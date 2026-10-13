"""Conversión URDF -> SDF con fricción anisotrópica para ruedas Mecanum.

NO MODIFICAR (viene resuelto en el examen). Lo usa simulacion.launch.py.

¿Por qué existe esto?
    Una rueda Mecanum real tiene rodillos libres a 45°. En Gazebo se emula con
    fricción anisotrópica en la colisión de cada rueda:
        mu  = 1  en la dirección fdir1 (la rueda "agarra")
        mu2 = 0  en la dirección perpendicular (el rodillo gira libre)
    fdir1 debe estar expresada en base_link (si se expresa en el link de la
    rueda, gira con ella y deja de apuntar a 45°). Eso se escribe en SDF como
        <fdir1 gz:expressed_in="base_link">1 -1 0</fdir1>
    pero el atributo gz:expressed_in se pierde cuando Gazebo convierte un URDF
    (por ejemplo con `create -topic robot_description`). Por eso el launch convierte el URDF a SDF con `gz sdf -p` e inserta aquí
    la fricción de cada rueda antes de hacer el spawn.

Direcciones (rodillos en configuración "X" vista desde arriba), coherentes con
ω = W⁺ ξ, ξ = [u, v, r]ᵀ, a = radio de rueda:
    ω_del_izq = (u - v - (lx+ly) r) / a      fdir1 = ( 1, -1, 0)
    ω_del_der = (u + v + (lx+ly) r) / a      fdir1 = ( 1,  1, 0)
    ω_tra_izq = (u + v - (lx+ly) r) / a      fdir1 = ( 1,  1, 0)
    ω_tra_der = (u - v + (lx+ly) r) / a      fdir1 = ( 1, -1, 0)
"""

import re
import subprocess
import tempfile

RUEDAS_FDIR1 = {
    'rueda_del_izq': '1 -1 0',
    'rueda_del_der': '1 1 0',
    'rueda_tra_izq': '1 1 0',
    'rueda_tra_der': '1 -1 0',
}


def _surface(fdir1: str) -> str:
    return (
        '<surface><friction><ode>'
        '<mu>1.0</mu><mu2>0.0</mu2>'
        f"<fdir1 gz:expressed_in='base_link'>{fdir1}</fdir1>"
        '</ode></friction></surface>'
    )


def insertar_friccion_mecanum(sdf: str) -> str:
    """Inserta la fricción anisotrópica en las colisiones de las 4 ruedas."""
    for link, fdir1 in RUEDAS_FDIR1.items():
        link_re = re.compile(
            r"(<link\s+name=['\"]" + re.escape(link) + r"['\"]\s*>)(.*?)(</link>)",
            re.DOTALL)
        m = link_re.search(sdf)
        if m is None:
            raise RuntimeError(
                f"No se encontró el link '{link}' en el SDF. Los links de las "
                "ruedas deben llamarse rueda_del_izq, rueda_del_der, "
                "rueda_tra_izq y rueda_tra_der.")
        body = m.group(2)

        def fix_collision(cm):
            inner = re.sub(r'<surface>.*?</surface>', '', cm.group(2), flags=re.DOTALL)
            return cm.group(1) + inner + _surface(fdir1) + cm.group(3)

        new_body, n = re.subn(r'(<collision\b[^>]*>)(.*?)(</collision>)',
                              fix_collision, body, flags=re.DOTALL)
        if n == 0:
            raise RuntimeError(f"El link '{link}' no tiene <collision>.")
        sdf = sdf[:m.start(2)] + new_body + sdf[m.end(2):]
    return sdf


def urdf_a_sdf_mecanum(urdf: str) -> str:
    """URDF (texto) -> SDF (texto) listo para spawn, usando `gz sdf -p`."""
    with tempfile.NamedTemporaryFile('w', suffix='.urdf', delete=False) as f:
        f.write(urdf)
        urdf_path = f.name
    result = subprocess.run(['gz', 'sdf', '-p', urdf_path],
                            capture_output=True, text=True)
    if result.returncode != 0 or '<sdf' not in result.stdout:
        raise RuntimeError('`gz sdf -p` falló al convertir el URDF:\n' + result.stderr)
    return insertar_friccion_mecanum(result.stdout)


def escribir_sdf_mecanum(urdf: str) -> str:
    """Convierte, inserta la fricción y devuelve la ruta de un .sdf temporal."""
    sdf = urdf_a_sdf_mecanum(urdf)
    with tempfile.NamedTemporaryFile('w', suffix='.sdf', delete=False) as f:
        f.write(sdf)
        return f.name
