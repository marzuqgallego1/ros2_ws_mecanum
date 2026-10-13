# Examen Robótica Móvil — Robot Mecanum (ROS 2 Jazzy + Gazebo Harmonic)

El enunciado está en Notion. Complete los `TODO` con los valores de **su variante**.

| Paquete | Contenido |
|---|---|
| `mi_robot_mecanum` | urdf, mundo `pista`, launch, rviz |
| `mi_mecanum_control` | nodos `cinematica` (W, W⁺, odometría) e `ir_a_puntos` (ec. 21) |

## Compilar

```bash
cd ~/ros2_ws_mecanum
colcon build --symlink-install
source install/setup.bash
```

## Ejecutar

```bash
# Terminal 1: Gazebo + robot + bridge + cinemática + RViz
ros2 launch mi_robot_mecanum simulacion.launch.py

# Terminal 2: prueba con teclado (Shift + J / L = desplazamiento lateral)
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Terminal 2: control a puntos
ros2 run mi_mecanum_control ir_a_puntos --ros-args -p use_sim_time:=true
```

## Dónde están los TODO

| Archivo | Parte |
|---|---|
| `mi_robot_mecanum/urdf/mi_robot.urdf.xacro` | 1 (ruedas, motores, encoders) y 2 (sensores) |
| `mi_robot_mecanum/worlds/pista.sdf` | 2 (sistema de sensores) |
| `mi_robot_mecanum/launch/simulacion.launch.py` | 3 (bridge y nodo cinematica) |
| `mi_robot_mecanum/rviz/control.rviz` | 3 |
| `mi_mecanum_control/mi_mecanum_control/cinematica.py` | 4 |
| `mi_mecanum_control/mi_mecanum_control/ir_a_puntos.py` | 5 |

**No modifique** `launch/sdf_mecanum.py` ni la `<collision>`.

## Depuración

```bash
xacro src/mi_robot_mecanum/urdf/mi_robot.urdf.xacro > /tmp/check.urdf && echo OK
gz topic -l
ros2 topic list
ros2 topic echo /joint_states --once
ros2 node info /cinematica
```
