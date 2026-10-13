#!/usr/bin/env python3
"""
cinematica.py — Cinemática del robot Mecanum (Parte 4)

    ξ = [u, v, r]ᵀ                       velocidades en el marco del robot
    ω = [ω_di, ω_dd, ω_ti, ω_td]ᵀ        ruedas: delantera izq/der, trasera izq/der

    Directa : ξ = W ω     (W de 3x4, ver enunciado)
    Inversa : ω = W⁺ ξ

Lazo:
    /cmd_vel --W⁺--> /vel_rueda_* --> Gazebo (JointController) --> ruedas
    /joint_states --W--> ξ --η̇ = J(φ)ξ, Euler--> /odom + TF odom -> base_link

    ros2 run mi_mecanum_control cinematica --ros-args -p use_sim_time:=true
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster

# ===== TODO: parámetros físicos de SU variante (deben coincidir con el xacro) =====
A_RUEDA = 0.06     # a  : rueda_radio
LX = 0.36 / 2.0          # lx : distancia_ejes / 2
LY = 0.32 / 2.0          # ly : separacion_ruedas / 2

RUEDAS = ['del_izq', 'del_der', 'tra_izq', 'tra_der']
JOINTS = [f'joint_rueda_{p}' for p in RUEDAS]
TOPICOS = [f'/vel_rueda_{p}' for p in RUEDAS]


def matriz_W(a, lx, ly):
    """Cinemática directa del Mecanum: ξ = W ω  (3x4)."""
    # TODO
    k = 1.0 / (lx + ly)
    return (a / 4.0) * np.array([
        [1.0,  1.0,  1.0,  1.0],
        [-1.0, 1.0,  1.0, -1.0],
        [-k,   k,   -k,    k],
    ])
    raise NotImplementedError


def envolver(a):
    """Envuelve un ángulo a [-π, π]."""
    return math.atan2(math.sin(a), math.cos(a))


def integrar(x, y, phi, u, v, r, dt):
    """η̇ = J(φ) ξ e integración de Euler. Devuelve (x, y, φ) con φ en [-π, π]."""
    # TODO: ojo, u y v están en el marco del robot, no en odom
    c, s = math.cos(phi), math.sin(phi)
    x_dot = u * c - v * s
    y_dot = u * s + v * c
    return x + x_dot * dt, y + y_dot * dt, envolver(phi + r * dt)
    raise NotImplementedError


class Cinematica(Node):
    def __init__(self):
        super().__init__('cinematica')
        self.W = matriz_W(A_RUEDA, LX, LY)
        self.W_pinv = np.linalg.pinv(self.W)    # TODO: ω = W⁺ ξ

        # Inversa
        self.create_subscription(Twist, '/cmd_vel', self.cb_cmd_vel, 10)
        self.pubs = [self.create_publisher(Float64, t, 10) for t in TOPICOS]

        # Directa + odometría
        self.create_subscription(JointState, '/joint_states', self.cb_joints, 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.tf = TransformBroadcaster(self)

        self.x = self.y = self.phi = 0.0
        self.t_ant = None

        np.set_printoptions(precision=4, suppress=True)
        self.get_logger().info(f'\nW =\n{self.W}\nW⁺ =\n{self.W_pinv}')

    # ---------- Cinemática inversa: ω = W⁺ ξ ----------
    def cb_cmd_vel(self, msg):
        # TODO: armar ξ desde msg, calcular ω y publicar cada ω en su publisher (Float64)
        xi = np.array([msg.linear.x, msg.linear.y, msg.angular.z])
        omega = self.W_pinv @ xi
        for pub, w in zip(self.pubs, omega):
            m = Float64()
            m.data = float(w)
            pub.publish(m)

    # ---------- Cinemática directa: ξ = W ω, y odometría ----------
    def cb_joints(self, msg):
        # ω en el orden de JOINTS
        try:
            omega = np.array([msg.velocity[msg.name.index(j)] for j in JOINTS])
        except (ValueError, IndexError):
            return

        ahora = self.get_clock().now()
        if self.t_ant is None:
            self.t_ant = ahora
            return
        dt = (ahora - self.t_ant).nanoseconds * 1e-9
        self.t_ant = ahora
        if dt <= 0.0:
            return

        # TODO: calcular u, v, r con W y actualizar self.x, self.y, self.phi
        #u = v = r = 0.0
        try:
            idx = [msg.name.index(j) for j in JOINTS]
        except ValueError:
            return
        if len(msg.velocity) > max(idx):
            omega = np.array([msg.velocity[i] for i in idx])
        elif len(msg.position) > max(idx):
            # respaldo: derivar la posición si el publisher no manda velocidad
            pos = np.array([msg.position[i] for i in idx])
            if self.pos_ant is None:
                self.pos_ant = pos
                self.t_ant = ahora
                return
            omega = (pos - self.pos_ant) / dt
            self.pos_ant = pos
        else:
            return
        self.t_ant = ahora

        u, v, r = self.W @ omega
        self.x, self.y, self.phi = integrar(self.x, self.y, self.phi, u, v, r, dt)

        qz, qw = math.sin(self.phi / 2.0), math.cos(self.phi / 2.0)
        stamp = ahora.to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        # TODO: llenar odom.pose.pose (posición y orientación) y odom.twist.twist (u, v, r)
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = float(u)
        odom.twist.twist.linear.y = float(v)
        odom.twist.twist.angular.z = float(r)
        self.pub_odom.publish(odom)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf.sendTransform(t)


def main():
    rclpy.init()
    nodo = Cinematica()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
