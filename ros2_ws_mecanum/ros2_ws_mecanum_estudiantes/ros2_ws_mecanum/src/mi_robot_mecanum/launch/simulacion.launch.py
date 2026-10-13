"""
Lanza la pista en Gazebo, el robot Mecanum, el bridge ROS 2 <-> Gazebo,
el nodo de cinemática y RViz.

    ros2 launch mi_robot_mecanum simulacion.launch.py
"""
import os
import sys

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sdf_mecanum import escribir_sdf_mecanum  # noqa: E402  (dado, NO MODIFICAR)


def generate_launch_description():
    pkg = get_package_share_directory('mi_robot_mecanum')
    xacro_file = os.path.join(pkg, 'urdf', 'mi_robot.urdf.xacro')
    world_file = os.path.join(pkg, 'worlds', 'pista.sdf')
    rviz_file = os.path.join(pkg, 'rviz', 'control.rviz')

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    rviz = LaunchConfiguration('rviz')

    # Dado: SDF con la fricción de los rodillos Mecanum (por eso el spawn usa -file)
    sdf_file = escribir_sdf_mecanum(xacro.process_file(xacro_file).toxml())

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )

    spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-name', 'mi_robot', '-file', sdf_file,
                   '-x', '0.0', '-y', '0.0', '-z', '0.1'],
        output='screen',
    )

    # ]  ROS -> Gazebo     [  Gazebo -> ROS
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # TODO (Parte 3): motores /vel_rueda_del_izq, /vel_rueda_del_der,
            #                 /vel_rueda_tra_izq, /vel_rueda_tra_der  (std_msgs/Float64)
            # TODO: encoders  /world/pista/model/mi_robot/joint_state
            # TODO: LiDAR     /scan
            # TODO: cámara    /camara/image, /camara/depth_image, /camara/points, /camara/camera_info
            '/vel_rueda_del_izq@std_msgs/msg/Float64]gz.msgs.Double',
            '/vel_rueda_del_der@std_msgs/msg/Float64]gz.msgs.Double',
            '/vel_rueda_tra_izq@std_msgs/msg/Float64]gz.msgs.Double',
            '/vel_rueda_tra_der@std_msgs/msg/Float64]gz.msgs.Double',
            '/world/pista/model/mi_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camara/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camara/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camara/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/camara/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        # TODO: remapping del joint_state de Gazebo a /joint_states
        remappings=[
            ('/world/pista/model/mi_robot/joint_state', '/joint_states'),
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # TODO (Parte 3): nodo 'cinematica' del paquete mi_mecanum_control, con use_sim_time
    cinematica = Node(
        package='mi_mecanum_control', executable='cinematica',
        name='cinematica',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )
    
    rviz_node = Node(
        package='rviz2', executable='rviz2', arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': True}], condition=IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        gazebo, robot_state_publisher, spawn, bridge, cinematica, rviz_node,
    ])
