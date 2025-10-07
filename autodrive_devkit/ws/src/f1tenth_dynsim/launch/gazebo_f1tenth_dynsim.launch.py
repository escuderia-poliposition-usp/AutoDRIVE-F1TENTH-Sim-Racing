#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # === Pacotes ===
    pkg_desc   = get_package_share_directory('f1tenth_description')   # Xacro do carro
    pkg_bridge = get_package_share_directory('autodrive_f1tenth')     # seu bridge + RViz
    pkg_slam   = get_package_share_directory('slam_toolbox')
    pkg_nav2   = get_package_share_directory('nav2_bringup')
    pkg_gz_ros = get_package_share_directory('gazebo_ros')
    pkg_dynsim = get_package_share_directory('f1tenth_dynsim')        # contém worlds/monza.world

    # === World do Gazebo ===
    world_file = os.path.join(pkg_dynsim, 'worlds', 'monza.world')

    # === Args ===
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # === URDF/Xacro -> robot_description ===
    urdf_xacro = os.path.join(pkg_desc, 'urdf', 'f1tenth_car.urdf.xacro')
    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', urdf_xacro, ' use_sim_time:=', use_sim_time]),
            value_type=str
        )
    }

    # === Publicadores de estado e TF ===
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

        # Caminho do YAML (adicione isso onde você já definiu os paths)
    ekf_yaml  = os.path.join(pkg_dynsim, 'config', 'ekf.yaml')

    # EKF (robot_localization)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_yaml, {'use_sim_time': use_sim_time}],
    )

    # === Gazebo (server+client) COM WORLD ===
    # Passa o argumento 'world' para o launch do gazebo_ros
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gz_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'verbose': 'true',
            'world': world_file
        }.items()
    )

    # === Spawn do robô (via tópico robot_description) ===
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_f1tenth',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'f1tenth_1',
            '-x', '-0.000202',
            '-y', '-0.003571',
            # se o modelo não aparece, suba um pouco o z (ex.: 0.05)
            '-z',  '0.050000',
            # deixe roll/pitch = 0.0 e só aplique yaw
            '-Y',  '1.757631',
        ],
    )

    # === Bridge + RViz do projeto (o seu bringup) ===
    sim_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bridge, 'launch', 'simulator_bringup_rviz.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # === SLAM toolbox (async) ===
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # === Nav2 Bringup (Ackermann config) ===
    nav2_params = os.path.join(pkg_dynsim, 'config', 'nav2_params.yaml')


    map_yaml = LaunchConfiguration('map', default='/root/mapa_corrida.yaml')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml,
            'params_file': nav2_params,
            'use_sim_time': 'true',
            'autostart': 'true'
        }.items()
    )


    # Ambiente: silencia warning do Qt dentro do container
    xdg_runtime = SetEnvironmentVariable('XDG_RUNTIME_DIR', '/tmp/runtime-root')

    return LaunchDescription([
        use_sim_time_arg,
        xdg_runtime,
        robot_state_pub,
        joint_state_pub,
        ekf_node,
        gazebo,          # sobe Gazebo com o MONZA.world
        spawn_entity,    # spawn do robô
        sim_bridge,      # seu bringup (inclui RViz)
        slam,            # SLAM toolbox
        #nav2,            # Nav2 demo
    ])
