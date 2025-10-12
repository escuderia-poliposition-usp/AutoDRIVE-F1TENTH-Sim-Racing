#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable, GroupAction
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ==== Pacotes / paths ====
    pkg_desc   = get_package_share_directory('f1tenth_description')   # URDF/Xacro do carro
    pkg_bridge = get_package_share_directory('autodrive_f1tenth')     # bridge + RViz (opcional)
    pkg_nav2   = get_package_share_directory('nav2_bringup')          # Nav2 bringup
    pkg_gz_ros = get_package_share_directory('gazebo_ros')            # Gazebo Classic (ROS)
    pkg_dynsim = get_package_share_directory('f1tenth_dynsim')        # worlds + configs locais

    world_file   = os.path.join(pkg_dynsim, 'worlds', 'monza.world')
    nav2_params  = os.path.join(pkg_dynsim, 'config', 'nav2_params.yaml')
    urdf_xacro   = os.path.join(pkg_desc, 'urdf', 'f1tenth_car.urdf.xacro')

    # ==== Args ====
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    use_sim_time     = LaunchConfiguration('use_sim_time')

    start_rviz_arg   = DeclareLaunchArgument('start_rviz',   default_value='true')
    start_rviz       = LaunchConfiguration('start_rviz')

    start_bridge_arg = DeclareLaunchArgument('start_bridge', default_value='false')  # evite conflito porta 4567
    start_bridge     = LaunchConfiguration('start_bridge')

    # ==== robot_description via xacro ====
    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', urdf_xacro, ' use_sim_time:=', use_sim_time]),
            value_type=str
        )
    }

    # ==== State publishers ====
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

    # ==== Gazebo (server+client) com WORLD ====
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gz_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'verbose': 'true',
            'world': world_file
        }.items()
    )

    # ==== Spawn do robô (via /robot_description) ====
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
            '-z',  '0.05',
            '-Y',  '1.757631',
        ],
    )

    # ==== (Opcional) RViz/bridge do seu pacote ====
    sim_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bridge, 'launch', 'simulator_bringup_rviz.launch.py')
        ),
        condition=IfCondition(start_bridge),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    rviz_only = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'rviz_launch.py')
        ),
        condition=IfCondition(start_rviz),
        launch_arguments={
            'use_namespace': 'false',
            'namespace': '',
        }.items()
    )

    # ==== CONTAINER composable do Nav2 (fundamental quando use_composition:=True) ====
    nav2_container = Node(
        package='rclcpp_components',
        executable='component_container_isolated',
        name='nav2_container',
        output='screen',
        parameters=[{'use_sim_time': True, 'autostart': True}],
        # remappings típicos do bringup (tf / tf_static)
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )

    # ==== Nav2 (APENAS navigation_launch.py) ====
    # Com o container acima, o navigation_launch vai carregar:
    # controller_server, planner_server, costmaps, bt_navigator, velocity_smoother, lifecycle_manager_navigation, etc.
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'namespace':        '',
            'use_sim_time':     'true',
            'autostart':        'true',
            'params_file':      nav2_params,
            'use_composition':  'True',
            'use_respawn':      'False',
            'container_name':   'nav2_container',
        }.items()
    )

    # Ambiente: silencia warning do Qt dentro do container
    xdg_runtime = SetEnvironmentVariable('XDG_RUNTIME_DIR', '/tmp/runtime-root')

    return LaunchDescription([
        use_sim_time_arg,
        start_rviz_arg,
        start_bridge_arg,
        xdg_runtime,

        # Robô + sim
        robot_state_pub,
        joint_state_pub,
        gazebo,
        spawn_entity,

        # Visualização (à sua escolha)
        sim_bridge,
        rviz_only,

        # Nav2 container + navigation (sem SLAM e sem localization)
        nav2_container,
        navigation,
    ])
