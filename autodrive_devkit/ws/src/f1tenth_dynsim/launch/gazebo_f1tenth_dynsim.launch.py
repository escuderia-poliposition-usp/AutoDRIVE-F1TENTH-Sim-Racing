#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable,
    TimerAction
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # === Pacotes / paths ===
    pkg_desc   = get_package_share_directory('f1tenth_description')   # URDF/Xacro do carro
    pkg_nav2   = get_package_share_directory('nav2_bringup')          # Nav2 bringup
    pkg_gz_ros = get_package_share_directory('gazebo_ros')            # Gazebo Classic (ROS)
    pkg_dynsim = get_package_share_directory('f1tenth_dynsim')        # worlds + configs locais
    pkg_slam   = get_package_share_directory('slam_toolbox')          # slam_toolbox local

    world_file   = os.path.join(pkg_dynsim, 'worlds', 'monza.world')
    nav2_params  = os.path.join(pkg_dynsim, 'config', 'nav2_params.yaml')
    urdf_xacro   = os.path.join(pkg_desc, 'urdf', 'f1tenth_car.urdf.xacro')

    # SLAM params do pacote slam_toolbox (você comentou que baixou local)
    slam_params  = os.path.join(pkg_slam, 'config', 'mapper_params_online_async.yaml')

    # BT XML **absoluto** do pacote nav2_bt_navigator (evita missing .so e segfault)
    bt_xml_default = '/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_w_replanning_and_recovery.xml'

    # === Args ===
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    use_sim_time     = LaunchConfiguration('use_sim_time')

    start_rviz_arg   = DeclareLaunchArgument('start_rviz', default_value='true')
    start_rviz       = LaunchConfiguration('start_rviz')

    with_slam_arg    = DeclareLaunchArgument('with_slam', default_value='true', description='Sobe o SLAM toolbox?')
    with_slam        = LaunchConfiguration('with_slam')

    use_composition_arg = DeclareLaunchArgument('use_composition', default_value='False', description='Nav2 em container?')
    use_composition     = LaunchConfiguration('use_composition')

    bt_xml_arg = DeclareLaunchArgument(
        'bt_xml',
        default_value=bt_xml_default,
        description='Caminho absoluto do Behavior Tree XML do Nav2'
    )
    bt_xml = LaunchConfiguration('bt_xml')

    # === robot_description via xacro ===
    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', urdf_xacro, ' use_sim_time:=', use_sim_time]),
            value_type=str
        )
    }

    # === State publishers ===
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

    # === Gazebo (server+client) com WORLD ===
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gz_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'verbose': 'true',
            'world': world_file
        }.items()
    )

    # === Spawn do robô (via /robot_description) ===
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

    # === (Opcional) RViz do Nav2 ===
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

    # === SLAM Toolbox (async) — opcional ===
    slam_toolbox = Node(
        condition=IfCondition(with_slam),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': use_sim_time}],
        remappings=[]
    )

    # === Nav2 (navigation_launch.py) — **sem composição** por padrão ===
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time':    use_sim_time,
            'autostart':       'true',
            'params_file':     nav2_params,
            'use_composition': use_composition,    # default False (estável)
            'use_respawn':     'False',
            'container_name':  'nav2_container',
            'bt_xml':          bt_xml,             # respeita caminho absoluto
        }.items()
    )

    # Ambiente: silencia warning do Qt dentro do container
    xdg_runtime = SetEnvironmentVariable('XDG_RUNTIME_DIR', '/tmp/runtime-root')

    # Pequeno delay antes do Nav2 para garantir TF/robot_description ok
    navigation_delayed = TimerAction(period=2.0, actions=[navigation])

    return LaunchDescription([
        # args
        use_sim_time_arg,
        start_rviz_arg,
        with_slam_arg,
        use_composition_arg,
        bt_xml_arg,

        # env
        xdg_runtime,

        # sim + robô
        robot_state_pub,
        joint_state_pub,
        gazebo,
        spawn_entity,

        # visualização
        rviz_only,

        # SLAM (opcional)
        slam_toolbox,

        # Nav2 com delay
        navigation_delayed,
    ])
