#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""All-in-one (Gazebo Classic) no estilo do tb3_simulation.launch.py, adaptado ao F1TENTH."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Diretórios base
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir  = os.path.join(bringup_dir, 'launch')
    pkg_desc    = get_package_share_directory('f1tenth_description')
    pkg_dynsim  = get_package_share_directory('f1tenth_dynsim')

    # === Launch configs (espelhando o tb3_simulation) ===
    slam            = LaunchConfiguration('slam')
    namespace       = LaunchConfiguration('namespace')
    use_namespace   = LaunchConfiguration('use_namespace')
    map_yaml_file   = LaunchConfiguration('map')
    use_sim_time    = LaunchConfiguration('use_sim_time')
    params_file     = LaunchConfiguration('params_file')
    autostart       = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn     = LaunchConfiguration('use_respawn')

    # Sim / Visualização
    rviz_config_file    = LaunchConfiguration('rviz_config_file')
    use_simulator       = LaunchConfiguration('use_simulator')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_joint_state_pub = LaunchConfiguration('use_joint_state_pub')
    use_rviz            = LaunchConfiguration('use_rviz')
    headless            = LaunchConfiguration('headless')
    world               = LaunchConfiguration('world')
    pose = {
        'x': LaunchConfiguration('x_pose', default='0.00'),
        'y': LaunchConfiguration('y_pose', default='0.00'),
        'z': LaunchConfiguration('z_pose', default='0.05'),
        'R': LaunchConfiguration('roll',  default='0.00'),
        'P': LaunchConfiguration('pitch', default='0.00'),
        'Y': LaunchConfiguration('yaw',   default='0.00'),
    }
    robot_name = LaunchConfiguration('robot_name')

    # Remappings (mesmo racional do oficial)
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # === Declare arguments ===
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace', default_value='false',
        description='Whether to apply a namespace to the navigation stack'
    )

    declare_slam_cmd = DeclareLaunchArgument(
        'slam', default_value='False', description='Whether run a SLAM'
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_dynsim, 'maps', 'tb3_sandbox.yaml'),
        description='Full path to map file to load'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dynsim, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file for all nodes'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack'
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='True',
        description='Whether to use composed bringup'
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes (when composition is disabled)'
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(bringup_dir, 'rviz', 'nav2_default_view.rviz'),
        description='Full path to the RVIZ config file to use'
    )

    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator', default_value='True',
        description='Whether to start the simulator'
    )

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub', default_value='True',
        description='Whether to start the robot state publisher'
    )

    declare_use_joint_state_pub_cmd = DeclareLaunchArgument(
        'use_joint_state_pub', default_value='True',
        description='Whether to start the joint state publisher'
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='True',
        description='Whether to start RVIZ'
    )

    declare_simulator_cmd = DeclareLaunchArgument(
        'headless', default_value='True',
        description='Whether to execute gzclient'
    )

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_dynsim, 'worlds', 'monza.world'),
        description='Full path to world model file to load'
    )

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name', default_value='f1tenth_1', description='name of the robot'
    )

    # === Gazebo Classic (server + client) ===
    start_gazebo_server_cmd = ExecuteProcess(
        condition=IfCondition(use_simulator),
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', world],
        output='screen'
    )

    start_gazebo_client_cmd = ExecuteProcess(
        condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])),
        cmd=['gzclient'],
        output='screen'
    )

    # === robot_description via XACRO ===
    urdf_xacro = os.path.join(pkg_desc, 'urdf', 'f1tenth_car.urdf.xacro')
    robot_description_param = {
        'robot_description': ParameterValue(
            Command(['xacro ', urdf_xacro, ' use_sim_time:=', use_sim_time]),
            value_type=str
        )
    }

    # === robot_state_publisher ===
    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[robot_description_param, {'use_sim_time': use_sim_time}],
        remappings=remappings
    )

    # === joint_state_publisher ===
    joint_state_publisher_cmd = Node(
        condition=IfCondition(use_joint_state_pub),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # === Spawner: usa o tópico /robot_description (com pequeno delay) ===
    start_gazebo_spawner_cmd = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                output='screen',
                arguments=[
                    '-entity', robot_name,
                    '-topic', 'robot_description',
                    '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
                    '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y']
                ]
            )
        ]
    )

    # === RViz ===
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': use_namespace,
            'rviz_config': rviz_config_file
        }.items()
    )

    # === Nav2 Bringup ===
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'bringup_launch.py')),
        launch_arguments={
            'namespace': namespace,
            'slam': slam,
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': use_composition,
            'use_respawn': use_respawn
        }.items()
    )

    # === LaunchDescription ===
    ld = LaunchDescription()

    # Declare options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_joint_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_robot_name_cmd)

    # Start sim & spawner
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(joint_state_publisher_cmd)
    ld.add_action(start_gazebo_spawner_cmd)

    # Nav2 & Viz
    ld.add_action(rviz_cmd)
    ld.add_action(bringup_cmd)

    return ld
