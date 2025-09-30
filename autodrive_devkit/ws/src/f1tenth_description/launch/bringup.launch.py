#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_desc   = get_package_share_directory('f1tenth_description')
    pkg_bridge = get_package_share_directory('autodrive_f1tenth')
    pkg_slam   = get_package_share_directory('slam_toolbox')
    pkg_nav2   = get_package_share_directory('nav2_bringup')

    # URDF/Xacro (instalado em share/f1tenth_description/urdf)
    urdf_xacro = os.path.join(pkg_desc, 'urdf', 'f1tenth_car.urdf.xacro')
    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', urdf_xacro, ' use_sim_time:=true']),
            value_type=str
        )
    }

    # Bridge + RViz do projeto
    sim_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bridge, 'launch', 'simulator_bringup_rviz.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # SLAM toolbox (async)
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Nav2 demo
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': '/opt/ros/humble/share/nav2_bringup/maps/turtlebot3_world.yaml',
            'params_file': '/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml',
            'use_sim_time': 'true',
            'autostart': 'true'
        }.items()
    )

    # TF estático temporário (odom->base_link) só pra custo/TF não chorarem
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_link_pub',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen',
        ),
        static_tf,   # remova quando usar odometria real (ekf/localization)
        sim_bridge,
        slam,
        nav2,
    ])
