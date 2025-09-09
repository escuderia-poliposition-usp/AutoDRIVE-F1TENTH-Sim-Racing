from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Resolve o share do pacote f1tenth_description (sem caminho absoluto!)
    desc_share = get_package_share_directory('f1tenth_description')
    xacro_file = os.path.join(desc_share, 'urdf', 'f1tenth_car.urdf.xacro')

    # Saída temporária do URDF gerado
    urdf_out = '/tmp/f1tenth_car.urdf'

    return LaunchDescription([
        # Gera o URDF a partir do Xacro e abre no Gazebo Classic
        ExecuteProcess(
            cmd=['bash', '-lc', f'xacro "{xacro_file}" > "{urdf_out}" && gazebo --verbose "{urdf_out}"'],
            output='screen'
        ),

        # Bridge 1: Twist -> steer/throttle/brake
        Node(
            package='f1tenth_dynsim',
            executable='twist_to_actuators.py',
            name='twist_to_actuators',
            output='screen',
            parameters=[{
                'wheelbase': 0.26,
                'steer_limit_deg': 40.0,
                'throttle_gain': 50.0,
                'brake_gain': 80.0,
                'cmd_vel': '/cmd_vel',
                'steer_topic': '/autodrive/f1tenth_1/steering_command',
                'throttle_topic': '/autodrive/f1tenth_1/throttle_command',
                'brake_topic': '/autodrive/f1tenth_1/brake_command',
            }]
        ),
    ])
