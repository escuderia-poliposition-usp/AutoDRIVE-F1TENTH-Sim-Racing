#!/usr/bin/env python3
# file: scripts/twist_to_actuators.py
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class TwistToActuators(Node):
    def __init__(self):
        super().__init__('twist_to_actuators')

        # Parâmetros
        self.declare_parameter('wheelbase', 0.26)  # m
        self.declare_parameter('steer_limit_deg', 40.0)
        self.declare_parameter('throttle_gain', 50.0)   # throttle% = gain * v [m/s]
        self.declare_parameter('brake_gain', 80.0)      # % por m/s de redução
        self.declare_parameter('cmd_vel', '/cmd_vel')
        self.declare_parameter('steer_topic', '/autodrive/f1tenth_1/steering_command')
        self.declare_parameter('throttle_topic', '/autodrive/f1tenth_1/throttle_command')
        self.declare_parameter('brake_topic', '/autodrive/f1tenth_1/brake_command')

        self.L = float(self.get_parameter('wheelbase').value)
        self.steer_lim = math.radians(float(self.get_parameter('steer_limit_deg').value))
        self.k_thr = float(self.get_parameter('throttle_gain').value)
        self.k_brk = float(self.get_parameter('brake_gain').value)

        self.pub_steer = self.create_publisher(Float32, self.get_parameter('steer_topic').value, 10)
        self.pub_thr   = self.create_publisher(Float32, self.get_parameter('throttle_topic').value, 10)
        self.pub_brk   = self.create_publisher(Float32, self.get_parameter('brake_topic').value, 10)

        self.sub = self.create_subscription(Twist, self.get_parameter('cmd_vel').value, self.on_twist, 10)

        self.prev_v = 0.0
        self.get_logger().info('TwistToActuators ON')

    def on_twist(self, msg: Twist):
        v = float(msg.linear.x)
        w = float(msg.angular.z)

        # Ângulo (modelo bicicleta) com proteção v~0
        v_eff = v if abs(v) > 0.05 else math.copysign(0.05, v if v != 0.0 else 1.0)
        theta = math.atan((self.L * w) / v_eff) if abs(v_eff) > 1e-6 else 0.0
        theta = max(-self.steer_lim, min(self.steer_lim, theta))

        # Throttle proporcional a v (clip 0..100)
        thr = max(0.0, min(100.0, self.k_thr * max(0.0, v)))

        # Brake entra quando reduzimos v alvo
        decel = max(0.0, self.prev_v - v)
        brk = max(0.0, min(100.0, self.k_brk * decel))

        self.prev_v = v

        self.pub_steer.publish(Float32(data=float(theta)))
        self.pub_thr.publish(Float32(data=float(thr)))
        self.pub_brk.publish(Float32(data=float(brk)))

def main():
    rclpy.init()
    node = TwistToActuators()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
