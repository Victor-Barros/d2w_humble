#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
#from .submodules.kinematics import d2w_inv_kine

class velPublisher(Node):
    def __init__(self):
        super().__init__('vel_publisher')
        self.publisher_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

def go(pub):
    while rclpy.ok():
        move = Twist()

        arr = input()
        l = list(map(float,arr.split(' ')))

        if len(l) == 3:
            move.linear.x = l[0]
            move.linear.y = l[1]
            move.linear.z = 0.0
            move.angular.x = 0.0
            move.angular.y = 0.0
            move.angular.z = l[2]
            pub.publisher_cmd_vel.publish(move)

        else:
            pub.get_logger().info('Erro!')

def main(args=None):
    rclpy.init(args=args)

    vel_publisher = velPublisher()

    go(vel_publisher)

    rclpy.spin(vel_publisher)
    vel_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
