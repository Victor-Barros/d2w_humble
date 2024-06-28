#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import TwistWithCovariance
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry

class Refpub(Node):
    def __init__(self):
        super().__init__('ref_pub')
        self.pub_ref = self.create_publisher(Float64MultiArray, '/ref', 10)
        self.pub_compute = self.create_publisher(Odometry, '/odom', 10)

        self.x0 = 0.
        self.y0 = 0.
        self.w0 = 0.

        self.delta_t = 0.01
        self.theta1_0 = 0.
        self.theta2_0 = 0.
        self.theta3_0 = 0.
        self.theta4_0 = 0.
        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.sub_Vel = self.create_subscription(TwistWithCovariance, 'twist_odom',self.listener_odometry,10 )

    def listener_odometry(self, sensor):
        self.vel_x = sensor.twist.linear.x
        self.vel_y = sensor.twist.linear.y
        self.vel_w = sensor.twist.angular.z

        x = self.x0 + self.vel_x * self.delta_t
        y = self.y0 + self.vel_y * self.delta_t
        w = self.w0 + self.vel_w * self.delta_t

        self.x0 = x
        self.y0 = y
        self.w0 = w


    def timer_callback(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.pose.pose.position.x = self.x0
        odom.pose.pose.position.y =  self.y0
        odom.pose.pose.orientation.w = self.w0
        

        '''        arr = input()
        l = list(map(float, arr.split(' ')))

        if len(l) > 3:
            msg.data = l[0:3]
            self.pub_ref.publish(msg)
        else:
            self.get_logger().info('Error!')'''

        self.pub_compute.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    ref_pub = Refpub()
    rclpy.spin(ref_pub)
    ref_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

