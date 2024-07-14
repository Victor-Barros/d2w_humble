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
        #publishers
        self.pub_ref = self.create_publisher(Float64MultiArray, 'ref', 10)
        self.pub_compute = self.create_publisher(Odometry, 'odom', 10)
        #timers
        timer_period = 0.01
        timer_period2 = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        #subscribers
        self.sub_Vel = self.create_subscription(TwistWithCovariance, 'twist_odom',self.listener_odometry,10 )
        #declared variables
        self.x0 = 0.
        self.y0 = 0.
        self.w0 = 0.

        self.leader = 0
        self.frame = "init"
        self.dir = 0

        self.last_x = 0.
        self.last_y = 0.
        self.last_z = 0.
        self.desired_x = 0.
        self.desired_y = 0.
        self.desired_w = 0.

        self.step_number = 100

        self.delta_t = 0.01
        self.theta1_0 = 0.
        self.theta2_0 = 0.
        self.theta3_0 = 0.
        self.theta4_0 = 0.

        self.i = 0

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
        self.trajectory()
        odom = Odometry()
        msg = Float64MultiArray()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.frame
        odom.pose.pose.position.x = self.x0
        odom.pose.pose.position.y =  self.y0
        odom.pose.pose.orientation.z = self.w0

        l =[self.vector_x[self.i], self.vector_y[self.i], self.vector_w[self.i]]

        msg.data = l

        self.pub_ref.publish(msg)
        self.pub_compute.publish(odom)

        if self.i == (self.step_number - 1):
            self.i = self.step_number - 1
        else:
            self.i = self.i + 1
    def trajectory(self):
        #square
        if self.leader == 0:
            if self.x0 < 1. and self.dir == 0:
                self.desired_x = 1.
                self.dir = 1
            if self.x0 > 0.93 and self.dir == 1:
                self.desired_y = 1.
                self.dir = 2
            if self.y0 > 0.93 and self.dir == 2:
                self.last_x = self.x0
                self.desired_x = 0.
                self.dir = 3
            if self.x0 < 0.1 and self.dir == 3:
                self.last_y = self.y0
                self.desired_y = 0.
                self.dir = 4
            if self.y0 < 0.1 and self.dir == 4:
                self.frame = "q_f"
        #triangle
        if self.leader == 1:
            if self.x0 < 1. and self.dir == 0:
                self.last_x = 0.
                self.last_y = 0.
                self.desired_x = 1.
                self.desired_y = 0.5
                self.dir = 1
            if self.x0 > 0.93 and self.y0 > 0.47 and self.dir == 1:
                self.last_x = 1.
                self.last_y = 0.5
                self.desired_x = 0.
                self.desired_y = 1.
                self.dir = 2
            if self.y0 > 0.93 and self.dir == 2:
                self.last_x = 0.
                self.last_y = 1.0
                self.desired_x = 0.
                self.desired_y = 0.
                self.dir = 3
            if self.y0 < 0.1 and self.dir == 3:
                self.frame = "t_f"
        #X
        if self.leader == 2:
            #left back
            if self.x0 == 0. and self.y0 == 0. and self.dir == 0:
                self.last_x = 0.
                self.last_y = 0.
                self.desired_x = -0.5
                self.desired_y = -0.5
                self.dir = 1
            #center
            if self.x0 < -0.46 and self.y0 < -0.46 and self.dir == 1:
                self.last_x = -0.5
                self.last_y = -0.5
                self.desired_x = 0.
                self.desired_y = 0.
                self.dir = 2
            #left front
            if self.x0 > -0.04 and self.y0 > -0.04 and self.dir == 2:
                self.last_x = 0.
                self.last_y = 0.
                self.desired_x = 0.5
                self.desired_y = -0.5
                self.dir = 3
            #center
            if self.x0 > 0.43 and self.y0 < -0.46 and self.dir == 3:
                self.last_x = 0.5
                self.last_y = -0.5
                self.desired_x = 0.
                self.desired_y = 0.
                self.dir = 4
            #front right
            if self.x0 < 0.04 and self.y0 > -0.06 and self.dir == 4:
                self.last_x = 0.
                self.last_y = 0.
                self.desired_x = 0.5
                self.desired_y = 0.5
                self.dir = 5
            #center
            if self.x0 > 0.46 and self.y0 > 0.44 and self.dir == 5:
                self.last_x = 0.5
                self.last_y = 0.5
                self.desired_x = 0.
                self.desired_y = 0.
                self.dir = 6
            #front back
            if self.x0 < 0.06 and self.y0 < 0.04 and self.dir == 6:
                self.last_x = 0.
                self.last_y = 0.
                self.desired_x = -0.5
                self.desired_y = 0.5
                self.dir = 7
            #center
            if self.x0 < -0.42 and self.y0 > 0.46 and self.dir == 7:
                self.last_x = -0.5
                self.last_y = 0.5
                self.desired_x = 0.
                self.desired_y = 0.
                self.dir = 8
            if self.y0 < 0.1 and self.dir == 8:
                self.frame = "c_f"
        self.vector_x = np.linspace(self.last_x, self.desired_x, self.step_number)
        self.vector_y = np.linspace(self.last_y, self.desired_y, self.step_number)
        self.vector_w = np.linspace(self.last_z, self.desired_w, self.step_number)


def main(args=None):
    rclpy.init(args=args)
    ref_pub = Refpub()
    rclpy.spin(ref_pub)
    ref_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

