#!/usr/bin/python3
import rclpy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, Float64
from d2w_ros2.kinematics import d2w_kine
from rclpy.node import Node

class ControllerPublisher(Node):
    def __init__(self):
        super().__init__('controller')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.go)
        self.sub_ref = self.create_subscription(Float64MultiArray, 'ref', self.ctrl_callback, 10)
        self.sub_odom =  self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        #self.R = 0.0485
        #self.l1 = 0.175
        #self.l2 = 0.165
        self.R = 0.127
        self.l1 = 0.5975
        self.l2 = 0.735
        self.k = 1
        self.k2 = 5
        self.phid = [0.,0.,0.,0.]
        self.state = [0.,0.,0.]
        self.ref = [0.,0.,0.]
        self.xc = 0.
        self.yc = 0.
        self.psi = 0.


    def ctrl_callback(self,msg):
        self.ref = msg.data

    def odom_callback(self,msg):
        self.xc = msg.pose.pose.position.x
        self.yc = msg.pose.pose.position.y
        self.psi = self.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]
        self.state = (self.xc, self.yc, self.psi)

    def go(self):
        psi = self.state[2]
        K=np.array([[(self.k*(np.cos(psi)+np.sin(psi)))/self.R, -((self.k*(np.cos(psi) - np.sin(psi)))/self.R), -((self.k*(self.l1+self.l2))/self.R)], [(self.k*(np.cos(psi) - np.sin(psi)))/self.R, (self.k*(np.cos(psi) + np.sin(psi)))/self.R, (self.k*(self.l1 + self.l2))/self.R], [(self.k*(np.cos(psi)-np.sin(psi)))/self.R, (self.k*(np.cos(psi) + np.sin(psi)))/self.R, -((self.k2*(self.l1+self.l2))/self.R)]])
        N=np.array([[(self.k*(np.cos(psi)+np.sin(psi)))/self.R, -((self.k*(np.cos(psi) - np.sin(psi)))/self.R), -((self.k*(self.l1+self.l2))/self.R)], [(self.k*(np.cos(psi) - np.sin(psi)))/self.R, (self.k*(np.cos(psi) + np.sin(psi)))/self.R, (self.k*(self.l1 + self.l2))/self.R], [(self.k*(np.cos(psi)-np.sin(psi)))/self.R, (self.k*(np.cos(psi) + np.sin(psi)))/self.R, -((self.k2*(self.l1+self.l2))/self.R)]])
        input = [0,0,0,0]
        input[0:3] = N.dot(self.ref)-K.dot(self.state)
        input[3] = input[0]+input[1]-input[2]
        self.phid = np.clip(input,-10,10)

        xdot = d2w_kine(self.state,self.phid)

        move = Twist()
        move.linear.x = xdot[0]
        move.linear.y = xdot[1]
        move.linear.z = 0.
        move.angular.x = 0.
        move.angular.y = 0.
        move.angular.z = xdot[2]

        self.pub.publish(move)

    def euler_from_quaternion(self,quaternion):
        x = quaternion[0]
        y = quaternion[1]
        z = quaternion[2]
        w = quaternion[3]

        sinr_cosp = 2 * (w * x * y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)
    ctrl_pub = ControllerPublisher()
    rclpy.spin(ctrl_pub)
    go()
    ctrl_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
