#!/usr/bin/python3
import rclpy
import tf
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import concatenate_matrices, euler_from_quaternion, quarternion_from_euler
from std_msgs.msg import Float64MultiArray, Float64
from kinematics import d2w_kine
from rclpy.node import Node

class ControllerPublisher(Node):
    def __init__(self):
        super().__init__('controller')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.ctrl_pub1 = self.create_publisher('/robot/joint_flw_velocity_controller/command',Float64,10)
        self.ctrl_pub2 = self.create_publisher('/robot/joint_frw_velocity_controller/command',Float64,10)
        self.ctrl_pub3 = self.create_publisher('/robot/joint_blw_velocity_controller/command',Float64,10)
        self.ctrl_pub4 = self.create_publisher('/robot/joint_brw_velocity_controller/command',Float64,10)

        self.R = 0.127
        self.l1 = 0.5975
        self.l2 = 0.735
        self.k = 1
        self.k2 = 5
        self.phid = [0.,0.,0.,0.]
        self.state = [0.,0.,0.]
        self.ref = [0.,0.,0.]

    def ctrl_callback(msg):
        self.ref = msg.data

    def odom_callback(msg):
        xc = msg.pose.pose.position.x
        yc = msg.pose.pose.position.y
        psi = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]
        self.state = (xc, yx, psi)

    def go():
        psi = self.state[2]
        K=np.array([[(self.k*(np.cos(psi)+np.sin(psi)))/self.R, -((self.k*(np.cos(psi) - np.sin(psi)))/self.R), -((k*(l1+l2))/self.R)], [(k*(np.cos(psi) - np.sin(psi)))/self.R, (k*(np.cos(psi) + np.sin(psi)))/self.R, (k*(l1 + l2))/self.R], [(k*(np.cos(psi)-np.sin(psi)))/self.R, (k*(np.cos(psi) + np.sin(psi)))/self.R, -((k2*(l1+l2))/self.R)]])
        N=np.array([[(self.k*(np.cos(psi)+np.sin(psi)))/self.R, -((self.k*(np.cos(psi) - np.sin(psi)))/self.R), -((k*(l1+l2))/self.R)], [(k*(np.cos(psi) - np.sin(psi)))/self.R, (k*(np.cos(psi) + np.sin(psi)))/self.R, (k*(l1 + l2))/self.R], [(k*(np.cos(psi)-np.sin(psi)))/self.R, (k*(np.cos(psi) + np.sin(psi)))/self.R, -((k2*(l1+l2))/self.R)]])
        input = [0,0,0,0]
        input[0:3] = N.dot(self.ref)-K.dot(self.state)
        input[3] = input[0]+input[1]-input[2]
        self.phid = np.clip(input,-10,10)

        xdot = d2w_kine(self.state,self.phid)

        move = Twist()
        move.linear.x = xdot[0]
        move.linear.y = xdot[1]
        move.linear.z = 0
        move.angular.x = 0
        move.angular.y = 0
        move.angular.z = xdot[2]

        self.pub.publish(move)
        self.ctrl_pub1(self.phid[0])
        self.ctrl_pub2(self.phid[1])
        self.ctrl_pub3(self.phid[2])
        self.ctrl_pub4(self.phid[3])

def main(args=None):
    rclpy.init(args=args)
    ctrl_pub = ControllerPublisher()
    rclpy.spin(ctrl_pub)
    ctrl_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
