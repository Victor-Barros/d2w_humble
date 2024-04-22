#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistWithCovariance 
#from nav_msgs.msg import Odometry
#import tf_transformations
from std_msgs.msg import Float64MultiArray, Float64
from d2w_ros2.kinematics import d2w_kine
from d2w_ros2.kinematics import d2w_inv_kine

class velDrive(Node):

    def __init__(self):
        super().__init__('vel_drive')

        #Common Variables
        self.phid = [0.,0.,0.,0.]
        self.state = [0.,0.,0.,0.]
        self.linear_state = [0.,0.,0.]
        self.move = Twist()

        #Publishers
        self.ctrl_pub_1 = self.create_publisher(Float64, '/robot/joint_flw_velocity_controller/command', 10)
        self.ctrl_pub_2 = self.create_publisher(Float64, '/robot/joint_frw_velocity_controller/command', 10)
        self.ctrl_pub_3 = self.create_publisher(Float64, '/robot/joint_blw_velocity_controller/command', 10)
        self.ctrl_pub_4 = self.create_publisher(Float64, '/robot/joint_brw_velocity_controller/command', 10)

        self.publisher_joint_vel = self.create_publisher(Float64MultiArray, '/joint_vel', 10)
        self.publisher_odom_twist = self.create_publisher(TwistWithCovariance, '/twist_odom', 10)

        #Subscribers
        self.odom_sub = self.create_subscription(Float64MultiArray, '/raw_odom', self.odom_callback, 10)
        self.odom_sub  # prevent unused variable warning
        #self.vel_sub = self.create_subscription(Float64MultiArray, '/vel', self.vel_callback, 10)
        self.vel_sub = self.create_subscription(Twist, '/cmd_vel', self.vel_callback, 10)
        self.vel_sub  # prevent unused variable warning

    """def timer_callback(self):
        xdot = d2w_kine(self.state,self.phid)

        move = Twist()
        move.linear.x = xdot[0]
        move.linear.y = xdot[1]
        move.linear.z = 0.
        move.angular.x = 0.
        move.angular.y = 0.
        move.angular.z = xdot[2]

        #self.get_logger().info('Estado: %s' % xdot)"""


    def odom_callback(self, msg):
        #xc = msg.pose.pose.position.x
        #yc = msg.pose.pose.position.y
        #psi = tf_transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]
        self.state = msg.data
        self.linear_state = d2w_kine([0,0,0],self.state)
        odom_twist_message = TwistWithCovariance()
        odom_twist_message.twist.linear.x = self.linear_state[0]
        odom_twist_message.twist.linear.y = self.linear_state[1]
        odom_twist_message.twist.angular.z = self.linear_state[2]
        self.publisher_odom_twist.publish(odom_twist_message)
        

    def vel_callback(self, msg):
        self.move = msg
        self.phid = d2w_inv_kine((self.move.linear.x,self.move.linear.y,self.move.angular.z))
        joint_msg = Float64MultiArray()
        joint_msg.data = self.phid
        self.publisher_joint_vel.publish(joint_msg)

        ctrl_msg = (Float64(),Float64(),Float64(),Float64())
        for i in range(0,4):
            ctrl_msg[i].data = self.phid[i]

        self.ctrl_pub_1.publish(ctrl_msg[0])
        self.ctrl_pub_2.publish(ctrl_msg[1])
        self.ctrl_pub_3.publish(ctrl_msg[2])
        self.ctrl_pub_4.publish(ctrl_msg[3])

def main(args=None):
    rclpy.init(args=args)

    vel_drive = velDrive()

    rclpy.spin(vel_drive)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    vel_drive.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
