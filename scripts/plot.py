#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np

class SubPlot(Node):
    def __init__(self):
        super().__init__('sub_plot')
        self.subscription = self.create_subscription(Odometry, 'odom', self.call_plot, 10)
        self.subscription
        self.position_x = 0.
        self.position_y = 0.
        self.orientation_z = 0.
        self.i = 0
        self.vet_x = []

        self.vet_y = []

    def call_plot(self,msg):
        self.position_x = msg.pose.pose.position.x
        self.position_y = msg.pose.pose.position.y
        self.orientation_z = msg.pose.pose.orientation.z

        self.vet_x.append(self.position_x)
        self.vet_y.append(self.position_y)
        self.i = self.i + 1

def main(args=None):
    rclpy.init(args=args)
    plot_subscriber = SubPlot()
    rclpy.spin(plot_subscriber)
    plot_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
