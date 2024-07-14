#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv

class SubPlot(Node):
    def __init__(self):
        super().__init__('sub_plot')
        self.subscription = self.create_subscription(Odometry, 'odom', self.call_plot, 10)
        self.subscription
        self.position_x = 0.
        self.position_y = 0.
        self.orientation_z = 0.
        self.frame = ""
        self.i = 0

        self.dados = [
                ["Deslocamento"],
                ["x"],
                ["y"],
                ["theta"]]

        self.nome_arquivo = "src/d2w_ros2/dados.csv"

    def call_plot(self,msg):
        self.frame = msg.header.frame_id
        self.position_x = msg.pose.pose.position.x
        self.position_y = msg.pose.pose.position.y
        self.orientation_z = msg.pose.pose.orientation.z

        if self.frame == "q_f":
            with open(self.nome_arquivo, mode='w', newline='') as arquivo:
                escritor_csv = csv.writer(arquivo)
                escritor_csv.writerows(self.dados)
            self.get_logger().info('escrito.................................................................')
        else:
            self.dados[1].append(self.position_x)
            self.dados[2].append(self.position_y)
            self.dados[3].append(self.orientation_z)


def main(args=None):
    rclpy.init(args=args)
    plot_subscriber = SubPlot()
    rclpy.spin(plot_subscriber)
    plot_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
