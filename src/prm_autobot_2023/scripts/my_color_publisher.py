#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class EnemyColorReader(Node):
    def __init__(self):
        super().__init__('enemy_color_reader')
        self.enemy_is_red = False
        self.create_subscription(String, 'target_robot_color', self.color_cb, 10)

    def color_cb(self, msg):
        self.enemy_is_red = (msg.data == "read")
        self.get_logger().info(f"enemy_is_red = {self.enemy_is_red}")

def main():
    rclpy.init()
    node = EnemyColorReader()
    rclpy.spin(node)

if __name__ == '__main__':
    main()