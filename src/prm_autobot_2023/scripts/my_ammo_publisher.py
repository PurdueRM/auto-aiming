#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Int16


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('ammo_publisher')
        self.publisher_ = self.create_publisher(Int16, 'ammo', 10)
        timer_period = 1.0 #0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        #self.arr = 1000
        self.arr = 40

    def timer_callback(self):
        """msg = Int16()
        msg.data = self.arr
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        #self.i += 1 
        if msg.data > 0: self.arr -= 4"""
        
        msg = Int16()
        
        if (self.arr >= 1):
        	self.arr -=1
        else:
        	self.arr = 40
        if (self.arr > 20):
        	msg.data = 700
        else:
        	msg.data = 500
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
