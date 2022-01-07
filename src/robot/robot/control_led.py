#!/usr/bin/env python3
# coding: utf-8

import os
import time
# for ROS2
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8

from .mylib import usb_pixel_ring_v2 as ring

class ControlLed(Node):
    def __init__(self):
        super().__init__('control_led_node')

        self.robot_id = int(os.environ['ROBOT_ID'])

        self.subscription = self.create_subscription(
            UInt8, 'select_robot_topic', self.control_led_cb, 10)
        self.subscription

    def control_led_cb(self, msg):
        pixel_ring = ring.find()
        if msg.data == self.robot_id:
            pixel_ring.set_color(0xFF0000)
            time.sleep(3)
        else:
            pixel_ring.set_color(0x0000FF)
            time.sleep(3)
        pixel_ring.off()
        time.sleep(3)
        pixel_ring.close()

def main(args=None):
    rclpy.init(args=args)

    node = ControlLed()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
