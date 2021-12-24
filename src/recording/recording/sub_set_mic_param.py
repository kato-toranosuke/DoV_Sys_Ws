#!/usr/bin/env python3
# coding: utf-8

import sys
# for ROS2
import rclpy
from rclpy.node import Node
from interfaces.msg import SetMicParam

from .lib_rec import tuning

class SetMicParamSubscriber(Node):
    def __init__(self):
        super().__init__('sub_set_mic_param')
        self.subscription = self.create_subscription(
            SetMicParam,
            'set_mic_param_topic',
            self.set_mic_param_cb,
            10)
        self.subscription

    def set_mic_param_cb(self, msg):
        dev = tuning.find()
        if not dev:
            self.get_logger().error('No device found')
            sys.exit(1)

        key = msg.key.upper()
        if key in tuning.PARAMETERS:
            dev.write(key, msg.value)
            self.get_logger().info('{}: {}'.format(key, msg.value))
        else:
            self.get_logger().warn('{} is not a valid key'.format(key))

        dev.close()

def main(args=None):
    rclpy.init(args=args)

    set_param_sub = SetMicParamSubscriber()

    rclpy.spin(set_param_sub)

    set_param_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
