#!/usr/bin/env python3
# coding: utf-8

# for ROS2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from interfaces.msg import SimpleControlRec

class MainRobotController(Node):
    def __init__(self):
        super().__init__('main_robot_controller')
        self.start_rec_sub = self.create_subscription(
            Bool, 'start_rec_topic', self.listener_cb, 10)
        self.start_rec_sub_2 = self.create_subscription(
            Bool, 'start_rec_topic_2', self.listener_cb_2, 10)

        self.start_rec_sub
        self.start_rec_sub_2

    def listener_cb(self, msg):
        print('listener_cb')

    def listener_cb_2(self, msg):
        print('listener_cb_2')

def main(args=None):
    rclpy.init(args=args)

    main_robot_controller = MainRobotController()

    rclpy.spin(main_robot_controller)

    main_robot_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
