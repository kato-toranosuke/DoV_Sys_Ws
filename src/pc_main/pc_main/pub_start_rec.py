#!/usr/bin/env python3
# coding: utf-8

import time
import datetime
import rclpy
from rclpy.node import Node
from interfaces.msg import StartRec

class StartRecTopic(Node):
    def __init__(self):
        super().__init__('pub_start_rec_node')
        self.publisher_ = self.create_publisher(
            StartRec, 'start_rec_topic', 10)
        self.no = 0
        self.keyboard_daemon()

    def keyboard_daemon(self):
        while True:
            key = input('Key Input: ')
            if key == 's':
                self.pub_msg_cb()

    def pub_msg_cb(self):
        msg = StartRec()
        msg.flag = True
        msg.time_stamp = time.time()
        # JST時刻に設定
        dt_now_jst = datetime.datetime.now(
            datetime.timezone(datetime.timedelta(hours=9))
        )
        msg.dir_name = dt_now_jst.strftime(
            '%Y-%m-%d_%H-%M-%S_no') + str(self.no)

        # publish
        self.publisher_.publish(msg)
        self.get_logger().info(
            '[Publishing] time_stamp: %lf, dir_name: %s' % (msg.time_stamp, msg.dir_name))

        self.no += 1


def main(args=None):
    rclpy.init(args=args)

    node = StartRecTopic()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
