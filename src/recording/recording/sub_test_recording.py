#!/usr/bin/env python3
# coding: utf-8

# for ROS2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from .lib_rec import rec_audio
from .lib_rec import load_constants

class RecordingSubscriber(Node):

    def __init__(self):
        super().__init__('recording_subscriber')
        self.subscription = self.create_subscription(
            Bool,
            'control_rec_topic',
            self.rec_cb,
            10)
        self.subscription  # prevent unused variable warning
        self.consts = load_constants.Rec_Consts(
            rate=48000, channels=6, width=2, index=2, chunk=1024 * 2, record_sec=2.0, output_path='../../out')

    def rec_cb(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        if msg.data == True:
            rec_audio.recording(self.consts)


def main(args=None):
    rclpy.init(args=args)

    recording_subscriber = RecordingSubscriber()

    rclpy.spin(recording_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    recording_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
