#!/usr/bin/env python3
# coding: utf-8

# for ROS2
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

# for recording
# from mylib.usb_4_mic_array.tuning import Tuning
from mylib.load_constants import Rec_Consts
from mylib import output_wav
from mylib import rec_audio

class RecordingSubscriber(Node):

    def __init__(self):
        super().__init__('recording_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # for recording
        self.consts = Rec_Consts(
            index=0, output_path="../out/", recording_sec=2.0, rate=48000, chunk=2048)

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        if msg.data == 1:
            part_of_output_file_path = output_wav.setupOutputEnv(
                self.consts, 's11', 'recording0', None, 'gym', 'nowall', 1, 0, 0)
            rec_audio.recording(self.consts, part_of_output_file_path)


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
