#!/usr/bin/env python3
# coding: utf-8

# for ROS2
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

# for recording
import pyaudio
import numpy as np

# constants
RESPEAKER_RATE = 48000
# change base on firmwares, default_firmware.bin as 1 or 6_firmware.bin as 6
RESPEAKER_CHANNELS = 6
RESPEAKER_WIDTH = 2
# run getDeviceInfo.py to get index
RESPEAKER_INDEX = 0  # refer to input device id
CHUNK = 1024 * 2
RECORD_SECONDS = 2
OUTPUT_PATH = "../out/"

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
        self.p = pyaudio.PyAudio()
        self.stream = self.p.open(
            format=self.p.get_format_from_width(RESPEAKER_WIDTH),
            # format=pyaudio.paInt16,
            rate=RESPEAKER_RATE,
            channels=RESPEAKER_CHANNELS,
            input=True,
            output=False,
            input_device_index=RESPEAKER_INDEX)

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        if msg.data == 1:
            self.recording()


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