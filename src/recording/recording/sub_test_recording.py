#!/usr/bin/env python3
# coding: utf-8

# for ROS2
import rclpy
from rclpy.node import Node
from interfaces.msg import SimpleControlRec

from .lib_rec import rec_audio
from .lib_rec import load_constants

class RecordingSubscriber(Node):

    def __init__(self):
        super().__init__('sub_test_recording')
        self.subscription = self.create_subscription(
            SimpleControlRec,
            'control_rec_topic',
            self.rec_cb,
            10)
        self.subscription  # prevent unused variable warning
        self.consts = load_constants.Rec_Consts(
            rate=48000, channels=6, width=2, index=0, chunk=1024 * 3, record_sec=2.0, output_path='/home/ubuntu/DoV_Sys_Ws/out')

    def rec_cb(self, msg):
        # self.get_logger().info('I heard: flag = %s, distance = %d, angle = %d, trial = %d, date = %s' %
        #                        (msg.flag, msg.distance, msg.angle, msg.trial, msg.date))
        if msg.flag == True:
            rec_audio.recording(self.consts, mode='test', distance=msg.distance,
                                angle=msg.angle, trial=msg.trial, date=msg.date)


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
