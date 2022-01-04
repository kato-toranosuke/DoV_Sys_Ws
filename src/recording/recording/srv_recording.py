#!/usr/bin/env python3
# coding: utf-8

import os
# for ROS2
import rclpy
from rclpy.node import Node
from interfaces.srv import RecordWav

# for recording
# from mylib.usb_4_mic_array.tuning import Tuning
from .lib_rec import rec_audio
from .lib_rec.load_constants import Rec_Consts

class RecordingService(Node):

    def __init__(self):
        # self.robot_id = os.environ['ROBOT_ID']
        # self.node_name = 'recording_service_' + self.robot_id
        # self.service_name = 'record_wav_srv_' + self.robot_id
        super().__init__('recording_service_node')
        self.srv = self.create_service(
            RecordWav, 'record_wav_srv', self.record_wav_cb)

    def record_wav_cb(self, request, response):
        self.get_logger().info('Incoming request\ndirname: %s, index: %d, sampling_rate: %d, recording_sec: %f, chunk_size: %d\n\n' %
                               (request.dirname, request.index, request.sampling_rate, request.recording_sec, request.chunk_size))
        # define constants for recording
        consts = Rec_Consts(index=request.index, output_path="../out",
                            record_sec=request.recording_sec, rate=request.sampling_rate, chunk=request.chunk_size)
        try:
            file_path = rec_audio.recording(consts, dirname=request.dirname)
        except Exception as e:
            self.get_logger().error(e)
            response.success = False
        else:
            response.file_path = file_path
            response.success = True
        finally:
            return response


def main(args=None):
    rclpy.init(args=args)

    recording_subscriber = RecordingService()

    rclpy.spin(recording_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    recording_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
