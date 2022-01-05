#!/usr/bin/env python3
# coding: utf-8

import time
import rclpy
from rclpy.node import Node
from interfaces.msg import PredictedDovResult

class ElectionSubscriber(Node):
    def __init__(self):
        super().__init__('sub_election_node')
        self.subscription = self.create_subscription(
            PredictedDovResult,
            'predicted_dov_result_topic',
            self.collect_result_cb,
            10)
        self.results = []
        self.n_robot = 5
        self.last_time_stamp = time.time()

    def collect_result_cb(self, msg):
        # over size -> initialize
        if len(self.results) == self.n_robot:
            self.get_logger().info('the size of results is 5, which is initialized.')
            self.results = []

        # time out -> initialize
        now = time.time()
        if (now - self.last_time_stamp) > 60.0:
            self.get_logger().info('time out, results array is initialized.')
            self.results = []

        self.last_time_stamp = now
        self.results.append(
            {'robot_id': msg.robot_id, 'predicted_class': msg.predicted_class, 'proba': msg.proba})

        if len(self.results) == 5:
            self.select_robot(self.results, 'proba')

    def select_robot(self, array, key):
        # sort
        class_one_results = [i for i in array if i['predicted_class'] == 1]
        max_proba_result = max(class_one_results, key=lambda x: x[key])

        print(max_proba_result)

def main(args=None):
    rclpy.init(args=args)

    node = ElectionSubscriber()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
