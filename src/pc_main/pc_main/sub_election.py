#!/usr/bin/env python3
# coding: utf-8

import rclpy
from rclpy.node import Node
from interfaces.msg import PredictedDovResult
from std_msgs.msg import UInt8

class ElectionSubscriber(Node):
    def __init__(self):
        super().__init__('sub_election_node')
        self.publisher_ = self.create_publisher(
            UInt8, 'select_robot_topic', 10)
        self.subscription = self.create_subscription(
            PredictedDovResult,
            'predicted_dov_result_topic',
            self.collect_result_cb,
            100)
        self.received_msgs = []
        self.n_robot = 5

    def collect_result_cb(self, msg):
        self.received_msgs.append(
            {'robot_id': msg.robot_id, 'predicted_class': msg.predicted_class, 'proba': msg.proba, 'time_stamp': msg.time_stamp})
        self.get_logger().info(
            '[Subscriber] robot_id: %d, proba: %lf, predicted_class: %d, time_stamp: %lf' % (msg.robot_id, msg.proba, msg.predicted_class, msg.time_stamp))

        cur_time_stamp_array = [
            i for i in self.received_msgs if i['time_stamp'] == msg.time_stamp]

        print('all array: %d' % len(self.received_msgs))
        print('current array: %d\n' % len(cur_time_stamp_array))

        if len(cur_time_stamp_array) == self.n_robot:
            # select robot
            selected_robot = self.select_robot(cur_time_stamp_array, 'proba')
            # publish message
            select_robot_msg = UInt8()
            select_robot_msg.data = selected_robot['robot_id']
            self.publisher_.publish(select_robot_msg)

            # delete a part of array
            self.received_msgs = [
                i for i in self.received_msgs if i['time_stamp'] != msg.time_stamp]

            print('deleted received_msgs array:')
            print(self.received_msgs)

    def select_robot(self, array, key):
        # sort
        class_one_results = [i for i in array if i['predicted_class'] == 1]
        max_proba_result = max(class_one_results, key=lambda x: x[key])

        print(max_proba_result)
        return max_proba_result

def main(args=None):
    rclpy.init(args=args)

    node = ElectionSubscriber()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
