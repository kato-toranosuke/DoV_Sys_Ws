#!/usr/bin/env python3
# coding: utf-8

import rclpy
from rclpy.node import Node
from interfaces.msg import PredictedDovResult
from std_msgs.msg import UInt8
import pprint

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
        self.declare_parameter('n_robot', 5)
        self.declare_parameter('elect_mode', '0angle')

    def collect_result_cb(self, msg):
        n_robot = self.get_parameter(
            'n_robot').get_parameter_value().integer_value
        elect_mode = self.get_parameter(
            'elect_mode').get_parameter_value().string_value

        self.received_msgs.append(
            {'robot_id': msg.robot_id, 'predicted_class': msg.predicted_class, 'proba': msg.proba, 'time_stamp': msg.time_stamp})
        self.get_logger().info(
            '[Subscriber] robot_id: %d, proba: %lf, predicted_class: %d, time_stamp: %lf' % (msg.robot_id, msg.proba, msg.predicted_class, msg.time_stamp))

        cur_time_stamp_array = [
            i for i in self.received_msgs if i['time_stamp'] == msg.time_stamp]

        if len(cur_time_stamp_array) == n_robot:
            # select robot
            if elect_mode == '0angle':
                selected_robot = self.select_robot(
                    cur_time_stamp_array, 'proba')
            else:
                selected_robot = self.select_robot_45(
                    cur_time_stamp_array, 'proba')

            # publish message
            select_robot_msg = UInt8()
            select_robot_msg.data = selected_robot['robot_id']
            self.publisher_.publish(select_robot_msg)

            # delete a part of array
            self.received_msgs = [
                i for i in self.received_msgs if i['time_stamp'] != msg.time_stamp]

    def select_robot(self, array, key):
        # sort
        class_one_results = [i for i in array if i['predicted_class'] == 1]

        if len(class_one_results) > 0:
            max_proba_result = max(class_one_results, key=lambda x: x[key])
        else:
            max_proba_result = {'robot_id': 404}

        pprint.pprint(array)
        print('\n')
        print(max_proba_result)
        print('-------------\n')
        return max_proba_result

    def select_robot_45(self, array, key):
        # extraction class 1
        class_one_results = [i for i in array if i['predicted_class'] == 1]
        # sort by robot_id
        sorted(class_one_results, key=lambda x: x['robot_id'])

        # 場合分け
        if len(class_one_results) == 1:
            max_proba_result = class_one_results[0]
        elif len(class_one_results) == 2:
            if class_one_results[0]['robot_id'] == 1 and class_one_results[1]['robot_id'] == 2:
                max_proba_result = class_one_results[0]
            elif class_one_results[0]['robot_id'] == 4 and class_one_results[1]['robot_id'] == 5:
                max_proba_result = class_one_results[1]
            else:
                max_proba_result = {'robot_id': 404}
        elif len(class_one_results) == 3:
            if class_one_results[0]['robot_id'] == 1 and class_one_results[1]['robot_id'] == 2 and class_one_results[2]['robot_id'] == 3:
                max_proba_result = class_one_results[1]
            elif class_one_results[0]['robot_id'] == 2 and class_one_results[1]['robot_id'] == 3 and class_one_results[2]['robot_id'] == 4:
                max_proba_result = class_one_results[1]
            elif class_one_results[0]['robot_id'] == 3 and class_one_results[1]['robot_id'] == 4 and class_one_results[2]['robot_id'] == 5:
                max_proba_result = class_one_results[1]
            else:
                max_proba_result = {'robot_id': 404}
        else:
            max_proba_result = {'robot_id': 404}

        pprint.pprint(array)
        print('\n')
        print(max_proba_result)
        print('-------------\n')
        return max_proba_result


def main(args=None):
    rclpy.init(args=args)

    node = ElectionSubscriber()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
