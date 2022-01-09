#!/usr/bin/env python3
# coding: utf-8

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterValue
from rcl_interfaces.srv import SetParameters

class SetParamClient(Node):

    def __init__(self, node_name, param_name, param_value):
        super().__init__('set_param_client_node')

        self.service_name = '/' + node_name + '/set_parameters'
        self.cli = self.create_client(SetParameters, self.service_name)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetParameters.Request()
        self.param_name = param_name
        self.param_value = param_value

    def send_request(self):
        parameter = Parameter()
        parameter.name = self.param_name
        value = ParameterValue()
        value.type = 4
        value.string_value = self.param_value
        parameter.value = value

        # set request
        self.req.parameters = [parameter]
        self.future = self.cli.call_async(self.req)

class SetParams():
    def __init__(self, n_robot=5):
        self.n_robot = n_robot
        self.input_daemon()

    def set_params(self, param_name, param_value):
        if param_name == 'elect_mode':
            full_node_name = 'sub_election_node'
            client = SetParamClient(full_node_name, param_name, param_value)
            client.send_request()

            while rclpy.ok():
                rclpy.spin_once(client)
                if client.future.done():
                    try:
                        response = client.future.result().results[0]
                    except Exception as e:
                        client.get_logger().info('Service call failed %r' % (e,))
                    else:
                        print('ok')
                        request = client.req.parameters[0]
                        client.get_logger().info(
                            'Result: [sub_election_node] key=%s, value=%s --> success=%s, reason=%s' %
                            (request.name, request.value.string_value, str(response.successful), response.reason))
                    break

            client.destroy_node()

        for i in range(self.n_robot):
            full_node_name = 'robot' + str(i + 1) + '_robot_main'
            client = SetParamClient(full_node_name, param_name, param_value)
            client.send_request()

            while rclpy.ok():
                rclpy.spin_once(client)
                if client.future.done():
                    try:
                        response = client.future.result().results[0]
                    except Exception as e:
                        client.get_logger().info('Service call failed %r' % (e,))
                    else:
                        print('ok')
                        request = client.req.parameters[0]
                        client.get_logger().info(
                            'Result: [robot%d] key=%s, value=%s --> success=%s, reason=%s' %
                            (i + 1, request.name, request.value.string_value, str(response.successful), response.reason))
                    break

            client.destroy_node()

    def input_daemon(self):
        while True:
            try:
                param_name = input('Param Name: ')
                param_value = input('Param Value: ')
                self.set_params(param_name, param_value)
            except KeyboardInterrupt:
                break


def main(args=None):
    rclpy.init(args=args)

    set_params = SetParams(5)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
