#!/usr/bin/env python3
# coding: utf-8

import rclpy
from rclpy.node import Node

from interfaces.srv import CalcProba

import joblib

class DovPredictService(Node):
    def __init__(self):
        # self.robot_id = os.environ['ROBOT_ID']
        # self.node_name = 'dov_pred_service_' + self.robot_id
        # self.service_name = 'calc_proba_srv_' + self.robot_id

        super().__init__('dov_pred_service_node')
        self.srv = self.create_service(
            CalcProba, 'calc_proba_srv', self.dov_predict_cb)

    def dov_predict_cb(self, request, response):
        try:
            # モデルの読み込み
            model = joblib.load(request.pkl_filepath)

            # classの予測
            X = request.feature_vals
            predicted_class = model.predict(X)
            proba = model.predict_proba(X)

            response.predicted_class = predicted_class
            response.proba = proba
            response.success = True
        except Exception as e:
            self.get_logger().error(e)
            response.success = False
        finally:
            return response


def main(args=None):
    rclpy.init(args=args)

    service = DovPredictService()

    rclpy.spin(service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
