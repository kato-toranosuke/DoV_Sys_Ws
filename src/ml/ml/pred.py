#!/usr/bin/env python3
# coding: utf-8

import rclpy
from rclpy.node import Node

from interfaces.srv import CalcProba

import joblib

class DovPredictService(Node):
    def __init__(self):
        super().__init__('dov_predict_service')
        self.srv = self.create_service(
            CalcProba, 'calc_proba', self.dov_predict_cb)

    def dov_predict_cb(self, request, response):
        # モデルの読み込み
        input_pkl_filepath = 'hoge'
        model = joblib.load(input_pkl_filepath)

        # classの予測
        X = request.feature_vals
        predicted_class = model.predict(X)
        proba = model.predict_proba(X)

        response.predicted_class = predicted_class
        response.proba = proba

        return response


def main(args=None):
    rclpy.init(args=args)

    srv = DovPredictService()

    rclpy.spin(srv)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
