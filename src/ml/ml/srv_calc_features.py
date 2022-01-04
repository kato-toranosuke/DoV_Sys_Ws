#!/usr/bin/env python3
# coding: utf-8

import rclpy
from rclpy.node import Node
from interfaces.srv import CalcFeatureVals

import os
from typing import List, Any, Union, Dict
import numpy as np
import itertools

from .lib_ml import cis
from .lib_ml import fetch_features as ff
from .lib_ml import fetch_features_func as fff

class ClacFeaturesService(Node):
    def __init__(self):
        # self.robot_id = os.environ['ROBOT_ID']
        # self.node_name = 'calc_features_service_' + self.robot_id
        # self.service_name = 'calc_feature_vals_srv_' + self.robot_id

        super().__init__('calc_features_service_node')
        self.srv = self.create_service(
            CalcFeatureVals, 'calc_feature_vals_srv', self.calc_features_cb)

    # def calc_features_cb(file_name: str, dir_path: str, gp_tdoa_mic_channels: List, w: int, N: Union[int, str] = 'full', overlap: Union[int, float] = 0.8) -> List:
    def calc_features_cb(self, request, response):
        '''
        音声ファイルに対応する特徴量の抽出。

        Parameters
        ----------
        dir_path: str
            ファイルが格納されているディレクトリまでのパス
        attr: dict
            属性値が格納された辞書
        gp_tdoa_mic_channels: 
        w: int
            ピーク検出に対する感度
        N : int or str
            FFTのサイズ
        overlap : int or float
            FFTのオーバーラップ率

        Returns
        -------
        row: list
            1行分のデータ
        '''
        try:
            # 定数
            N = request.n
            w = request.w
            overlap = request.overlap
            gp_tdoa_mic_channels = request.gp_tdoa_mic_channels

            ##########################
            ### GCC-PHAT & TDOA以外 ###
            ##########################
            # 指定したfile_pathに該当するwavがない場合、スキップする
            wav_0ch_file_path = request.file_dir_path + '/' + request.file_name + '_0.wav'
            if not os.path.isfile(wav_0ch_file_path):
                raise FileNotFoundError(wav_0ch_file_path + ' is not found!')

            # channel0の音声データを読み込む
            v, fs = cis.wavread(wav_0ch_file_path)

            # 特徴量を得る
            features = ff.FetchFeaturesFromMonoData(v, fs, N, overlap, w=w)

            #######################
            ### GCC-PHAT & TDOA ###
            #######################

            # channel1~4の音声データを読み込む
            voice_data = []
            for ch in gp_tdoa_mic_channels:
                fname = request.file_name + '_' + str(ch) + '.wav'
                fpath = request.file_dir_path + '/' + fname
                v_, fs_ = cis.wavread(fpath)
                voice_data.append([v_, fs_])

            # GCC-PHATの特徴量を格納する配列を生成
            pair_gp_tdoa_features = []

            # 各ペアについて計算
            mic_ch_pairs = itertools.combinations(gp_tdoa_mic_channels, 2)
            for pair in mic_ch_pairs:
                # マイクのチャンネル
                mic_ch1 = pair[0]
                mic_ch2 = pair[1]
                # インデックスを計算（mic channelは1から始まる）
                ix1 = mic_ch1 - 1
                ix2 = mic_ch2 - 1
                # 音声データを取得
                v1 = voice_data[ix1][0]
                v2 = voice_data[ix2][0]
                # サンプリング周波数を取得
                fs = voice_data[ix1][1]

                # GCC-PHATとTDOAを計算
                gp_max_val, gp_max_ix, gp_auc, tdoa = fff.GetGccPhatAndTdoa(
                    v1, v2, fs=fs, w=w)

                # 配列に格納する
                pair_gp_tdoa_features.append(
                    [gp_max_val, gp_max_ix, gp_auc, tdoa])

            # 標準偏差、範囲、最小値、最大値、平均を求める
            gp_tdoa_features = []
            np_pair_gp_tdoa_features = np.array(pair_gp_tdoa_features)
            for i in range(4):
                vals = np_pair_gp_tdoa_features[:, i]
                std = np.std(vals)
                max = np.max(vals)
                min = np.min(vals)
                r = max - min
                mean = np.mean(vals)

                gp_tdoa_features.extend([std, r, min, max, mean])

            # 行情報を生成
            response.feature_vals = features + gp_tdoa_features
        except Exception as e:
            self.get_logger().error(e)
            response.success = False
        else:
            self.get_logger().info('Success to calcurate feature values from a wav file.')
            response.success = True
        finally:
            return response

def main(args=None):
    rclpy.init(args=args)

    service = ClacFeaturesService()

    rclpy.spin(service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
