#!/usr/bin/env python3
# coding: utf-8

import os
import numpy as np
import itertools
import joblib

# for ROS2
import rclpy
from rclpy.node import Node
from interfaces.msg import StartRec, PredictedDovResult

# custom module
# for recording
from .mylib import rec_audio
from .mylib.load_constants import Rec_Consts
# for calculate features
from .mylib import cis
from .mylib import fetch_features as ff
from .mylib import fetch_features_func as fff

class Robot(Node):

    def __init__(self):
        super().__init__('robot_main_node')
        # constants
        self.consts = Rec_Consts(index=0, output_path="/home/ubuntu/DoV_Sys_Ws/out",
                                 record_sec=2.0, rate=48000, chunk=1024 * 3)

        self.filename_prefix = 'rec'
        self.N = 1024 * 4
        self.w = 1
        self.overlap = 80
        self.gp_tdoa_mic_channels = [1, 2, 3, 4]

        self.pkl_dir = '/home/ubuntu/DoV_Sys_Ws/pkl'

        self.robot_id = int(os.environ['ROBOT_ID'])

        self.publisher_ = self.create_publisher(
            PredictedDovResult, 'predicted_dov_result_topic', 10)
        self.subscription = self.create_subscription(
            StartRec,
            'start_rec_topic',
            self.main_cb,
            10)
        self.subscription  # prevent unused variable warning

        self.declare_parameter('elect_mode', '0angle')
        self.declare_parameter('agc_mode', 'noagc')
        self.declare_parameter(
            'agc_0angle_pkl_filename', 'AGC-0angle-under5m_2021-12-27_ExtraTreesClassifier_ClusterCentroids_2021-12-28_no8.sav')
        self.declare_parameter(
            'agc_45angle_pkl_filename', 'AGC-45angle-under5m_2021-12-27_ExtraTreesClassifier_SMOTE_2021-12-29_no4.sav')
        self.declare_parameter(
            'noagc_0angle_pkl_filename', 'NoAGC-0angle-under5m_2021-12-27_ExtraTreesClassifier_RandomOverSampler_2021-12-29_no3.sav')
        self.declare_parameter('noagc_45angle_pkl_filename',
                               'NoAGC-45angle-under5m_2021-12-27_ExtraTreesClassifier_NoResampler_2021-12-29_no0.sav')

    def test_cb(self, msg):
        # publish msg to pc
        robot_ids = [1, 2, 3, 4, 5]
        probas = [0.6, 0.85, 0.9, 0.85, 0.9]
        predicted_classes = [0, 0, 0, 1, 1]
        time_stamp = 1641450318.272768

        for i in range(1, 6):
            _i = i - 1
            result_msg = PredictedDovResult()
            result_msg.proba = probas[_i]
            result_msg.predicted_class = predicted_classes[_i]
            result_msg.robot_id = robot_ids[_i]
            result_msg.time_stamp = time_stamp
            self.publisher_.publish(result_msg)
            self.get_logger().info(
                '[Publishing] robot_id: %d, proba: %lf, predicted_class: %d' % (result_msg.robot_id, result_msg.proba, result_msg.predicted_class))

    def main_cb(self, msg):
        # recording
        wav_dir_path = self.rec_wav(msg.dir_name, self.get_logger())
        # calculate features
        features_array = self.calc_features(wav_dir_path, self.filename_prefix)
        # predict dov
        elect_mode = self.get_parameter(
            'elect_mode').get_parameter_value().string_value
        agc_mode = self.get_parameter(
            'agc_mode').get_parameter_value().string_value
        pkl_param_name = agc_mode + '_' + elect_mode + '_pkl_filename'
        pkl_filepath = self.pkl_dir + '/' + \
            self.get_parameter(
                pkl_param_name).get_parameter_value().string_value
        self.get_logger().info(
            f"elect_mode: {elect_mode}, agc_mode: {agc_mode}, pkl_filepath: {pkl_filepath}")
        # predict
        predicted_class, proba = self.dov_predict(
            features_array, pkl_filepath)

        # publish msg to pc
        result_msg = PredictedDovResult()
        result_msg.proba = proba
        result_msg.predicted_class = predicted_class
        result_msg.robot_id = self.robot_id
        result_msg.time_stamp = msg.time_stamp
        self.publisher_.publish(result_msg)
        self.get_logger().info(
            '[Publishing] robot_id: %d, proba: %lf, predicted_class: %d' % (result_msg.robot_id, result_msg.proba, result_msg.predicted_class))

    def rec_wav(self, dir_name, logger):
        try:
            wav_dir_path = rec_audio.recording(
                self.consts, logger, dirname=dir_name)
        except Exception as e:
            self.get_logger().error(e)
        else:
            self.get_logger().info('Success to record wav files.: %s' % (wav_dir_path))
            return wav_dir_path

    def calc_features(self, wav_dir_path, filename_prefix):
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
            N = self.N
            w = self.w
            overlap = self.overlap
            gp_tdoa_mic_channels = self.gp_tdoa_mic_channels

            ##########################
            ### GCC-PHAT & TDOA以外 ###
            ##########################
            # 指定したfile_pathに該当するwavがない場合、スキップする
            wav_0ch_file_path = wav_dir_path + '/' + filename_prefix + '_0.wav'
            if not os.path.isfile(wav_0ch_file_path):
                raise FileNotFoundError(wav_0ch_file_path + ' is not found!')

            # channel0の音声データを読み込む
            v, fs = cis.wavread(wav_0ch_file_path)

            # 特徴量を得る
            features = ff.FetchFeaturesFromMonoData(v, fs, N, overlap, w=w)

            self.get_logger().info('Complete to calculate features except GCC-PHAT & TDOA')

            #######################
            ### GCC-PHAT & TDOA ###
            #######################

            # channel1~4の音声データを読み込む
            voice_data = []
            for ch in gp_tdoa_mic_channels:
                fname = filename_prefix + '_' + str(ch) + '.wav'
                fpath = wav_dir_path + '/' + fname
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

            self.get_logger().info('Complete to calculate GCC-PHAT & TDOA')

            # 行情報を生成
            feature_vals = features + gp_tdoa_features
        except Exception as e:
            self.get_logger().error(e)
        else:
            self.get_logger().info('Success to calcurate feature values from a wav file.')
            return feature_vals

    def dov_predict(self, feature_vals, pkl_filepath):
        try:
            # モデルの読み込み
            model = joblib.load(pkl_filepath)

            # classの予測
            predicted_class = model.predict(feature_vals)
            proba = model.predict_proba(feature_vals)
        except Exception as e:
            self.get_logger().error(e)
        else:
            self.get_logger().info('Success to predict dov.')
            return predicted_class, proba


def main(args=None):
    rclpy.init(args=args)

    node = Robot()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
