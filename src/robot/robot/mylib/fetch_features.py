#!/usr/bin/env python3
# coding: utf-8

from . import fetch_features_func as fff
from .fft import fft
from typing import List, Any, Union, Dict
from srmrpy.srmr import *

def FetchFeaturesFromMonoData(v: List, fs: int, N: int, overlap: Union[int, float], th: Union[int, float] = 7000, w: int = 3, sec_range: float = 0.01, nth: int = 9):
    '''
    単一の音声データから特徴量を取得する。

    Parameters
    ----------
    v : array-like
        音声データ
    fs : int
        音声データのサンプリング周波数
    N : int
        FFTのサイズ
    overlap : int or float
        FFTのオーバーラップ率
    th: int or float, default 7000
        高周波成分と低周波成分の閾値[Hz]
    w: int, default 3
        ピーク検出の感度に関する値
    sec_range: float, default 0.01
        最大ピークから切り出す時間範囲[sec]
    nth: int, default 9
        最大ピークと比較する、次に高いn番目のピーク

    Returns
    -------
    rows: array-like
        各種特徴量が格納された配列
    '''

    # FFTを用いた周波数解析
    fft_array, fft_mean, fft_axis = fft(v, fs, N=N, overlap=overlap)

    # 低周波成分と高周波成分の取得
    low_power, high_power = fff.GetLHPower(fft_mean, fft_axis, th)
    # 低周波成分と高周波成分の比
    hlbr = fff.GetHLBR(low_power, high_power)

    # 128-bin FFT
    fft_array_128, fft_mean_128, fft_axis_128 = fft(v, fs, 128, 50)
    # 128-bin FFTにフィッティングした線形回帰
    coe1 = fff.FitFFT(fft_mean_128, fft_axis_128, 1)
    # 128-bin FFTにフィッティングした3度の多項式
    coe3 = fff.FitFFT(fft_mean_128, fft_axis_128, 3)

    # 最大ピークと、+-10ms以内の他のピークの平均値の比
    ratio_max_to_10ms_ave_peaks = fff.GetRatioMaxToOtherAvePeaks(
        v, w, fs, sec_range)
    # 最大ピークと、次に高い9つのピークの平均値の比
    ratio_max_to_9th_ave_peaks = fff.GetRatioMaxToNthAvePeaks(v, w, nth + 1)

    # 自己相関の標準偏差と曲線下面積
    ac, ac_std, ac_auc = fff.AutocorStdAuc(v)
    # 自己相関関数の微分の標準偏差と曲線下面積
    diff, diff_std, diff_auc = fff.DifferStdAuc(v)

    # SRMR
    srmr_val, _ = srmr(v, fs)

    return [low_power, high_power, hlbr, coe1[0], coe1[1], coe3[0], coe3[1], coe3[2], coe3[3], ratio_max_to_10ms_ave_peaks, ratio_max_to_9th_ave_peaks, ac_std, ac_auc, diff_std, diff_auc, srmr_val]
