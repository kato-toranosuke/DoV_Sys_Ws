#!/usr/bin/env python3
# coding: utf-8

import math
import heapq
from scipy import signal, integrate
import numpy as np
from numba import jit
import numba

# 型ヒントのサポート
from typing import List, Any, Union, Dict


###############################
### 低周波成分と高周波成分の分析 ###
###############################
# @jit
def GetLHPower(fft_array: List, fft_axis: List, th: Union[float, int]) -> float:
    '''
    低周波成分と高周波成分の合計パワーを算出する。

    Parameters
    ----------
    fft_array : array-like
        FFTの振幅レベル
    fft_axis : array-like
        周波数軸
    th : float or int
        低周波と高周波を分ける閾値となる周波数。

    Returns
    -------
    low_power : float
        低周波数部分の合計パワー
    high_power : float
        高周波数部分の合計パワー

    Notes
    -----
    以下のmodule, package, libraryに依存している。
    - scipy.fftpack
    - numpy

    '''

    # 閾値となる周波数に対応するインデックスを求める
    df = fft_axis[1] - fft_axis[0]
    th_ix = int(th / df)

    # パワーを計算
    power = np.power(np.abs(fft_array), 2)
    low_power = np.sum(power[:th_ix])
    high_power = np.sum(power[th_ix:])

    return low_power, high_power


def GetHLBR(low_power: float, high_power: float) -> float:
    '''
    低周波成分と高周波成分の合計パワーの比を算出する。

    Parameters
    ----------
    low_power : float
        低周波成分の合計パワー
    high_power : float
        高周波成分の合計パワー

    Returns
    -------
    hlbr : float
        低周波成分と高周波成分の合計パワーの比
    '''
    return high_power / low_power

#######################
### 関数フィッティング ###
#######################
# @jit
def FitFFT(fft_array: List[Any], freq: List[Any], deg: int) -> List[float]:
    # 係数
    coe = np.polyfit(freq, fft_array, deg)
    return coe


################
### ピーク検出 ###
################

# 純粋なピーク検出
def FindPeaks(x: List, y: List, n: int, w: int):
    '''
    波形(x, y)からn個のピークを幅wで検出する関数

    Parameters
    ----------
    x : array-like
        波形のx軸
    y : array-like
        波形のy軸
    n : int
        抽出するピークの数
    w : int
        ピークの感度に関する幅

    Returns
    -------
    ix_val : array-like
        n個のピークに対応したx軸の物理量（周波数など）
    peaks : array-like
        n個のピーク値を格納した配列
    index : array-like
        n個のピークに対応したindexを格納した配列
    index_all : array-like
        検出した全てのピークに対応するindexの配列
    '''
    index_all = list(signal.argrelmax(y, order=w))  # scipyのピーク検出
    index = []  # ピーク指標の空リスト
    peaks = []  # ピーク値の空リスト

    # n個分のピーク情報(指標、値）を格納
    for i in numba.prange(n):
        # ピークがn個に満たない場合、ループを抜ける（エラー処理）
        if i >= len(index_all[0]):
            break
        index.append(index_all[0][i])
        peaks.append(y[index_all[0][i]])

    # 個数の足りない分を0で埋める（エラー処理）
    if len(index) != n:
        index = index + ([0] * (n - len(index)))
        peaks = peaks + ([0] * (n - len(peaks)))

    # xの分解能dxをかけて指標を物理軸に変換
    dx = x[1] - x[0]
    ix_val = np.array(index) * dx

    return ix_val, peaks, index, index_all


# 最大ピークと、+-10ms以内の他のピークの平均値の比
def FindMaxPeak(data: List, w: int):
    '''
    最大ピークを検出する関数

    Parameters
    ----------
    data : array-like
        波形の値
    w : int
        ピークの感度に関する幅

    Returns
    -------
    max_peak_ix : int
        最大ピークに対応するindex
    all_peaks_ix : list[np.ndarray]
        検出した全てのピークに対応するindex
    max_peak_val : int
        最大ピークの値
    '''
    all_peaks_ix = list(signal.argrelmax(data, order=w))
    all_peaks_ix = all_peaks_ix[0]
    vals = data[all_peaks_ix]

    # 最大ピークを取得
    max_peak_val = max(vals)
    # 最大ピークのインデックス
    max_peak_ix = all_peaks_ix[np.argmax(vals)]

    return max_peak_ix, all_peaks_ix, max_peak_val


# @jit(nopython=True, parallel=True, cache=True)
def GetWaveWithinTimeRange(data: List[float], base_ix: int, sec_range: int, fs: int):
    '''
    基準インデックスから周囲ms_range[s]の波形を切り出す関数

    Parameters
    ----------
    data : array-like
        波形データ
    base_ix : int
        基準インデックス
    sec_range : int
        基準インデックスから切り出す範囲[s]
    fs : int
        サンプリング周波数

    Returns
    -------
    index : array-like
        切り出した範囲に対応するインデックス
    val : array-like
        切り出した範囲の値
    '''

    dt = 1.0 / fs  # インデックス間の時間間隔
    # 切り出す範囲のインデックスを計算
    delta_ix = math.floor(sec_range / dt)
    start_ix = base_ix - delta_ix
    end_ix = base_ix + delta_ix

    # 範囲外のインデックスに対する処理
    if start_ix < 0:
        start_ix = 0
    if end_ix > len(data) - 1:
        end_ix = len(data) - 1

    index = np.arange(start_ix, end_ix + 1)
    val = data[index]

    return index, val


def GetRatioMaxToOtherAvePeaks(data: List, w: int, fs: int, sec_range: float = 0.01) -> float:
    '''
    最大ピークと、sec_range[sec]以内の他のピークの平均値の比を取得する関数

    Parameters
    ----------
    data : array-like
        波形のy軸
    w : int
        ピーク検出における感度
    fs : int
        サンプリング周波数[Hz]
    sec_range : float, default 0.01
        切り出す時間範囲[sec]

    Returns
    -------
    ratio : float
        最大ピークと、sec_range[sec]以内の他のピークの平均値の比
    '''

    # 最大ピークとその時のインデックスを取得
    # max_ix: 最大ピークのインデックス
    # all_ix: ピークのインデックス
    # max_val: 最大ピークの値
    max_ix, all_ix, max_val = FindMaxPeak(data, w)

    # max_indexから+-10ms以内の波形を切り出す。
    # target_ix : 対象時間区間のインデックス（ピークとピークでないものも含む）
    # target_val : target_ixに対応する値
    target_ix, target_val = GetWaveWithinTimeRange(data, max_ix, sec_range, fs)

    # target_peak_ix : 対象時間範囲内のピーク部分に対応するインデックス
    target_peak_ix = [i for i in all_ix if (i >= target_ix[0]) and (
        i <= target_ix[-1]) and (i != max_ix)]

    # 最大ピーク以外の他の全てのピークの平均値を算出
    mean_other_peaks = np.mean(data[target_peak_ix])

    # 比率の計算
    ratio = mean_other_peaks / max_val
    return ratio

# 最大ピークと次に高い９つのピークの平均値の比


def FindNthMaxPeak(data: List, n: int, w: int) -> List:
    '''
    第n位までの大きさのピークの値を取得する関数

    Parameters
    ----------
    data : array-like
        波形データ
    n : int
        何番目までの大きさのピークを取得するかを指定
    w : int
        ピーク検出の感度に関する幅

    Returns
    -------
    peaks_val : array-like
        取得したピークの値
    '''

    index_all = list(signal.argrelmax(data, order=w))
    index_all = index_all[0]
    vals = data[index_all]

    # 検出したピークの個数がn個に満たない場合
    if len(index_all) < n:
        return vals.tolist()

    peaks_val = heapq.nlargest(n, vals)
    return peaks_val


def GetRatioMaxToNthAvePeaks(data: List, w: int, n: int = 10) -> float:
    '''
    最大ピークと、次に高い(n-1)個のピークの平均値の比を取得する関数

    Parameters
    ----------
    data : array-like
        波形データの配列
    w : int
        ピーク検出の感度
    n : int, default 10
        何番目に高いピークまで取得のかを指定する

    Returns
    -------
    ratio : float
        最大ピークと、次に高い(n-1)個のピークの平均値の比
    '''

    # n番目の大きさのピークまで取得
    peak_vals = FindNthMaxPeak(data, n, w)
    # 最大ピークを取得
    max_peak_val = max(peak_vals)
    # 最大ピークと次に高いn-1個のピークのlist
    peak_vals.remove(max_peak_val)
    # 平均値
    mean_peak_val = np.mean(peak_vals)
    # 比率を算出
    ratio = mean_peak_val / max_peak_val

    return ratio

###############
### 自己相関 ###
###############

# 自己相関の標準偏差と曲線下面積


def AutocorStdAuc(data):
    '''
    自己相関の標準偏差と曲線下面積を求める関数

    Parameters
    ----------
    data : array-like
        データ配列

    Returns
    -------
    ac : array-like
        自己相関
    ac_std : float
        自己相関の標準偏差
    ac_auc : float
        自己相関の曲線下面積
    '''

    # 自己相関を計算
    ac = np.correlate(data, data, "full")
    # 標準偏差
    ac_std = np.std(ac)
    # 曲線下面積
    ac_auc = CalcAUC(ac)

    return ac, ac_std, ac_auc


# @jit(nopython=True, parallel=True, cache=True)
def CalcAUC(data: List) -> float:
    '''
    曲線下面積を求める関数

    Parameters
    ----------
    data : array-like
        データ配列

    Returns
    -------
    S : float
        曲線下面積
    '''
    return np.sum(np.abs(data))

# 自己相関関数の微分の標準偏差と曲線下面積


# @jit(parallel=True, cache=True)
def CalcDiffer(data: List) -> List:
    '''
    微分値を求める関数

    Parameters
    ----------
    data : array-like
        波形データ

    Returns
    -------
    differs : array-like
        微分値が格納された配列

    Notes
    -----
    波形データのx軸はインデックスであることが前提。
    '''
    return np.abs(np.gradient(data, edge_order=2))


# @jit(parallel=True, cache=True)
def DifferStdAuc(data: List):
    '''
    微分値の標準偏差と曲線下面積を求める関数

    Parameters
    ----------
    data : array-like
        データ配列

    Returns
    -------
    diff : array-like
        微分値
    diff_std : float
        微分値の標準偏差
    diff_auc : float
        微分値の曲線下面積
    '''
    # 微分
    diff = CalcDiffer(data)
    # 標準偏差
    diff_std = np.std(diff)
    # 曲線下面積
    diff_auc = CalcAUC(diff)

    return diff, diff_std, diff_auc


################
### GCC-PHAT ###
################
# @jit(nopython=True, parallel=True, cache=True)
def gcc_phat_old(sig, refsig, interp=16):
    '''
    GCC-PHATを算出
    This function computes the offset between the signal sig and the reference signal refsig
    using the Generalized Cross Correlation - Phase Transform (GCC-PHAT)method.
    '''

    # make sure the length for the FFT is larger or equal than len(sig) + len(refsig)
    n = sig.shape[0] + refsig.shape[0]

    # Generalized Cross Correlation Phase Transform
    SIG = np.fft.rfft(sig, n=n)
    REFSIG = np.fft.rfft(refsig, n=n)
    R = SIG * np.conj(REFSIG)

    # ゼロ割り対策
    R_ABS = np.abs(R)
    D = np.divide(R, R_ABS, out=np.zeros_like(R), where=R_ABS != 0)

    # 純粋なGCC-PHATの値はこれ。
    # gcc = np.fft.irfft(R / np.abs(R), n=(interp * n))
    gcc = np.fft.irfft(D, n=(interp * n))

    return gcc, n


# @jit(nopython=True, parallel=True, cache=True)
def tdoa_from_gccphat(gp, n, fs=1, max_tau=None, interp=16):
    '''
    GCC-PHATをフィルタにTDOA(Time Difference Of Arrival)を算出する
    '''

    max_shift = int(interp * n / 2)
    if max_tau:
        max_shift = np.minimum(int(interp * fs * max_tau), max_shift)

    gp = np.concatenate((gp[-max_shift:], gp[:max_shift + 1]))

    # find max cross correlation index
    shift = np.argmax(np.abs(gp)) - max_shift

    tau = shift / float(interp * fs)

    return tau


def gcc_phat(sig, refsig, fs=1, max_tau=None, interp=16):
    '''
    This function computes the offset between the signal sig and the reference signal refsig
    using the Generalized Cross Correlation - Phase Transform (GCC-PHAT)method.
    '''
    try:
        # make sure the length for the FFT is larger or equal than len(sig) + len(refsig)
        n = sig.shape[0] + refsig.shape[0]
        # フレームサイズを求める（FFTの点数）
        ex = math.ceil(math.log2(n))
        n = 2**ex

        # Generalized Cross Correlation Phase Transform
        SIG = np.fft.rfft(sig, n=n)
        REFSIG = np.fft.rfft(refsig, n=n)
        R = SIG * np.conj(REFSIG)

        # ゼロ割り対策
        R_ABS = np.abs(R)
        Normalize_R = np.divide(
            R, R_ABS, out=np.zeros_like(R), where=R_ABS != 0)

        # cc = np.fft.irfft(R / np.abs(R), n=(interp * n))
        cc = np.fft.irfft(Normalize_R, n=(interp * n))

        max_shift = int(interp * n / 2)
        if max_tau:
            max_shift = np.minimum(int(interp * fs * max_tau), max_shift)

        cc = np.concatenate((cc[-max_shift:], cc[:max_shift + 1]))

        # find max cross correlation value
        cc_max = np.max(cc)

        # find max cross correlation index
        shift = np.argmax(np.abs(cc)) - max_shift

        tau = shift / float(interp * fs)

    # except ZeroDivisionError as e:
    except:
        print("何かのエラー!")

    return tau, cc, cc_max, shift

# @jit(parallel=True, cache=True)
def GetGccPhatAndTdoa(y1: List, y2: List, fs, max_delay: float = 0.000236, w: int = 3, sound_spd: float = 343.2, distance: float = 0.070):
    '''
    Parameters
    ----------
    y1 : array-like
        データのy軸
    y2 : array-like
        データのy軸
    fs : int
        サンプリング周波数
    max_delay : float, default 0.000236
        直交するマイク間の距離に基づく理論上の最大遅延[s]
    w : int, default 3
        ピーク検出の感度
    sound_spd : float, default 343.2
        音速[m/sec]
    distance : float, default 0.14
        マイク間距離

    Returns
    -------
    gp_max_val : float
        GCC-PHATの最大ピークの値
    gp_max_ix : int
        GCC-PHATの最大ピークのインデックス
    gp_auc : float
        GCC-PHATの曲線下面積
    tdoa : float
        TDOA
    '''
    # max_tauは時間
    max_tau = distance / sound_spd

    # GCC-PHATの計算
    tau, cc, cc_max, shift = gcc_phat(
        sig=y1, refsig=y2, fs=fs, max_tau=max_tau)

    # 曲線下面積
    # gcc-phatの最大ピークのインデックスを取得（時間空間）
    max_ix, _, _ = FindMaxPeak(cc, w)

    # 最大ピークから+-MAX_DELAYだけ切り出す
    _, target_val = GetWaveWithinTimeRange(cc, max_ix, max_delay, fs)

    # 曲線下面積を計算
    dt = 1 / fs
    cc_auc = np.sum(dt * np.abs(target_val))

    return cc_max, shift, cc_auc, tau
