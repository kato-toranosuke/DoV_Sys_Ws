#!/usr/bin/env python3
# coding: utf-8
# 「Pythonで学ぶ実践画像・音声処理入門」のサンプルプログラム用のパッケージ

import numpy as np
import simpleaudio as sa

def audioplay(y, fs):
    yout = np.iinfo(np.int16).max / np.max(np.abs(y)) * y
    yout = yout.astype(np.int16)
    play_obj = sa.play_buffer(yout, y.ndim, 2, fs)


import scipy.io.wavfile as sw

def wavread(wavefile):
    fs, y = sw.read(wavefile)
    if y.dtype == 'float32' or y.dtype == 'float64':
        max_y = 1
    elif y.dtype == 'uint8':
        y = y - 128
        max_y = 128
    elif y.dtype == 'int16':
        max_y = np.abs(np.iinfo(np.int16).min)
    else:
        max_y = np.abs(np.iinfo(np.int16).min)
    y = y / max_y
    y = y.astype(np.float32)
    return (y, fs)
