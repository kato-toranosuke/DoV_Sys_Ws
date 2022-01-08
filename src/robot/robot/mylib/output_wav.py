#!/usr/bin/env python3
# coding: utf-8

import pyaudio
import wave
from typing import List
import datetime
import os
import re
import sys
from .load_constants import Rec_Consts


def outputWaveFilesForTest(consts: Rec_Consts, frames: List, p: pyaudio.PyAudio, distance: int, angle: int, trial: int, date: str):
    '''
    同一ディレクトリにチャンネル毎のwavファイルを出力する。

    Parameters
    ----------
    nch: int
            チャンネル数
    output_path: str

    '''
    # ディレクトリを作成
    output_dir_path = consts.OUTPUT_PATH + '/' + date + \
        '_' + str(distance) + 'm_' + str(angle) + '_trial' + str(trial)
    os.mkdir(output_dir_path)

    for i in range(consts.RESPEAKER_CHANNELS):
        output_file_path = output_dir_path + '/rec_' + str(i) + '.wav'
        outputWaveFile(
            output_file_path, frames[i], p, consts.RESPEAKER_WIDTH, consts.RESPEAKER_RATE)

def outputWaveFilesForService(consts: Rec_Consts, frames: List, p: pyaudio.PyAudio, dirname: str) -> str:
    '''
    同一ディレクトリにチャンネル毎のwavファイルを出力する。

    Parameters
    ----------
    nch: int
            チャンネル数
    output_path: str

    '''
    try:
        # ディレクトリを作成
        output_dir_path = consts.OUTPUT_PATH + '/' + dirname
        os.makedirs(output_dir_path, exist_ok=True)

        for i in range(consts.RESPEAKER_CHANNELS):
            output_file_path = output_dir_path + '/rec_' + str(i) + '.wav'
            outputWaveFile(
                output_file_path, frames[i], p, consts.RESPEAKER_WIDTH, consts.RESPEAKER_RATE)
        return output_dir_path
    except:
        raise Exception('Failed to save wav files.')


def outputWaveFiles(consts: Rec_Consts, frames: List, p: pyaudio.PyAudio):
    '''
    同一ディレクトリにチャンネル毎のwavファイルを出力する。

    Parameters
    ----------
    nch: int
            チャンネル数
    output_path: str

    '''
    # ディレクトリを作成
    output_dir_path = consts.OUTPUT_PATH + '/' + getDirnameInTimeFormat()
    os.mkdir(output_dir_path)

    for i in range(consts.RESPEAKER_CHANNELS):
        output_file_path = output_dir_path + '/rec_' + str(i) + '.wav'
        outputWaveFile(
            output_file_path, frames[i], p, consts.RESPEAKER_WIDTH, consts.RESPEAKER_RATE)


def outputWaveFile(file_path: str, frame: List, p: pyaudio.PyAudio, width: int, fs: int) -> None:
    '''
    音声データをwavファイルを出力する。

    Parameters
    ----------
    file_path: str
            ファイルパス
    frame: array-like
            音声データが格納された配列
    pa: pyaudio.PyAudio
            pyaudio
    '''
    try:
        wf = wave.open(file_path, 'wb')
        wf.setnchannels(1)
        wf.setsampwidth(p.get_sample_size(p.get_format_from_width(width)))
        wf.setframerate(fs)
        wf.writeframes(b''.join(frame))
    except:
        raise Exception(f'Failed to write audio file.: {file_path}')
    else:
        print(f'Success to write audio file.: {file_path}')
    finally:
        wf.close()
