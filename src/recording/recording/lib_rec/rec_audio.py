#!/usr/bin/env python3
# coding: utf-8

import pyaudio
import numpy as np
from .load_constants import Rec_Consts
from .output_wav import outputWaveFiles, outputWaveFilesForTest

def recording(consts: Rec_Consts, **kwargs) -> None:
    '''
    単純なレコーディングを行う関数

    Parameters
    ----------
    consts: load_constants.Rec_Consts
        定数
    part_of_output_file_path: str, default = None
        channelの表記を除いたファイルのパス。
    '''
    try:
        p = pyaudio.PyAudio()

        stream = p.open(
            format=p.get_format_from_width(consts.RESPEAKER_WIDTH),
            channels=consts.RESPEAKER_CHANNELS,
            rate=consts.RESPEAKER_RATE,
            input=True,
            output=False,
            input_device_index=consts.RESPEAKER_INDEX,
            frames_per_buffer=consts.CHUNK)

        print("* recording")

        # # 音声データを格納する二次元配列
        frames = [[] for i in range(consts.RESPEAKER_CHANNELS)]

        for _ in range(0, int(consts.RESPEAKER_RATE / consts.CHUNK * consts.RECORD_SECONDS)):
            data = stream.read(consts.CHUNK)

            # channelごとの音声データの取得
            for j in range(consts.RESPEAKER_CHANNELS):
                ch_data = np.frombuffer(data, dtype=np.int16)[
                    j::consts.RESPEAKER_CHANNELS]
                frames[j].append(ch_data.tobytes())

        print("* done recording")

        stream.stop_stream()
        stream.close()
        p.terminate()
    except:
        print(f'Failed to record audio.')
    else:
        # wavファイルに出力
        outputWaveFilesForTest(
            consts, frames, p, kwargs['distance'], kwargs['angle'], kwargs['trial'], kwargs['date'])


if __name__ == "__main__":
    consts = Rec_Consts(index=0, record_sec=1.5,
                        output_path="/home/toranosuke/dov_sys_ws/out")
    recording(consts)
