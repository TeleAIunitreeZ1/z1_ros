import pyaudio
import time
import numpy as np
from iat_ws_python3 import *
from extract import *
from parameter import *
from setout import *
from setout_keda import *
import os
import sys
 

RATE = 16000 # 修改采样率为16k
CHUNK = 1024
FORMAT = pyaudio.paInt16 # 16位，+-32768
CHANNELS = 1
RECORD_SECONDS = 2
THRESHOLD = 500
audio = pyaudio.PyAudio()



# 获取设备名称
DEVICE_INDEX= 5
 # 请替换为实际的输入设备名称



stream = audio.open(format=FORMAT,
                    channels=CHANNELS,
                    rate=RATE,
                    input=True,
                    input_device_index=DEVICE_INDEX,
                    frames_per_buffer=CHUNK)

frames = []
silent_frames = 0
flag = 0

def save_pcm(frames):
    filename = "audio.pcm"
    with open(filename, 'wb') as f:
        f.write(b''.join(frames))
        print(f"Saved file: {filename}")

def detect_silence(data):
    audio_data = np.frombuffer(data, dtype=np.int16)
    max_val = np.max(np.abs(audio_data))
    return max_val < THRESHOLD

print("Listening...")

try:
    while True:
        data = stream.read(CHUNK)
        frames.append(data)

        if detect_silence(data):
            silent_frames += 1
        else:
            flag += 1
            silent_frames = 0

        if silent_frames >= int(RATE / CHUNK * RECORD_SECONDS):
            stream.stop_stream()
            print("Silence detected.")
            if flag >= 3:
                save_pcm(frames)
                text = run()
                if text == "None":
                    stream.start_stream()
                    frames = []
                    silent_frames = 0
                    continue


                # text
                # print(text)
                sentence = [text]
                keywords_found = extract(sentence, keyword_list)  

                # 输出结果  
                
                print(f"句子: {text}")  
                print(f"提取的关键词: {keywords_found}")
                
                reply = "好的，我现在就去拿"
                
                if(len(keywords_found[0]) == 0):
                  reply_re = "对不起，我这里没有你想要的东西，可以换个别的吗？"
                else:
                  reply_re = reply + keywords_found[0][0]
#                speak_text(reply_re)
                # speak_text_pytts(reply_re)
                tts_main(reply_re)

                flag = 0
            print("Listening...")
            frames = []
            silent_frames = 0
            stream.start_stream()  # clear

except KeyboardInterrupt:
    if frames and flag == 1:
        save_pcm(frames)
        print("Recording stopped.")

stream.stop_stream()
stream.close()
audio.terminate()
