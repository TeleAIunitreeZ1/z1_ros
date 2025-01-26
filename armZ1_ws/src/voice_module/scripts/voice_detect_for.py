import sys 
import os

# 获取脚本所在目录 
script_path = os.path.dirname(os.path.abspath(__file__)) 
# 将脚本所在目录添加到 Python 的系统路径中 
sys.path.append(script_path)

import pyaudio
import numpy as np
from iat_ws_python3_for import *
from extract import *
from parameter import *
from setout import *
from setout_keda import *

current_file_path = os.path.abspath(__file__)
voice_module_path = os.path.dirname(current_file_path) # /home/jianghao/armZ1_ws/src/voice_module/scripts
voice_module_root = os.path.dirname(voice_module_path) # /home/jianghao/armZ1_ws/src/voice_module
sys.path.append(voice_module_root)

from age_model.face_model import*

import rospy
from std_msgs.msg import Int32
from std_msgs.msg import String


class VoiceRecognition:
    def __init__(self, device_index=5, rate=16000, chunk=1024, channels=1, threshold=4000, record_seconds=1):
        
        rospy.init_node('voice_recognition_node', anonymous=True)
        self.keyword_pub = rospy.Publisher('keyword_topic', String, queue_size=10)

        self.RATE = rate
        self.CHUNK = chunk
        self.FORMAT = pyaudio.paInt16  # 16位
        self.CHANNELS = channels
        self.RECORD_SECONDS = record_seconds
        self.THRESHOLD = threshold
        self.audio = pyaudio.PyAudio()
        self.DEVICE_INDEX = device_index
        
        self.frames = []
        self.silent_frames = 0
        self.flag = 0
        
        self.stream = self.audio.open(format=self.FORMAT,
                                      channels=self.CHANNELS,
                                      rate=self.RATE,
                                      input=True,
                                      input_device_index=self.DEVICE_INDEX,
                                      frames_per_buffer=self.CHUNK)
    
    def save_pcm(self, frames):
        filename = "audio.pcm"
        with open(filename, 'wb') as f:
            f.write(b''.join(frames))
        print(f"Saved file: {filename}")
    
    def detect_silence(self, data):
        audio_data = np.frombuffer(data, dtype=np.int16)
        max_val = np.max(np.abs(audio_data))
        return max_val < self.THRESHOLD
       
    def listen(self):
        print("Listening...")

        try:
            while not rospy.is_shutdown():
                
                data = self.stream.read(self.CHUNK)
                self.frames.append(data)
                
                if self.detect_silence(data):
                    self.silent_frames += 1
                else:
                    self.flag += 1
                    self.silent_frames = 0
                
                if self.silent_frames >= int(self.RATE / self.CHUNK * self.RECORD_SECONDS):
                    self.stream.stop_stream()
                    print("Silence detected.")
                    if self.flag >= 3:
                        self.save_pcm(self.frames)
                        text = run()  
                        if text == "None":
                            self.stream.start_stream()
                            continue
                                
                        sentence = [text]
                        keywords_found = extract(sentence, keyword_list)
                                    
                        # rospy.loginfo(f"句子: {text}")
                        # rospy.loginfo(f"提取的关键词: {keywords_found}")
                        
                        if len(keywords_found[0]) == 0:
                            reply_re = "对不起，我这里没有您想要的商品，请确认商品名称或者重新选择。"

                            tts_main(reply_re)
                            self.flag = 0

                        else:
                            rospy.loginfo(f"Publishing keyword: {keywords_found[0][0]}")
                            self.keyword_pub.publish(keywords_found[0][0])

                            if "东西" in keywords_found[0][0]:
                                r = "您好，欢迎光临小卖部。您可以选取您想要的饮品。"
                                tts_main(r)
                            else:
                                reply = "好的，我现在就去拿"
                                reply_re = reply + keywords_found[0][0]
                                tts_main(reply_re)


                                rospy.loginfo("机械臂即将夹取饮品..")
                                rospy.sleep(3)
                                self.flag = 0

                    else:
                        self.flag = self.flag


                    print("Listening...")
                    self.frames = []
                    self.silent_frames = 0
                    self.stream.start_stream()  # clear

        except KeyboardInterrupt:
            if self.frames and self.flag == 1:
                self.save_pcm(self.frames)
                print("Recording stopped.")
            
            
        self.stream.stop_stream()
        self.stream.close()
        self.audio.terminate()

voice_recognition = VoiceRecognition()

voice_recognition.listen()

