import sys 
import os

# 获取脚本所在目录 
script_path = os.path.dirname(os.path.abspath(__file__)) 
# 将脚本所在目录添加到 Python 的系统路径中 
sys.path.append(script_path)
import wave

import pyaudio
import numpy as np
from iat_ws_python3 import *
from iat_ws_python3_for import *
from extract import *
from parameter import *
from setout import *
from setout_keda import *
from pydub import AudioSegment

from threading import Lock

current_file_path = os.path.abspath(__file__)
voice_module_path = os.path.dirname(current_file_path) # /home/jianghao/armZ1_ws/src/voice_module/scripts
voice_module_root = os.path.dirname(voice_module_path) # /home/jianghao/armZ1_ws/src/voice_module
sys.path.append(voice_module_root)

from age_model.face_model import*

import rospy
from std_msgs.msg import Int32
from std_msgs.msg import String

class VoiceRecognition:
    def __init__(self, device_index=5, rate=16000, chunk=1024, channels=1, threshold=2000):
        self.rate = rate
        self.chunk = chunk
        self.channels = channels
        self.threshold = threshold
        self.audio = pyaudio.PyAudio()
        self.device_index = device_index

        self.stream = self.audio.open(format=pyaudio.paInt16,
                                      channels=self.channels,
                                      rate=self.rate,
                                      input=True,
                                      input_device_index=self.device_index,
                                      frames_per_buffer=self.chunk)

        # self.robot_state_pub = rospy.Publisher('robot_state_topic', Int32, queue_size=10)
        self.keyword_pub = rospy.Publisher('keyword_topic', String, queue_size=10)

    def detect_silence(self, data):
        audio_data = np.frombuffer(data, dtype=np.int16)
        max_val = np.max(np.abs(audio_data))
        return max_val < self.threshold

    def process_audio(self):
        # """
        # 处理音频数据，返回识别结果
        # """
        print("Listening for a command...")
        data = self.stream.read(self.chunk, exception_on_overflow=False)

        if self.detect_silence(data):
            return None  # 如果是静音，返回None

        # 调用语音识别模块 (run 是你的语音识别逻辑函数)
        text = run()
        return text

    def process_command(self, text, keyword_list):
        # """
        # 处理语音识别结果，进行关键词匹配和指令发布
        # """
        if not text or text == "None":
            rospy.loginfo("No valid speech detected.")
            return

        sentence = [text]
        keywords_found = extract(sentence, keyword_list)

        if not keywords_found[0]:
            tts_main("对不起，我这里没有你想要的东西，可以换个别的吗？")
        else:
            rospy.loginfo(f"Keyword found: {keywords_found[0][0]}")
            self.keyword_pub.publish(keywords_found[0][0])

            # 根据关键词执行不同的动作
            if "东西" in keywords_found[0][0]:
                tts_main("您好，欢迎光临小卖部。您可以选取您想要的饮品。")

            # elif "鸡尾酒" in keywords_found[0][0] or "啤酒" in keywords_found[0][0]:
            #     self.robot_state_pub.publish(4)
            #     tts_main("即将开始识别是否成年")
            #     if facedetect():  # 判断是否成年
            #         tts_main("已成年，可以购买酒类饮品")
            #     else:
            #         tts_main("未成年不允许购买酒类饮品")
            else:
                # self.robot_state_pub.publish(2)
                rospy.loginfo("机械臂即将夹取饮品..")
                rospy.sleep(3)

    def shutdown(self):
        # """
        # 清理音频资源
        # """
        self.stream.stop_stream()
        self.stream.close()
        self.audio.terminate()
        rospy.loginfo("Audio stream closed.")

def main():
    # 初始化ROS节点
    rospy.init_node('voice_recognition_node', anonymous=True)

    # 初始化语音识别模块
    voice_recognition = VoiceRecognition()

    try:
        while not rospy.is_shutdown():
            # 监听并处理语音
            text = voice_recognition.process_audio()
            if text:
                voice_recognition.process_command(text, keyword_list)

    except rospy.ROSInterruptException:
        rospy.loginfo("ROS node interrupted.")
    finally:
        voice_recognition.shutdown()

if __name__ == "__main__":
    main()