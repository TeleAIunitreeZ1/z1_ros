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
from extract import *
from parameter import *
from setout import *
from setout_keda import *
from pydub import AudioSegment

current_file_path = os.path.abspath(__file__)
voice_module_path = os.path.dirname(current_file_path) # /home/jianghao/armZ1_ws/src/voice_module/scripts
voice_module_root = os.path.dirname(voice_module_path) # /home/jianghao/armZ1_ws/src/voice_module
sys.path.append(voice_module_root)

from age_model.face_model import*

import rospy
from std_msgs.msg import Int32
from std_msgs.msg import String

with open('/home/jianghao/armZ1_ws/src/voice_module/scripts/output1.txt', "w") as f:
    f.write("None"+ "\n")

class VoiceRecognition:
    def __init__(self, device_index=5, rate=16000, chunk=1024, channels=1, threshold=2000, record_seconds=1,zhen=10):
        rospy.init_node('voice_recognition_node', anonymous=True)
        self.robot_state_pub = rospy.Publisher('robot_state_topic', Int32, queue_size=10)
        self.keyword_pub = rospy.Publisher('keyword_topic', String, queue_size=10)
        self.photo_nowine_pub = rospy.Publisher('photo_nowine_topic', Int32, queue_size=10)

        self.RATE = rate
        self.ZHEN = zhen
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
        
    def publish_robot_state(self, state):
        self.robot_state_pub.publish(state)
        #rospy.loginfo(f"Published robot state {state}")

    def publish_photo_nowine_state(self, state):
        self.photo_nowine_pub.publish(state)
        #rospy.loginfo(f"Published photo_nowine_state {state}")
    
    def save_pcm(self, frames):
        # filename = "temp_audio.wav"
        # audio_data = np.concatenate(self.frames)
        # # 保存音频文件
        # with wave.open(filename, 'wb') as wf:
        #     wf.setnchannels(1)
        #     wf.setsampwidth(2)
        #     wf.setframerate(16000)
        #     wf.writeframes(audio_data.tobytes())
        # audio=AudioSegment.from_file(filename)
        # audio =audio.set_frame_rate(16000)
        # audio.export(filename,format="wav")
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
                flag = True
                # text = run()
                text = "None"

                # while(flag):
                #     try:
                #         with open('/home/jianghao/armZ1_ws/src/voice_module/scripts/output1.txt', "r") as f:
                #             text = f.readline().strip()
                        
                #         with open('/home/jianghao/armZ1_ws/src/voice_module/scripts/output1.txt', "w") as f:
                #             f.write("None" + "\n")
                #         flag = False
                #     except:
                #         continue
                # print(text)
                # if text == "None" or text == "":
                #     self.flag = self.flag
                #     self.publish_robot_state(-1)
                #     self.publish_photo_nowine_state(1)
                #     continue

                detection_data = np.load('/home/jianghao/armZ1_ws/src/vision_module/detect.npy', allow_pickle=True).item()
       
                while(flag):
                    self.publish_robot_state(-1)
                    self.publish_photo_nowine_state(1)
                    try:
                        with open('/home/jianghao/armZ1_ws/src/voice_module/scripts/output1.txt', "r") as f:
                            text = f.readline().strip()
                        
                        with open('/home/jianghao/armZ1_ws/src/voice_module/scripts/output1.txt', "w") as f:
                            f.write("None" + "\n")

                            
                        if text != 'None' and text != "":
                            print(text)
                            flag = False

                    except:
                        continue
                sentence = [text]
                keywords_found = extract(sentence, keyword_list)
                            
                # rospy.loginfo(f"句子: {text}")
                # rospy.loginfo(f"提取的关键词: {keywords_found}")
                
                if len(keywords_found[0]) == 0:
                    reply_re = "对不起，我这里没有你想要的东西，可以换个别的吗？"
                    self.publish_robot_state(-1)
                    tts_main(reply_re)
                    self.flag = 0

                else:
                    rospy.loginfo(f"Publishing keyword: {keywords_found[0][0]}")
                    self.keyword_pub.publish(keywords_found[0][0])
                    if "东西" in keywords_found[0][0]:
                        self.publish_photo_nowine_state(1)
                        r = "您好，欢迎光临小卖部。您可以选取您想要的饮品。"
                        tts_main(r)
                        
                    elif ("鸡尾酒" in keywords_found[0][0]) or ("啤酒" in keywords_found[0][0]):
                        self.publish_photo_nowine_state(0)
                        self.publish_robot_state(1)
                        r = "即将开始识别是否成年"
                        tts_main(r)
                        rospy.loginfo("Begin Face recognition..")

                        is_adult_result = facedetect()   # 调用人脸识别年龄模块，判断是否成年
                        rospy.loginfo("Result of Age Estimation: %s ", is_adult_result)
                        if is_adult_result:

                            re = "已成年，可以购买酒类饮品"
                            tts_main(re)
                            self.publish_robot_state(4)
                            rospy.sleep(5)
                            self.flag = 0

                        else:
                            rep = "未成年不允许购买酒类饮品"
                            tts_main(rep)
                            self.publish_robot_state(3)
        
                            rospy.loginfo("Back to Start..")
                            rospy.sleep(5)
                            self.flag = 0

                    elif keywords_found[0][0] in detection_data:
                        self.publish_photo_nowine_state(1)
                        self.publish_robot_state(2)
                        # reply = "好的，我现在就去拿"
                        # reply_re = reply + keywords_found[0][0]
                        # tts_main(reply_re)

                        rospy.loginfo("机械臂即将夹取饮品..")
                        rospy.sleep(3)
                        self.flag = 0
                    else:
                        self.publish_photo_nowine_state(1)
                        self.publish_robot_state(-1)
                        rospy.loginfo("没有看见该饮品..")
                      
                    
                flag = True


            

        except KeyboardInterrupt:
            if self.frames and self.flagflag == 1:
                self.save_pcm(self.frames)
                print("Recording stopped.")
            
            
        self.stream.stop_stream()
        self.stream.close()
        self.audio.terminate()

    def listen1(self):
        print("shooting...")

        # 定义一个映射字典
        product_map = {
            0: "椰子真果粒",
            1: "草莓真果粒",
            2: "黄桃真果粒",
            3: "百岁山",
            4: "可口可乐",
            5: "哇哈哈",
            6: "农夫山泉",
            7: "怡宝"
        }

        try:
            while not rospy.is_shutdown():
                flag = True
                
                # 使用键盘输入来代替读取txt文件
                text = input("请输入数字（0-7）以选择对应产品: ")
                
                # 如果输入不在有效范围内，则继续请求输入
                while not text.isdigit() or int(text) not in product_map:
                    print("无效的输入，请输入0-7之间的数字！")
                    text = input("请输入数字（0-7）以选择对应产品: ")
                
                # 获取对应的产品名称
                text = product_map[int(text)]
                
                print(f"输入的产品名称: {text}")
                sentence = [text]

                # 提取关键词
                keywords_found = extract(sentence, keyword_list)

                if len(keywords_found[0]) == 0:
                    reply_re = "对不起，我这里没有你想要的东西，可以换个别的吗？"
                    self.publish_robot_state(-1)
                    tts_main(reply_re)
                    self.flag = 0
                else:
                    rospy.loginfo(f"Publishing keyword: {keywords_found[0][0]}")
                    self.keyword_pub.publish(keywords_found[0][0])

                    if "东西" in keywords_found[0][0]:
                        self.publish_photo_nowine_state(1)
                        r = "您好，欢迎光临小卖部。您可以选取您想要的饮品。"
                        tts_main(r)

                    elif ("鸡尾酒" in keywords_found[0][0]) or ("啤酒" in keywords_found[0][0]):
                        self.publish_photo_nowine_state(0)
                        self.publish_robot_state(1)
                        r = "即将开始识别是否成年"
                        tts_main(r)
                        rospy.loginfo("Begin Face recognition..")

                        is_adult_result = facedetect()   # 调用人脸识别年龄模块，判断是否成年
                        rospy.loginfo("Result of Age Estimation: %s ", is_adult_result)
                        if is_adult_result:
                            re = "已成年，可以购买酒类饮品"
                            tts_main(re)
                            self.publish_robot_state(4)
                            rospy.sleep(5)
                            self.flag = 0
                        else:
                            rep = "未成年不允许购买酒类饮品"
                            tts_main(rep)
                            self.publish_robot_state(3)
                            rospy.loginfo("Back to Start..")
                            rospy.sleep(5)
                            self.flag = 0

                    elif keywords_found[0][0] in detection_data:
                        self.publish_photo_nowine_state(1)
                        self.publish_robot_state(2)
                        rospy.loginfo("机械臂即将夹取饮品..")
                        rospy.sleep(3)
                        self.flag = 0
                    else:
                        self.publish_photo_nowine_state(1)
                        self.publish_robot_state(-1)
                        rospy.loginfo("没有看见该饮品..")

                flag = True

        except KeyboardInterrupt:
            if self.frames and self.flag == 1:
                self.save_pcm(self.frames)
                print("Recording stopped.")
            
        self.stream.stop_stream()
        self.stream.close()
        self.audio.terminate()


voice_recognition = VoiceRecognition()

voice_recognition.listen1()

