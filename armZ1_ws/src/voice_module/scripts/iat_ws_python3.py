# -*- coding: utf-8 -*-
import sounddevice as sd
import numpy as np
import wave
import time as tm
import os
import threading
import websocket
import datetime
import hashlib
import base64
import hmac
import json
from urllib.parse import urlencode
import ssl
from wsgiref.handlers import format_date_time
from datetime import datetime
from time import mktime
import librosa
import soundfile as sf
from scipy.io import wavfile
from scipy.signal import resample
from extract import *
STATUS_FIRST_FRAME = 0  # 第一帧的标识
STATUS_CONTINUE_FRAME = 1  # 中间帧标识
STATUS_LAST_FRAME = 2  # 最后一帧的标识


# 设置参数
sample_rate = 44100  # 采样率
threshold = 0.045  # 音量阈值
silence_limit = 0.5  # 无声音时停止录音的时间（秒）
recording = False  # 是否正在录音
recorded_frames = []  # 存储录音数据
result=""
recognize_event = threading.Event()
sua = None


class Ws_Param(object):
    # 初始化
    def __init__(self, APPID, APIKey, APISecret, AudioFile):
        self.APPID = APPID
        self.APIKey = APIKey
        self.APISecret = APISecret
        self.AudioFile = AudioFile
        # 公共参数(common)
        self.CommonArgs = {"app_id": self.APPID}
        # 业务参数(business)，更多个性化参数可在官网查看
        self.BusinessArgs = {"domain": "iat", "language": "zh_cn", "accent": "mandarin", "vinfo": 1, "vad_eos": 10000}
    
    # 生成url
    def create_url(self):
        url = 'wss://ws-api.xfyun.cn/v2/iat'
        # 生成RFC1123格式的时间戳
        now = datetime.now()
        date = format_date_time(mktime(now.timetuple()))
        # 拼接字符串
        signature_origin = "host: " + "ws-api.xfyun.cn" + "\n"
        signature_origin += "date: " + date + "\n"
        signature_origin += "GET " + "/v2/iat " + "HTTP/1.1"
        # 进行hmac-sha256进行加密
        signature_sha = hmac.new(self.APISecret.encode('utf-8'), signature_origin.encode('utf-8'), 
                                 digestmod=hashlib.sha256).digest()
        signature_sha = base64.b64encode(signature_sha).decode(encoding='utf-8')
        authorization_origin = "api_key=\"%s\", algorithm=\"%s\", headers=\"%s\", signature=\"%s\"" % (
            self.APIKey, "hmac-sha256", "host date request-line", signature_sha)
        authorization = base64.b64encode(authorization_origin.encode('utf-8')).decode(encoding='utf-8')
        # 将请求的鉴权参数组合为字典
        v = {
            "authorization": authorization,
            "date": date,
            "host": "ws-api.xfyun.cn"
        }
        # 拼接鉴权参数，生成url
        url = url + '?' + urlencode(v)
        return url

# 收到websocket消息的处理
def on_message(ws, message):
    global result
    global sua
    try:
        response = json.loads(message)
        code = response["code"]
        sid = response["sid"]
        if code != 0:
            errMsg = response["message"]
            print("sid:%s call error:%s code is:%s" % (sid, errMsg, code))
        else:
            data = response["data"]["result"]["ws"]
            for item in data:
                for w in item["cw"]:
                    result += w["w"]
            # print("1111111", result[:-1])
            if(sua != result[:-1]):
                print("sid:%s call success!, result is:%s" % (sid, result))
                sua = result
                # with open("output1.txt", "w") as f:  # 以追加模式打开文件
                #     f.write(result + "\n") 
                sentence = [result]
                keywords_found = extract(sentence, keyword_list)
    except Exception as e:
        print("receive msg, but parse exception:", e)

# 收到websocket错误的处理
def on_error(ws, error):
    print("### error:", error)

# 收到websocket关闭的处理
def on_close(ws, a, b):
    print("### closed ###")

# 收到websocket连接建立的处理
def on_open(ws):
    def run(*args):
        frameSize = 8000  # 每一帧的音频大小
        intervel = 0.04  # 发送音频间隔(单位:s)
        status = STATUS_FIRST_FRAME  # 音频的状态信息，标识音频是第一帧，还是中间帧、最后一帧
        with open(ws.audio_file, "rb") as fp:
            while True:
                buf = fp.read(frameSize)
                # 文件结束
                if not buf:
                    status = STATUS_LAST_FRAME
                # 第一帧处理
                if status == STATUS_FIRST_FRAME:
                    d = {"common": ws.common_args, 
                         "business": ws.business_args, 
                         "data": {"status": 0, "format": "audio/L16;rate=16000",
                                  "audio": str(base64.b64encode(buf), 'utf-8'),
                                  "encoding": "raw"}}
                    d = json.dumps(d)
                    ws.send(d)
                    status = STATUS_CONTINUE_FRAME
                # 中间帧处理
                elif status == STATUS_CONTINUE_FRAME:
                    d = {"data": {"status": 1, "format": "audio/L16;rate=16000",
                                  "audio": str(base64.b64encode(buf), 'utf-8'),
                                  "encoding": "raw"}}
                    ws.send(json.dumps(d))
                # 最后一帧处理
                elif status == STATUS_LAST_FRAME:
                    d = {"data": {"status": 2, "format": "audio/L16;rate=16000",
                                  "audio": str(base64.b64encode(buf), 'utf-8'),
                                  "encoding": "raw"}}
                    ws.send(json.dumps(d))
                    tm.sleep(1)
                    break
                tm.sleep(intervel)
        ws.close()
    thread = threading.Thread(target=run)
    thread.start()
def find_usb_mic():
        devices = sd.query_devices()
        print("\n可用的音频设备:")
        print("-" * 60)
        
        usb_mic_index = None
        
        for i, dev in enumerate(devices):
            # 只显示输入设备
            if dev['max_input_channels'] > 0:
                print(f"设备 {i}: {dev['name']}")
                print(f"   ALSA设备: {dev.get('hostapi_name')} - {dev.get('name')}")
                print(f"   输入通道: {dev['max_input_channels']}")
                print(f"   采样率: {dev['default_samplerate']}Hz")
                print("-" * 60)
                
                if 'usb' in dev['name'].lower() and 'mic' in dev['name'].lower():
                    usb_mic_index = i
        
        if usb_mic_index is not None:
            print(f"\n找到USB麦克风! sounddevice设备号: {usb_mic_index}")
        else:
            print("\n未找到USB麦克风设备")
        
        return usb_mic_index
        
def recognize_audio_xfyun():
    global result
    while True:
        recognize_event.wait()  # 等待事件触发
        recognize_event.clear()  # 清除事件状态
        print("开始识别")
        time1 = datetime.now()
        #sua
        #ws_param = Ws_Param(APPID='72aebd3f', APISecret='NzAzOTdjNTZiNzJhMDgzMDU0MDU1MWJh',
         #                   APIKey='eb4c56ac4a4e5cc28ccb7ef0008ab0c2', AudioFile='output.wav')
        #ybw
        ws_param = Ws_Param(APPID='f879c9a6', APISecret='NzY3ZGQwZGViODc5ZmY1NzMxYmY4NmM1',
                             APIKey='d1faf503ef453239e49690582b14a87f', AudioFile='output.wav')
        #sua 小号
        # ws_param = Ws_Param(APPID='1a732a4f', APISecret='NzNmNjZmNTBmMWRkMmJjYmU0ZGNhYjRk',
        #                     APIKey='7d9db7844585ef02bf4e05e3f735facf', AudioFile='output.wav')

        
        websocket.enableTrace(False)
        ws_url = ws_param.create_url()
        ws = websocket.WebSocketApp(ws_url, on_message=on_message, on_error=on_error, on_close=on_close)
        ws.common_args = ws_param.CommonArgs  # 添加common_args属性
        ws.business_args = ws_param.BusinessArgs  # 添加business_args属性
        ws.audio_file = ws_param.AudioFile  # 添加audio_file属性
        ws.on_open = on_open
        ws.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE})
        time2 = datetime.now()
        print(time2-time1)

def audio_callback(indata, frames, time_info, status):  # 重命名time为time_info
    global recording, recorded_frames, last_sound_time
    if status:
        print(status)
    # 计算音量（均方根值）
    float_data = indata.astype(np.float32) / 32768.0  # int16 转换为 float32
    volume = np.sqrt(np.mean(float_data ** 2))
    print(volume)
    if volume > threshold:
        if not recording:
            print("检测到声音，开始录音...")
            recording = True
            recorded_frames = []
        recorded_frames.append(indata.copy())
        last_sound_time = tm.time()
    else:
        if recording:
            silence_time = tm.time() - last_sound_time
            if silence_time > silence_limit:
                print("停止录音，保存文件...")
                recording = False
                save_recording(recorded_frames)
                recorded_frames = []
            else:
                recorded_frames.append(indata.copy())

def save_recording(frames):
    global result
    filename = "output.wav"
    audio_data = np.concatenate(frames)
    with wave.open(filename, 'wb') as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(sample_rate)
        wf.writeframes(audio_data.tobytes())
    print("录音已保存为 " + filename)
    

    # 读取音频数据和采样率
    sr, data = wavfile.read("output.wav")

    # 计算目标采样点数量
    target_sr = 16000
    num_samples = int(len(data) * target_sr / sr)

    # 重新采样
    data_resampled = resample(data, num_samples)

# 保存为 16 kHz 的 WAV 文件
    wavfile.write("output.wav", target_sr, data_resampled.astype(np.int16))
    result=""
    recognize_event.set()  # 唤醒识别线程

def audio_monitoring():
    blocksize = 4096  # 调整缓冲区大小
    with sd.InputStream(callback=audio_callback, channels=1, dtype='int16', samplerate=sample_rate, blocksize=blocksize,device=find_usb_mic()):
        try:
            while True:
                tm.sleep(0.01)
        except KeyboardInterrupt:
            print("停止监听")

if __name__ == "__main__":
    recognization_thread = threading.Thread(target=recognize_audio_xfyun)
    recognization_thread.start()
    audio_monitoring_thread = threading.Thread(target=audio_monitoring)
    audio_monitoring_thread.start()

    # try:
    #     while True:
    #         tm.sleep(1)
    # except KeyboardInterrupt:
    #     print("停止监听")
    #     recognization_thread.join()
    #     audio_monitoring_thread.join()
