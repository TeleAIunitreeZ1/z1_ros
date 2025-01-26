from gtts import gTTS
import os
from playsound import playsound

def speak_text(text):
    # 创建 gTTS 对象
  tts = gTTS(text=text, lang='zh')

    # 保存音频文件
  audio_file = "output.mp3"
  tts.save(audio_file)

    # 播放音频文件
  playsound(audio_file)

#if __name__ == "__main__":
 # text_to_speak = "你好，欢迎使用外接麦克风输出声音！"
  #speak_text(text_to_speak)
 

 	 
 	


