from pydub import AudioSegment

def resample_audio(input_file, output_file, target_rate=16000):
    # 加载 44100 Hz 的音频
    audio = AudioSegment.from_file(input_file, format="pcm", frame_rate=44100, sample_width=2, channels=1)
    # 转换为 16000 Hz
    audio = audio.set_frame_rate(target_rate)
    # 导出为新的 PCM 文件
    audio.export(output_file, format="pcm")