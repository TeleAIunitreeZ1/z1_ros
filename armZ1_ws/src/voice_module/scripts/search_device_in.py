import pyaudio  

# 创建 PyAudio 对象  
p = pyaudio.PyAudio()  

# 列出所有音频输入设备  
print("可用的音频输入设备:")  
for i in range(p.get_device_count()):  
    device_info = p.get_device_info_by_index(i)  
    if device_info['maxInputChannels'] > 0:  # 只列出输入设备  
        print(f"设备索引: {i}, 设备名称: {device_info['name']}")  

# 关闭 PyAudio 对象  
p.terminate()
