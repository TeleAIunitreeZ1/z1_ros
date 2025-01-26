import sounddevice as sd  

def list_audio_output_devices():  
    # Get the list of all audio devices  
    devices = sd.query_devices()  
    
    print("Available Audio Output Devices:")  
    for i, device in enumerate(devices):  
        # Check if the device is an output device  
        if device['max_output_channels'] > 0:  
            print(f"{i}: {device['name']} (Channels: {device['max_output_channels']})")  

if __name__ == "__main__":  
    list_audio_output_devices()
