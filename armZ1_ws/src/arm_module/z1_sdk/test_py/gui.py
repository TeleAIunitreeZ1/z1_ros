import sys
import os
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout
import time

class ArmControlGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Mechanical Arm Control - Manual Mode')
        self.setGeometry(100, 100, 400, 300)  # 调整窗口高度
        
        # Create layout
        layout = QVBoxLayout()
        
        # Create buttons for each module
        self.voice_button = QPushButton('Run Voice Module', self)
        self.voice_button.clicked.connect(self.run_voice_module)
        layout.addWidget(self.voice_button)
        
        self.stop_voice_button = QPushButton('Stop Voice Module', self)
        self.stop_voice_button.clicked.connect(self.stop_voice_module)
        layout.addWidget(self.stop_voice_button)
        
        self.vision_button = QPushButton('Run Vision Module', self)
        self.vision_button.clicked.connect(self.run_vision_module)
        layout.addWidget(self.vision_button)
        
        self.arm_button = QPushButton('Run Arm Movement', self)
        self.arm_button.clicked.connect(self.run_arm_module)
        layout.addWidget(self.arm_button)
        
        self.stop_arm_button = QPushButton('Stop Arm Module', self)
        self.stop_arm_button.clicked.connect(self.stop_arm_module)
        layout.addWidget(self.stop_arm_button)
        
        self.setLayout(layout)
    
    def run_voice_module(self):
        os.system('cd ~/Z1/Z1_model/ && python3 voice_detect.py')
    
    def stop_voice_module(self):
        os.system('pkill -f voice_detect.py')  # 停止语音模块
    
    def run_vision_module(self):
        os.system('cd ~/Z1/Yolov5/ && python3 run_yolo.py')
    
    def run_arm_module(self):
        os.system('cd ~/Z1/test_ws/ && roslaunch unitree_gazebo z1.launch &')  # 后台运行以允许执行后续命令
        time.sleep(3)
        os.system('cd ~/Z1/test_ws/src/z1_controller/build/ && ./sim_ctrl &')   # 后台运行
        time.sleep(3)
        os.system('cd ~/Z1/test_ws/src/z1_sdk/robot/ && python3 main.py')  # 执行最后一个python脚本

    def stop_arm_module(self):
        os.system('pkill -f roslaunch')  # 停止roslaunch命令
        time.sleep(3)
        os.system('pkill -f sim_ctrl')   # 停止sim_ctrl命令
        time.sleep(3)
        os.system('pkill -f main.py')  # 停止matrix_transform.py

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = ArmControlGUI()
    window.show()
    sys.exit(app.exec_())
