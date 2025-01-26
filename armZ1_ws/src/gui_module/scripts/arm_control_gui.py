# import sys
# import os
# import rospy
# from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout
# from std_msgs.msg import String
# import subprocess
# import threading
# import time

# class ArmControlApp(QWidget):
#     def __init__(self):
#         super().__init__()
#         self.initUI()

#     def initUI(self):
#         self.setWindowTitle('机械臂识别控制')

#         # Create a button to start recognition
#         start_button = QPushButton('开始识别', self)
#         start_button.clicked.connect(self.start_recognition)

#         # Layout
#         layout = QVBoxLayout()
#         layout.addWidget(start_button)
#         self.setLayout(layout)
#     def start_recognition(self):
#         # Start ROS modules via subprocess or ROS communication
#         rospy.init_node('arm_control_gui', anonymous=True)

#         # Example of starting different modules
#         subprocess.Popen(['gnome-terminal', '--', 'bash', '-c','roslaunch vision_module run_yolo.launch; exec bash'])
#         subprocess.Popen(['gnome-terminal', '--', 'bash', '-c','roslaunch voice_module voice_detect_for.launch; exec bash'])
#         subprocess.Popen(['gnome-terminal', '--', 'bash', '-c','roslaunch arm_module arm_controller.launch; exec bash'])
#         # Launch your custom command in a separate terminal
#         subprocess.Popen(['gnome-terminal', '--', 'bash', '-c','cd ~/armZ1_ws/src/arm_module/z1_controller/build && ./z1_ctrl; exec bash'])
#         # Alternatively, you can use ROS publishers or services to trigger actions
#         # pub = rospy.Publisher('/start_recognition', String, queue_size=10)
#         # pub.publish('start')

# if __name__ == '__main__': 
#     app = QApplication(sys.argv) 
#     ex = ArmControlApp() 
#     ex.show() 
#     sys.exit(app.exec_())


import sys
import os
import rospy
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QLabel
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont, QPixmap
from std_msgs.msg import String
import subprocess

class ArmControlApp(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        self.setWindowTitle('机械臂识别控制')
        self.setGeometry(100, 100, 800, 600)  # Set the size of the window

        # Set a font for the buttons
        font = QFont()
        font.setPointSize(10)
        font.setBold(True)

        self.image_label = QLabel(self)
        self.pixmap = QPixmap('/home/jianghao/下载/arm.jpeg')  # Load the image
        if not self.pixmap.isNull():
            self.image_label.setPixmap(self.pixmap)
            self.image_label.setAlignment(Qt.AlignCenter)
        else:
            print("Failed to load image.")
            self.image_label.setText("Failed to load image.")
            self.image_label.setAlignment(Qt.AlignCenter)

        start_button = QPushButton('开始识别', self)
        start_button.setFixedHeight(50)  
        start_button.setFont(font)
        start_button.clicked.connect(self.start_recognition)
        
        stop_button = QPushButton('停止识别', self)
        stop_button.setFixedHeight(50)  
        stop_button.setFont(font)
        stop_button.clicked.connect(self.stop_recognition)
        
        manual_control_button = QPushButton('手动控制', self)
        manual_control_button.setFixedHeight(50) 
        manual_control_button.setFont(font)
        manual_control_button.clicked.connect(self.manual_control)
        
        auto_control_button = QPushButton('自动控制', self)
        auto_control_button.setFixedHeight(50) 
        auto_control_button.setFont(font)
        auto_control_button.clicked.connect(self.auto_control)

        # Layout
        main_layout = QVBoxLayout()

        # Add image label to the top 
        main_layout.addWidget(self.image_label) 
        # Add buttons to the bottom 
        buttons_layout = QHBoxLayout() 
        buttons_layout.addWidget(start_button) 
        buttons_layout.addWidget(stop_button) 
        buttons_layout.addWidget(manual_control_button) 
        buttons_layout.addWidget(auto_control_button) 
        main_layout.addLayout(buttons_layout) 
        self.setLayout(main_layout)

    def start_recognition(self):
        # Start ROS modules via subprocess or ROS communication
        rospy.init_node('arm_control_gui', anonymous=True)
        subprocess.Popen(['gnome-terminal', '--', 'bash', '-c','roslaunch vision_module run_yolo.launch; exec bash'])
        subprocess.Popen(['gnome-terminal', '--', 'bash', '-c','roslaunch voice_module voice_detect_for.launch; exec bash'])
        subprocess.Popen(['gnome-terminal', '--', 'bash', '-c','roslaunch arm_module arm_controller.launch; exec bash'])
        subprocess.Popen(['gnome-terminal', '--', 'bash', '-c','cd ~/armZ1_ws/src/arm_module/z1_controller/build && ./z1_ctrl; exec bash'])

    def stop_recognition(self):
        # Here you should add the code to stop the recognition process
        pass

    def manual_control(self):
        # Here you should add the code to start manual control
        pass

    def auto_control(self):
        # Here you should add the code to start auto control
        pass

if __name__ == '__main__': 
    app = QApplication(sys.argv) 
    ex = ArmControlApp() 
    ex.show() 
    sys.exit(app.exec_())
