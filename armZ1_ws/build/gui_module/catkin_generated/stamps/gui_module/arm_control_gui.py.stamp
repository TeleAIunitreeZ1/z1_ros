import sys
import rospy
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout
from std_msgs.msg import String
import subprocess

class ArmControlApp(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        self.setWindowTitle('机械臂识别控制')

        # Create a button to start recognition
        start_button = QPushButton('开始识别', self)
        start_button.clicked.connect(self.start_recognition)

        # Layout
        layout = QVBoxLayout()
        layout.addWidget(start_button)
        self.setLayout(layout)

    def start_recognition(self):
        # Start ROS modules via subprocess or ROS communication
        rospy.init_node('arm_control_gui', anonymous=True)

        # Example of starting different modules
        subprocess.Popen(['roslaunch', 'vision_module', 'run_yolo.launch'])
        subprocess.Popen(['roslaunch', 'voice_module', 'voice_detect_for.launch'])
        subprocess.Popen(['roslaunch', 'arm_module', 'arm_controller.launch'])

        # Alternatively, you can use ROS publishers or services to trigger actions
        # pub = rospy.Publisher('/start_recognition', String, queue_size=10)
        # pub.publish('start')

