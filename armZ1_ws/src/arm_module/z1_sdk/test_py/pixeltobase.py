import rospy
import numpy as np

from std_msgs.msg import String
from vision_module.msg import ObjectDetection

import sys
sys.path.append("../lib")
import unitree_arm_interface

class Transformation:
    def __init__(self):
        rospy.init_node('coordinate_transformation_node', anonymous=True)
        self.arm =  unitree_arm_interface.ArmInterface(hasGripper=True)
        self.armModel = self.arm._ctrlComp.armModel
        self.armState = unitree_arm_interface.ArmFSMState

        self.product_positions = {}
        self.keyword_sub = rospy.Subscriber('keyword_topic', String, self.handle_keyword)
        self.detection_sub = rospy.Subscriber('detection', ObjectDetection, self.handle_detection)
        
        self.current_effector_pos = None

    def handle_detection(self, msg):
        self.product_positions[msg.label] = (msg.x, msg.y, msg.depth)
        rospy.loginfo(f"Received detection for {msg.label}: (x={msg.x}, y={msg.y}, depth={msg.depth})")
        self.pixeltobase(msg.label)

    def handle_keyword(self, msg):
        keyword = msg.data
        rospy.loginfo(f"Received keyword: {keyword}")
        self.pixeltobase(keyword)

    def pixeltobase(self, keyword):

        rospy.loginfo(f"Processing position for {keyword}")
        
        #手眼矩阵
        matrixHand2Camera = np.array([[ 0.01016862, 0.00842434, 0.99991281, 0.10144304],
                                    [-0.99859197, 0.05215064, 0.00971582, 0.03207163],
                                    [-0.05206424, -0.9986037, 0.00894278, -0.05421758],
                                    [ 0, 0, 0, 1]])   

        #末端姿态矩阵（手到基座）每次重新计算
        state = Transformation.self.arm.lowstate.getQ()
        start_pos = Transformation.self.armModel.forwardKinematics(state, 5)
        matrixBase2Hand = np.array(start_pos) 
        print("末端姿态矩阵：",matrixBase2Hand)

        #内参矩阵（color）
        matrixCamera2Pixel = np.array([[609.5366821289062, 0, 324.4776916503906],
        [0, 608.2521362304688, 247.832275390625],
        [0, 0, 1]]) 

        matrixBase2Camera = np.dot(matrixBase2Hand,matrixHand2Camera)
        matrixCamera2Base = np.linalg.inv(matrixBase2Camera)

        if keyword in self.product_positions:
            u, v, d = self.product_positions[keyword]
            d = d * 0.001
            rospy.loginfo(f"Coordinates for {keyword}: {u}, {v}, {d}")
        else:
            rospy.logwarn(f"No data for target: {keyword}")

        base_pos = np.dot(np.linalg.inv(matrixCamera2Base[0:3,0:3]),d*np.dot(np.linalg.inv(matrixCamera2Pixel),np.array([u,v,1]).reshape(3,1))-matrixCamera2Base[:3,3].reshape(3,1))
        #a=d*np.dot(np.linalg.inv(matrixCamera2Pixel),np.array([u,v,1]).reshape(3,1))
        #print("a:",a)
        effec_pos = np.zeros((1, 6))
        effec_pos[0, 3] = base_pos[0, 0]
        effec_pos[0, 4] = base_pos[1, 0]
        effec_pos[0, 5] = base_pos[2, 0]

        #设立初始位置
        effector_pos = effec_pos.flatten()
        effector_pos[0] = effector_pos[0] + 1.57    #夹爪姿态
        effector_pos[3] = effector_pos[3] + 0.03    #补偿坐标到物体的实际坐标

        self.current_effector_pos = effector_pos
        rospy.loginfo("Coordinate for J5: {}".format(self.current_effector_pos))
