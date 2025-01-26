# import keyboard
# import sys 
import os

# # 获取脚本所在目录 
# script_path = os.path.dirname(os.path.abspath(__file__)) 
# # 将脚本所在目录添加到 Python 的系统路径中 
# sys.path.append(script_path)

import numpy as np
import math
import function as f

import sys
print(sys.executable)
sys.path.append("../lib")
import unitree_arm_interface

import threading

import rospy
from std_msgs.msg import Int32
from vision_module.msg import ObjectDetection



np.set_printoptions(precision=8, suppress=True)

break_flag = False
# def on_esc_press(event):
#     global break_flag
#     break_flag =  True
#     rospy.loginfo("ESC key pressed. Exiting program.")

# keyboard.on_press_key('esc', on_esc_press)
# rospy.loginfo('Press ESC to exit the program.')

def user_input_listener():
    global break_flag
    while True:
        user_input = input("Enter 'q' to quit: ")
        if user_input.lower() == 'q':
            break_flag = True
            break

# 使用线程监听用户输入
input_thread = threading.Thread(target=user_input_listener)
input_thread.start()

def pixeltobase(u,v,d):
    
    # #手眼矩阵(一个螺母)
    # matrixHand2Camera = np.array([[ 0.01016862, 0.00842434, 0.99991281, 0.10144304],
    #                             [-0.99859197, 0.05215064, 0.00971582, 0.03207163],
    #                             [-0.05206424, -0.9986037, 0.00894278, -0.05421758],
    #                             [ 0, 0, 0, 1]])   

    #手眼矩阵（三个螺母）
    matrixHand2Camera = np.array([[0.00253969, -0.03171856, 0.99949361, 0.10205003],
                                [-0.99953336, 0.03034445, 0.00350276, 0.03116234],
                                [-0.03044018, -0.99903611, -0.0316267, -0.06167485],
                                [ 0, 0, 0, 1]])   

    #末端姿态矩阵（手到基座）每次重新计算
    state = arm.lowstate.getQ()
    start_pos = armModel.forwardKinematics(state, 5)
    matrixBase2Hand = np.array(start_pos) 
    #print("末端姿态矩阵：",matrixBase2Hand)

    #内参矩阵（color）
    matrixCamera2Pixel = np.array([[609.5366821289062, 0, 324.4776916503906],
    [0, 608.2521362304688, 247.832275390625],
    [0, 0, 1]]) 

    matrixBase2Camera = np.dot(matrixBase2Hand,matrixHand2Camera)
    matrixCamera2Base = np.linalg.inv(matrixBase2Camera)

    d = d * 0.001
    rospy.loginfo(f"Coordinates for detection: {u}, {v}, {d}")

    #获取饮品实际中心点坐标
    base_pos = np.dot(np.linalg.inv(matrixCamera2Base[0:3,0:3]),d*np.dot(np.linalg.inv(matrixCamera2Pixel),np.array([u,v,1]).reshape(3,1))-matrixCamera2Base[:3,3].reshape(3,1))
    #a=d*np.dot(np.linalg.inv(matrixCamera2Pixel),np.array([u,v,1]).reshape(3,1))
    #print("a:",a)
    effec_pos = np.zeros((1, 6))
    effec_pos[0, 3] = base_pos[0, 0]
    effec_pos[0, 4] = base_pos[1, 0]
    effec_pos[0, 5] = base_pos[2, 0]

    effector_pos = effec_pos.flatten()
    effector_pos[0] = effector_pos[0] + 1.57    #夹爪姿态
    effector_pos[3] = effector_pos[3] + 0.03    #补偿坐标到物体的实际坐标
    rospy.loginfo("Coordinate for J5: ", effector_pos)

    return effector_pos


def Joint_ctl_ct(z1_pos,joint_hand,joint_speed,z1_pos_d,z1_pos_h):        #完成对位置坐标的改变，同时进行控制
    z1_pos_ch = np.copy(z1_pos)
    z1_pos_ch[3] = z1_pos[3] + z1_pos_d*z1_pos[3]/math.sqrt(z1_pos[3]**2 + z1_pos[4]**2)
    z1_pos_ch[4] = z1_pos[4] - abs(z1_pos_d)*z1_pos[4]/math.sqrt(z1_pos[3]**2 + z1_pos[4]**2)
    z1_pos_ch[5] = z1_pos[5] + z1_pos_h
    z1_pos_ctrl = f.Hand_ctl_mt(z1_pos_ch, joint_hand, joint_speed)
    en_act = f.Hand_ctl_act(z1_pos_ctrl)
    if(en_act == 1):
        arm.MoveJ(z1_pos_ctrl, joint_hand, joint_speed)
        rospy.loginfo("Arrived New Position:", z1_pos_ctrl)
    else:
        rospy.logwarn("Cannot Arrived Position:",z1_pos_ctrl)
    return en_act

def handle_detection(msg):
    rospy.loginfo(f"Received detection for {msg.label}: (x={msg.x}, y={msg.y}, depth={msg.depth})")
    global effector_pos, effector_pos_ctl
    effector_pos = pixeltobase(msg.x, msg.y, msg.depth)
    if effector_pos is not None: 
        effector_pos_ctl = effector_pos

def callback_robot_state(data):
    global robot_state
    robot_state = data.data
    rospy.loginfo("Received robot_state: %d", robot_state)


rospy.init_node('arm_controller_node', anonymous=True)
robot_state_sub = rospy.Subscriber("robot_state_topic", Int32, callback_robot_state)
detection_sub = rospy.Subscriber('detection', ObjectDetection, handle_detection)
rate = rospy.Rate(10)

arm =  unitree_arm_interface.ArmInterface(hasGripper=True)
armState = unitree_arm_interface.ArmFSMState
armModel = arm._ctrlComp.armModel

arm.loopOn()

while not rospy.is_shutdown():
    try:
        if break_flag: 
            rospy.loginfo("Break flag triggered. Exiting the loop.") 
            break

        if robot_state == -1 or effector_pos is None:
            continue

        if effector_pos_ctl is None: 
            rospy.logwarn("effector_pos_ctl is not set yet. Waiting for valid detection data.") 
            continue

        #状态1 扭头进行人脸识别的位置
        elif robot_state == 1:

            arm.labelRun("lookface")
            rospy.loginfo("Move after 5s..")
            rospy.sleep(5) 
            
        elif robot_state == 2:
            
            arm.labelRun("forward")

            #移动到目标位置前方
            act = Joint_ctl_ct(effector_pos, 0, 1.2 ,-0.21 ,-0.01)
            if(act == 1):
                rospy.loginfo("1.Arrived the forward position..\n",)
            else:
                rospy.logwarn("Cannot arrived the position. Stop moving..")
            rospy.sleep(2)

            #打开抓夹
            act = Joint_ctl_ct(effector_pos, -1.0 ,2.0 ,-0.21 ,-0.01)
            if(act == 1):
                rospy.loginfo("Open the Claw..\n")
            else:
                rospy.logwarn("Cannot arrived the position. Stop moving..")
            rospy.sleep(2)

            #移动到目标位置
            act = Joint_ctl_ct(effector_pos, -1.0 ,0.5 ,-0.09 ,-0.01)
            if(act == 1):
                rospy.loginfo("2.Arrived the target position..\n")
            else:
                rospy.logwarn("Cannot arrived the position. Stop moving..")
            rospy.sleep(2)

            #关闭抓夹
            act = Joint_ctl_ct(effector_pos, -0.6 ,0.5 ,-0.09 ,-0.01)
            if(act == 1):
                rospy.loginfo("Close the Claw..\n")
            else:
                rospy.logwarn("Cannot arrived the position. Stop moving..")
            rospy.sleep(2)

            #移动到目标位置上方
            act = Joint_ctl_ct(effector_pos, -0.6 ,0.5 ,-0.09 ,0.09)
            if(act == 1):
                rospy.loginfo("3.Arrived the upper position..\n")
            else:
                rospy.logwarn("Cannot arrived the position. Stop moving..")
            rospy.sleep(2)

            #收回机械臂
            rospy.loginfo("Withdraw the Claw..")
            effector_pos_ctl = np.array([1.57, 0, 0, 0.5, 0, 0.35])
            arm.MoveJ(effector_pos_ctl, -0.6, 2.0)
            rospy.sleep(2)

            rospy.loginfo("Move to Destination..")
            effector_pos_ctl = np.array([0, 1.57, -1.57, 0.1, -0.3, 0.2])
            arm.MoveJ(effector_pos_ctl, -0.6, 1.0)
            rospy.sleep(2)

            rospy.loginfo("Open the Claw..")
            arm.MoveJ(effector_pos_ctl, -1.0, 0.3)

            rospy.loginfo("Back to Start after 2s..")
            rospy.sleep(2)
            arm.backToStart()

            # reply_re = "您好，请取走您的商品"
            # tts_main(reply_re)

        elif robot_state == 3:

            rospy.loginfo("MINOR, Back to Start..")
            arm.backToStart()

        elif robot_state == 4:

            arm.labelRun("forward")

            #移动到目标位置前方
            act = Joint_ctl_ct(effector_pos, 0, 1.2 ,-0.21 ,-0.04)
            if(act == 1):
                rospy.loginfo("1.Arrived the forward position..\n",)
            else:
                rospy.logwarn("Cannot arrived the position. Stop moving..")
            rospy.sleep(2)

            #打开抓夹
            act = Joint_ctl_ct(effector_pos, -1.0 ,2.0 ,-0.21 ,-0.04)
            if(act == 1):
                rospy.loginfo("Open the Claw..\n")
            else:
                rospy.logwarn("Cannot arrived the position. Stop moving..")
            rospy.sleep(2)

            #移动到目标位置
            act = Joint_ctl_ct(effector_pos, -1.0 ,0.5 ,-0.1 ,-0.04)
            if(act == 1):
                rospy.loginfo("2.Arrived the target position..\n")
            else:
                rospy.logwarn("Cannot arrived the position. Stop moving..")
            rospy.sleep(2)

            #关闭抓夹
            act = Joint_ctl_ct(effector_pos, -0.73 ,0.5 ,-0.1 ,-0.04)
            if(act == 1):
                rospy.loginfo("Close the Claw..\n")
            else:
                rospy.logwarn("Cannot arrived the position. Stop moving..")
            rospy.sleep(2)

            #移动到目标位置上方
            act = Joint_ctl_ct(effector_pos, -0.73 ,0.5 ,-0.1 ,0.09)
            if(act == 1):
                rospy.loginfo("3.Arrived the upper position..\n")
            else:
                rospy.logwarn("Cannot arrived the position. Stop moving..")
            rospy.sleep(2)

            #收回机械臂
            rospy.loginfo("Withdraw the Claw..")
            effector_pos_ctl = np.array([1.57, 0, 0, 0.5, 0, 0.35])
            arm.MoveJ(effector_pos_ctl, -0.73, 2.0)
            rospy.sleep(2)

            rospy.loginfo("Move to Destination..")
            effector_pos_ctl = np.array([0, 1.57, -1.57, 0.1, -0.3, 0.2])
            arm.MoveJ(effector_pos_ctl, -0.73, 1.0)
            rospy.sleep(2)

            rospy.loginfo("Open the Claw..")
            arm.MoveJ(effector_pos_ctl, -1.0, 0.3)

            rospy.loginfo("Back to Start after 2s..")
            rospy.sleep(2)
            arm.backToStart()

        else:
            rospy.loginfo("Unknown state: %d", robot_state)

    except Exception as e:
        rospy.logerr("Error handling state %d: %s", robot_state, str(e))
        arm.backToStart()

input_thread.join()
try:
    rospy.loginfo("Break..")
    arm.backToStart()
    arm.loopOff()
    rospy.loginfo("End of Movement..")
except Exception as e: 
    rospy.logerr(f"Error during shutdown: {e}")
