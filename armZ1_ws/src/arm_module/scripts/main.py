#!/usr/bin/env python3
import select
import tty
import termios
import time
import sys

def getKey():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1) # 设置超时
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

sys.path.append("/home/jianghao/armZ1_ws/src/arm_module/z1_sdk/lib")
import unitree_arm_interface

from math import atan2
import numpy as np
import math
#sys.path.append("/home/jianghao/armZ1_ws/src/arm_module/scripts")
#import function as f

import rospy
from std_msgs.msg import Int32
from arm_module.msg import ObjectDetection

np.set_printoptions(precision=8, suppress=True)


def pixeltobase(u,v,d):

    #手眼矩阵
    # matrixHand2Camera = np.array([[0.00140756, -0.00114016, 0.99999836, 0.09783193],
    #                             [-0.99768202, 0.06803221, 0.00148187, 0.03635455],
    #                             [-0.06803379, -0.99768247, -0.00104176, -0.0611316],
    #                             [ 0, 0, 0, 1]])    

    matrixHand2Camera = np.array([[0.03669228, -0.0197634, 0.99913116,0.08646369],
                                [-0.998902, 0.0284172, 0.03724597, 0.03133078],
                                [-0.02912862, -0.99940076, -0.018699, -0.05673134],
                                [ 0, 0, 0, 1 ]])
    # matrixHand2Camera = np.array([[-0.08539153, 0.15181322, 0.98471368,0.08270564],
    #                             [-0.89684246, 0.41882278, -0.14234146, 0.04645956],
    #                             [-0.43402983, -0.89528779, 0.10038862, -0.03863054],
    #                             [ 0, 0, 0, 1 ]])
                        

    #末端姿态矩阵（手到基座）每次重新计算
    state = arm.lowstate.getQ()
    start_pos = armModel.forwardKinematics(state, 5)
    matrixBase2Hand = np.array(start_pos) 
    # rospy.loginfo(f"matrixBase2Hand——start:%s",matrixBase2Hand)

    #内参矩阵（color）
    matrixCamera2Pixel = np.array([[609.5366821289062, 0, 324.4776916503906],
    [0, 608.2521362304688, 247.832275390625],
    [0, 0, 1]]) 

    matrixBase2Camera = np.dot(matrixBase2Hand,matrixHand2Camera)
    matrixCamera2Base = np.linalg.inv(matrixBase2Camera)

    d = d * 0.001
    #rospy.loginfo(f"Coordinates for detection: {u}, {v}, {d}")

    # #获取饮品实际中心点坐标
    # rospy.loginfo("matrixCamera2Base: %s",matrixCamera2Base)
    # rospy.loginfo("d:%f",d)
    # rospy.loginfo("matrixCamera2Pixel: %s",matrixCamera2Pixel)
    # rospy.loginfo("u:%f",u)
    # rospy.loginfo("v:%f",v)
    base_pos = np.dot(np.linalg.inv(matrixCamera2Base[0:3,0:3]),d*np.dot(np.linalg.inv(matrixCamera2Pixel),np.array([u,v,1]).reshape(3,1))-matrixCamera2Base[:3,3].reshape(3,1))
    #a=d*np.dot(np.linalg.inv(matrixCamera2Pixel),np.array([u,v,1]).reshape(3,1))
    #print("a:",a)
    effec_pos = np.zeros((1, 6))
    effec_pos[0, 3] = base_pos[0, 0]
    effec_pos[0, 4] = base_pos[1, 0]
    effec_pos[0, 5] = base_pos[2, 0]

    effector_pos = effec_pos.flatten()
    effector_pos[0] = effector_pos[0] + 1.57    #夹爪姿态
    effector_pos[3] = effector_pos[3] + 0.04    #补偿坐标到物体的实际坐标
    effector_pos[4] = effector_pos[4]  - 0.05  #y轴右减左加
    rospy.loginfo("Coordinate for J5: %s" , effector_pos)

    return effector_pos

##################位姿1*6转为4*4矩阵####################
def calculate(wanted_pos):
    #wanted_pos是一个pitch yaw roll x y z的1*6的数组(np.array)
    #本函数的作用是将pitch yaw roll x y z的表示转化为4*4旋转矩阵的表示
    pitch_angle = wanted_pos[0]
    yaw_angle = wanted_pos[1]
    roll_angle = wanted_pos[2]
    x_pos = wanted_pos[3]
    y_pos = wanted_pos[4]
    z_pos = wanted_pos[5]
    cartesian_pos = np.array([[x_pos],[y_pos],[z_pos]]) #3*1的x y z
    other_elements = np.array([0,0,0,1])
    R = np.zeros((3,3))
    R_x = np.array([[1,0,0],[0,np.cos(pitch_angle),-1*np.sin(pitch_angle)],[0,np.sin(pitch_angle),np.cos(pitch_angle)]])
    R_y = np.array([[np.cos(yaw_angle),0,np.sin(yaw_angle)],[0,1,0],[-1*np.sin(yaw_angle),0,np.cos(yaw_angle)]])
    R_z = np.array([[np.cos(roll_angle),-1*np.sin(roll_angle),0],[np.sin(roll_angle),np.cos(roll_angle),0],[0,0,1]])

    R = np.dot(np.dot(R_z,R_y),R_x) #旋转矩阵
    mid_matrix = np.hstack((R,cartesian_pos))
    final_matrix = np.vstack((mid_matrix,other_elements)) #4*4的旋转位姿矩阵
    return final_matrix

################矩阵转换#############
#4*4矩阵转为位姿1*6，XYZ格式
def rotation_to_position_angles_2(T):
    x,y,z = T[0,3],T[1,3],T[2,3]
    R = T[:3,:3]
    assert R.shape == (3,3)

    #按照XYZ的顺序进行矩阵变换
    pitch = math.atan2(-R[2,0],math.sqrt(R[0,0]**2+R[1,0]**2)) #贝塔，绕Y
    if (pitch > 1.57):
        roll = math.atan(R[0,1],R[1,1])
        pitch = 1.57
        yaw = 0
    elif(pitch < -1.57):
        roll = -math.atan(R[0,1],R[1,1])
        pitch = -1.57
        yaw = 0
    else:
        roll = math.atan2(R[2,1], R[2,2]) #伽马, 绕X
        pitch = math.atan2(-R[2,0],math.sqrt(R[0,0]**2+R[1,0]**2)) #贝塔, 绕Y
        yaw =math.atan2(R[1,0], R[0,0]) #阿尔法，绕Z

    position = np.array([x,y,z])
    angles = np.array([roll,pitch,yaw])
    p_combine_a = np.concatenate((angles,position))
    return p_combine_a

################更换夹爪控制逻辑#############
def Hand_ctl_mt(z1_pos,joint_hand,joint_speed):
    z1_pos[2] = atan2(z1_pos[4],z1_pos[3])+ z1_pos[1]  #姿态角要改变一下
    #print("z1_pos\n",z1_pos)
    D_j62t = np.array([0,math.radians(-22.5*joint_hand),0,0,0,0]) #（j6）j6到打开夹爪的1*6笛卡尔变换
    #print("D_j62t\n",D_j62t)
    T_j62t = calculate(D_j62t)                       #（j6）上述的4*4变换矩阵
    #print("T_b2t\n",T_j62t)
    T_j02j6 = calculate(z1_pos)                      #（j0）j0到j6的4*4变换矩阵
    #print("T_j02b\n",T_j02j6)
    T_j02t = np.dot(T_j02j6,T_j62t)                    #（j0）j0到打开夹爪的4*4变换矩阵
    #print("T_j02t\n",T_j02t)
    D_j02t = rotation_to_position_angles_2(T_j02t)   #（j0）上述的1*6笛卡尔坐标系
    D_j02t[4:6] = z1_pos[4:6]                          #坐标不变
    #print("D_j02t\n",D_j02t)

    return D_j02t

################判断能否到达#############
def Hand_ctl_act(z1_pos):
    global act_bg
    z = z1_pos[5]
    y = z1_pos[4]
    x = z1_pos[3]
    length = math.sqrt(x**2+y**2)
    #print(length)
    if(z<=0.11):
        #print("当前高度过低,无法到达")
        act_bg = 0
    elif(z>0.11 and z<=0.17):
        act_bg = 1 if (length>0.68-z and length<0.74) else 0
        if(act_bg == 0):
            rospy.logwarn("当前长度无法到达，为：%f",length)
    elif(z>0.17 and z<=0.21):
        act_bg = 1 if (length>0.85-2*z and length<0.74) else 0
        if(act_bg == 0):
            rospy.logwarn("当前长度无法到达，为：%f",length)
    elif(z>0.22 and z<=0.4):
        act_bg = 1 if (length>0.53 and length<0.74) else 0
        if(act_bg == 0):
            rospy.logwarn("当前长度无法到达，为：%f",length)
    elif(z>0.4):
            rospy.logwarn("当前高度有可能可以到达")
            act_bg = 0
    return act_bg

def Joint_ctl_ct(z1_pos,joint_hand,joint_speed,z1_pos_d,z1_pos_h):        #完成对位置坐标的改变，同时进行控制
    z1_pos_ch = np.copy(z1_pos)
    z1_pos_ch[3] = z1_pos[3] + z1_pos_d*z1_pos[3]/math.sqrt(z1_pos[3]**2 + z1_pos[4]**2)
    z1_pos_ch[4] = z1_pos[4] +  z1_pos_d*z1_pos[4]/math.sqrt(z1_pos[3]**2 + z1_pos[4]**2)
    z1_pos_ch[5] = z1_pos[5] + z1_pos_h
    z1_pos_ctrl = Hand_ctl_mt(z1_pos_ch, joint_hand, joint_speed)
    en_act = Hand_ctl_act(z1_pos_ctrl)
    if(en_act == 1):
        arm.MoveJ(z1_pos_ctrl, joint_hand, joint_speed)
        rospy.loginfo("Arrived New Position: %s", z1_pos_ctrl)
    else:
         rospy.logwarn("Cannot Arrived Position: %s",z1_pos_ctrl)
    return en_act

def handle_detection(msg):
    rospy.loginfo(f"Received detection for {msg.label}: (x={msg.x}, y={msg.y}, depth={msg.depth})")
    global effector_pos, effector_pos_ctl
    effector_pos = pixeltobase(msg.x, msg.y, msg.depth)
    # if effector_pos is not None: 
    #     if effector_pos[3] < 某个阈值 and effector_pos[3] > 某个阈值:
    #         effector_pos_ctl = effector_pos
    #     else:
    #         rospy.loginfo(f"深度不在规定阈值:")
    if effector_pos is not None: 
        effector_pos_ctl = effector_pos

def callback_robot_state(data):
    global robot_state
    robot_state = data.data
    #rospy.loginfo("Received robot_state: %d", robot_state)

rospy.init_node('arm_controller_node', anonymous=True)
robot_state_sub = rospy.Subscriber("robot_state_topic", Int32, callback_robot_state)
detection_sub = rospy.Subscriber('detection', ObjectDetection, handle_detection)

# photo_state_pub = rospy.Publisher('photo_state_topic', Int32, queue_size=10)
# def publish_photo_state(state):
#     photo_state_pub.publish(state)
#     #rospy.loginfo(f"Published photo state {state}")

rate = rospy.Rate(100)

arm =  unitree_arm_interface.ArmInterface(hasGripper=True)
armState = unitree_arm_interface.ArmFSMState
armModel = arm._ctrlComp.armModel

arm.loopOn()
effector_pos = None
effector_pos_ctl = None

state = 1
# arm.labelRun("forward")
# arm.backToStart()

while not rospy.is_shutdown():
    try:
        key = getKey()
        if key == 'q': # 按下 'q' 键中断程序
            rospy.logwarn("INTERRUPT")
            break
        if key:
            rospy.loginfo(f"PRESS q to interrupt..")

        if effector_pos_ctl is None: 
            rospy.logwarn("effector_pos_ctl is not set yet. Waiting for valid detection data.") 
            continue

        if robot_state==2:
 
            arm.labelRun("forward")

            #移动到目标位置前方
            act = Joint_ctl_ct(effector_pos, 0, 2.5 ,-0.19 ,-0.02)#增加是减少深度本来是-0.17
            if(act == 1):
                rospy.loginfo("1.Arrived the forward position..")
            else:
                rospy.logwarn("Cannot arrived the position. Stop moving..")

            #打开抓夹
            #act = Joint_ctl_ct(effector_pos, -1.0 ,2.0 ,-0.19 ,-0.02)#增加绝对值是减少深度本来是0.17
            act = Joint_ctl_ct(effector_pos, -1 ,2.0 ,-0.19 ,-0.02)#增加绝对值是减少深度本来是0.17
            if(act == 1):
                rospy.loginfo("Open the Claw..")
            else:
                rospy.logwarn("Cannot arrived the position. Stop moving..")

            #移动到目标位置
            #act = Joint_ctl_ct(effector_pos, -1.0 , 2.5 ,-0.07 ,-0.02)#增加是减少深度本来是0.07
            act = Joint_ctl_ct(effector_pos, -1 , 2.5 ,-0.07 ,-0.02)#增加是减少深度本来是0.07
            if(act == 1):
                rospy.loginfo("2.Arrived the target position..")
            else:
                rospy.logwarn("Cannot arrived the position. Stop moving..")
            rospy.sleep(1)

            #关闭抓夹
            act = Joint_ctl_ct(effector_pos, -0.6 ,2.0 ,-0.07 ,-0.02)
            if(act == 1):
                rospy.loginfo("Close the Claw..")
            else:
                rospy.logwarn("Cannot arrived the position. Stop moving..")

            #移动到目标位置上方
            act = Joint_ctl_ct(effector_pos, -0.6 , 2.5 ,-0.12 ,0.04)
            if(act == 1):
                rospy.loginfo("3.Arrived the upper position..")
            else:
                rospy.logwarn("Cannot arrived the position. Stop moving..")

            #收回机械臂
            rospy.loginfo("Withdraw the Claw..")
            effector_pos_ctl = np.array([1.57, 0, 0, 0.5, 0, 0.35])
            arm.MoveJ(effector_pos_ctl, -0.6, 2.5)

            rospy.loginfo("Move to Destination..")
            effector_pos_ctl = np.array([0, 0.8, 1.6, 0.015, 0.5, 0.364])
            arm.MoveJ(effector_pos_ctl, -0.6, 2.5)

            rospy.loginfo("Open the Claw..")
            arm.MoveJ(effector_pos_ctl, -1.0, 2.5)

            arm.backToStart()
            robot_state=0

            # #末端姿态矩阵（手到基座）每次重新计算
            # state = arm.lowstate.getQ()
            # start_pos = armModel.forwardKinematics(state, 5)
            # matrixBase2Hand = np.array(start_pos) 
            # rospy.loginfo(f"matrixBase2Hand_backtostart:%s",matrixBase2Hand)

    except Exception as e:
        rospy.logerr("Error handling state %d: %s", robot_state, str(e))
        arm.backToStart()


try:
    rospy.loginfo("Break..")
    arm.backToStart()
    arm.loopOff()
    rospy.loginfo("End of Movement..")

except Exception as e: 
    rospy.logerr(f"Error during shutdown: {e}")
