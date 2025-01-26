import sys
sys.path.append("../lib")
import csv
import numpy as np
import unitree_arm_interface
import math
# import pandas as pd
import time
from math import atan2, asin, sqrt

# -*- coding: utf-8 -*-
# encoding: utf-8print
import pyrealsense2 as rs
import cv2

arm =  unitree_arm_interface.ArmInterface(hasGripper=True)
armState = unitree_arm_interface.ArmFSMState
armModel = arm._ctrlComp.armModel

################setArm函数################
def setArmposition(arm,label_array,duration,gripper):
    armModel = arm._ctrlComp.armModel
    lastPos = arm.lowstate.getQ()
    targetPos = np.array(label_array,dtype=np.float64)
    # print('arm.gripperQ:',arm.gripperQ)

    #0——关  -1——开
    for i in range(0, duration):
        arm.q = lastPos*(1-i/duration) + targetPos*(i/duration)# set position
        arm.qd = (targetPos-lastPos)/(duration*0.002) # set velocity
        arm.tau = armModel.inverseDynamics(arm.q, arm.qd, np.zeros(6), np.zeros(6)) # set torque
        if gripper == 'open1':
            arm.gripperQ = -1*(i/(duration))
        if gripper == 'open2':
            arm.gripperQ = -0.86-0.14*(i/(duration))
        if gripper == 'on':
            arm.gripperQ = -1
        if gripper == 'close':
            arm.gripperQ = -1*(1-0.14*i/(duration))
        if gripper == 'off':
            arm.gripperQ = -0.86

        arm.setArmCmd(arm.q, arm.qd, arm.tau)
        arm.setGripperCmd(arm.gripperQ, arm.gripperQd, arm.gripperTau)
        arm.sendRecv()# udp connection
        # print(arm.lowstate.getQ())
        time.sleep(arm._ctrlComp.dt)

################csv中找label函数################
def find_label_array(file_path, label_name):
    # 定义一个空列表来存储转换后的浮点数
    label_array = []
    try:
        with open(file_path, 'r') as csvfile:
            # 创建CSV阅读器
            reader = csv.reader(csvfile)
            
            # 遍历CSV文件中的每一行
            for row in reader:
                # 检查是否是指定标签行
                if row and row[0] == label_name:
                    # 遍历行中的值，跳过第一个元素（标签）
                    for value in row[1:]:
                        # 检查值是否非空且非仅空格
                        if value.strip():
                            try:
                                # 尝试将非空字符串转换为浮点数并添加到列表
                                label_array.append(float(value.strip()))
                            except ValueError as e:
                                # 如果转换失败，打印错误信息并中断循环
                                print(f"Error converting '{value}' to float: {e}")
                                break
                    # 如果成功找到并处理了标签行，可以提前退出循环
                    if label_array:
                        break
    except FileNotFoundError:
        print(f"The file {file_path} does not exist.")
    except Exception as e:
        print(f"An error occurred: {e}")

    return label_array


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


# ##################抓取定位#######################
# def pixeltobase():
    
#     #手眼矩阵
#     matrixHand2Camera = np.array([[ 0.01016862, 0.00842434, 0.99991281, 0.10144304],
#                                 [-0.99859197, 0.05215064, 0.00971582, 0.03207163],
#                                 [-0.05206424, -0.9986037, 0.00894278, -0.05421758],
#                                 [ 0, 0, 0, 1]])   

#     #末端姿态矩阵（手到基座）每次重新计算
#     state = arm.lowstate.getQ()
#     start_pos = armModel.forwardKinematics(state, 5)
#     matrixBase2Hand = np.array(start_pos) 
#     #print("末端姿态矩阵：",matrixBase2Hand)

#     #内参矩阵（color）
#     matrixCamera2Pixel = np.array([[609.5366821289062, 0, 324.4776916503906],
#     [0, 608.2521362304688, 247.832275390625],
#     [0, 0, 1]]) 

#     matrixBase2Camera = np.dot(matrixBase2Hand,matrixHand2Camera)
#     matrixCamera2Base = np.linalg.inv(matrixBase2Camera)


#     #提取饮品像素点坐标
#     model_path = '/home/jianghao/Z1/keywords.txt'    #语音
#     detect_path = '/home/jianghao/Z1/Yolov5/detect.npy'    #视觉

#     products = np.load(detect_path, allow_pickle=True).tolist()
#     print("所有饮品像素坐标：",products)

#     with open(model_path, 'r') as file:
#         key = file.read()
#         print("目标饮品：",key)
#     file.close()

#     u = 0
#     v = 0
#     d = 0
#     u, v, d = products[key.strip()]
#     d=d*0.001
#     print("目标饮品坐标：",f'u = {u}, v = {v}, depth = {d}')

#     #获取饮品实际中心点坐标
#     base_pos = np.dot(np.linalg.inv(matrixCamera2Base[0:3,0:3]),d*np.dot(np.linalg.inv(matrixCamera2Pixel),np.array([u,v,1]).reshape(3,1))-matrixCamera2Base[:3,3].reshape(3,1))
#     #a=d*np.dot(np.linalg.inv(matrixCamera2Pixel),np.array([u,v,1]).reshape(3,1))
#     #print("a:",a)
#     effec_pos = np.zeros((1, 6))
#     effec_pos[0, 3] = base_pos[0, 0]
#     effec_pos[0, 4] = base_pos[1, 0]
#     effec_pos[0, 5] = base_pos[2, 0]

#     #设立初始位置
#     effector_pos = effec_pos.flatten()
#     effector_pos[0] = effector_pos[0] + 1.57    #夹爪姿态
#     effector_pos[3] = effector_pos[3] + 0.03    #补偿坐标到物体的实际坐标
#     print("末端实际坐标为：", effector_pos)

#     return effector_pos



################矩阵转换（by shh）#############
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
    # print(p_combine_a)
    return p_combine_a


# file_path = '/home/jianghao/Z1/test_ws/src/z1_controller/config/savedArmStates.csv'
# label_array_grisp = []
# label_array_grisp = f.find_label_array(file_path,'forward')
# fk_grisp= armModel.forwardKinematics(label_array_grisp, 6)
# combine_grisp = f.rotation_to_position_angles(fk_grisp)
# print("当前坐标",combine_grisp)


################更换夹爪控制逻辑（by shh）#############
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
            print("当前长度无法到达，为：\n",length)
    elif(z>0.17 and z<=0.21):
        act_bg = 1 if (length>0.85-2*z and length<0.74) else 0
        if(act_bg == 0):
            print("当前长度无法到达，为：\n",length)
    elif(z>0.22 and z<=0.4):
        act_bg = 1 if (length>0.53 and length<0.74) else 0
        if(act_bg == 0):
            print("当前长度无法到达，为：\n",length)
    elif(z>0.4):
            print("当前高度过高,有可能可以到达，但我没写")
            act_bg = 0
    return act_bg


################矩阵转换（by sua）#############
def extract_transformation(T):
    x,y,z = T[0,3],T[1,3],T[2,3]
    R = T[:3,:3]
    return np.array([x,y,z]),R

def rotation_matrix_to_euler_angles(R):
    assert R.shape == (3,3)
    pitch = asin(-R[1,2])
    if np.isclose(pitch, np.pi/2) or np.isclose(pitch, -np.pi/2):
        roll = 0
        yaw = atan2(R[0,1], R[0,0])
    else:
        roll = atan2(R[2,1], R[2,2])
        yaw = atan2(R[1,0], R[0,0])
    return roll,pitch,yaw

def rotation_to_position_angles(T):
    x,y,z = T[0,3],T[1,3],T[2,3]
    R = T[:3,:3]
    assert R.shape == (3,3)
    pitch = asin(-R[1,2])
    if np.isclose(pitch, np.pi/2) or np.isclose(pitch, -np.pi/2):
        roll = 0
        yaw = atan2(R[0,1], R[0,0])
    else:
        roll = atan2(R[2,1], R[2,2])
        yaw = atan2(R[1,0], R[0,0])

    position = np.array([x,y,z])
    angles = np.array([roll,pitch,yaw])

    p_combine_a = np.concatenate((angles,position))

    # print(p_combine_a)
    return p_combine_a


##################抓取定位#######################
def pixeltobase_natie():
    
    #手眼矩阵
    matrixHand2Camera = np.array([[ 0.01016862, 0.00842434, 0.99991281, 0.10144304],
                                [-0.99859197, 0.05215064, 0.00971582, 0.03207163],
                                [-0.05206424, -0.9986037, 0.00894278, -0.05421758],
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


    #提取饮品像素点坐标
    key = "拿铁咖啡"    #语音
    detect_path = '/home/jianghao/Z1/Yolov5/detect.npy'    #视觉

    products = np.load(detect_path, allow_pickle=True).tolist()
    print("所有饮品像素坐标：",products)

    u = 0
    v = 0
    d = 0
    u, v, d = products[key.strip()]
    d=d*0.001
    print("目标饮品坐标：",f'u = {u}, v = {v}, depth = {d}')

    #获取饮品实际中心点坐标
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
    print("末端实际坐标为：", effector_pos)

    return effector_pos
