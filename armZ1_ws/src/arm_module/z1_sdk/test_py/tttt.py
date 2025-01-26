import keyboard
import sys
print(sys.executable)
sys.path.append("../lib")
import numpy as np
import unitree_arm_interface
import time
import math
import function as f
from math import atan2, asin, sqrt
# from Z1_model.setout_keda import *

#使用ESC可以退出程序
break_flag = False
def on_esc_press(event):
    global break_flag
    break_flag =  True
    print("ECS key pressed. Exiting program.")
    # exit()

# print("Press ctrl+\ to quit process.")
keyboard.on_press_key('esc', on_esc_press)
print('Press ESC to exit the program.')
# keyboard.wait('esc')

def Joint_ctl_ct(z1_pos,joint_hand,joint_speed,z1_pos_d,z1_pos_h):
    #完成对位置坐标的改变，同时进行控制
    z1_pos_ch = np.copy(z1_pos)     #copy才不会改变原本的格式类型
    z1_pos_ch[3] = z1_pos[3] + z1_pos_d*z1_pos[3]/math.sqrt(z1_pos[3]**2 + z1_pos[4]**2)
    z1_pos_ch[4] = z1_pos[4] - abs(z1_pos_d)*z1_pos[4]/math.sqrt(z1_pos[3]**2 + z1_pos[4]**2)
    z1_pos_ch[5] = z1_pos[5] + z1_pos_h
    #print("计算后的值是：",z1_pos_ch)
    z1_pos_ctrl = f.Hand_ctl_mt(z1_pos_ch, joint_hand, joint_speed)
    en_act = f.Hand_ctl_act(z1_pos_ctrl)
    if(en_act == 1):
        arm.MoveJ(z1_pos_ctrl, joint_hand, joint_speed)
        print("可以到达，位姿是：", z1_pos_ctrl)
    else:
        print("不可以到达，位姿是：",z1_pos_ctrl)
    return en_act


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


################矩阵转换#############
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





#输出精度
np.set_printoptions(precision=8, suppress=True)

#机械臂
arm =  unitree_arm_interface.ArmInterface(hasGripper=True)
armState = unitree_arm_interface.ArmFSMState
armModel = arm._ctrlComp.armModel

arm.loopOn()