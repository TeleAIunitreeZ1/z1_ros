import sys
sys.path.append("../lib")
import unitree_arm_interface
import numpy as np
import math

np.set_printoptions(precision=3, suppress=True)
arm =  unitree_arm_interface.ArmInterface(hasGripper = True)
armModel = arm._ctrlComp.armModel
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

print('--------------------------FK & IK------------------------')
q_FORWARD = np.array([1.6, 2.0, -2.2, 1.0, 0, 0])
q_near_forward = np.array([1.665, 2.111, -2.243, 1.11, 0, 0])
# 1. FK
T_forward = armModel.forwardKinematics(q_FORWARD, 6)
print(T_forward)
effector = rotation_to_position_angles_2(T_forward)
print("effector",effector)
