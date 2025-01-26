import sys
sys.path.append("../lib")
import unitree_arm_interface
import time
import numpy as np
import math
import function as f
print("Press ctrl+\ to quit process.")

np.set_printoptions(precision=3, suppress=True)
arm =  unitree_arm_interface.ArmInterface(hasGripper=True)
armState = unitree_arm_interface.ArmFSMState
armModel = arm._ctrlComp.armModel
arm.loopOn()
q_f = arm.lowstate.getQ()
T_forward0 = armModel.forwardKinematics(q_f, 0)
T_forward1 = armModel.forwardKinematics(q_f, 1)
T_forward2 = armModel.forwardKinematics(q_f, 2)
T_forward3 = armModel.forwardKinematics(q_f, 3)
T_forward4 = armModel.forwardKinematics(q_f, 4)
T_forward5 = armModel.forwardKinematics(q_f, 5)
T_forward6 = armModel.forwardKinematics(q_f, 6)
print(T_forward0)
print(T_forward1)
print(T_forward2)
print(T_forward3)
print(T_forward4)
print(T_forward5)
print(T_forward6)

wanted_pos = np.array([0.25,0.25,0,0.5,0.0,0.2])
pitch_angle = wanted_pos[0]
yaw_angle = wanted_pos[1]
roll_angle = wanted_pos[2]
x_pos = wanted_pos[3]
y_pos = wanted_pos[4]
z_pos = wanted_pos[5]
cartesian_pos = np.array([[x_pos],[y_pos],[z_pos]])
other_elements = np.array([0,0,0,1])
R = np.zeros((3,3))
R_x = np.array([[1,0,0],[0,np.cos(pitch_angle),-1*np.sin(pitch_angle)],[0,np.sin(pitch_angle),np.cos(pitch_angle)]])

R_y = np.array([[np.cos(yaw_angle),0,np.sin(yaw_angle)],[0,1,0],[-1*np.sin(yaw_angle),0,np.cos(yaw_angle)]])

R_z = np.array([[np.cos(roll_angle),-1*np.sin(roll_angle),0],[np.sin(roll_angle),np.cos(roll_angle),0],[0,0,1]])

R = np.dot(np.dot(R_z,R_y),R_x)

mid_matrix = np.hstack((R,cartesian_pos))

final_matrix = np.vstack((mid_matrix,other_elements))

hasIK, q_forward = armModel.inverseKinematics(final_matrix, np.zeros(6), True)
print(q_forward)
arm.backToStart()
arm.loopOff()

#####################关节角控制移动################
np.set_printoptions(precision=3, suppress=True)
arm = unitree_arm_interface.ArmInterface(hasGripper=True)
armModel = arm._ctrlComp.armModel
arm.setFsmLowcmd()

# arm.backToStart()

duration = 2500
file_path = '/home/jianghao/Z1/test_ws/src/z1_controller/config/savedArmStates.csv'
label_array_grisp = []
# label_array_grisp = [1.316,2.26,-1.124,-1.136,-1.316,0]
label_array_grisp = f.find_label_array(file_path,'beer_test1')#左上角

# 打印结果
print("label_array_grisp",label_array_grisp)
#移动到抓取的位置的上方,打开抓夹
# f.setArmposition(arm,label_array_grisp,2500,'open1')
# 经过前向运动学进行求解
fk_grisp= armModel.forwardKinematics(label_array_grisp, 6)
#将前向运动学算出的4*4矩阵进行提取和计算，得到angles和position
combine_grisp = f.rotation_to_position_angles(fk_grisp)

arm.labelSave("test")

current_angle=arm.lowstate.getQ()
print("angle:")
print(current_angle)