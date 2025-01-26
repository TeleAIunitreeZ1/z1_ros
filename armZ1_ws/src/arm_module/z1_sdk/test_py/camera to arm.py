import sys
sys.path.append("../lib")
import unitree_arm_interface
import time
import numpy as np
import pandas as pd
from rotation_to_angles import rotation_to_position_angles
from z1_sdk.examples_py.function import find_label_array


print("Press ctrl+\ to quit process.")

np.set_printoptions(precision=3, suppress=True)
arm =  unitree_arm_interface.ArmInterface(hasGripper=True)
armState = unitree_arm_interface.ArmFSMState

arm.loopOn()
arm.backToStart()
armModel = arm._ctrlComp.armModel

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
print("Press ctrl+\ to quit process.")

start_pos = [0,0,0,-0.0932,0.483,0.223]


init_angle = calculate(start_pos)
print("angle")
print(init_angle)
hasIK, q_forward = armModel.inverseKinematics(init_angle, np.zeros(6), True)
print("q_forward")
print(q_forward)
lastPos = arm.lowstate.getQ()
targetPos = q_forward
print("targetpos type")
print(type(targetPos[0]))
print("last type")
print(type(lastPos[0]))
print(find_label_array)

for i in range(0,2500):
    arm.q = lastPos*(1-i/2500) + targetPos*(i/2500)# set position
    arm.qd = (targetPos-lastPos)/(2500*0.002) # set velocity
    arm.tau = armModel.inverseDynamics(arm.q, arm.qd, np.zeros(6), np.zeros(6)) # set torque
    arm.gripperQ = 0
    arm.setArmCmd(arm.q, arm.qd, arm.tau)
    arm.setGripperCmd(arm.gripperQ,arm.gripperQd,arm.gripperTau)
    arm.sendRecv()# udp connection
    # print(arm.lowstate.getQ())
    time.sleep(arm._ctrlComp.dt)

arm.loopOn()



arm.backToStart()
arm.loopOff()