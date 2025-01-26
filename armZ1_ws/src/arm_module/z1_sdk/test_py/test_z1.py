import sys
sys.path.append("../lib")
import unitree_arm_interface
import time
import numpy as np

print("Press ctrl+\ to quit process.")

np.set_printoptions(precision=3, suppress=True)
arm =  unitree_arm_interface.ArmInterface(hasGripper=True)
armState = unitree_arm_interface.ArmFSMState

cup_pos = [0.4, 0, 0.4]

arm.loopOn()
arm.backToStart()
arm.labelRun("forward")
cartesian_speed = 0.5
# arm.MoveL(np.array([0,0,0,0.3,0,0.45]), gripper_pos, cartesian_speed)
gripper_pos = 0 # 0 or -1
jnt_speed = 0.5
for i in range(0,500): #gripper close
    arm.jointCtrlCmd(np.array([0,0,0,0,0,0,1]), jnt_speed)
    time.sleep(arm._ctrlComp.dt)
for i in range(0,1700): #up
    arm.jointCtrlCmd(np.array([0,0,0,0,0,-1,0]), jnt_speed)
    time.sleep(arm._ctrlComp.dt)
current_angle=arm.lowstate.getQ()
print("angle:")
print(current_angle)
arm.labelSave("test")#-0.000008, 1.496696, -0.990038, -0.529235, 0.000003, -1.732675, 
print("yes")
# for i in range(0,1500): #turn right 
#     arm.jointCtrlCmd(np.array([-1,0,0,0,0,0,0]), jnt_speed)
#     time.sleep(arm._ctrlComp.dt)
for i in range(0,250):  #turn right
    arm.jointCtrlCmd(np.array([-1,0,0,0,0,0,0]), jnt_speed)
    time.sleep(arm._ctrlComp.dt)
for i in range(0,1000):  #gripper open
    arm.jointCtrlCmd(np.array([0,0,0,0,0,0,-1]), jnt_speed)
    time.sleep(arm._ctrlComp.dt)
for i in range(0,1100):  #gripper down
    arm.jointCtrlCmd(np.array([0,1,-1,0,0,0,0]), jnt_speed)
    time.sleep(arm._ctrlComp.dt)
for i in range(0,1000):  #gripper close
    arm.jointCtrlCmd(np.array([0,0,0,0,0,0,1]), jnt_speed)
    time.sleep(arm._ctrlComp.dt)
# for i in range(0, 750):
#     arm.jointCtrlCmd(np.array([-1,0,1,-1,1,0,0]), jnt_speed)
#     time.sleep(arm._ctrlComp.dt)
for i in range(0,480):  #turn right
    arm.jointCtrlCmd(np.array([-1,0,0,0,0,0,0]), jnt_speed)
    time.sleep(arm._ctrlComp.dt)
for i in range(0,100):
    arm.jointCtrlCmd(np.array([0,0,0,1,0,0,0]), jnt_speed)
    time.sleep(arm._ctrlComp.dt)
for i in range(0,500):
    arm.jointCtrlCmd(np.array([0,-1,1,0,0,0,0]), jnt_speed)
    time.sleep(arm._ctrlComp.dt)
for i in range(0,500):
    arm.jointCtrlCmd(np.array([0,0,1,-1,0,0,0]), jnt_speed)
    time.sleep(arm._ctrlComp.dt)
for i in range(0,250):
    arm.jointCtrlCmd(np.array([0,0,0,-1,0,0,0]), jnt_speed)
    time.sleep(arm._ctrlComp.dt)
for i in range(0,150):
    arm.jointCtrlCmd(np.array([0,1,0,0,0,0,0]), jnt_speed)
    time.sleep(arm._ctrlComp.dt)
time.sleep(10)
for i in range(0,1000):  #gripper open
    arm.jointCtrlCmd(np.array([0,0,0,0,0,0,-1]), jnt_speed)
    time.sleep(arm._ctrlComp.dt)
time.sleep(3)
arm.backToStart()
arm.loopOff()

