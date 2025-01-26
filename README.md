# z1_ros
A Beverage Grasping System for Robotic Arms Based on the ROS Framework
硬件和软件配置
Ubuntu20.04
ROS-noetic版本:后续的安装都需要把melodic版本的安装修改成Noetic
双目相机D435i
宇树机械臂Z1
基础环境配置
按照Z1的官方仓库：https://github.com/unitreerobotics/z1_sdk的教程安装完前置环境
Z1-ROS包的安装（后续标定中需要Moveit!功能）
Z1支持ROS、Moveit!包的官方仓库：https://github.com/unitreerobotics/z1_ros
按照官方仓库安装：https://github.com/unitreerobotics/z1_ros/blob/noetic/doc/setup.md
注意：在安装Pinocchio library的时候，请不要按照官方的指令，官方的setup文档中安装的是当时（August 2023）Pinocchio是最新版本的，我们需要去Pinocchio library安装2.6.19Version（具体原因请查看：https://github.com/unitreerobotics/z1_ros/issues/11）
手眼标定
	将以下文档与链接中的https://blog.csdn.net/gyxx1998/article/details/122238052内容配合阅读。
机械臂与相机配合抓取任务有两种方式：眼在手上、眼在手外。眼在手上：相机固定于机械臂末端，随着机械臂的运动而运动，机械臂末端与相机的相对位置是不变的。眼在手外：相机坐标系固定在机械臂外。本项目由于需要比较高的灵活度（经常需要变动位置），采用的是眼在手上的形式。因此需要手眼标定，将机械臂与相机的转换矩阵标定出来，标定的结果直接影响了最后的精度。
	机械臂的基坐标系和相机的坐标系不同。相机读取的坐标是像素坐标系下的，通过相机本身的内参，转换到相机坐标系下，再通过机械臂与相机之间的固定旋转矩阵，将相机坐标系下的坐标转换到机械臂末端上，最后通过已知的机械臂末端的位姿，转换到机械臂（base）的坐标系下。
	接下来是求机械臂末端与相机之间的坐标转换过程（手眼标定）。
安装过程：
1、	安装aruco_ros：这是标定板
cd ~/ur_ws/src
git clone -b melodic-devel https://github.com/pal-robotics/aruco_ros.git
cd ..
catkin_make
2、	aruco二维码设置
从https://chev.me/arucogen/中下载aruco二维码并打印出来
Dictionary 一定要选 Original ArUco
Marker ID 和 Marker size 自选，在launch （roslaunch aruco_ros single.launch）文件中做相应的修改
打印时注意选择原始大小，否则要测量一下打印出来的真实大小
3、	安装vision_visp/visp_hand2eye_calibration：这是手眼标定的一个包
cd ~/ur_ws/src
sudo apt-get install ros-melodic-visp
git clone -b melodic-devel https://github.com/lagadic/vision_visp.git
cd ..
catkin_make --pkg visp_hand2eye_calibration
catkin_make
4、	安装easy_handeye：这是一款手眼标定工具
cd ~/ur_ws/src
git clone https://github.com/IFL-CAMP/easy_handeye
cd ..
catkin_make
启动标定过程：
1、	启动realsense
roslaunch realsense2_camera rs_camera.launch
2、	启动aruco
cd eye_ws/
source devel/setup.bash
roslaunch aruco_ros single.launch
3、	启动image_view节点显示图像
rosrun image_view image_view image:=/aruco_single/result
rostopic echo /aruco_single/pose
4、	启动机械臂  moveit
source /opt/ros/noetic/setup.bash
source ~/z1_ws/devel/setup.bash
roslaunch z1_bringup real_arm.launch rviz:=true
5、	启动easy_handeye节点
cd eye_ws/
source devel/setup.bash
roslaunch easy_handeye eye_in_hand_calibrate.launch
全部启动后，会出现3个窗口。如果没有启动成功请检查tf树是否连接正确：
1、	tf树合并
rosrun tf2_ros static_transform_publisher x y z qx qy qz /world /camera_link
2、	发布tf
cd eye_ws/
source devel/setup.bash
roslaunch easy_handeye publish.launch
rosrun tf tf_echo /link00 /camera_link
标定过程：
1、	首先打开一个终端，输入rqt，点击菜单栏的 Plugins -> Visulization -> Image View，选择 /aruco_tracker/result 话题。当识别出aruco码时，则可以进行下一步。
2、	在第三个屏幕中点击check starting pose，若检查成功，界面会出现： 0/17，ready to start
3、	在第三个窗口点击next pose -> plan -> execute，当点完 plan ，出现绿色框，则说明规划成功，然后可以点击 execute让机械臂执行动作
4、	然后在第二个窗口，点击take sample采样
5、	然后再次回到第三个窗口使机械臂执行规划动作。
当17个动作执行完成，回到第二个界面，点击compute，然后出现结果的姿态矩阵，然后可以点击save保存
6、	Calibration saved to /home/jianghao/.ros/easy_handeye/z1_rs_d435i_eye_on_hand.yaml
参考博客：
https://blog.csdn.net/gyxx1998/article/details/122238052
https://blog.csdn.net/Thinkin9/article/details/123743924
矩阵转换：
将上面标定出的手眼标定矩阵代入公式（在文件中是）：
import numpy as np
matrixHand2Camera = np.array([[ 0.56186877, -0.82718502, -0.00827197, 0.00665606],
                            [ 0.82722097,0.56180082, 0.00923615, -0.0420225],
                            [-0.00299281 , -0.01203225, 0.99992313, 0.02549419],
                [ 0,          0,          0,          1         ]]) # 手眼矩阵THC
matrixBase2Hand = np.array([[0.26195007,	-0.14356031, 0.95434407, 0.319418241631],
                            [-0.67221037, -0.73668364, 0.07369148,	-0.010993728332],
                            [0.69247049, -0.66082346, -0.28947706, 0.689715275419],
                [0,           0,          0,          1         ]]) # 末端姿态TBH
matrixCamera2Pixel = np.array([[922.78685321,   0,            657.06183115],
                             [  0,             923.92329954, 366.06665478],
                [  0,             0,              1         ]]) # 内参
matrixBase2Camera = np.dot(matrixBase2Hand,matrixHand2Camera)
matrixCamera2Base = np.linalg.inv(matrixBase2Camera)
zc = 0.495
u = 801
v = 452
# 直接变换
outputBase2=np.dot(np.linalg.inv(matrixCamera2Base[0:3,0:3]),zc*np.dot(np.linalg.inv(matrixCamera2Pixel),np.array([u,v,1]).reshape(3,1))-matrixCamera2Base[:3,3].reshape(3,1))
print("直接变换",outputBase2)
参考链接：https://blog.csdn.net/road_of_god/article/details/126891153

模块流程图（最新版本中的代码没有添加酒类饮品的检测过程）
 
 
仿真与实机调试本项目：
仿真：
#如果有修改代码
catkin_make
source devel/setup.bash
#进入工作空间
cd armZ1_ws
#运行gazebo仿真软件
roslaunch unitree_gazebo z1.launch
#进入编译完成的文件中
cd armZ1_ws/src/arm_module/z1_controller/build
#运行可执行文件
./sim_ctrl
#视觉节点
rosrun vision_module run_yolo.py
#roslaunch vision_module run_yolo.launch
#语音节点
roslaunch voice_module voice_detect_for.launch
#机械臂
roslaunch arm_module arm_controller.launch
实机运行：
#如果有修改代码
catkin_make
source devel/setup.bash
#进入工作空间
cd armZ1_ws/src/arm_module/z1_controller/build
./z1_ctrl
#视觉节点
rosrun vision_module run_yolo.py
#roslaunch vision_module run_yolo.launch
#语音节点
roslaunch voice_module voice_detect_for.launch
#机械臂
roslaunch arm_module arm_controller.launch
如果使用gui界面
roscore
rosrun gui_module arm_control_gui
主要程序
armZ1_ws\src\voice_module\scripts\ voice_detect_for
语音识别模块：该模块将用户语音转化为文本，关键词通过ROS传递至视觉模块，驱动后续商品抓取任务。
armZ1_ws\src\vision_module\scripts\run_yolo.py
视觉定位模块：通过Realsense深度相机和YOLOv5算法，识别商品并获取坐标与类别标签。结合深度信息，ROS传输至机械臂控制模块，确保精准定位和抓取，即使在复杂环境下也能稳定工作。
人脸识别与年龄估算模块：在识别为酒类饮品时，系统采集用户面部图像并通过深度学习进行年龄估算。若用户年龄不足18岁，系统拒绝操作并语音提示；符合条件则继续下一步操作。
armZ1_ws\src\arm_module\scripts\main.py
机械臂控制模块：本模块通过ROS控制宇树Z1机械臂完成路径规划和抓取任务。系统先进行手眼标定，实现深度相机与机械臂坐标系的精确对接。随后，通过蒙特卡洛方法规划可行工作区间，并利用逆运动学生成抓取轨迹。抓取成功后，机械臂将商品放置到指定位置，确保过程平滑、安全且高效。
