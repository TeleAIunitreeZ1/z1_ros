# -*- coding: utf-8 -*-
# encoding: utf-8print
import pyrealsense2 as rs
import cv2


#根据目标饮品中心像素坐标检测深度
pc = rs.pointcloud()
points = rs.points()
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)

pipe_profile = pipeline.start(config)

depth_sensor = pipe_profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: ", depth_scale) 

align_to = rs.stream.color
align = rs.align(align_to)

u = 362.0
v = 257.0

depth_pixel = [u, v] 
num_samples = 30 
depth_sum = 0 
valid_samples = 0

for i in range(num_samples):
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    aligned_depth_frame = aligned_frames.get_depth_frame()

    x = int(depth_pixel[0])
    y = int(depth_pixel[1])
    dis = aligned_depth_frame.get_distance(x, y) # 深度单位是m

    # 检查深度值是否有效
    if dis > 0:
        depth_sum += dis
        valid_samples += 1

    print(f'第{i + 1}次深度采样 (m): {dis}')
    cv2.waitKey(100)

# 计算平均深度
if valid_samples > 0:
    zc = depth_sum / valid_samples
    print(f'深度平均值 (m): {zc}')
else:
    print('没有获取到有效的深度数据')

pipeline.stop()
