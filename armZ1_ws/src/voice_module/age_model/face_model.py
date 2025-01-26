import cv2
import dlib
import pyrealsense2 as rs
import numpy as np

from age_model.Facenet.models.mtcnn import MTCNN
from age_model.AgeNet.models import Model
import torch
from torchvision import transforms as T
from PIL import Image
import os

#计数文件
def load_save_count(file_path):
    try:
        with open(file_path, 'r') as f:
            return int(f.read())
    except FileNotFoundError:
        return 0

def save_save_count(file_path, count):
    with open(file_path, 'w') as f:
        f.write(str(count))

#判断是否成年
def is_adult(age):
    return age >= 18


#年龄预测模型
class AgeEstimator():
    def __init__(self, weights_path, device='cpu'):
        self.device = device
        self.facenet_model = MTCNN(device=self.device)
        self.model = Model().to(self.device)
        self.model.eval()
        self.model.load_state_dict(torch.load(weights_path, map_location=torch.device('cpu')))
        self.transform = T.Compose([
            T.Resize((64, 64)),
            T.ToTensor(),
            T.Normalize(mean=[0.5, 0.5, 0.5], std=[0.5, 0.5, 0.5])
        ])
        
    def predict(self, img_path):
        image = Image.open(img_path)
        transformed_image = self.transform(image)
        transformed_image = transformed_image.unsqueeze(0).to(self.device)
        with torch.no_grad():
            genders, ages = self.model(transformed_image)
        return genders, ages


def face_distance(face1, face2):
    return np.linalg.norm(np.array([face1.left(), face1.top(), face1.right(), face1.bottom()]) - 
                          np.array([face2.left(), face2.top(), face2.right(), face2.bottom()]))


def facedetect():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    
    detector = dlib.get_frontal_face_detector()
    color_green = (0, 255, 0)
    line_width = 3
    save_count = load_save_count("save_count.txt")  #加载保存的序号
    weights_path = "/home/jianghao/armZ1_ws/src/voice_module/age_model/weights/weights.pt" # 模型权重路径       #####需要重新修改路径
    age_estimator = AgeEstimator(weights_path, device='cpu') # 初始化模型
    
    count=0

    try:
        while True:
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            rgb_image = np.asanyarray(color_frame.get_data())
            cv2.imshow('camera', rgb_image)   #显示图像

            dets = detector(rgb_image)
            for det in dets:
                cv2.rectangle(rgb_image, (det.left(), det.top()), (det.right(), det.bottom()), color_green, line_width)
     
            # 保存视频截图
            if len(dets) > 0:
                count += 1

                if count == 40:
                    file_path = f"/home/jianghao/armZ1_ws/src/voice_module/age_model/photo_age/frame_{save_count}.jpg"
                    cv2.imwrite(file_path, rgb_image)
                    save_count += 1
                    save_save_count("save_count.txt", save_count) #保存序号
                
                    is_adult_result = False  # 默认未成年

                    # 使用模型进行预测
                    genders, ages = age_estimator.predict(file_path)
                    for gender, age in zip(genders, ages):
                        is_adult_result = is_adult(age.item())
                        if is_adult_result:
                            age_label = "Adult"
                        else:
                            age_label = "Minor"
                        print(f"Age: {age.item()}, {age_label}")
                        return is_adult_result
                    
                    count = 0     
                

            key = cv2.waitKey(1)
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break
    finally:
        pipeline.stop()