#!/usr/bin/env python3
# YOLOv5 🚀 by Ultralytics, GPL-3.0 license
"""
Run inference on images, videos, directories, streams, etc.

Usage - sources:
    $ python path/to/detect.py --weights yolov5s.pt --source 0              # webcam
                                                             img.jpg        # image
                                                             vid.mp4        # video
                                                             path/          # directory
                                                             path/*.jpg     # glob
                                                             'https://youtu.be/Zgi9g1ksQHc'  # YouTube
                                                             'rtsp://example.com/media.mp4'  # RTSP, RTMP, HTTP stream

Usage - formats:
    $ python path/to/detect.py --weights yolov5s.pt                 # PyTorch
                                         yolov5s.torchscript        # TorchScript
                                         yolov5s.onnx               # ONNX Runtime or OpenCV DNN with --dnn
                                         yolov5s.xml                # OpenVINO
                                         yolov5s.engine             # TensorRT
                                         yolov5s.mlmodel            # CoreML (macOS-only)
                                         yolov5s_saved_model        # TensorFlow SavedModel
                                         yolov5s.pb                 # TensorFlow GraphDef
                                         yolov5s.tflite             # TensorFlow Lite
                                         yolov5s_edgetpu.tflite     # TensorFlow Edge TPU
"""
import argparse
import os
import sys
from pathlib import Path
import torch
import torch.backends.cudnn as cudnn
import cv2 as cv
import pyrealsense2 as rs
import numpy as np
from glob import glob
import rospy
from vision_module.msg import ObjectDetection
from std_msgs.msg import String,Int32
import threading
import queue
rospy.init_node('vision_module_node', anonymous=True)
pub = rospy.Publisher('detection', ObjectDetection, queue_size=10)


FILE = Path(__file__).resolve()
ROOT = FILE.parents[1]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

from models.common import DetectMultiBackend
from utils.dataloaders import IMG_FORMATS, VID_FORMATS, LoadImages, LoadStreams
from utils.general import (LOGGER, check_file, check_img_size, check_imshow, check_requirements, colorstr, cv2,
                           increment_path, non_max_suppression, print_args, scale_coords, strip_optimizer, xyxy2xywh)
from utils.plots import Annotator, colors, save_one_box
from utils.torch_utils import select_device, time_sync

task_queue = queue.Queue()

@torch.no_grad()
def run(

        weights=ROOT / 'yolov5s.pt',  # model.pt path(s)
        source=ROOT / 'data/images',  # file/dir/URL/glob, 0 for webcam
        data=ROOT / 'data/coco128.yaml',  # dataset.yaml path
        imgsz=(640, 640),  # inference size (height, width)
        conf_thres=0.25,  # confidence threshold
        iou_thres=0.45,  # NMS IOU threshold
        max_det=1000,  # maximum detections per image
        device='0',  # cuda device, i.e. 0 or 0,1,2,3 or cpu
        view_img=False,  # show results
        save_txt=False,  # save results to *.txt
        save_conf=False,  # save confidences in --save-txt labels
        save_crop=True,  # save cropped prediction boxes
        nosave=False,  # do not save images/videos
        classes=None,  # filter by class: --class 0, or --class 0 2 3
        agnostic_nms=False,  # class-agnostic NMS
        augment=False,  # augmented inference
        visualize=False,  # visualize features
        update=False,  # update all models
        project=ROOT / 'runs/detect',  # save results to project/name
        name='exp',  # save results to project/name
        exist_ok=False,  # existing project/name ok, do not increment
        line_thickness=3,  # bounding box thickness (pixels)
        hide_labels=False,  # hide labels
        hide_conf=False,  # hide confidences
        half=False,  # use FP16 half-precision inference
        dnn=False,  # use OpenCV DNN for ONNX inference
):

    pipeline = rs.pipeline()
    config1 = rs.config()
    config1.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config1.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipe_profile = pipeline.start(config1)

    depth_sensor = pipe_profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    #print("Depth Scale is: ", depth_scale)     

    align_to = rs.stream.color
    align = rs.align(align_to)

    save_count=0

    try:
        while not rospy.is_shutdown() and save_count < 69:
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            aligned_depth_frame = aligned_frames.get_depth_frame()
            aligned_color_frame = aligned_frames.get_color_frame()
            if not aligned_depth_frame or not aligned_color_frame:
                continue
            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            rgb_image = np.asanyarray(aligned_color_frame.get_data())
            save_count += 1
                
            # 保存视频截图
            if save_count > 59:
                if save_count == 60:
                    img_path = f"{source}/frame_{save_count}.jpg"
                    cv.imwrite(img_path, rgb_image)
                
                depth_path = f"{source}/framedepth_{save_count}.npy"
                # cv.imwrite(depth_path, depth_image)
                np.save(depth_path, depth_image)

    finally:
        pipeline.stop()  
    
    source = str(source)
    save_img = not nosave and not source.endswith('.txt')  # save inference images
    is_file = Path(source).suffix[1:] in (IMG_FORMATS + VID_FORMATS)
    is_url = source.lower().startswith(('rtsp://', 'rtmp://', 'http://', 'https://'))
    webcam = source.isnumeric() or source.endswith('.txt') or (is_url and not is_file)
    if is_url and is_file:
        source = check_file(source)  # download

    # Directories
    save_dir = increment_path(Path(project) / name, exist_ok=exist_ok)  # increment run
    (save_dir / 'labels' if save_txt else save_dir).mkdir(parents=True, exist_ok=True)  # make dir

    # Load model
    device = select_device(device)
    model = DetectMultiBackend(weights, device=device, dnn=dnn, data=data, fp16=half)
    stride, names, pt = model.stride, model.names, model.pt
    imgsz = check_img_size(imgsz, s=stride)  # check image size

    # Dataloader
    if webcam:
        view_img = check_imshow()
        cudnn.benchmark = True  # set True to speed up constant image size inference
        dataset = LoadStreams(source, img_size=imgsz, stride=stride, auto=pt)
        bs = len(dataset)  # batch_size
    else:
        dataset = LoadImages(source, img_size=imgsz, stride=stride, auto=pt)
        bs = 1  # batch_size
    vid_path, vid_writer = [None] * bs, [None] * bs

    # Run inference
    model.warmup(imgsz=(1 if pt else bs, 3, *imgsz))  # warmup
    dt, seen = [0.0, 0.0, 0.0], 0
    for path, im, im0s, vid_cap, s in dataset:
        t1 = time_sync()
        im = torch.from_numpy(im).to(device)
        im = im.half() if model.fp16 else im.float()  # uint8 to fp16/32
        im /= 255  # 0 - 255 to 0.0 - 1.0
        if len(im.shape) == 3:
            im = im[None]  # expand for batch dim
        t2 = time_sync()
        dt[0] += t2 - t1

        # Inference
        visualize = increment_path(save_dir / Path(path).stem, mkdir=True) if visualize else False
        pred = model(im, augment=augment, visualize=visualize)
        t3 = time_sync()
        dt[1] += t3 - t2

        # NMSnames
        pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)
        dt[2] += time_sync() - t3

        # Second-stage classifier (optional)
        # pred = utils.general.apply_classifier(pred, classifier_model, im, im0s)

        # Process predictions
        for i, det in enumerate(pred):  # per image
            seen += 1
            if webcam:  # batch_size >= 1
                p, im0, frame = path[i], im0s[i].copy(), dataset.c, im, im0s, vid_cap, s
            else:
                p, im0, frame = path, im0s.copy(), getattr(dataset, 'frame', 0)

            p = Path(p)  # to Path
            save_path = str(save_dir / p.name)  # im.jpg
            txt_path = str(save_dir / 'labels' / p.stem) + ('' if dataset.mode == 'image' else f'_{frame}')  # im.txt
            s += '%gx%g ' % im.shape[2:]  # print string
            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
            imc = im0.copy() if save_crop else im0  # for save_crop
            annotator = Annotator(im0, line_width=line_thickness, example="中文")

            depth_image = 0
            depth_files = glob(f'{source}/*.npy')
            for file in depth_files:
                data = np.load(file, allow_pickle=True)
                depth_image += data
            depth_image = depth_image / len(depth_files)

            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(im.shape[2:], det[:, :4], im0.shape).round()

                # Print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                    s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string

                # Write results
                write_dict = {}
                for *xyxy, conf, cls in reversed(det):
                    c = int(cls)  # integer class
                    dict_name = {'0':"椰子真果粒", '1':"草莓真果粒", '2':"黄桃真果粒", '3':"芦荟真果粒",
                                '4':"百岁山", '5':"巴马铂泉", '6':"可口可乐", '7':"果汁", '8':"拿铁咖啡", '9':"啤酒", 
                                '10':"鸡尾酒", '11':"娃哈哈",'12':"农夫山泉",'13':"怡宝",'14':"依云"}
                    p_name = dict_name[names[c]]
                    # xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                    xywh = (torch.tensor(xyxy).view(1, 4)).view(-1).tolist()  # normalized xywh
                    
                    # line = (p_name, *xywh, conf) if save_conf else (p_name, *xywh)  # label format
                    line = p_name
                    # for xy in xywh:
                    #     line = line + ' ' + str(xy)
                    xc, yc = (xywh[0] + xywh[2]) // 2, (xywh[1] + xywh[3]) // 2
                    depth = depth_image[int(yc)][int(xc)]
                    write_dict[p_name] = (xc, yc, depth)

                    if save_img or save_crop or view_img:  # Add bbox to image
                        c = int(cls)  # integer class
                        dict_name = {'0':"椰子真果粒", '1':"草莓真果粒", '2':"黄桃真果粒", '3':"芦荟真果粒",
                                    '4':"百岁山", '5':"巴马铂泉", '6':"可口可乐", '7':"果汁", '8':"拿铁咖啡", '9':"啤酒", 
                                    '10':"鸡尾酒", '11':"娃哈哈",'12':"农夫山泉",'13':"怡宝",'14':"依云"}

                        label = None if hide_labels else (dict_name[names[c]] if hide_conf else f'{dict_name[names[c]]} {conf:.2f}')
                        annotator.box_label(xyxy, label, color=colors(c, True))

                    if save_crop:
                        save_one_box(xyxy, imc, file=save_dir / 'crops' / names[c] / f'{p.stem}.jpg', BGR=True)
                np.save('/home/jianghao/armZ1_ws/src/vision_module/detect.npy', write_dict)

            # Stream results
            im0 = annotator.result()
            if view_img:
                cv2.imshow(str(p), im0)
                cv2.waitKey(1)  # 1 millisecond

            # Save results (image with detections)
            if save_img:
                if dataset.mode == 'image':
                    cv2.imwrite(save_path, im0)
                else:  # 'video' or 'stream'
                    if vid_path[i] != save_path:  # new video
                        vid_path[i] = save_path
                        if isinstance(vid_writer[i], cv2.VideoWriter):
                            vid_writer[i].release()  # release previous video writer
                        if vid_cap:  # video
                            fps = vid_cap.get(cv2.CAP_PROP_FPS)
                            w = int(vid_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                            h = int(vid_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                        else:  # stream
                            fps, w, h = 30, im0.shape[1], im0.shape[0]
                        save_path = str(Path(save_path).with_suffix('.mp4'))  # force *.mp4 suffix on results videos
                        vid_writer[i] = cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
                    vid_writer[i].write(im0)

        # Print time (inference-only)
        LOGGER.info(f'{s}Done. ({t3 - t2:.3f}s)')

    # Print results
    t = tuple(x / seen * 1E3 for x in dt)  # speeds per image
    LOGGER.info(f'Speed: %.1fms pre-process, %.1fms inference, %.1fms NMS per image at shape {(1, 3, *imgsz)}' % t)
    if save_txt or save_img:
        s = f"\n{len(list(save_dir.glob('labels/*.txt')))} labels saved to {save_dir / 'labels'}" if save_txt else ''
        LOGGER.info(f"Results saved to {colorstr('bold', save_dir)}{s}")
    if update:
        strip_optimizer(weights)  # update model (to fix SourceChangeWarning)



def parse_opt():
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', nargs='+', type=str, default='/home/jianghao/armZ1_ws/src/vision_module/weights/best.pt', help='model path(s)')
    parser.add_argument('--source', type=str, default='/home/jianghao/armZ1_ws/src/vision_module/source', help='file/dir/URL/glob, 0 for webcam')
    parser.add_argument('--data', type=str, default='/home/jianghao/armZ1_ws/src/vision_module/data/Product.yaml', help='(optional) dataset.yaml path')
    parser.add_argument('--imgsz', '--img', '--img-size', nargs='+', type=int, default=[640], help='inference size h,w')
    parser.add_argument('--conf-thres', type=float, default=0.25, help='confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.45, help='NMS IoU threshold')
    parser.add_argument('--max-det', type=int, default=1000, help='maximum detections per image')
    parser.add_argument('--device', default='', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--view-img', action='store_true', help='show results')
    parser.add_argument('--save-txt', default='True', help='save results to *.txt')
    parser.add_argument('--save-conf', action='store_true', help='save confidences in --save-txt labels')
    parser.add_argument('--save-crop', action='store_true', help='save cropped prediction boxes')
    parser.add_argument('--nosave', action='store_true', help='do not save images/videos')
    parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --classes 0, or --classes 0 2 3')
    parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
    parser.add_argument('--augment', action='store_true', help='augmented inference')
    parser.add_argument('--visualize', action='store_true', help='visualize features')
    parser.add_argument('--update', action='store_true', help='update all models')
    parser.add_argument('--project', default='/home/jianghao/armZ1_ws/src/vision_module/runs/detect', help='save results to project/name')
    parser.add_argument('--name', default='exp', help='save results to project/name')
    parser.add_argument('--exist-ok', action='store_true', help='existing project/name ok, do not increment')
    parser.add_argument('--line-thickness', default=3, type=int, help='bounding box thickness (pixels)')
    parser.add_argument('--hide-labels', default=False, action='store_true', help='hide labels')
    parser.add_argument('--hide-conf', default=False, action='store_true', help='hide confidences')
    parser.add_argument('--half', action='store_true', help='use FP16 half-precision inference')
    parser.add_argument('--dnn', action='store_true', help='use OpenCV DNN for ONNX inference')
    opt = parser.parse_args()
    opt.imgsz *= 2 if len(opt.imgsz) == 1 else 1  # expand
    print_args(vars(opt))
    return opt

def main(opt):
    check_requirements(exclude=('tensorboard', 'thop'))
    run(**vars(opt))

def publish_detection(label,x,y,depth):
    detection_msg = ObjectDetection()
    detection_msg.label = label
    detection_msg.x = x
    detection_msg.y = y
    detection_msg.depth = depth
    rospy.loginfo(f"Publishing detection: {label}, x={x}, y={y}, depth={depth}")
    pub.publish(detection_msg)

robot_state_pub = rospy.Publisher('robot_state_topic', Int32, queue_size=10)
def publish_robot_state(state):
    robot_state_pub.publish(state)
    rospy.loginfo(f"Published photo state {state}")

def keyword_callback(msg):
    keyword = msg.data
    rospy.loginfo(f"Received keyword: {keyword}")
    task_queue.put(keyword)


# 任务工作线程
def task_worker():
    while not rospy.is_shutdown():
        try:
            keyword = task_queue.get(block=True, timeout=1)
            print("keyword:",keyword)
            main(opt)
            try:
                    # Load detect.npy and find the corresponding entry
                detection_data = np.load('/home/jianghao/armZ1_ws/src/vision_module/detect.npy', allow_pickle=True).item()
                if keyword in detection_data:
                    x, y, depth = detection_data[keyword]
                    publish_detection(keyword, x, y, depth)
                    publish_robot_state(2)

                else:
                    rospy.logwarn(f"Keyword '{keyword}' not found in detection data.")
            except Exception as e:
                rospy.logerr(f"Failed to load detection data: {e}")

        except queue.Empty:
            continue
        except Exception as e:
            rospy.logerr(f"Error in task_worker: {e}")   





if __name__ == "__main__":
    opt = parse_opt()
    rospy.init_node('vision_module_node',anonymous = True)

    rospy.Subscriber('keyword_topic', String, keyword_callback)

    task_thread = threading.Thread(target=task_worker)
    task_thread.start()
    # rospy.Subscriber('photo_state_topic', Int32, callback_photo_state)

    rospy.spin()