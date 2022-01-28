#!/home/grego/Documents/envs/cv_inference/bin/python

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
                                         yolov5s.mlmodel            # CoreML (MacOS-only)
                                         yolov5s_saved_model        # TensorFlow SavedModel
                                         yolov5s.pb                 # TensorFlow GraphDef
                                         yolov5s.tflite             # TensorFlow Lite
                                         yolov5s_edgetpu.tflite     # TensorFlow Edge TPU
"""

import argparse
import os
from socket import MsgFlag
import sys
from pathlib import Path

import cv2
import torch
import torch.backends.cudnn as cudnn

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

from models.common import DetectMultiBackend
from utils.datasets import IMG_FORMATS, VID_FORMATS, LoadImages, LoadStreams
from utils.general import (LOGGER, check_file, check_img_size, check_imshow, check_requirements, colorstr,
                           increment_path, non_max_suppression, print_args, scale_coords, strip_optimizer, xyxy2xywh)
from utils.plots import Annotator, colors, save_one_box
from utils.torch_utils import select_device, time_sync

# For kalman tracking
import track.sort  
import time

# For ROS
import rospy
from yolo_ros.msg import ShipInfo
from sbg_driver.msg import SbgEkfQuat, SbgEkfNav
from yolo_ros.msg import ContactsList
from projection import Projecteur


    

@torch.no_grad()
def run(weights=ROOT / 'yolov5s.pt',  # model.pt path(s)
        source=ROOT / 'data/images',  # file/dir/URL/glob, 0 for webcam
        data=ROOT / 'data/coco128.yaml',  # dataset.yaml path
        imgsz=(640, 640),  # inference size (height, width)
        conf_thres=0.25,  # confidence threshold
        iou_thres=0.45,  # NMS IOU threshold
        max_det=1000,  # maximum detections per image
        device='',  # cuda device, i.e. 0 or 0,1,2,3 or cpu
        view_img=False,  # show results
        save_txt=False,  # save results to *.txt
        save_conf=False,  # save confidences in --save-txt labels
        save_crop=False,  # save cropped prediction boxes
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
        no_kalman=False, # use kalman filtering
        min_hits=5, # parameter for kalman filter
        max_age=1, # parameter of duration
        save_kalman=False,
        reticle=False,
        rt=False,
        ):
    
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
    model = DetectMultiBackend(weights, device=device, dnn=dnn, data=data)
    stride, names, pt, jit, onnx, engine = model.stride, model.names, model.pt, model.jit, model.onnx, model.engine
    imgsz = check_img_size(imgsz, s=stride)  # check image size

    # Initialize Kalman filtering.  
    if not no_kalman :
        mot_tracker = track.sort.Sort(min_hits=min_hits, max_age=max_age)
    
    # Initialize ROS node
    rospy.init_node('VisionTrackProducer', anonymous=True)    
    publisher = rospy.Publisher('vision_detection', ContactsList, queue_size = 10)
    
    # Half
    half &= (pt or jit or onnx or engine) and device.type != 'cpu'  # FP16 supported on limited backends with CUDA
    if pt or jit:
        model.model.half() if half else model.model.float()

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


    # To prevent an error if no videos
    dataset.frame = 0
    # model_name = weights[0].replace('.\\','').replace('.pt','')
    
    # Run inference
    model.warmup(imgsz=(bs, 3, *imgsz), half=half)  # warmup
    dt, seen = [0.0, 0.0, 0.0], 0
    
    if rt:
        skip_frames = 0
        time_delta = 0
        fps = 30
        
    QuatMsg = SbgEkfQuat()
    QuatMsg.quaternion.x = 1
    NavMsg = SbgEkfNav()
    # Define the callback function when receiving CI data.
    def callback_nav(data):
        nonlocal NavMsg
        NavMsg = data

    def callback_quat(data):
        nonlocal QuatMsg
        QuatMsg = data

    rospy.Subscriber('Quat', SbgEkfQuat, callback_quat, queue_size = 1)    
    rospy.Subscriber('Nav', SbgEkfNav, callback_nav, queue_size = 1)    
    rate = rospy.Rate(10)
    projecteur = Projecteur()
    for path, im, im0s, vid_cap, s in dataset:
        if rt:
            if skip_frames > 0:
                skip_frames -= 1
                continue
            else :
                time.sleep(time_delta)
        
        # Update proj with current values.
        projecteur.update(
            QuatMsg=QuatMsg,
            NavMsg=NavMsg,
        )
        
        t1 = time_sync()
        if (not no_kalman) & (dataset.frame == 1) :
            del mot_tracker
            track.sort.KalmanBoxTracker.count = 0
            mot_tracker = track.sort.Sort(min_hits=min_hits, max_age=max_age)
            if save_kalman:
                save_kalman_path = '.'.join(dataset.files[dataset.count].split('.')[:-1]) + f'_{model_name}' +'.csv'
                line = ('frame', 'obj_id','detect_score', 'xtl','ytl','xbr','ybr')
                with open(save_kalman_path, 'w') as f:
                    f.write(('%s,' * len(line)).rstrip()[:-1] % line + '\n')
        im = torch.from_numpy(im).to(device)
        im = im.half() if half else im.float()  # uint8 to fp16/32
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

        # NMS
        pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)
        dt[2] += time_sync() - t3

        # Second-stage classifier (optional)
        # pred = utils.general.apply_classifier(pred, classifier_model, im, im0s)

        # Process predictions
        for i, det in enumerate(pred):  # per image
            seen += 1
            if webcam:  # batch_size >= 1
                p, im0, frame = path[i], im0s[i].copy(), dataset.count
                s += f'{i}: '
            else:
                p, im0, frame = path, im0s.copy(), getattr(dataset, 'frame', 0)

            p = Path(p)  # to Path
            save_path = str(save_dir / p.name)  # im.jpg
            txt_path = str(save_dir / 'labels' / p.stem) + ('' if dataset.mode == 'image' else f'_{frame}')  # im.txt
            s += '%gx%g ' % im.shape[2:]  # print string
            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
            imc = im0.copy() if save_crop else im0  # for save_crop
            annotator = Annotator(im0, line_width=line_thickness, example=str(names))
            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(im.shape[2:], det[:, :4], im0.shape).round()

                # Print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                    s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string

                if not no_kalman :
                    tracked_objects = mot_tracker.update(reversed(det).cpu())
                    detection_number = 0
                    confs = det[:,4]
                    dt[2] += time_sync() - t3

                else:
                    tracked_objects = reversed(det)

                # Write results
                for *xyxy, conf, cls in tracked_objects:
                    if not no_kalman:
                        obj_id = conf
                        conf = confs[detection_number]
                        detection_number += 1

                    if True:           # Format and send the message !
                        camera_id = i # Not this generally but will depend which camera we use simutaneously.
                        xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                        out_msg = projecteur(xywh, camera_id, obj_id, rosmsg=True)
                        publisher.publish(out_msg)
                    
                    if save_txt & (not save_kalman):  # Write to file
                        xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                        line = (cls, *xywh, conf) if save_conf else (cls, *xywh)  # label format
                        with open(txt_path + '.txt', 'a') as f:
                            f.write(('%g ' * len(line)).rstrip() % line + '\n')

                    if (not no_kalman) & save_kalman :
                        # save_path = dataset.files[dataset.count].split(os.sep)[-1].split('.')[0]
                        xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                        rounding = lambda x : round(x,4)
                        line = (dataset.frame, int(obj_id), conf,*[rounding(el) for el in xywh])
                        with open(save_kalman_path, 'a') as f:
                            f.write(('%g,' * len(line)).rstrip()[:-1] % line + '\n')

                    if save_img or save_crop or view_img:  # Add bbox to image
                        if not no_kalman:
                            c = int(obj_id)
                            label = None if hide_labels else (str(c) if hide_conf else f'{c} {conf:.3f}')
                        else:
                            c = int(cls)  # integer class
                            label = None if hide_labels else (names[c] if hide_conf else f'{names[c]} {conf:.3f}')
                        annotator.box_label(xyxy, label, color=colors(c, True))
                        if save_crop:
                            save_one_box(xyxy, imc, file=save_dir / 'crops' / names[c] / f'{p.stem}.jpg', BGR=True)          
            
            # Add the middle rectangle to visualize the center.
            if reticle:
                h,w,_ = im0.shape
                size = 0.02*h
                xyxy = [0.5*w + size, 0.5*h + size,0.5*w - size,0.5*h - size]
                annotator.box_label(xyxy, None, color=(0,153,255))
                size = 0.003*h
                xyxy = [0.5*w + size, 0.5*h + size,0.5*w - size,0.5*h - size]
                annotator.box_label(xyxy, None, color=(0,20,153))

            # Print time (inference-only)
            LOGGER.info(f'{s}Done. ({t3 - t2:.3f}s)')

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
                            save_path += '.mp4'
                        vid_writer[i] = cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
                    vid_writer[i].write(im0)
            
        rate.sleep()
        
        
        if rt:
            t_end = time_sync()
            if dataset.mode != 'image':
                
                # Compute number of frames to skip in order to be real time.
                skip_frames = int(fps * (t_end - t1))
                time_delta = (fps * (t_end - t1) - skip_frames)/fps
            
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
    parser.add_argument('--weights', nargs='+', type=str, default=ROOT / 'yolov5s.pt', help='model path(s)')
    parser.add_argument('--source', type=str, default=ROOT / 'data/images', help='file/dir/URL/glob, 0 for webcam')
    parser.add_argument('--data', type=str, default=ROOT / 'data/coco128.yaml', help='(optional) dataset.yaml path')
    parser.add_argument('--imgsz', '--img', '--img-size', nargs='+', type=int, default=[640], help='inference size h,w')
    parser.add_argument('--conf-thres', type=float, default=0.25, help='confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.45, help='NMS IoU threshold')
    parser.add_argument('--max-det', type=int, default=1000, help='maximum detections per image')
    parser.add_argument('--device', default='', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--view-img', action='store_true', help='show results')
    parser.add_argument('--save-txt', action='store_true', help='save results to *.txt')
    parser.add_argument('--save-conf', action='store_true', help='save confidences in --save-txt labels')
    parser.add_argument('--save-crop', action='store_true', help='save cropped prediction boxes')
    parser.add_argument('--nosave', action='store_true', help='do not save images/videos')
    parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --classes 0, or --classes 0 2 3')
    parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
    parser.add_argument('--augment', action='store_true', help='augmented inference')
    parser.add_argument('--visualize', action='store_true', help='visualize features')
    parser.add_argument('--update', action='store_true', help='update all models')
    parser.add_argument('--project', default=ROOT / 'runs/detect', help='save results to project/name')
    parser.add_argument('--name', default='exp', help='save results to project/name')
    parser.add_argument('--exist-ok', action='store_true', help='existing project/name ok, do not increment')
    parser.add_argument('--line-thickness', default=3, type=int, help='bounding box thickness (pixels)')
    parser.add_argument('--hide-labels', default=False, action='store_true', help='hide labels')
    parser.add_argument('--hide-conf', default=False, action='store_true', help='hide confidences')
    parser.add_argument('--half', action='store_true', help='use FP16 half-precision inference')
    parser.add_argument('--dnn', action='store_true', help='use OpenCV DNN for ONNX inference')
    
    # Kalman args
    parser.add_argument('--save-kalman', action='store_true', help='save labels as a csv file with object tracking')
    parser.add_argument('--no-kalman', action='store_true', help="don't use kalman filtering")
    parser.add_argument('--min-hits', default=5, type=int, help='kalman min hits')
    parser.add_argument('--max-age', default=1, type=int, help='kalman max age')
    
    # ROS args
    parser.add_argument('--rt', action='store_true', help="play the video in real-time, meaning that it might skip frames")
    
    # IPD args    
    parser.add_argument('--reticle', action='store_true', help='show center of screen.')
    
    opt = parser.parse_args()
    opt.imgsz *= 2 if len(opt.imgsz) == 1 else 1  # expand
    print_args(FILE.stem, opt)
    return opt


def main(opt):
    check_requirements(exclude=('tensorboard', 'thop'))
    run(**vars(opt))


if __name__ == "__main__":
    opt = parse_opt()
    # We set the following default values:
    opt.view_img = True
    opt.source = ROOT/"ShipSpotting1.mp4"
    opt.weights = ROOT/'gdn.pt'
    opt.nosave = True
    opt.rt = True
    main(opt)
