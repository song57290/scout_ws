#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
from pathlib import Path
import torch

FILE = Path(__file__).absolute()
sys.path.append(FILE.parents[0].as_posix())

# YOLOv5 internals (repo 동봉 utils/models)
from models.common import DetectMultiBackend
from utils.general import check_img_size, non_max_suppression, scale_boxes
from utils.plots import Annotator, colors
from utils.torch_utils import select_device, smart_inference_mode

bridge = CvBridge()

IMAGE_TOPIC = '/camera/image_raw'                       # 입력 원본 이미지
ANNOTATED_COMPRESSED_TOPIC = '/yolo/annotated_image/compressed'  # 주석 그린 CompressedImage 퍼블리시
WEIGHTS = 'best.pt'                                     # 가중치 경로(스크립트 폴더 기준)
DATA_CFG = 'data/coco128.yaml'                          # 클래스 이름 로딩용
IMG_SIZE = (640, 480)                                   # 추론 입력 크기 (w,h)
CONF_THRES = 0.25
IOU_THRES = 0.45
MAX_DET = 1000
LINE_THICKNESS = 3
HIDE_LABELS = False
HIDE_CONF = False
USE_FP16 = False                                        # GPU에서만 의미
CONF_NOTIFY = 0.60                                      # 알림 임계값
JPEG_QUALITY = 90                                       # 퍼블리시 JPEG 품질(0~100)


class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')

        # ---- Device & model ----
        device_num = ''  # '' auto / '0' cuda:0 / 'cpu'
        self.device = select_device(device_num)
        self.model = DetectMultiBackend(WEIGHTS, device=self.device, dnn=False, data=DATA_CFG, fp16=USE_FP16)
        stride, self.names, pt = self.model.stride, self.model.names, self.model.pt
        imgsz = check_img_size(IMG_SIZE, s=stride)
        self.model.warmup(imgsz=(1 if pt or self.model.triton else 1, 3, *imgsz))

        # ---- State ----
        self.fire_detected = False
        self.count = 0

        # ---- QoS ----
        sensor_qos = QoSProfile(depth=5)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        sensor_qos.history = HistoryPolicy.KEEP_LAST

        # ---- Subscriber ----
        self.subscription = self.create_subscription(
            Image,
            IMAGE_TOPIC,
            self.camera_callback,
            sensor_qos
        )

        # ---- Publisher (Annotated CompressedImage) ----
        self.pub_annot = self.create_publisher(CompressedImage, ANNOTATED_COMPRESSED_TOPIC, sensor_qos)

        self.get_logger().info(f'Subscribed to image topic: {IMAGE_TOPIC}')
        self.get_logger().info(f'Publishing annotated to: {ANNOTATED_COMPRESSED_TOPIC}')
        self.get_logger().info(f'Using device: {self.device}')

    @smart_inference_mode()
    def camera_callback(self, msg: Image):
        # ROS Image -> OpenCV BGR
        img0 = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')  # (H,W,3) BGR uint8

        # ---- Preprocess: BGR->RGB, HWC->CHW, [0,1] ----
        img = img0.copy()
        img = img[..., ::-1]                    # BGR->RGB
        img = np.transpose(img, (2, 0, 1))      # HWC->CHW
        img = np.expand_dims(img, 0)            # BxCxHxW
        img = np.ascontiguousarray(img)
        img = torch.from_numpy(img).to(self.model.device)
        img = img.half() if self.model.fp16 else img.float()
        img /= 255.0

        # ---- Inference ----
        pred = self.model(img, augment=False, visualize=False)

        # ---- NMS ----
        pred = non_max_suppression(
            pred, CONF_THRES, IOU_THRES,
            classes=None, agnostic=False, max_det=MAX_DET
        )

        # ---- Postprocess & Draw ----
        annotator = Annotator(img0, line_width=LINE_THICKNESS, example=str(self.names))

        self.count += 1
        if self.count > 1:
            self.fire_detected = False

        for det in pred:
            if len(det):
                self.count = 0
                det[:, :4] = scale_boxes(img.shape[2:], det[:, :4], img0.shape).round()

                for *xyxy, conf, cls in reversed(det):
                    c = int(cls)
                    if HIDE_LABELS:
                        label = None
                    else:
                        if HIDE_CONF:
                            label = self.names[c]
                        else:
                            label = f'{self.names[c]} {float(conf):.2f}'

                    # 알림(임계값 초과 시 1회 출력)
                    if conf.item() > CONF_NOTIFY and not self.fire_detected:
                        self.get_logger().info(
                            "\n==============================================\n"
                            "Fire detected! Notification sent to the owner\n"
                            "==============================================\n"
                        )
                        self.fire_detected = True

                    annotator.box_label(xyxy, label, color=colors(c, True))

        # 주석 그려진 프레임
        annotated = annotator.result()

        # ---- Publish annotated as CompressedImage (JPEG) ----
        try:
            ok, enc = cv2.imencode('.jpg', annotated, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
            if ok:
                msg_out = CompressedImage()
                msg_out.header = msg.header  # 입력 헤더를 그대로 복사(타임스탬프/프레임)
                msg_out.format = 'jpeg'
                msg_out.data = enc.tobytes()
                self.pub_annot.publish(msg_out)
        except Exception as e:
            self.get_logger().warn(f'Failed to encode/publish annotated image: {e}')

        # ---- (선택) 로컬 미리보기 ----
        try:
            cv2.imshow("YOLO Annotated", annotated)
            cv2.waitKey(1)
        except Exception:
            pass


def main():
    rclpy.init(args=None)
    node = CameraSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass


if __name__ == '__main__':
    main()

