#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from pathlib import Path
import torch

FILE = Path(__file__).absolute()
sys.path.append(FILE.parents[0].as_posix())

# YOLOv5 internals (repo 동봉 utils/models)
from models.common import DetectMultiBackend
from utils.general import (
    check_img_size, increment_path, non_max_suppression,
    scale_boxes
)
from utils.plots import Annotator, colors
from utils.torch_utils import select_device, smart_inference_mode

bridge = CvBridge()


class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')

        # ---- Parameters (필요시 런치/CLI에서 덮어쓰기) ----
        # scout_mini 공통 RGB 토픽 기본값
        self.declare_parameter('image_topic', '/camera/image_raw')
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value

        # YOLO 설정
        weights = 'best.pt'              # 가중치 파일 경로 (스크립트 폴더 기준)
        self.imgsz = (640, 480)          # 입력 크기
        self.conf_thres = 0.25           # confidence threshold
        self.iou_thres = 0.45            # NMS IoU threshold
        self.max_det = 1000
        self.classes = None
        self.agnostic_nms = False
        self.augment = False
        self.visualize = False
        self.line_thickness = 3
        self.hide_labels = False
        self.hide_conf = False
        self.half = False                # FP16 사용 여부 (GPU에서만 의미)
        self.data = 'data/coco128.yaml'  # 클래스 이름 로딩용
        self.fire_detected = False
        self.count = 0

        # ---- Device 선택 및 모델 로드 ----
        device_num = ''  # '' -> auto select, '0' -> cuda:0, 'cpu' -> CPU 강제
        self.device = select_device(device_num)
        self.model = DetectMultiBackend(weights, device=self.device, dnn=False, data=self.data, fp16=self.half)
        stride, self.names, pt = self.model.stride, self.model.names, self.model.pt
        imgsz = check_img_size(self.imgsz, s=stride)
        self.model.warmup(imgsz=(1 if pt or self.model.triton else 1, 3, *imgsz))

        # ---- 카메라 구독 (sensor_data QoS 권장) ----
        sensor_qos = QoSProfile(depth=5)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        sensor_qos.history = HistoryPolicy.KEEP_LAST

        self.subscription = self.create_subscription(
            Image,
            image_topic,           # 예: '/camera/image_raw'
            self.camera_callback,
            sensor_qos
        )

        self.get_logger().info(f'Subscribed to image topic: {image_topic}')
        self.get_logger().info(f'Using device: {self.device}')

    @smart_inference_mode()
    def camera_callback(self, msg: Image):
        # msg.encoding 이 rgb8 이라도, OpenCV BGR로 쓰기 위해 "bgr8"으로 변환 요청
        img0 = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')  # (H,W,3) BGR uint8

        # ---- Preprocess: BGR->RGB, HWC->CHW, [0,1] ----
        img = img0.copy()
        img = img[np.newaxis, :, :, :]               # BHWC
        img = np.stack(img, 0)                       # B=1
        img = img[..., ::-1].transpose((0, 3, 1, 2)) # BGR->RGB, BHWC->BCHW
        img = np.ascontiguousarray(img)
        img = torch.from_numpy(img).to(self.model.device)
        img = img.half() if self.model.fp16 else img.float()
        img /= 255.0
        if img.ndim == 3:
            img = img[None]

        # ---- Inference ----
        pred = self.model(img, augment=self.augment, visualize=False)

        # ---- NMS ----
        pred = non_max_suppression(
            pred, self.conf_thres, self.iou_thres,
            self.classes, self.agnostic_nms, max_det=self.max_det
        )

        # ---- Postprocess & Draw ----
        for i, det in enumerate(pred):
            annotator = Annotator(img0, line_width=self.line_thickness, example=str(self.names))

            # 연속 프레임 관리 (간단히 중복 알림 방지)
            self.count += 1
            if self.count > 1:
                self.fire_detected = False

            if len(det):
                self.count = 0
                det[:, :4] = scale_boxes(img.shape[2:], det[:, :4], img0.shape).round()

                for *xyxy, conf, cls in reversed(det):
                    c = int(cls)
                    label = None if self.hide_labels else (
                        self.names[c] if self.hide_conf else f'{self.names[c]} {float(conf):.2f}'
                    )

                    # 알림 (임계값 0.60)
                    if conf.item() > 0.60 and not self.fire_detected:
                        self.get_logger().info("\n==============================================\n"
                                               "Fire detected! Notification sent to the owner\n"
                                               "==============================================\n")
                        self.fire_detected = True

                    annotator.box_label(xyxy, label, color=colors(c, True))

        # ---- Show window (X11 필요). 서버/헤드리스면 생략됨 ----
        try:
            cv2.imshow("IMAGE", img0)
            cv2.waitKey(1)
        except Exception:
            # 헤드리스 환경이면 그냥 무시
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
