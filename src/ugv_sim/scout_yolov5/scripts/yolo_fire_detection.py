#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from pathlib import Path
import torch

FILE = Path(__file__).absolute()
sys.path.append(FILE.parents[0].as_posix())

# YOLOv5 internals
from models.common import DetectMultiBackend
from utils.general import check_img_size, non_max_suppression, scale_boxes
from utils.plots import Annotator, colors
from utils.torch_utils import select_device, smart_inference_mode

bridge = CvBridge()

# --- Topics & params ---
RGB_TOPIC   = '/camera/image_raw'                   # 입력 RGB
DEPTH_TOPIC = '/depth_camera/depth/image_raw'       # 입력 Depth (32FC1 또는 16UC1)
INFO_TOPIC  = '/depth_camera/depth/camera_info'     # 카메라 내·외부파라미터
ANNOTATED_COMPRESSED_TOPIC = '/fire/annotated_image/compressed'
FIRE_DETECTED_TOPIC        = '/fire/detected'
FIRE_POSITION_TOPIC        = '/fire/position'

WEIGHTS  = 'best.pt'
DATA_CFG = 'data/coco128.yaml'
IMG_SIZE = (640, 480)
CONF_THRES = 0.25
IOU_THRES  = 0.45
MAX_DET    = 1000
LINE_THICKNESS = 3
HIDE_LABELS = False
HIDE_CONF   = False
USE_FP16    = False
CONF_NOTIFY = 0.60
JPEG_QUALITY = 90

# 깊이 샘플링 윈도우 (bbox 중심 주변 median 사용)
DEPTH_WIN = 5           # 홀수 권장
MIN_DEPTH = 0.05        # m
MAX_DEPTH = 20.0        # m


class FireDetector(Node):
    def __init__(self):
        super().__init__('fire_detector')

        # ---- Device & model ----
        self.device = select_device('')
        self.model = DetectMultiBackend(WEIGHTS, device=self.device, dnn=False, data=DATA_CFG, fp16=USE_FP16)
        stride, self.names, pt = self.model.stride, self.model.names, self.model.pt
        imgsz = check_img_size(IMG_SIZE, s=stride)
        self.model.warmup(imgsz=(1 if pt or self.model.triton else 1, 3, *imgsz))

        # ---- State ----
        self.fire_detected_once = False
        self.last_depth = None          # np.ndarray (H,W), float32 m 단위
        self.have_depth = False
        self.fx = self.fy = self.cx = self.cy = None
        self.depth_frame_id = None
        self.have_intrinsics = False

        # ---- QoS ----
        sensor_qos = QoSProfile(depth=5)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        sensor_qos.history = HistoryPolicy.KEEP_LAST

        # ---- Subscribers ----
        self.sub_rgb   = self.create_subscription(Image, RGB_TOPIC, self.cb_rgb, sensor_qos)
        self.sub_depth = self.create_subscription(Image, DEPTH_TOPIC, self.cb_depth, sensor_qos)
        self.sub_info  = self.create_subscription(CameraInfo, INFO_TOPIC, self.cb_info, sensor_qos)
        self._info_logged = False
        self._last_info = None


        # ---- Publishers ----
        self.pub_annot = self.create_publisher(CompressedImage, ANNOTATED_COMPRESSED_TOPIC, sensor_qos)
        self.pub_flag  = self.create_publisher(Bool, FIRE_DETECTED_TOPIC, 10)
        self.pub_pos   = self.create_publisher(PointStamped, FIRE_POSITION_TOPIC, 10)

        # ---- Logs ----
        self.get_logger().info(f'[INFO]: RGB topic:   {RGB_TOPIC}')
        self.get_logger().info(f'[INFO]: DEPTH topic: {DEPTH_TOPIC}')
        self.get_logger().info(f'[INFO]: INFO topic:  {INFO_TOPIC}')
        self.get_logger().info(f'[INFO]: Annotated out: {ANNOTATED_COMPRESSED_TOPIC}')
        self.get_logger().info(f'[INFO]: Using device: {self.device}')
        self.get_logger().warn('[WARN]: No valid depth/intrinsics yet — 3D position will publish once ready.')

    # ---------------- Depth & intrinsics ----------------
    def cb_info(self, msg: CameraInfo):
        if len(msg.k) == 9:
            self.fx, self.fy = float(msg.k[0]), float(msg.k[4])
            self.cx, self.cy = float(msg.k[2]), float(msg.k[5])
            self.have_intrinsics = True
            cur = (self.fx, self.fy, self.cx, self.cy, msg.header.frame_id)
            if not self._info_logged or cur != self._last_info:
                self.get_logger().info(
                    f'CameraInfo received: fx={self.fx:.1f}, fy={self.fy:.1f}, '
                    f'cx={self.cx:.1f}, cy={self.cy:.1f}, frame={msg.header.frame_id}'
                )
                self._last_info = cur
                self._info_logged = True
        self.depth_frame_id = msg.header.frame_id or self.depth_frame_id

    def cb_depth(self, msg: Image):
        # Convert depth to meters float32
        try:
            depth = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if depth is None:
                return
            if depth.dtype == np.uint16:
                # 16UC1: 보통 mm → m 변환 필요
                depth_m = depth.astype(np.float32) / 1000.0
            else:
                # 32FC1 (이미 meters)
                depth_m = depth.astype(np.float32)
            # NaN/Inf 클리핑
            depth_m[~np.isfinite(depth_m)] = 0.0
            self.last_depth = depth_m
            self.have_depth = True
            # frame_id를 depth 이미지에서 가져올 경우 (카메라옵티컬프레임일 때 선호)
            if msg.header.frame_id:
                self.depth_frame_id = msg.header.frame_id
        except Exception as e:
            self.get_logger().warn(f'Failed to convert depth: {e}')

    # ---------------- RGB / YOLO ----------------
    @smart_inference_mode()
    def cb_rgb(self, msg: Image):
        img0 = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')  # BGR uint8
        self.rgb_h, self.rgb_w = img0.shape[:2]


        # Preprocess
        img = img0[..., ::-1]                    # BGR->RGB
        img = np.transpose(img, (2, 0, 1))       # HWC->CHW
        img = np.expand_dims(img, 0)             # 1xCxHxW
        img = np.ascontiguousarray(img)
        img = torch.from_numpy(img).to(self.model.device)
        img = img.half() if self.model.fp16 else img.float()
        img /= 255.0

        # Inference
        pred = self.model(img, augment=False, visualize=False)

        # NMS
        pred = non_max_suppression(pred, CONF_THRES, IOU_THRES, classes=None, agnostic=False, max_det=MAX_DET)

        # Draw & publish
        annotator = Annotator(img0, line_width=LINE_THICKNESS, example=str(self.names))

        # 기본값: 검출 없음
        fire_flag = False
        best_hit = None  # (u,v,conf,cls,xyxy)

        for det in pred:
            if len(det):
                det[:, :4] = scale_boxes(img.shape[2:], det[:, :4], img0.shape).round()

                # 신뢰도 높은 박스 우선
                for *xyxy, conf, cls in reversed(det):
                    c = int(cls)
                    label = None
                    if not HIDE_LABELS:
                        label = self.names[c] if HIDE_CONF else f'{self.names[c]} {float(conf):.2f}'
                    annotator.box_label(xyxy, label, color=colors(c, True))

                    # “fire” 클래스만 3D 산출하고 싶다면 아래 한 줄 활성화
                    # if str(self.names[c]).lower() != 'fire': continue

                    # 가장 높은 conf 1개만 3D로
                    if best_hit is None or conf.item() > best_hit[2]:
                        x1, y1, x2, y2 = [int(t.item()) for t in xyxy]
                        u = (x1 + x2) / 2.0
                        v = (y1 + y2) / 2.0
                        best_hit = (u, v, conf.item(), c, (x1, y1, x2, y2))

                    # 알림 로그(1회성)
                    if conf.item() > CONF_NOTIFY and not self.fire_detected_once:
                        self.get_logger().info(
                            "\n==============================================\n"
                            "Fire detected! Notification sent to the owner\n"
                            "==============================================\n"
                        )
                        self.fire_detected_once = True

        # 퍼블리시: 주석 이미지
        annotated = annotator.result()
        try:
            ok, enc = cv2.imencode('.jpg', annotated, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
            if ok:
                msg_out = CompressedImage()
                msg_out.header = msg.header
                msg_out.format = 'jpeg'
                msg_out.data = enc.tobytes()
                self.pub_annot.publish(msg_out)
        except Exception as e:
            self.get_logger().warn(f'Failed to encode/publish annotated image: {e}')

        # 3D 좌표 계산 & 퍼블리시 (가능할 때만)
        if best_hit is not None:
            fire_flag = True
            self._maybe_publish_3d(best_hit, msg)

        # 검출 플래그 퍼블리시
        self.pub_flag.publish(Bool(data=fire_flag))

        # 로컬 미리보기(선택)
        try:
            cv2.imshow("YOLO Annotated", annotated)
            cv2.waitKey(1)
        except Exception:
            pass

    # ---------------- 2D -> 3D ----------------
    def _maybe_publish_3d(self, best_hit, rgb_msg):
        if not (self.have_depth and self.have_intrinsics and self.last_depth is not None):
            # 아직 준비되지 않음
            return

        u, v, conf, c, (x1, y1, x2, y2) = best_hit
        H, W = self.last_depth.shape[:2]          # depth H,W
        # RGB(u,v) -> Depth 좌표로 스케일 (크기 다를 때 중요)
        sw = W / float(getattr(self, "rgb_w", W) or W)
        sh = H / float(getattr(self, "rgb_h", H) or H)
        uc = int(np.clip(round(u * sw), 0, W - 1))
        vc = int(np.clip(round(v * sh), 0, H - 1))

        # 중심 주변 DEPTH_WIN x DEPTH_WIN 윈도우 median
        half = DEPTH_WIN // 2
        u0, u1 = max(0, uc - half), min(W, uc + half + 1)
        v0, v1 = max(0, vc - half), min(H, vc + half + 1)
        patch = self.last_depth[v0:v1, u0:u1]

        # 유효 깊이 필터링
        valid = patch[(patch > MIN_DEPTH) & (patch < MAX_DEPTH) & np.isfinite(patch)]
        if valid.size == 0:
            # 깊이 실패
            return
        Z = float(np.median(valid))

        # 핀홀 역투영
        X = (u - self.cx) / self.fx * Z
        Y = (v - self.cy) / self.fy * Z

        pt = PointStamped()
        # depth 프레임을 신뢰(보통 *_optical_frame)
        pt.header.stamp = rgb_msg.header.stamp
        pt.header.frame_id = self.depth_frame_id if self.depth_frame_id else 'depth_camera_optical_frame'
        pt.point.x = X
        pt.point.y = Y
        pt.point.z = Z
        self.pub_pos.publish(pt)

        # 디버그 로그 (필요 시 주석 해제)
        # self.get_logger().info(f'fire@px({u:.1f},{v:.1f}) -> cam({X:.2f},{Y:.2f},{Z:.2f}) m, conf={conf:.2f}')

def main():
    rclpy.init(args=None)
    node = FireDetector()
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
