#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
요구사항:
- 자율주행(Nav2) 중이라도 화재 감지(conf > 0.6) 즉시 Nav2 모든 goal을 취소(CancelAll)하고
  (map) 좌표 (1.0, 3.0, yaw=0.0)으로 복귀(goToPose).
- 액션 서버 바쁠 때 경합/거절 방지: status 토픽/그레이스 타임으로 idle 판단 후 복귀 goal 1회 전송.
- 기존 기능 유지: YOLO 추론, 주석 이미지 퍼블리시, fire 플래그, 2D->3D(/fire/position),
  (선택) TF로 map 포인트(/fire/target_map), 감지 순간 AMCL 포즈(/fire/robot_pose).
- 도착 시 그리퍼 시퀀스(열기→닫기) 1회 수행 (토픽/메시지 타입 파라미터화).
- 리콜 종료 후 자동으로 /auto_mode=True 라치 발행하여 waypoint 순찰 재개(파라미터로 on/off 가능).
"""

import sys
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.qos import DurabilityPolicy

from std_msgs.msg import Bool, Float64, Float64MultiArray
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from pathlib import Path
import torch

# YOLOv5 internals
FILE = Path(__file__).absolute()
sys.path.append(FILE.parents[0].as_posix())
from models.common import DetectMultiBackend
from utils.general import check_img_size, non_max_suppression, scale_boxes
from utils.plots import Annotator, colors
from utils.torch_utils import select_device, smart_inference_mode

# Nav2(Simple Commander)
try:
    from nav2_simple_commander.robot_navigator import BasicNavigator
    _HAS_NAV2 = True
except Exception:
    _HAS_NAV2 = False

# TF2 (카메라 포인트를 map으로 변환 시 사용)
try:
    from tf2_ros import Buffer, TransformListener
    import tf2_geometry_msgs  # noqa: F401
    _HAS_TF2 = True
except Exception:
    _HAS_TF2 = False

# ROS2 Actions 상태/취소
from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal

bridge = CvBridge()

# --- Topics & params ---
RGB_TOPIC   = '/camera/image_raw'                    # 입력 RGB
DEPTH_TOPIC = '/depth_camera/depth/image_raw'        # 입력 Depth (32FC1 또는 16UC1)
INFO_TOPIC  = '/depth_camera/depth/camera_info'      # 카메라 내·외부파라미터

ANNOTATED_COMPRESSED_TOPIC = '/fire/annotated_image/compressed'
FIRE_DETECTED_TOPIC        = '/fire/detected'
FIRE_POSITION_TOPIC        = '/fire/position'        # 카메라 프레임 기준 3D
FIRE_ROBOT_POSE_TOPIC      = '/fire/robot_pose'      # 감지 순간 로봇(map) 포즈
FIRE_TARGET_MAP_TOPIC      = '/fire/target_map'      # (선택) TF로 map 변환된 화재 포인트
FIRE_APPROACH_GOAL_TOPIC   = '/fire/approach_goal'   # 복귀 목표(시각화용)

# YOLO/추론 파라미터
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

# 깊이 샘플링 파라미터
DEPTH_WIN = 5
MIN_DEPTH = 0.05
MAX_DEPTH = 20.0

# ===== 복귀(리콜) 설정 =====
RECALL_ON_FIRE     = True
RECALL_XYZ_YAW     = (1.0, 3.0, 0.0)  # (x, y, yaw) in 'map'
RECALL_COOLDOWN_S  = 10.0             # 같은 복귀를 너무 자주 트리거하지 않도록

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
        self.last_depth = None          # np.ndarray (H,W), float32 m
        self.have_depth = False
        self.fx = self.fy = self.cx = self.cy = None
        self.depth_frame_id = None
        self.have_intrinsics = False
        self.rgb_h = None
        self.rgb_w = None

        # AMCL 포즈 캐시
        self.have_amcl = False
        self.amcl_pose = None

        # Nav2
        if _HAS_NAV2:
            self.nav = BasicNavigator()
            self.nav.waitUntilNav2Active()
        else:
            self.nav = None

        # TF2
        if _HAS_TF2:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        else:
            self.tf_buffer = None
            self.tf_listener = None

        # 복귀 상태머신
        self.recall_active = False        # 복귀 시퀀스 활성화 여부
        self.recall_state  = 'idle'       # 'idle' | 'canceling' | 'sending' | 'tracking'
        self._last_recall_ns = 0          # 최근 복귀 트리거 시각(ns)
        self._recall_cancel_sent = False  # CancelAll 1회 전송 여부
        self._cancel_first_time_ns = 0    # 최초 CancelAll 시각 (그레이스 판정용)
        self._cancel_last_try_ns   = 0    # 재취소(0.5s 간격) 마지막 시각
        self.CANCEL_GRACE_SEC = 2.0       # CancelAll 이후 강제 전이 허용 지연

        # 도착 이후 그리퍼 1회만 수행
        self._gripper_done = False

        # YOLO 트리거 히스테리시스(트리거 직후 N초간 행동 유발 무시)
        self._detect_freeze_until_ns = 0

        # goal 전송/워치독
        self._last_goal_send_ns = 0
        self._goal_sent_once = False

        # 자동 재개 옵션/타이머
        self.AUTO_RESUME = bool(self.declare_parameter('auto_resume_after_recall', True).value)
        self.AUTO_RESUME_DELAY = float(self.declare_parameter('auto_resume_delay_sec', 1.0).value)
        self._resume_timer = None

        # CameraInfo 로그 관리
        self._info_logged = False
        self._last_info = None

        # ---- QoS ----
        sensor_qos = QoSProfile(depth=5)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        sensor_qos.history = HistoryPolicy.KEEP_LAST

        # 라치 QoS (TRANSIENT_LOCAL) - /auto_mode 안정 전송용
        qos_latched = QoSProfile(depth=1)
        qos_latched.reliability = ReliabilityPolicy.RELIABLE
        qos_latched.history = HistoryPolicy.KEEP_LAST
        qos_latched.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # ---- Subscribers ----
        self.sub_rgb   = self.create_subscription(Image, RGB_TOPIC,   self.cb_rgb,   sensor_qos)
        self.sub_depth = self.create_subscription(Image, DEPTH_TOPIC, self.cb_depth, sensor_qos)
        self.sub_info  = self.create_subscription(CameraInfo, INFO_TOPIC, self.cb_info, sensor_qos)
        self.sub_amcl  = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.cb_amcl, 10)

        # ---- Publishers ----
        self.pub_annot = self.create_publisher(CompressedImage, ANNOTATED_COMPRESSED_TOPIC, sensor_qos)
        self.pub_flag  = self.create_publisher(Bool, FIRE_DETECTED_TOPIC, 10)
        self.pub_pos   = self.create_publisher(PointStamped, FIRE_POSITION_TOPIC, 10)
        self.pub_robot_pose = self.create_publisher(PoseStamped, FIRE_ROBOT_POSE_TOPIC, 10)
        self.pub_goal  = self.create_publisher(PoseStamped, FIRE_APPROACH_GOAL_TOPIC, 10)
        self.pub_target_map = self.create_publisher(PointStamped, FIRE_TARGET_MAP_TOPIC, 10)

        # waypoint_follower 제어용 (라치!)
        self.pub_auto = self.create_publisher(Bool, '/auto_mode', qos_latched)

        # ---- Nav2 액션 상태(status) 구독 → 바쁜지 확인 ----
        self._busy_nav = False
        self._busy_ntp = False
        self._busy_fw  = False
        self._stat_nav_seen = False
        self._stat_ntp_seen = False
        self._stat_fw_seen  = False
        self._debug_status_logged = False

        self.sub_stat_nav = self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status', self._on_status_nav, 10)
        self.sub_stat_ntp = self.create_subscription(
            GoalStatusArray, '/navigate_through_poses/_action/status', self._on_status_ntp, 10)
        self.sub_stat_fw = self.create_subscription(
            GoalStatusArray, '/follow_waypoints/_action/status', self._on_status_fw, 10)

        # ---- CancelAll 서비스 클라이언트 ----
        self.cancel_nav = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self.cancel_ntp = self.create_client(CancelGoal, '/navigate_through_poses/_action/cancel_goal')
        self.cancel_fw  = self.create_client(CancelGoal, '/follow_waypoints/_action/cancel_goal')

        # ---- Gripper 파라미터 & 퍼블리셔 ----
        self.GRIPPER_TOPIC = self.declare_parameter('gripper_topic', '/gripper_controller/commands').value
        # 'multi' → Float64MultiArray([pos]), 'single' → Float64(pos)
        self.GRIPPER_MSG   = self.declare_parameter('gripper_msg', 'multi').value
        self.GRIPPER_OPEN  = float(self.declare_parameter('gripper_open', 0.02).value)
        self.GRIPPER_CLOSE = float(self.declare_parameter('gripper_close', -0.02).value)
        if self.GRIPPER_MSG == 'single':
            self.pub_gripper = self.create_publisher(Float64, self.GRIPPER_TOPIC, 10)
        else:
            self.pub_gripper = self.create_publisher(Float64MultiArray, self.GRIPPER_TOPIC, 10)

        # ---- Logs ----
        if _HAS_NAV2:
            self.get_logger().info('Nav2 is ready for use!')
        else:
            self.get_logger().warn('Nav2(Simple Commander) not available — recall will not navigate.')
        self.get_logger().info(f'[INFO]: RGB topic:   {RGB_TOPIC}')
        self.get_logger().info(f'[INFO]: DEPTH topic: {DEPTH_TOPIC}')
        self.get_logger().info(f'[INFO]: INFO topic:  {INFO_TOPIC}')
        self.get_logger().info(f'[INFO]: Annotated out: {ANNOTATED_COMPRESSED_TOPIC}')
        self.get_logger().info(f'[INFO]: Using device: {self.device}')
        self.get_logger().warn('[WARN]: No valid depth/intrinsics yet — 3D position will publish once ready.')

    # ---------------- Status callbacks ----------------
    def _on_status_nav(self, msg: GoalStatusArray):
        self._busy_nav = len(msg.status_list) > 0
        self._stat_nav_seen = True

    def _on_status_ntp(self, msg: GoalStatusArray):
        self._busy_ntp = len(msg.status_list) > 0
        self._stat_ntp_seen = True

    def _on_status_fw(self, msg: GoalStatusArray):
        self._busy_fw = len(msg.status_list) > 0
        self._stat_fw_seen = True

    # ---------------- CameraInfo ----------------
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

    # ---------------- Depth ----------------
    def cb_depth(self, msg: Image):
        try:
            depth = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if depth is None:
                return
            if depth.dtype == np.uint16:
                depth_m = depth.astype(np.float32) / 1000.0
            else:
                depth_m = depth.astype(np.float32)
            depth_m[~np.isfinite(depth_m)] = 0.0
            self.last_depth = depth_m
            self.have_depth = True
            if msg.header.frame_id:
                self.depth_frame_id = msg.header.frame_id
        except Exception as e:
            self.get_logger().warn(f'Failed to convert depth: {e}')

    # ---------------- AMCL pose ----------------
    def cb_amcl(self, msg: PoseWithCovarianceStamped):
        self.amcl_pose = msg
        self.have_amcl = True

    # ---------------- RGB / YOLO ----------------
    @smart_inference_mode()
    def cb_rgb(self, msg: Image):
        # 트리거 직후 잠깐은 YOLO '행동' 로직만 무시 (표시는 계속)
        now_ns = self.get_clock().now().nanoseconds
        detect_frozen = now_ns < self._detect_freeze_until_ns

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

                for *xyxy, conf, cls in reversed(det):
                    c = int(cls)
                    label = None
                    if not HIDE_LABELS:
                        label = self.names[c] if HIDE_CONF else f'{self.names[c]} {float(conf):.2f}'
                    annotator.box_label(xyxy, label, color=colors(c, True))

                    # 최고 신뢰도 1개만 3D로
                    if best_hit is None or conf.item() > best_hit[2]:
                        x1, y1, x2, y2 = [int(t.item()) for t in xyxy]
                        u = (x1 + x2) / 2.0
                        v = (y1 + y2) / 2.0
                        best_hit = (u, v, conf.item(), c, (x1, y1, x2, y2))

                    # 알림(임계값 초과 시 1회 출력) + 복귀 트리거 (히스테리시스 적용)
                    if conf.item() > CONF_NOTIFY and (not self.fire_detected_once) and (not detect_frozen):
                        self.get_logger().info(
                            "\n==============================================\n"
                            "Fire detected! Notification sent to the owner\n"
                            "==============================================\n"
                        )
                        self.fire_detected_once = True
                        if RECALL_ON_FIRE:
                            self._trigger_recall()

        # 주석 프레임 퍼블리시
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

        # 3D 좌표 계산 & 퍼블리시
        if best_hit is not None:
            fire_flag = True
            _ = self._maybe_publish_3d(best_hit, msg)

        # 복귀 상태머신 진행
        if RECALL_ON_FIRE:
            self._recall_tick()

        # 검출 플래그 퍼블리시
        self.pub_flag.publish(Bool(data=fire_flag))

        # 로컬 미리보기(선택)
        try:
            cv2.imshow("YOLO Annotated", annotated)
            cv2.waitKey(1)
        except Exception:
            pass

    # ---------------- 2D -> 3D (카메라 좌표계) ----------------
    def _maybe_publish_3d(self, best_hit, rgb_msg):
        if not (self.have_depth and self.have_intrinsics and self.last_depth is not None):
            return None

        u, v, conf, c, (x1, y1, x2, y2) = best_hit
        H, W = self.last_depth.shape[:2]
        sw = W / float(self.rgb_w or W)
        sh = H / float(self.rgb_h or H)
        uc = int(np.clip(round(u * sw), 0, W - 1))
        vc = int(np.clip(round(v * sh), 0, H - 1))

        half = DEPTH_WIN // 2
        u0, u1 = max(0, uc - half), min(W, uc + half + 1)
        v0, v1 = max(0, vc - half), min(H, vc + half + 1)
        patch = self.last_depth[v0:v1, u0:u1]

        valid = patch[(patch > MIN_DEPTH) & (patch < MAX_DEPTH) & np.isfinite(patch)]
        if valid.size == 0:
            return None
        Z = float(np.median(valid))

        X = (u - self.cx) / self.fx * Z
        Y = (v - self.cy) / self.fy * Z

        pt = PointStamped()
        pt.header.stamp = rgb_msg.header.stamp
        pt.header.frame_id = self.depth_frame_id if self.depth_frame_id else 'depth_camera_optical_frame'
        pt.point.x = X
        pt.point.y = Y
        pt.point.z = Z
        self.pub_pos.publish(pt)

        if _HAS_TF2 and self.tf_buffer is not None:
            try:
                tf = self.tf_buffer.lookup_transform('map', pt.header.frame_id, rclpy.time.Time())
                pt_map = tf2_geometry_msgs.do_transform_point(pt, tf)
                self.pub_target_map.publish(pt_map)
            except Exception:
                pass

        return pt

    # ---------------- 복귀 상태머신 ----------------
    def _trigger_recall(self):
        if not _HAS_NAV2 or self.nav is None:
            self.get_logger().warn('[RECALL] Nav2 not available — cannot navigate.')
            return

        # 기존 자동재개 타이머가 있으면 취소(중복 방지)
        if self._resume_timer is not None:
            try:
                self._resume_timer.cancel()
            except Exception:
                pass
            self._resume_timer = None

        # auto_mode 종료 → waypoint_follower 재시작 금지 (라치로 보냄)
        try:
            self.pub_auto.publish(Bool(data=False))
            self.get_logger().info('[RECALL] /auto_mode -> False (pause waypoint follower)')
        except Exception as e:
            self.get_logger().warn(f'[RECALL] failed to publish /auto_mode False: {e}')

        now_ns = self.get_clock().now().nanoseconds

        # YOLO 행동 유발 억제: 2.0초 동안 추가 트리거/취소 방지
        FREEZE_SEC = 2.0
        self._detect_freeze_until_ns = now_ns + int(FREEZE_SEC * 1e9)

        if (now_ns - self._last_recall_ns) < RECALL_COOLDOWN_S * 1e9:
            return

        # 새 리콜 시작 → 상태 초기화
        self._gripper_done = False
        self._goal_sent_once = False

        self._last_recall_ns = now_ns
        self.recall_active = True
        self.recall_state  = 'canceling'
        self._recall_cancel_sent = False
        self._debug_status_logged = False
        self.get_logger().info('[RECALL] Triggered. Cancel current navigation and return to extinguisher.')

        self._publish_robot_pose_now()

    def _recall_tick(self):
        if not self.recall_active or self.nav is None:
            return

        # 1) canceling 단계
        if self.recall_state == 'canceling':
            if not self._recall_cancel_sent:
                # 서비스 준비 대기 짧게
                for cli in (self.cancel_nav, self.cancel_ntp, self.cancel_fw):
                    try:
                        cli.wait_for_service(timeout_sec=0.1)
                    except Exception:
                        pass
                self._cancel_all_nav_goals()
                self._recall_cancel_sent = True
                now_ns = self.get_clock().now().nanoseconds
                self._cancel_first_time_ns = now_ns   # 최초 시각 (고정)
                self._cancel_last_try_ns   = now_ns   # 재시도 기준
                self.get_logger().info('[RECALL] Requested CancelAll on Nav2 action servers.')
                return

            # 디버그 1회
            if not self._debug_status_logged:
                self.get_logger().info(
                    f'[RECALL] status busy? nav={self._busy_nav}, ntp={self._busy_ntp}, fw={self._busy_fw}'
                )
                self._debug_status_logged = True

            now_ns = self.get_clock().now().nanoseconds
            elapsed_grace = (now_ns - self._cancel_first_time_ns) / 1e9 if self._cancel_first_time_ns else 0.0
            elapsed_since_try = (now_ns - self._cancel_last_try_ns) / 1e9 if self._cancel_last_try_ns else 999.0

            # 0.5s 마다 재취소 시도 → last_try만 갱신 (first는 절대 갱신 금지!)
            if (self._busy_nav or self._busy_ntp or self._busy_fw) and elapsed_since_try >= 0.5:
                self._cancel_all_nav_goals()
                self._cancel_last_try_ns = now_ns

            # 전환 조건
            all_idle_seen   = (not self._busy_nav) and (not self._busy_ntp) and (not self._busy_fw)
            no_status_topics = (not self._stat_nav_seen) and (not self._stat_ntp_seen) and (not self._stat_fw_seen)
            grace_passed    = elapsed_grace >= self.CANCEL_GRACE_SEC

            if all_idle_seen or no_status_topics or grace_passed:
                self.recall_state = 'sending'
                # 내부 상태 보정
                try:
                    self.nav.cancelTask()
                except Exception:
                    pass
            else:
                return

        # 2) sending 단계: 복귀 goal 1회 전송
        gx, gy, gyaw = RECALL_XYZ_YAW
        gz = math.sin(gyaw / 2.0)
        gw = math.cos(gyaw / 2.0)
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = gx
        goal.pose.position.y = gy
        goal.pose.orientation.z = gz
        goal.pose.orientation.w = gw

        if self.recall_state == 'sending':
            # (권장) 코스트맵 클리어
            try: self.nav.clearLocalCostmap()
            except Exception: pass
            try: self.nav.clearGlobalCostmap()
            except Exception: pass

            self.get_logger().info(f'[RECALL] Navigating to extinguisher at ({gx:.2f}, {gy:.2f}, yaw={gyaw:.1f})')
            try:
                self.nav.goToPose(goal)
                self.recall_state = 'tracking'
                self._last_goal_send_ns = self.get_clock().now().nanoseconds
                self._goal_sent_once = True
            except Exception as e:
                self.get_logger().warn(f'[RECALL] goToPose error: {e}')
                return

            # 시각화/검증용 공개
            self.pub_goal.publish(goal)

        # 3) tracking 단계: 도착 감지 → 그리퍼 시퀀스 1회 + 워치독
        elif self.recall_state == 'tracking':
            # 워치독: 3초 이상 경과했고 서버가 한가하면 1회 재전송
            now_ns = self.get_clock().now().nanoseconds
            if self._goal_sent_once and (now_ns - self._last_goal_send_ns) > int(3e9):
                if (not self._busy_nav) and (not self._busy_ntp) and (not self._busy_fw):
                    try:
                        self.get_logger().warn('[RECALL] Goal seems inactive. Resending once...')
                        self.nav.goToPose(goal)
                        self._last_goal_send_ns = now_ns
                    except Exception as e:
                        self.get_logger().warn(f'[RECALL] resend goToPose error: {e}')

            try:
                if self.nav.isTaskComplete():
                    _ = self.nav.getResult()
                    # 확실히 정지
                    try:
                        self.nav.cancelTask()
                    except Exception:
                        pass

                    if not self._gripper_done:
                        self._send_gripper(self.GRIPPER_OPEN)
                        time.sleep(0.5)
                        self._send_gripper(self.GRIPPER_CLOSE)
                        self._gripper_done = True
                        self.get_logger().info('[RECALL] Gripper sequence executed.')

                    self.get_logger().info('[RECALL] Reached extinguisher. Recall finished.')
                    self.recall_active = False
                    self.recall_state  = 'idle'

                    # 자동 순찰 재개 예약 (재개 직후 재트리거 방지 위해 freeze 추가 3초)
                    self._schedule_auto_resume(extra_freeze_sec=3.0)
            except Exception:
                pass

    # ---- CancelAll helper ----
    def _cancel_all_nav_goals(self):
        req = CancelGoal.Request()
        req.goal_info.goal_id.uuid = [0] * 16  # 빈 UUID → 가능한 goal 모두 취소
        try:
            if self.cancel_nav.service_is_ready():
                self.cancel_nav.call_async(req)
            if self.cancel_ntp.service_is_ready():
                self.cancel_ntp.call_async(req)
            if self.cancel_fw.service_is_ready():
                self.cancel_fw.call_async(req)
        except Exception as e:
            self.get_logger().warn(f'[RECALL] CancelAll call failed: {e}')

    # ---- Gripper helper ----
    def _send_gripper(self, pos: float):
        try:
            if self.GRIPPER_MSG == 'single':
                msg = Float64()
                msg.data = float(pos)
                self.pub_gripper.publish(msg)
            else:
                msg = Float64MultiArray()
                msg.data = [float(pos)]
                self.pub_gripper.publish(msg)
        except Exception as e:
            self.get_logger().warn(f'Failed to publish gripper command: {e}')

    # ---- Auto-resume helper ----
    def _schedule_auto_resume(self, extra_freeze_sec=3.0):
        """리콜 종료 후 일정 시간 뒤 /auto_mode=True 라치 퍼블리시"""
        if not self.AUTO_RESUME:
            return

        # YOLO 트리거 동결을 조금 더 늘려, 재개 직후 다시 취소되지 않도록 보호
        now_ns = self.get_clock().now().nanoseconds
        self._detect_freeze_until_ns = max(
            self._detect_freeze_until_ns,
            now_ns + int(extra_freeze_sec * 1e9)
        )

        # 기존 타이머 있으면 정리
        if self._resume_timer is not None:
            try:
                self._resume_timer.cancel()
            except Exception:
                pass
            self._resume_timer = None

        def _resume_cb():
            try:
                # 다음 화재를 위해 플래그 리셋
                self.fire_detected_once = False
                # 순찰 재개
                self.pub_auto.publish(Bool(data=True))  # 라치 QoS
                self.get_logger().info('[RECALL] Auto-resume: /auto_mode -> True (waypoint follower resumes)')
            finally:
                try:
                    self._resume_timer.cancel()
                except Exception:
                    pass

        self._resume_timer = self.create_timer(self.AUTO_RESUME_DELAY, _resume_cb)

    # ---------------- 유틸 ----------------
    def _publish_robot_pose_now(self):
        if not self.have_amcl or self.amcl_pose is None:
            self.get_logger().warn('Cannot publish /fire/robot_pose — no AMCL pose yet.')
            return
        ps = PoseStamped()
        ps.header = self.amcl_pose.header
        ps.pose   = self.amcl_pose.pose.pose
        self.pub_robot_pose.publish(ps)

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
