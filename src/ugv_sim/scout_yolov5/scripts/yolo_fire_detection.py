#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys, math, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Bool, Float64, Float64MultiArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal

from cv_bridge import CvBridge
import cv2, numpy as np
from pathlib import Path
import torch

# --- YOLOv5 internals ---
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

bridge = CvBridge()

# --- Topics ---
RGB_TOPIC   = '/camera/image_raw'
INFO_TOPIC  = '/depth_camera/depth/camera_info'

ANNOTATED_COMPRESSED_TOPIC = '/fire/annotated_image/compressed'
FIRE_DETECTED_TOPIC        = '/fire/detected'
FIRE_ROBOT_POSE_TOPIC      = '/fire/robot_pose'
FIRE_APPROACH_GOAL_TOPIC   = '/fire/approach_goal'
RECALL_ACTIVE_TOPIC        = '/recall_active'          # NEW: 리콜 상태(라치)
NAV_WAKE_TOPIC             = '/cmd_vel_nav'            # cmd_vel 스위치 웨이크업용

# --- YOLO params ---
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

# --- Recall params ---
RECALL_ON_FIRE     = True
RECALL_XYZ_YAW     = (13, 3.54, 0.0)   # map 좌표 (x,y,yaw)
RECALL_COOLDOWN_S  = 10.0

# 웨이크업(joy→nav 선택 유도)
NAV_WAKE_SEC       = 1.5     # 이 시간 동안 20Hz로 zero twist 발행

class FireNode(Node):
    def __init__(self):
        super().__init__('yolo_fire_detection')

        # Model
        self.device = select_device('')
        self.model = DetectMultiBackend(WEIGHTS, device=self.device, dnn=False, data=DATA_CFG, fp16=USE_FP16)
        stride, self.names, pt = self.model.stride, self.model.names, self.model.pt
        imgsz = check_img_size(IMG_SIZE, s=stride)
        self.model.warmup(imgsz=(1 if pt or self.model.triton else 1, 3, *imgsz))

        # State
        self.fire_detected_once = False
        self.rgb_h = self.rgb_w = None
        self.have_amcl = False
        self.amcl_pose = None

        # Recall FSM
        self.recall_active = False
        self.recall_state  = 'idle'
        self._last_recall_ns = 0
        self._recall_cancel_sent = False
        self._cancel_first_time_ns = 0
        self._cancel_last_try_ns   = 0
        self.CANCEL_GRACE_SEC = 2.0

        self._last_goal_send_ns = 0
        self._goal_sent_once = False

        # 히스테리시스: 트리거 직후 YOLO 행동 억제
        self._detect_freeze_until_ns = 0

        # Auto resume (요청: 화재 때도 True 유지)
        self.AUTO_RESUME = True
        self.AUTO_RESUME_DELAY = 1.0
        self._resume_timer = None

        # QoS
        sensor_qos = QoSProfile(depth=5)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        sensor_qos.history = HistoryPolicy.KEEP_LAST

        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.history = HistoryPolicy.KEEP_LAST
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # Subs
        self.sub_rgb  = self.create_subscription(Image, RGB_TOPIC, self.cb_rgb, sensor_qos)
        self.sub_info = self.create_subscription(CameraInfo, INFO_TOPIC, self.cb_info, sensor_qos)
        self.sub_amcl = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.cb_amcl, 10)

        # Pubs
        self.pub_annot = self.create_publisher(CompressedImage, ANNOTATED_COMPRESSED_TOPIC, sensor_qos)
        self.pub_fire  = self.create_publisher(Bool, FIRE_DETECTED_TOPIC, 10)
        self.pub_auto  = self.create_publisher(Bool, '/auto_mode', latched_qos)     # 라치
        self.pub_goal  = self.create_publisher(PoseStamped, FIRE_APPROACH_GOAL_TOPIC, 10)
        self.pub_robot_pose = self.create_publisher(PoseStamped, FIRE_ROBOT_POSE_TOPIC, 10)
        self.pub_recall = self.create_publisher(Bool, RECALL_ACTIVE_TOPIC, latched_qos)  # NEW 라치
        self.pub_nav_wake = self.create_publisher(Twist, NAV_WAKE_TOPIC, 10)             # NEW 웨이크업

        # Nav2
        if _HAS_NAV2:
            self.nav = BasicNavigator()
            self.nav.waitUntilNav2Active()
            self.get_logger().info('Nav2 is ready for use!')
        else:
            self.nav = None
            self.get_logger().warn('Nav2(Simple Commander) not available — recall disabled.')

        # Nav2 status + cancel services
        self._busy_nav = self._busy_ntp = self._busy_fw = False
        self._stat_nav_seen = self._stat_ntp_seen = self._stat_fw_seen = False
        self._debug_status_logged = False

        self.sub_stat_nav = self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self._on_status_nav, 10)
        self.sub_stat_ntp = self.create_subscription(GoalStatusArray, '/navigate_through_poses/_action/status', self._on_status_ntp, 10)
        self.sub_stat_fw  = self.create_subscription(GoalStatusArray, '/follow_waypoints/_action/status', self._on_status_fw, 10)

        self.cancel_nav = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self.cancel_ntp = self.create_client(CancelGoal, '/navigate_through_poses/_action/cancel_goal')
        self.cancel_fw  = self.create_client(CancelGoal, '/follow_waypoints/_action/cancel_goal')

        # nav-wake timer
        self._wake_timer = None
        self._wake_end_ns = 0

        self.get_logger().info(f'[INFO] RGB: {RGB_TOPIC}')
        self.get_logger().info(f'[INFO] INFO: {INFO_TOPIC}')
        self.get_logger().info(f'[INFO] Annotated: {ANNOTATED_COMPRESSED_TOPIC}')
        self.get_logger().info(f'[INFO] Using device: {self.device}')

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

    # ---------------- CameraInfo / AMCL ----------------
    def cb_info(self, msg: CameraInfo):
        self.rgb_h = None  # keep minimal

    def cb_amcl(self, msg: PoseWithCovarianceStamped):
        self.have_amcl = True
        self.amcl_pose = msg

    # ---------------- RGB / YOLO ----------------
    @smart_inference_mode()
    def cb_rgb(self, msg: Image):
        now_ns = self.get_clock().now().nanoseconds
        detect_frozen = now_ns < self._detect_freeze_until_ns

        img0 = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.rgb_h, self.rgb_w = img0.shape[:2]

        # Preprocess
        img = img0[..., ::-1]
        img = np.transpose(img, (2, 0, 1))
        img = np.expand_dims(img, 0)
        img = np.ascontiguousarray(img)
        img = torch.from_numpy(img).to(self.model.device)
        img = img.half() if self.model.fp16 else img.float()
        img /= 255.0

        # Inference + NMS
        pred = self.model(img, augment=False, visualize=False)
        pred = non_max_suppression(pred, CONF_THRES, IOU_THRES, classes=None, agnostic=False, max_det=MAX_DET)

        annotator = Annotator(img0, line_width=LINE_THICKNESS, example=str(self.names))
        fire_flag = False

        for det in pred:
            if len(det):
                det[:, :4] = scale_boxes(img.shape[2:], det[:, :4], img0.shape).round()
                for *xyxy, conf, cls in reversed(det):
                    c = int(cls)
                    label = self.names[c] if HIDE_CONF else f'{self.names[c]} {float(conf):.2f}'
                    if HIDE_LABELS: label = None
                    annotator.box_label(xyxy, label, color=colors(c, True))

                    if conf.item() > CONF_NOTIFY:
                        fire_flag = True
                        if not self.fire_detected_once and not detect_frozen:
                            self.get_logger().info(
                                "\n==============================================\n"
                                "Fire detected! Notification sent to the owner\n"
                                "==============================================\n"
                            )
                            self.fire_detected_once = True
                            if RECALL_ON_FIRE:
                                self._trigger_recall()

        # publish fire flag
        self.pub_fire.publish(Bool(data=fire_flag))

        # annotate publish
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
            self.get_logger().warn(f'encode/publish annotated failed: {e}')

        # recall FSM
        if RECALL_ON_FIRE:
            self._recall_tick()

        # 로컬 미리보기 (옵션)
        try:
            cv2.imshow("YOLO Annotated", annotated); cv2.waitKey(1)
        except Exception:
            pass

    # ---------------- Recall FSM ----------------
    def _trigger_recall(self):
        if self.nav is None:
            self.get_logger().warn('[RECALL] Nav2 not available — skip.')
            return

        # 요청사항: 화재 감지 시에도 자동 모드 True 유지(사용자 babysit 방지)
        self.pub_auto.publish(Bool(data=True))

        now_ns = self.get_clock().now().nanoseconds
        self._detect_freeze_until_ns = now_ns + int(2.0 * 1e9)  # 2초 억제

        if (now_ns - self._last_recall_ns) < RECALL_COOLDOWN_S * 1e9:
            return

        # 상태 초기화
        self._last_recall_ns = now_ns
        self.recall_active = True
        self.recall_state  = 'canceling'
        self._recall_cancel_sent = False
        self._debug_status_logged = False
        self._last_goal_send_ns = 0
        self._goal_sent_once = False
        self.get_logger().info('[RECALL] Triggered. CancelAll then return to (1,3,0).')

        # 리콜 상태 알림 (라치 True)
        self.pub_recall.publish(Bool(data=True))

        # 감지 순간 로봇 포즈 공개
        self._publish_robot_pose()

    def _recall_tick(self):
        if not self.recall_active or self.nav is None:
            return

        # 1) canceling
        if self.recall_state == 'canceling':
            if not self._recall_cancel_sent:
                for cli in (self.cancel_nav, self.cancel_ntp, self.cancel_fw):
                    try: cli.wait_for_service(timeout_sec=0.1)
                    except Exception: pass
                self._cancel_all()
                self._recall_cancel_sent = True
                self._cancel_first_time_ns = self.get_clock().now().nanoseconds
                self._cancel_last_try_ns   = self._cancel_first_time_ns
                self.get_logger().info('[RECALL] CancelAll requested.')
                return

            if not self._debug_status_logged:
                self.get_logger().info(f'[RECALL] status busy? nav={self._busy_nav}, ntp={self._busy_ntp}, fw={self._busy_fw}')
                self._debug_status_logged = True

            now_ns = self.get_clock().now().nanoseconds
            elapsed_grace = (now_ns - self._cancel_first_time_ns)/1e9 if self._cancel_first_time_ns else 0.0
            elapsed_since_try = (now_ns - self._cancel_last_try_ns)/1e9 if self._cancel_last_try_ns else 999.0

            if (self._busy_nav or self._busy_ntp or self._busy_fw) and elapsed_since_try >= 0.5:
                self._cancel_all()
                self._cancel_last_try_ns = now_ns

            all_idle_seen = (not self._busy_nav) and (not self._busy_ntp) and (not self._busy_fw)
            no_status_topics = (not self._stat_nav_seen) and (not self._stat_ntp_seen) and (not self._stat_fw_seen)
            grace_passed = elapsed_grace >= self.CANCEL_GRACE_SEC

            if all_idle_seen or no_status_topics or grace_passed:
                self.recall_state = 'sending'
                try: self.nav.cancelTask()
                except Exception: pass
            else:
                return

        # 2) sending
        gx, gy, gyaw = RECALL_XYZ_YAW
        gz, gw = math.sin(gyaw/2.0), math.cos(gyaw/2.0)

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = gx
        goal.pose.position.y = gy
        goal.pose.orientation.z = gz
        goal.pose.orientation.w = gw

        if self.recall_state == 'sending':
            try: self.nav.clearLocalCostmap()
            except Exception: pass
            try: self.nav.clearGlobalCostmap()
            except Exception: pass

            self.get_logger().info(f'[RECALL] goToPose -> ({gx:.2f}, {gy:.2f}, yaw={gyaw:.1f})')
            try:
                self.nav.goToPose(goal)
                self.recall_state = 'tracking'
                self._last_goal_send_ns = self.get_clock().now().nanoseconds
                self._goal_sent_once = True
            except Exception as e:
                self.get_logger().warn(f'[RECALL] goToPose error: {e}')
                return

            self.pub_goal.publish(goal)

            # ★ 웨이크업: joy가 0값을 계속 쏘아도 nav가 선택되도록 NAV_WAKE_SEC 동안 20Hz zero twist 발행
            self._start_nav_wake()

        # 3) tracking
        elif self.recall_state == 'tracking':
            # 필요 시 완료 감지 후 자동 재개
            try:
                if self.nav.isTaskComplete():
                    _ = self.nav.getResult()
                    try: self.nav.cancelTask()
                    except Exception: pass

                    self.get_logger().info('[RECALL] Reached goal. Recall finished.')
                    self.recall_active = False
                    self.recall_state  = 'idle'
                    self.pub_recall.publish(Bool(data=False))  # 리콜 종료(라치 False)

                    # 다음 화재 다시 트리거 가능
                    self._schedule_auto_resume()
            except Exception:
                pass

    # ---- helpers ----
    def _cancel_all(self):
        req = CancelGoal.Request(); req.goal_info.goal_id.uuid = [0]*16
        try:
            if self.cancel_nav.service_is_ready(): self.cancel_nav.call_async(req)
            if self.cancel_ntp.service_is_ready(): self.cancel_ntp.call_async(req)
            if self.cancel_fw.service_is_ready():  self.cancel_fw.call_async(req)
        except Exception as e:
            self.get_logger().warn(f'[RECALL] CancelAll call failed: {e}')

    def _publish_robot_pose(self):
        if not self.have_amcl or self.amcl_pose is None:
            self.get_logger().warn('No AMCL pose yet — /fire/robot_pose skipped.')
            return
        ps = PoseStamped()
        ps.header = self.amcl_pose.header
        ps.pose   = self.amcl_pose.pose.pose
        self.pub_robot_pose.publish(ps)

    def _schedule_auto_resume(self):
        # 화재 한 번 감지 후엔 더 이상 재감지하지 않도록 플래그 유지
        # (다시 순찰 중 새 화재가 나오면 그때 새로 트리거)
        def _resume_cb():
            try:
                self.fire_detected_once = False
                self.pub_auto.publish(Bool(data=True))  # 순찰 재개 신호(라치)
                self.get_logger().info('[RECALL] Auto-resume: /auto_mode -> True (waypoints resume)')
            finally:
                try: self._resume_timer.cancel()
                except Exception: pass

        if self._resume_timer is not None:
            try: self._resume_timer.cancel()
            except Exception: pass
        self._resume_timer = self.create_timer(self.AUTO_RESUME_DELAY, _resume_cb)

    # ---- NAV wake 기능 ----
    def _start_nav_wake(self):
        # NAV_WAKE_SEC 동안 20Hz로 zero Twist를 /cmd_vel_nav로 발행하여 스위치를 Nav로 전환
        self._wake_end_ns = self.get_clock().now().nanoseconds + int(NAV_WAKE_SEC * 1e9)
        if self._wake_timer is None:
            self._wake_timer = self.create_timer(0.05, self._nav_wake_tick)  # 20Hz

    def _nav_wake_tick(self):
        now = self.get_clock().now().nanoseconds
        if now >= self._wake_end_ns:
            try:
                self._wake_timer.cancel()
            except Exception:
                pass
            self._wake_timer = None
            return
        self.pub_nav_wake.publish(Twist())  # zero twist

# ---------------- main ----------------
def main():
    rclpy.init(args=None)
    node = FireNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        try: cv2.destroyAllWindows()
        except Exception: pass

if __name__ == '__main__':
    main()
