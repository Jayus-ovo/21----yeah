#!/usr/bin/env python3
# -- Team 2 障碍规避模块 — 二维码检测+锥桶YOLO绕行+丢线恢复逻辑 --
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from ai_msgs.msg import PerceptionTargets
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, String


class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')

        self.declare_parameter('stop_line_p', 425)
        self.declare_parameter('stop_line_qr', 160)
        self.declare_parameter('stop_line_zt', 148)

        self.declare_parameter('avoid_speed', 0.7)
        self.declare_parameter('avoid_gain', 0.0045)
        self.declare_parameter('avoid_center_x', 332.0)
        self.declare_parameter('force_right_avoid', 0)
        self.declare_parameter('point_speed', 0.6)
        self.declare_parameter('point_gain', 0.0046)
        self.declare_parameter('point_center_x', 322.0)
        self.declare_parameter('qr_speed', 0.6)
        self.declare_parameter('qr_gain', 0.0048)
        self.declare_parameter('qr_center_x', 320.0)
        self.declare_parameter('enable_qr_tracking', 1)

        self.declare_parameter('conf_threshold_p', 0.72)
        self.declare_parameter('conf_threshold_qr', 0.72)
        self.declare_parameter('conf_threshold_zt', 0.72)
        self.declare_parameter('min_frames_p', 3)
        self.declare_parameter('min_frames_qr', 3)
        self.declare_parameter('min_frames_avoid', 3)
        self.declare_parameter('lost_hold_frames_p', 15)
        self.declare_parameter('lost_hold_frames_qr', 15)
        self.declare_parameter('lost_slow_frames_p', 45)
        self.declare_parameter('lost_slow_frames_qr', 45)
        self.declare_parameter('lost_hold_speed_p', 0.7)
        self.declare_parameter('lost_hold_speed_qr', 0.7)
        self.declare_parameter('reverse_speed_p', 0.7)
        self.declare_parameter('reverse_speed_qr', 0.7)
        self.declare_parameter('reverse_slow_speed_p', 0.6)
        self.declare_parameter('reverse_slow_speed_qr', 0.6)

        self.stop_line_p = int(self.get_parameter('stop_line_p').value)
        self.stop_line_qr = int(self.get_parameter('stop_line_qr').value)
        self.stop_line_zt = int(self.get_parameter('stop_line_zt').value)
        self.avoid_speed = float(self.get_parameter('avoid_speed').value)
        self.avoid_gain = float(self.get_parameter('avoid_gain').value)
        self.avoid_center_x = float(self.get_parameter('avoid_center_x').value)
        self.force_right_avoid = (
            int(self.get_parameter('force_right_avoid').value) == 1)
        self.point_speed = float(self.get_parameter('point_speed').value)
        self.point_gain = float(self.get_parameter('point_gain').value)
        self.point_center_x = float(self.get_parameter('point_center_x').value)
        self.qr_speed = float(self.get_parameter('qr_speed').value)
        self.qr_gain = float(self.get_parameter('qr_gain').value)
        self.qr_center_x = float(self.get_parameter('qr_center_x').value)
        self.enable_qr_tracking = (
            int(self.get_parameter('enable_qr_tracking').value) == 1)
        self.conf_threshold_p = float(
            self.get_parameter('conf_threshold_p').value)
        self.conf_threshold_qr = float(
            self.get_parameter('conf_threshold_qr').value)
        self.conf_threshold_zt = float(
            self.get_parameter('conf_threshold_zt').value)
        self.min_frames_p = max(
            1, int(self.get_parameter('min_frames_p').value))
        self.min_frames_qr = max(
            1, int(self.get_parameter('min_frames_qr').value))
        self.min_frames_avoid = max(
            1, int(self.get_parameter('min_frames_avoid').value))
        self.lost_hold_frames_p = max(
            0, int(self.get_parameter('lost_hold_frames_p').value))
        self.lost_hold_frames_qr = max(
            0, int(self.get_parameter('lost_hold_frames_qr').value))
        self.lost_slow_frames_p = max(
            self.lost_hold_frames_p,
            int(self.get_parameter('lost_slow_frames_p').value))
        self.lost_slow_frames_qr = max(
            self.lost_hold_frames_qr,
            int(self.get_parameter('lost_slow_frames_qr').value))
        self.lost_hold_speed_p = max(
            0.0, float(self.get_parameter('lost_hold_speed_p').value))
        self.lost_hold_speed_qr = max(
            0.0, float(self.get_parameter('lost_hold_speed_qr').value))
        self.reverse_speed_p = max(
            0.0, float(self.get_parameter('reverse_speed_p').value))
        self.reverse_speed_qr = max(
            0.0, float(self.get_parameter('reverse_speed_qr').value))
        self.reverse_slow_speed_p = max(
            0.0, float(self.get_parameter('reverse_slow_speed_p').value))
        self.reverse_slow_speed_qr = max(
            0.0, float(self.get_parameter('reverse_slow_speed_qr').value))

        qos_best = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        qos_persist = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.detection_sub = self.create_subscription(
            PerceptionTargets,
            '/perception_yolo_detection',
            self._on_detection,
            qos_best,
        )
        self.qr_sub = self.create_subscription(
            Int32, '/barcode_id', self._on_qr_received, 10)
        self.qr_done_sub = self.create_subscription(
            Int32,
            '/barcode_action_done',
            self._on_qr_completed,
            qos_persist,
        )
        self.track_state_sub = self.create_subscription(
            String,
            '/resnet_mode',
            self._on_track_state,
            qos_persist,
        )
        self.point_pub = self.create_publisher(Int32, '/halt_cmd', 10)
        self.avoid_state_pub = self.create_publisher(Int32, '/hazard_active', 10)
        self.cmd_pub = self.create_publisher(
            Twist, '/perception_cmd', qos_best)
        self.visual_state_pub = self.create_publisher(
            String, '/detection_mode', qos_persist)

        self.current_state = 'idle'
        self.track_state = 'yellow'
        self.locked_target = None
        self.p_valid_frames = 0
        self.qr_valid_frames = 0
        self.p_lost_frames = 0
        self.qr_lost_frames = 0
        self.qr_code_value = 0
        self.qr_action_done = False

        self.p_error_history = 0.0
        self.qr_error_history = 0.0
        self.last_p_angular = 0.0
        self.last_qr_angular = 0.0
        self.best_p_angular = 0.0
        self.best_qr_angular = 0.0
        self.avoid_error_history = 0.0
        self.avoid_frame_counter = 0
        self.avoid_direction = 0
        self.avoidance_level = 0

        self._publish_state()

        self.get_logger().info(
            '视觉避障模块已就绪 '
            f'(定点: v={self.point_speed:.2f}, kp={self.point_gain:.4f}; '
            f'码标: v={self.qr_speed:.2f}, '
            f'kp={self.qr_gain:.4f}, '
            f'追踪={int(self.enable_qr_tracking)}; '
            f'绕行: v={self.avoid_speed:.2f}, kp={self.avoid_gain:.4f}, '
            f'x={self.avoid_center_x:.1f}, '
            f'强制右转={int(self.force_right_avoid)}; '
            f'丢失: 保持={self.lost_hold_frames_p}/{self.lost_hold_frames_qr}, '
            f'减速={self.lost_slow_frames_p}/{self.lost_slow_frames_qr})'
        )

    @staticmethod
    def find_best_region(msg, target_type, confidence_threshold):
        best = None
        max_area_val = 0.0
        for target in msg.targets:
            if target.type != target_type:
                continue
            for roi in target.rois:
                if roi.confidence < confidence_threshold:
                    continue
                y_bottom = roi.rect.y_offset + roi.rect.height
                if not ((120 - 1) <= y_bottom <= (480 - 1)):
                    continue
                area = float(roi.rect.width * roi.rect.height)
                if area > max_area_val:
                    max_area_val = area
                    best = roi
        return best

    @staticmethod
    def region_position(roi):
        y_bottom = float(roi.rect.y_offset + roi.rect.height)
        center_x = float(roi.rect.x_offset + roi.rect.width / 2.0)
        return y_bottom, center_x

    @staticmethod
    def apply_low_pass(raw_error, history):
        if abs(raw_error) <= 3.0:
            return 0.0
        return raw_error * 0.7 + history * 0.3

    def _set_state(self, state_name):
        if state_name == self.current_state:
            return
        self.current_state = state_name
        self.p_error_history = 0.0
        self.qr_error_history = 0.0
        self._publish_state()

    def _publish_state(self):
        msg = String()
        msg.data = self.current_state
        self.visual_state_pub.publish(msg)

    def _send_velocity(self, linear_v, angular_v):
        cmd = Twist()
        cmd.linear.x = float(linear_v)
        cmd.angular.z = float(max(-5.0, min(5.0, angular_v)))
        self.cmd_pub.publish(cmd)

    def _set_avoidance_level(self, level):
        self.avoidance_level = int(level)
        msg = Int32()
        msg.data = self.avoidance_level
        self.avoid_state_pub.publish(msg)

    def _reset_avoidance(self):
        self.avoid_frame_counter = 0
        self.avoid_direction = 0
        self.avoid_error_history = 0.0

    def _track_point(self, center_x):
        self._set_state('track_p')
        error_raw = self.point_center_x - center_x
        error = self.apply_low_pass(error_raw, self.p_error_history)
        self.p_error_history = error
        self.last_p_angular = error * self.point_gain
        if abs(self.last_p_angular) > 1e-6:
            self.best_p_angular = self.last_p_angular
        self._send_velocity(self.point_speed, self.last_p_angular)

    def _track_qr(self, center_x):
        self._set_state('track_qrcode')
        error_raw = self.qr_center_x - center_x
        error = self.apply_low_pass(error_raw, self.qr_error_history)
        self.qr_error_history = error
        self.last_qr_angular = error * self.qr_gain
        if abs(self.last_qr_angular) > 1e-6:
            self.best_qr_angular = self.last_qr_angular
        self._send_velocity(self.qr_speed, self.last_qr_angular)

    def _on_qr_received(self, msg: Int32):
        if msg.data not in (3, 4) or self.qr_code_value in (3, 4):
            return
        self.qr_code_value = msg.data
        if self.enable_qr_tracking:
            self.get_logger().info(
                f'二维码 {self.qr_code_value} 已缓存，追踪至停车阈值')
        else:
            self.get_logger().info(
                f'二维码 {self.qr_code_value} 已缓存，等待主线到达停车点')

    def _on_qr_completed(self, msg: Int32):
        if msg.data not in (3, 4):
            return
        self.qr_code_value = msg.data
        self.qr_action_done = True
        self.locked_target = None
        self.qr_lost_frames = 0
        self._set_state('idle')

    def _on_track_state(self, msg: String):
        if msg.data not in {'yellow', 'stop'}:
            self.get_logger().warn('忽略未知循迹状态')
            return
        if msg.data == self.track_state:
            return

        self.track_state = msg.data
        self._reset_avoidance()
        self.get_logger().info(
            '循迹状态已更新，绕行方向已重置')

    def _handle_target_loss(self, target_type):
        if target_type == 'p':
            self._set_state('track_p')
            if self.p_lost_frames <= self.lost_hold_frames_p:
                self._send_velocity(
                    self.lost_hold_speed_p, self.best_p_angular)
            else:
                if self.p_lost_frames == self.lost_hold_frames_p + 1:
                    self.get_logger().warn('定点持续丢失，启动回转搜索')
                reverse_ang = -self.best_p_angular * 4.0
                if self.p_lost_frames <= self.lost_slow_frames_p:
                    self._send_velocity(self.reverse_speed_p, reverse_ang)
                else:
                    if self.p_lost_frames == self.lost_slow_frames_p + 1:
                        self.get_logger().warn('定点持续丢失，降低回转速度')
                    self._send_velocity(
                        self.reverse_slow_speed_p, reverse_ang)
            return

        self._set_state('track_qrcode')
        if self.qr_lost_frames <= self.lost_hold_frames_qr:
            self._send_velocity(
                self.lost_hold_speed_qr, self.best_qr_angular)
        else:
            if self.qr_lost_frames == self.lost_hold_frames_qr + 1:
                self.get_logger().warn('二维码持续丢失，启动回转搜索')
            reverse_ang = -self.best_qr_angular * 4.0
            if self.qr_lost_frames <= self.lost_slow_frames_qr:
                self._send_velocity(
                    self.reverse_speed_qr, reverse_ang)
            else:
                if self.qr_lost_frames == self.lost_slow_frames_qr + 1:
                    self.get_logger().warn('二维码持续丢失，降低回转速度')
                self._send_velocity(
                    self.reverse_slow_speed_qr, reverse_ang)

    def _avoid_obstacle(
            self, center_zt, center_p, center_qr,
            p_locked, qr_locked):
        self._set_state('avoid')

        if self.avoid_frame_counter <= 0:
            force_right = self.force_right_avoid
            if force_right:
                direction = 1
            elif p_locked:
                direction = -1 if center_p <= center_zt else 1
            elif qr_locked:
                direction = (
                    -1 if center_qr <= center_zt else 1)
            else:
                direction = (
                    -1 if center_zt >= self.avoid_center_x else 1)
            self.avoid_direction = direction
            self.avoid_frame_counter = self.min_frames_avoid - 1
        else:
            direction = self.avoid_direction
            self.avoid_frame_counter -= 1

        if direction == -1:
            raw_error = (640.0 - 1.0) - center_zt
        else:
            raw_error = 0.0 - center_zt

        filtered = self.apply_low_pass(raw_error, self.avoid_error_history)
        self.avoid_error_history = filtered
        self._set_avoidance_level(5)
        self._send_velocity(self.avoid_speed, filtered * self.avoid_gain)

    def _on_detection(self, msg: PerceptionTargets):
        try:
            best_p = self.find_best_region(msg, 'p', self.conf_threshold_p)
            best_qr = self.find_best_region(
                msg, 'qrcode', self.conf_threshold_qr)
            best_zt = self.find_best_region(msg, 'zt', self.conf_threshold_zt)

            y_p = cx_p = 0.0
            y_qr = cx_qr = 0.0
            y_zt = cx_zt = 0.0

            if best_p is not None:
                y_p, cx_p = self.region_position(best_p)
                self.p_lost_frames = 0
                if self.locked_target is None:
                    self.p_valid_frames = min(
                        self.min_frames_p, self.p_valid_frames + 1)
                elif self.locked_target != 'p':
                    self.p_valid_frames = 0
            elif self.locked_target == 'p':
                self.p_lost_frames += 1
            else:
                self.p_valid_frames = 0

            if best_qr is not None:
                y_qr, cx_qr = self.region_position(best_qr)
                self.qr_lost_frames = 0
                if self.locked_target is None:
                    self.qr_valid_frames = min(
                        self.min_frames_qr, self.qr_valid_frames + 1)
                elif self.locked_target != 'qrcode':
                    self.qr_valid_frames = 0
            elif self.locked_target == 'qrcode':
                self.qr_lost_frames += 1
            else:
                self.qr_valid_frames = 0

            if best_zt is not None:
                y_zt, cx_zt = self.region_position(best_zt)

            p_confirmed = self.p_valid_frames >= self.min_frames_p
            qr_confirmed = (
                self.qr_valid_frames >= self.min_frames_qr)

            if self.locked_target is None:
                if p_confirmed:
                    self.locked_target = 'p'
                    self.p_lost_frames = 0
                    self.get_logger().info('定点连续检测达标，锁定追踪')
                elif (self.enable_qr_tracking and qr_confirmed and
                      not self.qr_action_done):
                    self.locked_target = 'qrcode'
                    self.qr_lost_frames = 0
                    self.get_logger().info(
                        '二维码连续检测达标，锁定追踪')

            if self.current_state == 'stop_p':
                msg_stop = Int32()
                msg_stop.data = 1
                self.point_pub.publish(msg_stop)
                self._set_avoidance_level(0)
                self._send_velocity(0.0, 0.0)
                return

            if self.current_state == 'stop_qrcode':
                self._set_avoidance_level(0)
                self._send_velocity(0.0, 0.0)
                return

            if (self.locked_target == 'p' and best_p is not None and
                    y_p >= self.stop_line_p):
                self._set_state('stop_p')
                self._reset_avoidance()
                msg_stop = Int32()
                msg_stop.data = 1
                self.point_pub.publish(msg_stop)
                self._send_velocity(0.0, 0.0)
                self._set_avoidance_level(0)
                self.get_logger().info(f'定点到达停车线: y={y_p:.1f}')
                return

            qr_ready = (
                self.locked_target == 'qrcode' or
                (not self.enable_qr_tracking and
                 self.locked_target is None and qr_confirmed)
            )
            if (qr_ready and best_qr is not None and
                    y_qr >= self.stop_line_qr):
                self._set_state('stop_qrcode')
                self._reset_avoidance()
                self._set_avoidance_level(0)
                self._send_velocity(0.0, 0.0)
                self.get_logger().info(
                    f'二维码到达停车线: y={y_qr:.1f}, '
                    f'追踪={int(self.enable_qr_tracking)}')
                return

            zt_detected = best_zt is not None and y_zt >= self.stop_line_zt
            if zt_detected:
                self._avoid_obstacle(
                    cx_zt,
                    cx_p,
                    cx_qr,
                    best_p is not None and (
                        p_confirmed or self.locked_target == 'p'),
                    best_qr is not None and (
                        qr_confirmed or self.locked_target == 'qrcode'),
                )
                return

            self._reset_avoidance()
            self._set_avoidance_level(0)

            if self.locked_target == 'p':
                msg_p = Int32()
                msg_p.data = 2
                self.point_pub.publish(msg_p)
                if best_p is not None:
                    self._track_point(cx_p)
                else:
                    self._handle_target_loss('p')
                return

            if self.qr_action_done:
                self._set_state('idle')
                return

            if self.locked_target == 'qrcode':
                if best_qr is not None:
                    self._track_qr(cx_qr)
                else:
                    self._handle_target_loss('qrcode')
                return

            self._set_state('idle')

        except Exception as exc:
            self.get_logger().error(f'视觉处理异常: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
