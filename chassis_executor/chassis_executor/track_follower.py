#!/usr/bin/env python3

import rclpy
import time
import threading
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from geometry_msgs.msg import Twist
from ai_msgs.msg import PerceptionTargets
from std_msgs.msg import Int32, String


class TrackFollower(Node):

    def __init__(self):
        super().__init__('track_follower')

        self.declare_parameter('yellow_speed', 0.7)
        self.declare_parameter('yellow_gain', 0.0061)
        self.declare_parameter('center_yellow', 320.0)
        self.declare_parameter('track_topic', 'racing_track_center_detection_back')

        self.yellow_speed = float(self.get_parameter('yellow_speed').value)
        self.yellow_gain = float(self.get_parameter('yellow_gain').value)
        self.center_yellow = float(self.get_parameter('center_yellow').value)
        self.track_topic = str(self.get_parameter('track_topic').value)

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        qos_persistent = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.thread_lock = threading.Lock()
        self.avoidance_level = 0
        self.target_x = 0.0
        self.last_error = 0.0
        self.is_stopped = False
        self.qr_code_id = 0

        # ---- 黄色中线（唯一循迹源） ----
        self.sub_track = self.create_subscription(
            PerceptionTargets,
            self.track_topic,
            self._process_track_data,
            qos_profile
        )

        # ---- QR / 绕障 / 停车 ----
        self.sub_qr = self.create_subscription(
            Int32, '/barcode_id', self._on_qr_number, 10)
        self.sub_avoid = self.create_subscription(
            Int32, '/hazard_active', self._on_avoid_signal, 10)
        self.sub_point = self.create_subscription(
            Int32, '/halt_cmd', self._on_point_signal, 10)

        # ---- 输出 ----
        self.cmd_pub = self.create_publisher(
            Twist, '/track_cmd', qos_profile)
        self.control_mode_pub = self.create_publisher(
            String, '/resnet_mode', qos_persistent)

        self._publish_control_mode('yellow')
        self.get_logger().info(
            f'循迹模块已启动 (yellow: v={self.yellow_speed}, '
            f'kp={self.yellow_gain}, center={self.center_yellow:.1f}, '
            f'topic={self.track_topic})')

    def _publish_control_mode(self, mode):
        msg = String()
        msg.data = mode
        self.control_mode_pub.publish(msg)

    def _on_point_signal(self, msg: Int32):
        if msg.data == 1 and not self.is_stopped:
            self.is_stopped = True
            self._publish_control_mode('stop')
            self.get_logger().info("循迹输出已暂停")

    def _on_qr_number(self, msg: Int32):
        if msg.data not in (3, 4):
            return
        self.qr_code_id = msg.data
        direction = '顺时针' if msg.data == 3 else '逆时针'
        self.get_logger().info(f'qr={msg.data} ({direction})')

    def _on_avoid_signal(self, msg: Int32):
        with self.thread_lock:
            if msg.data == 0 and self.avoidance_level == 0:
                self.avoidance_level = 0
            elif msg.data >= self.avoidance_level:
                self.avoidance_level = 5

    def _process_track_data(self, msg: PerceptionTargets):
        if self.is_stopped:
            return
        try:
            if not msg.targets:
                return
            target = msg.targets[0]
            if not target.points:
                return
            point_group = target.points[0]
            if not point_group.point:
                return
            self.target_x = float(point_group.point[0].x)
            self._compute_and_publish()
        except Exception as e:
            self.get_logger().warn(f"解析追踪坐标出错: {e}")

    def _compute_and_publish(self):
        raw_error = self.center_yellow - self.target_x

        if abs(raw_error) <= 3.0:
            filtered_error = 0.0
            self.last_error = 0.0
        else:
            filtered_error = raw_error * 0.7 + self.last_error * 0.3
            self.last_error = filtered_error

        angular_vel = filtered_error * self.yellow_gain

        with self.thread_lock:
            if self.avoidance_level > 0:
                angular_vel *= 0.5
                self.avoidance_level -= 1

        angular_vel = max(-5.0, min(5.0, angular_vel))

        output = Twist()
        output.linear.x = float(self.yellow_speed)
        output.angular.z = float(angular_vel)
        self.cmd_pub.publish(output)

    def destroy_node(self):
        self.is_stopped = True
        self._publish_control_mode('stop')
        time.sleep(0.05)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TrackFollower()
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
