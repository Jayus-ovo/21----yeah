#!/usr/bin/env python3
"""2队路线指挥器：使用里程累计和通道入口复检确认完整一圈。"""
import math
import time
from dataclasses import dataclass
from enum import Enum

import rclpy
from ai_msgs.msg import PerceptionTargets
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32, String


class Phase(Enum):
    ACQUIRE_ORDER = 'acquire_order'
    BRANCH_TO_B = 'branch_to_b'
    CROSS_GATE_IN = 'cross_gate_in'
    ACCUMULATE_LAP = 'accumulate_lap'
    CROSS_GATE_OUT = 'cross_gate_out'
    HOMEWARD = 'homeward'
    COMPLETE = 'complete'
    SAFETY_STOP = 'safety_stop'


@dataclass
class Pose2:
    x: float
    y: float


class MissionCoordinator(Node):
    def __init__(self):
        super().__init__('bravo_route_conductor')
        for name, default in (
            ('match_limit_sec', 180.0), ('lap_distance_min', 2.5),
            ('home_radius', 0.18), ('input_timeout', 0.25),
            ('direction_turn_sec', 1.10), ('direction_turn_rate', 0.72)):
            self.declare_parameter(name, default)
        self.limit = float(self.get_parameter('match_limit_sec').value)
        self.lap_min = float(self.get_parameter('lap_distance_min').value)
        self.home_radius = float(self.get_parameter('home_radius').value)
        self.timeout = float(self.get_parameter('input_timeout').value)
        self.turn_sec = float(self.get_parameter('direction_turn_sec').value)
        self.turn_rate = float(self.get_parameter('direction_turn_rate').value)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.snap_pub = self.create_publisher(Int32, '/snapshot_cmd', 10)
        self.phase_pub = self.create_publisher(String, '/bravo_route_phase', 10)
        self.elapsed_pub = self.create_publisher(Float32, '/bravo_elapsed', 10)
        self.speech_pub = self.create_publisher(String, '/speech_cmd', 10)
        self.done_pub = self.create_publisher(Bool, '/mission_completed', 10)
        self.create_subscription(Twist, '/track_cmd', self._track, 10)
        self.create_subscription(Twist, '/perception_cmd', self._avoid, 10)
        self.create_subscription(Int32, '/hazard_active', self._hazard, 10)
        self.create_subscription(Int32, '/barcode_id', self._order, 10)
        self.create_subscription(Int32, '/halt_cmd', self._halt_mark, 10)
        self.create_subscription(String, '/scene_caption', self._caption, 10)
        self.create_subscription(PerceptionTargets, '/perception_yolo_detection',
                                 self._targets, 10)
        self.create_subscription(Odometry, '/odom', self._odometry, 10)

        self.phase = Phase.ACQUIRE_ORDER
        self.start_clock = time.monotonic()
        self.phase_clock = self.start_clock
        self.direction = 0.0
        self.turn_deadline = 0.0
        self.track_cmd = Twist()
        self.avoid_cmd = Twist()
        self.track_clock = 0.0
        self.avoid_clock = 0.0
        self.hazard_active = False
        self.gate_visible = False
        self.gate_was_entered = False
        self.caption_ok = False
        self.snapshot_sent = False
        self.origin = None
        self.previous_pose = None
        self.total_distance = 0.0
        self.lap_origin_distance = 0.0
        self.hard_stop = False
        self.create_timer(0.05, self._tick)
        self._emit_phase()

    def _change(self, phase):
        if phase == self.phase:
            return
        self.get_logger().info(f'phase {self.phase.value} -> {phase.value}')
        self.phase = phase
        self.phase_clock = time.monotonic()
        self._emit_phase()

    def _emit_phase(self):
        msg = String()
        msg.data = self.phase.value
        self.phase_pub.publish(msg)

    def _track(self, msg):
        self.track_cmd, self.track_clock = msg, time.monotonic()

    def _avoid(self, msg):
        self.avoid_cmd, self.avoid_clock = msg, time.monotonic()

    def _hazard(self, msg):
        self.hazard_active = msg.data != 0

    def _order(self, msg):
        if self.phase != Phase.ACQUIRE_ORDER or msg.data not in (3, 4):
            return
        self.direction = -1.0 if msg.data == 3 else 1.0
        self.turn_deadline = time.monotonic() + self.turn_sec
        self._change(Phase.BRANCH_TO_B)

    def _halt_mark(self, msg):
        if msg.data == 1 and self.phase == Phase.HOMEWARD:
            self._finish()

    def _caption(self, msg):
        if self.phase in (Phase.ACCUMULATE_LAP, Phase.CROSS_GATE_OUT) and msg.data.strip():
            self.caption_ok = True
            spoken = String()
            spoken.data = msg.data.strip()
            self.speech_pub.publish(spoken)

    @staticmethod
    def _classes(msg):
        return {str(getattr(item, 'type', '')).lower() for item in msg.targets}

    def _targets(self, msg):
        labels = self._classes(msg)
        gate = 'tongdao' in labels
        if self.phase == Phase.BRANCH_TO_B and gate:
            self.gate_was_entered = True
            self._change(Phase.CROSS_GATE_IN)
        elif self.phase == Phase.CROSS_GATE_IN and self.gate_visible and not gate:
            self.lap_origin_distance = self.total_distance
            self._change(Phase.ACCUMULATE_LAP)
        elif self.phase == Phase.ACCUMULATE_LAP:
            if labels.intersection({'person', 'tuwen'}) and not self.snapshot_sent:
                request = Int32()
                request.data = 1
                self.snap_pub.publish(request)
                self.snapshot_sent = True
            travelled = self.total_distance - self.lap_origin_distance
            if gate and not self.gate_visible and travelled >= self.lap_min and self.caption_ok:
                self._change(Phase.CROSS_GATE_OUT)
        elif self.phase == Phase.CROSS_GATE_OUT and self.gate_visible and not gate:
            self._change(Phase.HOMEWARD)
        self.gate_visible = gate

    def _odometry(self, msg):
        pose = Pose2(msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.origin is None:
            self.origin = pose
        if self.previous_pose is not None:
            step = math.hypot(pose.x - self.previous_pose.x,
                              pose.y - self.previous_pose.y)
            if step < 0.5:  # 丢帧跳变不计入圈长
                self.total_distance += step
        self.previous_pose = pose
        if self.phase == Phase.HOMEWARD and self.origin is not None:
            if math.hypot(pose.x - self.origin.x, pose.y - self.origin.y) <= self.home_radius:
                self._finish()

    def _finish(self):
        self.hard_stop = True
        self._change(Phase.COMPLETE)
        msg = Bool()
        msg.data = True
        self.done_pub.publish(msg)

    def _motion(self, now):
        if now < self.turn_deadline:
            cmd = Twist()
            cmd.linear.x = 0.16
            cmd.angular.z = self.direction * self.turn_rate
            return cmd
        if self.hazard_active and now - self.avoid_clock <= self.timeout:
            return self.avoid_cmd
        if now - self.track_clock <= self.timeout:
            return self.track_cmd
        return Twist()

    def _tick(self):
        now = time.monotonic()
        elapsed = now - self.start_clock
        elapsed_msg = Float32()
        elapsed_msg.data = float(elapsed)
        self.elapsed_pub.publish(elapsed_msg)
        if not self.hard_stop and elapsed >= self.limit:
            self.hard_stop = True
            self._change(Phase.SAFETY_STOP)
        self.cmd_pub.publish(Twist() if self.hard_stop else self._motion(now))


def main(args=None):
    rclpy.init(args=args)
    node = MissionCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.hard_stop = True
        for _ in range(5):
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
