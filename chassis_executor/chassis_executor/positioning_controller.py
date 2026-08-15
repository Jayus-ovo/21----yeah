#!/usr/bin/env python3
# -- Team 2 精准定位 — 确保返回P点时2/3车身在蓝白区域内 --
"""
智慧医疗比赛精准定位模块
确保车模返回P点时满足"2/3在蓝白区域内"的要求
"""

import rclpy
import math
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, String


class PositioningController(Node):
    """精准定位控制器"""

    # 车身参数
    CAR_LENGTH = 0.276  # 车身长度(米)
    CAR_WIDTH = 0.164   # 车身宽度(米)
    POSITION_RATIO = 2.0 / 3.0  # 定位精度要求

    # 定位参数
    POSITION_TOLERANCE = 0.055  # 位置容差(米)
    ANGLE_TOLERANCE = 0.09     # 角度容差(弧度)
    ALIGN_SPEED = 0.6          # 对准速度
    ALIGN_ANGULAR_GAIN = 0.75  # 对准角速度增益

    def __init__(self):
        super().__init__('positioning_controller')

        qos_reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        qos_best_effort = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 声明参数
        self.declare_parameter('p_point_x', 0.0)
        self.declare_parameter('p_point_y', 0.0)
        self.declare_parameter('p_point_yaw', 0.0)
        self.declare_parameter('position_tolerance', 0.055)
        self.declare_parameter('angle_tolerance', 0.09)
        self.declare_parameter('approach_speed', 0.6)
        self.declare_parameter('fine_position_speed', 0.6)

        # 目标位置
        self.target_x = self.get_parameter('p_point_x').value
        self.target_y = self.get_parameter('p_point_y').value
        self.target_yaw = self.get_parameter('p_point_yaw').value
        self.target_received = False

        # 当前位置
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.position_updated = False

        # 定位状态
        self.positioning_active = False
        self.positioning_phase = 'idle'  # idle, approach, align, fine, done
        self.positioning_start_time = 0.0
        self.positioning_timeout = 35.0

        # 发布者
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', qos_reliable)
        self.positioned_pub = self.create_publisher(Bool, '/positioning_done', qos_reliable)
        self.distance_pub = self.create_publisher(Float32, '/distance_to_p', qos_reliable)
        self.status_pub = self.create_publisher(String, '/positioning_status', qos_reliable)

        # 订阅者
        self.odom_sub = self.create_subscription(
            Odometry, '/odom_combined',
            self._on_odometry, qos_best_effort
        )

        self.target_sub = self.create_subscription(
            PoseStamped, '/p_point_position',
            self._on_target, qos_reliable
        )

        self.start_sub = self.create_subscription(
            Bool, '/start_positioning',
            self._on_start, qos_reliable
        )

        # 定时器
        self.control_timer = self.create_timer(0.05, self._control_tick)

        self.get_logger().info("精准定位控制器已启动")

    def _on_target(self, msg: PoseStamped):
        """接收目标位置"""
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y

        # 计算yaw
        q = msg.pose.orientation
        self.target_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        self.target_received = True
        self.get_logger().info(
            f"目标位置: ({self.target_x:.2f}, {self.target_y:.2f}), yaw={self.target_yaw:.2f}"
        )

    def _on_odometry(self, msg: Odometry):
        """更新当前位置"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        self.position_updated = True

    def _on_start(self, msg: Bool):
        """启动定位"""
        if msg.data and not self.positioning_active:
            self._start_positioning()

    def _start_positioning(self):
        """开始定位流程"""
        if not self.target_received:
            self.get_logger().warn("目标位置未设置，无法定位")
            return

        self.positioning_active = True
        self.positioning_phase = 'approach'
        self.positioning_start_time = time.monotonic()
        self.get_logger().info("开始精准定位流程")

    def _control_tick(self):
        """控制循环"""
        if not self.positioning_active or not self.position_updated:
            return

        # 检查超时
        if time.monotonic() - self.positioning_start_time > self.positioning_timeout:
            self._handle_timeout()
            return

        # 计算当前位置误差
        distance = self._calculate_distance_to_target()
        angle_error = self._calculate_angle_error()

        # 发布距离
        dist_msg = Float32()
        dist_msg.data = distance
        self.distance_pub.publish(dist_msg)

        # 根据阶段执行控制
        if self.positioning_phase == 'approach':
            self._execute_approach(distance, angle_error)
        elif self.positioning_phase == 'align':
            self._execute_align(angle_error)
        elif self.positioning_phase == 'fine':
            self._execute_fine_positioning(distance, angle_error)
        elif self.positioning_phase == 'done':
            self._publish_done()

    def _execute_approach(self, distance: float, angle_error: float):
        """接近阶段"""
        if distance < 0.35:  # 接近到35cm内
            self.positioning_phase = 'align'
            self._publish_status('align', '接近完成，开始对准')
            return

        # 计算到目标的方向
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        target_direction = math.atan2(dy, dx)

        # 计算角度差
        direction_error = self._normalize_angle(target_direction - self.current_yaw)

        cmd = Twist()
        cmd.linear.x = min(0.35, distance * 0.55)
        cmd.angular.z = direction_error * 2.2
        cmd.angular.z = max(-1.0, min(1.0, cmd.angular.z))

        self.cmd_pub.publish(cmd)

    def _execute_align(self, angle_error: float):
        """对准阶段"""
        if abs(angle_error) < self.ANGLE_TOLERANCE:
            self.positioning_phase = 'fine'
            self._publish_status('fine', '对准完成，开始精定位')
            return

        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = angle_error * self.ALIGN_ANGULAR_GAIN
        cmd.angular.z = max(-0.55, min(0.55, cmd.angular.z))

        self.cmd_pub.publish(cmd)

    def _execute_fine_positioning(self, distance: float, angle_error: float):
        """精确定位阶段"""
        if distance < self.POSITION_TOLERANCE and abs(angle_error) < self.ANGLE_TOLERANCE:
            self.positioning_phase = 'done'
            self._publish_status('done', '精准定位完成')
            return

        cmd = Twist()

        # 小距离调整
        if distance >= self.POSITION_TOLERANCE:
            cmd.linear.x = min(0.12, distance)
        else:
            cmd.linear.x = 0.0

        # 小角度调整
        if abs(angle_error) >= self.ANGLE_TOLERANCE:
            cmd.angular.z = angle_error * 0.55
            cmd.angular.z = max(-0.28, min(0.28, cmd.angular.z))

        self.cmd_pub.publish(cmd)

    def _publish_done(self):
        """发布定位完成"""
        # 发送停止命令
        stop_cmd = Twist()
        for _ in range(5):
            self.cmd_pub.publish(stop_cmd)

        # 发布完成状态
        done_msg = Bool()
        done_msg.data = True
        self.positioned_pub.publish(done_msg)

        # 计算最终误差
        distance = self._calculate_distance_to_target()
        angle_error = abs(self._calculate_angle_error())

        self.get_logger().info(
            f"精准定位完成: 误差={distance:.3f}m, 角度={angle_error:.2f}rad"
        )

        # 停止定时器
        self.control_timer.cancel()

    def _handle_timeout(self):
        """处理超时"""
        self.get_logger().error("定位超时")

        stop_cmd = Twist()
        self.cmd_pub.publish(stop_cmd)

        self._publish_status('timeout', '定位超时')

    def _calculate_distance_to_target(self) -> float:
        """计算到目标的距离"""
        return math.sqrt(
            (self.target_x - self.current_x) ** 2 +
            (self.target_y - self.current_y) ** 2
        )

    def _calculate_angle_error(self) -> float:
        """计算角度误差"""
        return self._normalize_angle(self.target_yaw - self.current_yaw)

    def _normalize_angle(self, angle: float) -> float:
        """归一化角度到[-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def _publish_status(self, phase: str, message: str):
        """发布状态"""
        status_msg = String()
        status_msg.data = f"{phase}:{message}"
        self.status_pub.publish(status_msg)
        self.get_logger().info(message)


def main(args=None):
    rclpy.init(args=args)
    controller = PositioningController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()