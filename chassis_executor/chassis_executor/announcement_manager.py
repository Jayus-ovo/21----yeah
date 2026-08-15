#!/usr/bin/env python3
# -- Team 2 语音播报管理器 — TTS合成 + 屏幕显示双重反馈 --
"""
智慧医疗比赛语音播报模块
支持语音合成和屏幕显示双重反馈
"""

import rclpy
import threading
import time
from queue import Queue
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import String


class AnnouncementManager(Node):
    """语音播报和屏幕显示管理器"""

    def __init__(self):
        super().__init__('announcement_manager')

        qos_reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        qos_persistent = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 订阅需要播报的文本
        self.text_sub = self.create_subscription(
            String, '/speech_cmd',
            self._on_tts_request, qos_reliable
        )

        # 订阅屏幕显示文本
        self.display_sub = self.create_subscription(
            String, '/screen_text',
            self._on_display_request, qos_reliable
        )

        # 发布到TTS服务
        self.tts_pub = self.create_publisher(
            String, '/tts_input', qos_reliable
        )

        # 发布到屏幕显示
        self.screen_pub = self.create_publisher(
            String, '/screen_display', qos_persistent
        )

        # 播报队列和状态
        self.announce_queue = Queue()
        self.current_announce = None
        self.announce_lock = threading.Lock()

        # 任务状态文本映射
        self.status_messages = {
            'mission_start': '智慧医疗比赛任务开始',
            'task1_navigate': '正在前往任务发布点',
            'task1_scanning': '正在扫描二维码',
            'task1_success': '二维码扫描成功，获取任务信息',
            'task1_complete': '子任务一完成',
            'task2_enter': '正在进入黄色通道',
            'task2_tracking': '正在通道内行驶，注意人形立牌',
            'task2_detected': '检测到人形立牌',
            'task2_complete': '子任务二完成',
            'task3_return': '正在返回起点',
            'task3_positioning': '正在进行精准定位',
            'task3_complete': '子任务三完成',
            'mission_complete': '所有任务完成',
            'timeout_warning': '注意，剩余时间不足',
            'error_recovery': '检测到异常，正在恢复',
        }

        # 定时处理队列
        self.announce_timer = self.create_timer(0.1, self._process_queue)

        self.get_logger().info("播报管理器已启动")

    def _on_tts_request(self, msg: String):
        """处理TTS请求"""
        self._add_announcement(msg.data)

    def _on_display_request(self, msg: String):
        """处理屏幕显示请求"""
        # 立即显示
        display_msg = String()
        display_msg.data = msg.data
        self.screen_pub.publish(display_msg)
        self.get_logger().info(f"屏幕显示: {msg.data}")

    def _add_announcement(self, text: str):
        """添加播报任务到队列"""
        with self.announce_lock:
            self.announce_queue.put(text)
            self.get_logger().debug(f"添加播报任务: {text}")

    def _process_queue(self):
        """处理播报队列"""
        if self.current_announce is not None:
            return  # 正在播报

        try:
            text = self.announce_queue.get_nowait()
        except:
            return

        with self.announce_lock:
            self.current_announce = text

        # 发送到TTS
        tts_msg = String()
        tts_msg.data = text
        self.tts_pub.publish(tts_msg)

        # 同时显示到屏幕
        display_msg = String()
        display_msg.data = text
        self.screen_pub.publish(display_msg)

        self.get_logger().info(f"播报: {text}")

        # 启动线程等待播报完成
        threading.Thread(
            target=self._wait_announce_complete,
            args=(len(text) * 0.15 + 0.5),  # 估算播报时间
            daemon=True
        ).start()

    def _wait_announce_complete(self, duration: float):
        """等待播报完成"""
        time.sleep(duration)
        with self.announce_lock:
            self.current_announce = None

    def announce_status(self, status_key: str):
        """播报预定义状态"""
        message = self.status_messages.get(status_key, '')
        if message:
            self._add_announcement(message)


def main(args=None):
    rclpy.init(args=args)
    manager = AnnouncementManager()

    try:
        rclpy.spin(manager)
    except KeyboardInterrupt:
        pass
    finally:
        manager.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()