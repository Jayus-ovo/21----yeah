# -- Team 2 车载信息展示面板 — Tkinter 实时渲染二维码+VLM识别结果 --
"""车载视觉信息展示面板

本模块通过 ROS2 订阅码标识别与场景文本话题，
在 Tkinter 图形窗口中实时呈现识别数据。
"""

import queue
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import Int32, String

try:
    import tkinter as tk
    from tkinter import font as tkfont
except ImportError as exc:
    tk = None
    tkfont = None
    TK_IMPORT_ERROR = exc
else:
    TK_IMPORT_ERROR = None


class DisplayPanelSubscriber(Node):
    #-- ROS2 订阅节点：接收码标与场景文本识别结果 --#
    def __init__(self, text_queue):
        super().__init__("info_display_panel")
        self.text_queue = text_queue

        qos_reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_run_start = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.display_text_sub = self.create_subscription(
            String, "/display_text", self.text_callback, qos_reliable)
        self.barcode_data_sub = self.create_subscription(
            String, "/barcode_data", self.qrcode_callback, qos_reliable)
        self.mission_cycle_sub = self.create_subscription(
            Int32,
            "/mission_cycle",
            self.run_start_callback,
            qos_run_start,
        )
        self.get_logger().info(
            "车载信息面板已订阅显示文本与码标数据话题")

    def text_callback(self, msg):
        text = msg.data.strip()
        if text:
            self.text_queue.put(("text", text))

    def qrcode_callback(self, msg):
        text = msg.data.strip()
        if text:
            self.text_queue.put(("qrcode", text))

    def run_start_callback(self, _msg):
        self.text_queue.put(("run_start", None))


class InfoDisplayPanel:
    #-- 车载信息终端图形化面板 --#
    QRCODE_WAITING_TEXT = "码标：待译码"
    TUWEN_WAITING_TEXT = "场景：等待分析"

    def __init__(self, text_queue):
        if tk is None:
            raise RuntimeError(f"tkinter is not available: {TK_IMPORT_ERROR}")

        self.text_queue = text_queue
        self.qrcode_text = self.QRCODE_WAITING_TEXT
        self.tuwen_text = self.TUWEN_WAITING_TEXT
        self.qrcode_locked = False
        self.tuwen_locked = False
        self.resize_after_id = None
        self.last_update_time = "--:--:--"

        self.root = tk.Tk()
        self.root.title("医疗机器人 · 可视化面板")
        self.root.configure(bg="#061a2e")
        self.root.attributes("-fullscreen", True)
        self.root.bind(
            "<f>",
            lambda _event: self.root.attributes(
                "-fullscreen",
                not self.root.attributes("-fullscreen"),
            ),
        )
        self.root.bind("<q>", lambda _event: self.root.destroy())
        self.root.bind("<Configure>", self.schedule_fit)

        #-- 尝试设置内置信息图标 --#
        try:
            self.root.iconbitmap(bitmap="info")
        except tk.TclError:
            pass

        self.qrcode_font = self.pick_font(24, "bold")
        self.tuwen_font = self.pick_font(20, "normal")
        self.timestamp_font = self.pick_font(14, "normal")
        self.status_font = self.pick_font(14, "normal")

        #-- 网格布局权重配置 --#
        self.root.grid_rowconfigure(0, weight=0)
        self.root.grid_rowconfigure(1, weight=0)
        self.root.grid_rowconfigure(2, weight=0)
        self.root.grid_rowconfigure(3, weight=1)
        self.root.grid_rowconfigure(4, weight=0)
        self.root.grid_columnconfigure(0, weight=1)

        #-- 系统运行状态指示（右上角绿色圆点） --#
        self.status_label = tk.Label(
            self.root,
            text="●",
            fg="#48bb78",
            bg="#061a2e",
            font=self.status_font,
            anchor="e",
        )
        self.status_label.grid(
            row=0, column=0, sticky="e", padx=30, pady=(30, 0))

        #-- 码标识别结果显示区 --#
        self.qrcode_label = tk.Label(
            self.root,
            text=self.qrcode_text,
            fg="#e8a838",
            bg="#061a2e",
            font=self.qrcode_font,
            anchor="center",
            justify="left",
        )
        self.qrcode_label.grid(
            row=1, column=0, sticky="ew", padx=30, pady=(15, 15))

        #-- 视觉分隔线 --#
        divider = tk.Frame(self.root, bg="#1a3a5c", height=2)
        divider.grid(row=2, column=0, sticky="ew", padx=30)

        #-- 场景文本识别结果显示区 --#
        self.tuwen_label = tk.Label(
            self.root,
            text=self.tuwen_text,
            fg="#e0e6ed",
            bg="#061a2e",
            font=self.tuwen_font,
            anchor="nw",
            justify="left",
        )
        self.tuwen_label.grid(
            row=3, column=0, sticky="nsew", padx=30, pady=(15, 30))

        #-- 最近更新时间标签 --#
        self.timestamp_label = tk.Label(
            self.root,
            text="更新: --:--:--",
            fg="#506680",
            bg="#061a2e",
            font=self.timestamp_font,
            anchor="e",
        )
        self.timestamp_label.grid(
            row=4, column=0, sticky="e", padx=30, pady=(0, 15))

        self.root.after(100, self.fit_content)

    @staticmethod
    def pick_font(size, weight):
        #-- 中文字体优先级：黑体/楷体优先，确保显示效果 --#
        families = set(tkfont.families())
        for family in (
            "SimHei",
            "KaiTi",
            "Noto Sans CJK SC",
            "WenQuanYi Zen Hei",
            "Microsoft YaHei",
            "Arial",
        ):
            if family in families:
                return tkfont.Font(family=family, size=size, weight=weight)
        return tkfont.Font(size=size, weight=weight)

    def run(self):
        #-- 启动消息轮询与主事件循环 --#
        self.poll_queue()
        self.root.mainloop()

    def poll_queue(self):
        #-- 定时从队列提取消息分发给对应处理方法 --#
        while True:
            try:
                event_type, payload = self.text_queue.get_nowait()
            except queue.Empty:
                break
            if event_type == "run_start":
                self.handle_run_start()
            elif event_type == "qrcode":
                self.handle_qrcode(payload)
            elif event_type == "text":
                self.handle_text(payload)
        self.root.after(100, self.poll_queue)

    def handle_run_start(self):
        #-- 重置所有状态至初始等待值 --#
        self.qrcode_text = self.QRCODE_WAITING_TEXT
        self.tuwen_text = self.TUWEN_WAITING_TEXT
        self.qrcode_locked = False
        self.tuwen_locked = False
        self.last_update_time = "--:--:--"
        self.render()

    def handle_qrcode(self, text):
        #-- 处理码标识别回调数据 --#
        if self.qrcode_locked:
            return

        parsed = self.parse_qrcode(text)
        if parsed is None:
            return

        value, direction = parsed
        self.qrcode_text = f"码标指令：{value}  [{direction}]"
        self.qrcode_locked = True
        self.render()

    def handle_text(self, text):
        #-- 处理场景文本识别回调数据 --#
        normalized = "\n".join(
            " ".join(line.split())
            for line in text.splitlines()
            if line.strip()
        )
        lowered = normalized.lower()

        if lowered == "start":
            if not self.tuwen_locked:
                self.tuwen_text = "场景：推理中..."
        elif lowered == "error":
            if not self.tuwen_locked:
                self.tuwen_text = "场景：调用超时"
        elif not self.tuwen_locked:
            self.tuwen_text = f"识别结果：{normalized}"
            self.tuwen_locked = True
        else:
            return

        self.render()

    @staticmethod
    def parse_qrcode(text):
        #-- 解析码标字符串并推断运行方向 --#
        normalized = text.strip()
        if normalized == "ClockWise":
            return normalized, "顺时针"
        if normalized == "AntiClockWise":
            return normalized, "逆时针"

        try:
            number = int(normalized)
        except ValueError:
            return None
        if not 1 <= number <= 9999:
            return None

        direction = "逆时针" if number % 2 == 0 else "顺时针"
        return normalized, direction

    def render(self):
        #-- 异步刷新显示内容并更新最后更新时间 --#
        self.last_update_time = time.strftime("%H:%M:%S")
        self.timestamp_label.configure(
            text=f"更新: {self.last_update_time}")
        self.root.after_idle(self.fit_content)

    def schedule_fit(self, event):
        #-- 窗口尺寸变化时延迟执行自适应缩放 --#
        if event.widget is not self.root:
            return
        if self.resize_after_id is not None:
            self.root.after_cancel(self.resize_after_id)
        self.resize_after_id = self.root.after(80, self.fit_content)

    def fit_content(self):
        #-- 根据面板实际宽度动态调节字体大小与布局 --#
        self.resize_after_id = None
        available_width = self.root.winfo_width() - 60
        if available_width <= 1:
            self.root.after(100, self.fit_content)
            return

        qrcode_display = self.fit_line(
            self.qrcode_font,
            self.qrcode_text,
            preferred_size=24,
            minimum_size=12,
            available_width=available_width,
        )
        self.qrcode_label.configure(text=qrcode_display)
        self.tuwen_font.configure(size=16)
        self.tuwen_label.configure(
            text=self.tuwen_text,
            wraplength=available_width,
        )

        #-- 同步调节时间戳显示字号 --#
        ts_size = max(10, self.tuwen_font.cget("size") - 2)
        self.timestamp_font.configure(size=ts_size)

    @staticmethod
    def fit_line(
            font, text, preferred_size, minimum_size, available_width):
        #-- 逐级缩小字号适配文本宽度，超宽时追加省略符 --#
        for size in range(preferred_size, minimum_size - 1, -1):
            font.configure(size=size)
            if font.measure(text) <= available_width:
                return text

        suffix = "..."
        low = 0
        high = len(text)
        while low < high:
            middle = (low + high + 1) // 2
            candidate = text[:middle].rstrip() + suffix
            if font.measure(candidate) <= available_width:
                low = middle
            else:
                high = middle - 1
        return text[:low].rstrip() + suffix


def spin_ros(text_queue):
    #-- ROS2 独立线程入口 --#
    rclpy.init()
    node = DisplayPanelSubscriber(text_queue)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    #-- 程序入口：启动 ROS 后台线程后进入 GUI 主循环 --#
    del args
    text_queue = queue.Queue()
    ros_thread = threading.Thread(
        target=spin_ros, args=(text_queue,), daemon=True)
    ros_thread.start()

    window = InfoDisplayPanel(text_queue)
    window.run()


if __name__ == "__main__":
    main()
