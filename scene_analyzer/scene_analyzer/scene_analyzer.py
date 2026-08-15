# -- Team 2 场景分析 — 火山引擎 doubao-lite-128k 视觉推理调用 --
import base64
import os
import threading
import time

import cv2
import numpy as np
import rclpy
from ai_msgs.msg import PerceptionTargets
from hbm_img_msgs.msg import HbmMsg1080P
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32, String
from volcenginesdkarkruntime import Ark


class SceneAnalyzerNode(Node):
    """2队场景分析节点：按需抓帧+火山引擎VLM推理+结果发布"""

    def __init__(self):
        super().__init__("scene_analyzer_node")

        self.declare_parameter("save_snapshot", False)
        self.declare_parameter("snapshot_dir", "/tmp/scene_snap")
        self.declare_parameter("crop_margin", 0.18)
        self.declare_parameter("snapshot_quality", 92)
        self.declare_parameter(
            "api_endpoint", "https://open.volcengineapi.com/ark/v3"
        )
        self.declare_parameter("api_model", "doubao-lite-128k")
        self.declare_parameter(
            "scene_prompt",
            "识别画面中的医用标识人物模型，用简短语句输出其特征。",
        )
        self.declare_parameter("vlm_timeout", 25.0)
        self.declare_parameter("vlm_max_retries", 2)

        self.save_snapshot = self.get_parameter("save_snapshot").value
        self.snapshot_dir = self.get_parameter("snapshot_dir").value
        self.crop_margin = max(
            0.0, float(self.get_parameter("crop_margin").value)
        )
        self.snapshot_quality = max(
            1,
            min(100, int(self.get_parameter("snapshot_quality").value)),
        )
        self.api_model = self.get_parameter("api_model").value
        self.scene_prompt = self.get_parameter("scene_prompt").value
        self.vlm_timeout = float(self.get_parameter("vlm_timeout").value)
        self.vlm_max_retries = int(self.get_parameter("vlm_max_retries").value)

        self.api_key = os.getenv("VOLC_ACCESS_KEY") or os.getenv("ARK_API_KEY") or ""
        self.api_endpoint = self.get_parameter("api_endpoint").value
        self.add_no_proxy_host("open.volcengineapi.com")
        self.client = Ark(
            base_url=self.api_endpoint,
            api_key=self.api_key,
            timeout=self.vlm_timeout,
            max_retries=self.vlm_max_retries,
        )

        self.qos_image = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        qos_reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.nv12_sub = None
        self.snapshot_trigger_sub = self.create_subscription(
            Int32,
            "/snapshot_cmd",
            self.snapshot_trigger_callback,
            qos_reliable,
        )
        self.scene_detection_sub = self.create_subscription(
            PerceptionTargets,
            "/perception_yolo_detection",
            self.scene_detection_callback,
            qos_reliable,
        )

        self.caption_pub = self.create_publisher(String, "/scene_caption", qos_reliable)
        self.display_pub = self.create_publisher(String, "/display_text", qos_reliable)
        self.analysis_pub = self.create_publisher(
            String, "/scene_analysis", qos_reliable
        )
        self.snapshot_img_pub = self.create_publisher(
            CompressedImage, "/snapshot_image", qos_reliable
        )
        self.snapshot_path_pub = self.create_publisher(
            String, "/snapshot_path", qos_reliable
        )

        self.lock = threading.Lock()
        self.latest_roi = None
        self.capture_pending = False
        self.capture_started_at = 0.0
        self.capture_frame_timeout = 1.0
        self.is_calling_vlm = False
        self.snapshot_count = 0
        self.capture_timeout_timer = self.create_timer(
            0.1, self.capture_timeout_callback
        )

        os.makedirs(self.snapshot_dir, exist_ok=True)
        self.get_logger().info(
            f"场景分析就绪: 模型={self.api_model}, "
            f"抓帧源=/nv12_img(按需), 画质={self.snapshot_quality}"
        )

    @staticmethod
    def add_no_proxy_host(host):
        hosts = [
            value.strip()
            for value in os.environ.get("NO_PROXY", "").split(",")
            if value.strip()
        ]
        if host not in hosts:
            hosts.append(host)
        no_proxy = ",".join(hosts)
        os.environ["NO_PROXY"] = no_proxy
        os.environ["no_proxy"] = no_proxy

    def nv12_callback(self, msg):
        with self.lock:
            if not self.capture_pending:
                return

        width = int(msg.width)
        height = int(msg.height)
        step = int(msg.step)
        if width <= 0 or height <= 0 or step < width:
            self.get_logger().warn(
                f"帧参数异常: 宽={width}, 高={height}, 步长={step}"
            )
            return

        encoding = bytes(msg.encoding).split(b"\x00", 1)[0].decode(
            "utf-8", errors="ignore"
        )
        if encoding != "nv12":
            self.get_logger().warn(f"编码格式不符: {encoding}")
            return

        expected_size = step * height * 3 // 2
        if len(msg.data) < expected_size:
            self.get_logger().warn(
                f"数据长度不足: 实际={len(msg.data)}, 需要至少={expected_size}"
            )
            return

        try:
            frame_data = memoryview(msg.data)[:expected_size].tobytes()
        except TypeError:
            frame_data = bytes(msg.data[:expected_size])

        with self.lock:
            if not self.capture_pending:
                return
            self.capture_pending = False
            self.capture_started_at = 0.0
            roi = self.latest_roi
            nv12_sub = self.nv12_sub
            self.nv12_sub = None

        if nv12_sub is not None:
            self.destroy_subscription(nv12_sub)

        self.process_captured_frame(
            (frame_data, width, height, step), roi
        )

    def scene_detection_callback(self, msg):
        best_roi = None
        max_area = 0
        for target in msg.targets:
            if target.type != "tuWen":
                continue
            for roi in target.rois:
                area = roi.rect.width * roi.rect.height
                if area > max_area:
                    max_area = area
                    best_roi = (
                        int(roi.rect.x_offset),
                        int(roi.rect.y_offset),
                        int(roi.rect.width),
                        int(roi.rect.height),
                    )
        if best_roi is not None:
            with self.lock:
                self.latest_roi = best_roi

    def snapshot_trigger_callback(self, msg):
        if msg.data != 1:
            return
        if self.is_calling_vlm:
            self.get_logger().info("VLM运行中，跳过本次抓帧请求")
            return

        with self.lock:
            if self.capture_pending:
                self.get_logger().info("已有待处理帧，跳过重复请求")
                return
            self.capture_pending = True
            self.capture_started_at = time.monotonic()
            try:
                self.nv12_sub = self.create_subscription(
                    HbmMsg1080P,
                    "/nv12_img",
                    self.nv12_callback,
                    self.qos_image,
                )
            except Exception as exc:
                self.capture_pending = False
                self.capture_started_at = 0.0
                self.nv12_sub = None
                self.get_logger().error(f"图像订阅失败: {exc}")
                return

        self.get_logger().info("收到抓帧触发，准备截取下一帧")

    def capture_timeout_callback(self):
        with self.lock:
            if not self.capture_pending:
                return
            if time.monotonic() - self.capture_started_at < self.capture_frame_timeout:
                return
            self.capture_pending = False
            self.capture_started_at = 0.0
            nv12_sub = self.nv12_sub
            self.nv12_sub = None

        if nv12_sub is not None:
            self.destroy_subscription(nv12_sub)
        self.get_logger().warn("等待帧超时，放弃本次截取")

    def process_captured_frame(self, nv12_frame, roi):
        image_msg = self.encode_snapshot(nv12_frame, roi)
        if image_msg is None:
            return

        image_path = self.save_snapshot_file(image_msg.data)
        self.snapshot_img_pub.publish(image_msg)

        path_msg = String()
        path_msg.data = image_path
        self.snapshot_path_pub.publish(path_msg)

        base64_image = base64.b64encode(image_msg.data).decode("utf-8")
        self.is_calling_vlm = True
        threading.Thread(
            target=self.call_vlm,
            args=(base64_image, image_path),
            daemon=True,
        ).start()

    def encode_snapshot(self, nv12_frame, roi):
        data, image_width, image_height, step = nv12_frame
        try:
            nv12 = np.frombuffer(data, dtype=np.uint8)
            nv12 = nv12.reshape((image_height * 3 // 2, step))
            if step != image_width:
                nv12 = np.ascontiguousarray(nv12[:, :image_width])
            image = cv2.cvtColor(nv12, cv2.COLOR_YUV2BGR_NV12)
        except (ValueError, cv2.error) as exc:
            self.get_logger().error(f"帧数据解析失败: {exc}")
            return None

        cropped = image
        crop_desc = "full frame"
        if roi is not None:
            x, y, width, height = roi
            margin_x = int(round(width * self.crop_margin))
            margin_y = int(round(height * self.crop_margin))
            x1 = max(0, x - margin_x)
            y1 = max(0, y - margin_y)
            x2 = min(image_width, x + width + margin_x)
            y2 = min(image_height, y + height + margin_y)
            if x2 > x1 and y2 > y1:
                cropped = image[y1:y2, x1:x2]
                crop_desc = f"({x1},{y1})-({x2},{y2})"
            else:
                self.get_logger().warn("裁剪区域越界，回退到全幅")
        else:
            self.get_logger().warn("未检测到裁剪区域，采用全幅")

        success, encoded = cv2.imencode(
            ".jpg",
            cropped,
            [cv2.IMWRITE_JPEG_QUALITY, self.snapshot_quality],
        )
        if not success:
            self.get_logger().error("JPEG编码失败")
            return None

        cropped_msg = CompressedImage()
        cropped_msg.format = "jpeg"
        cropped_msg.data = encoded.tobytes()
        self.get_logger().info(
            f"抓帧编码完成: 裁剪={crop_desc}, "
            f"质量={self.snapshot_quality}, 体积={len(cropped_msg.data)}字节"
        )
        return cropped_msg

    def save_snapshot_file(self, image_bytes):
        if self.save_snapshot:
            name = f"scene_{int(time.time())}_{self.snapshot_count}.jpg"
        else:
            name = "latest_scene.jpg"
        self.snapshot_count += 1
        image_path = os.path.join(self.snapshot_dir, name)
        with open(image_path, "wb") as f:
            f.write(image_bytes)
        self.get_logger().info(f"抓帧已落盘: {image_path}")
        return image_path

    def publish_text(self, text):
        msg = String()
        msg.data = text
        self.caption_pub.publish(msg)
        self.display_pub.publish(msg)
        self.analysis_pub.publish(msg)

    @staticmethod
    def response_text(response):
        def value(item, name, default=None):
            if isinstance(item, dict):
                return item.get(name, default)
            return getattr(item, name, default)

        direct_text = value(response, "output_text")
        if direct_text:
            return direct_text.strip()

        texts = []
        for item in value(response, "output", []) or []:
            if value(item, "type") != "message":
                continue
            for content in value(item, "content", []) or []:
                if value(content, "type") == "output_text":
                    text = value(content, "text", "")
                    if text:
                        texts.append(text)
        return "\n".join(texts).strip()

    @staticmethod
    def response_debug(response):
        if hasattr(response, "model_dump_json"):
            return response.model_dump_json()[:2000]
        if hasattr(response, "json"):
            try:
                return response.json()[:2000]
            except TypeError:
                pass
        return str(response)[:2000]

    def call_vlm(self, base64_image, image_path):
        try:
            self.publish_text("start")
            if not self.api_key:
                raise RuntimeError("VOLC_ACCESS_KEY/ARK_API_KEY is not set")

            response = self.client.responses.create(
                model=self.api_model,
                thinking={"type": "disabled"},
                input=[
                    {
                        "role": "user",
                        "content": [
                            {
                                "type": "input_image",
                                "image_url": f"data:image/jpeg;base64,{base64_image}",
                            },
                            {"type": "input_text", "text": self.scene_prompt},
                        ],
                    }
                ],
            )
            result_text = self.response_text(response)
            if not result_text:
                raise RuntimeError(
                    "Ark response contains no output_text: "
                    f"{self.response_debug(response)}"
                )
            self.publish_text(result_text)
            self.get_logger().info(f"场景推理结果: {result_text}")
        except Exception as e:
            self.get_logger().error(f"VLM调用异常: {e}")
            self.publish_text("error")
        finally:
            if not self.save_snapshot:
                try:
                    os.remove(image_path)
                except OSError:
                    pass
            self.is_calling_vlm = False


def main(args=None):
    rclpy.init(args=args)
    node = SceneAnalyzerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
