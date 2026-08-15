// -- Team 2 二维码解码 — ZBar 读取 + 方向编码 --
#include <rclcpp/rclcpp.hpp>

#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"

#include <opencv2/opencv.hpp>
#include <zbar.h>

#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#include <functional>
#include <string>
#include <stdexcept>


//=== 码标解码器：从视频帧中识别二维码/条形码，解析运动方向指令 ===

class CodeMarkDecoder : public rclcpp::Node
{
public:
  CodeMarkDecoder()
      : Node("codemark_decoder"),
        last_number_(0),
        frame_count_(0),
        debounce_count_(0),
        last_direction_(0)
  {
    rclcpp::QoS qos(1);
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    //=== 注册图像数据订阅回调 ===
    image_subscriber_ =
        this->create_subscription<hbm_img_msgs::msg::HbmMsg1080P>(
            "/nv12_img",
            qos,
            std::bind(&CodeMarkDecoder::subscription_callback, this, std::placeholders::_1));

    //=== 创建方向指令发布器（3=顺时针，4=逆时针） ===
    direction_publisher_ =
        this->create_publisher<std_msgs::msg::Int32>("/barcode_id", 10);

    //=== 创建原始码值发布器，用于调试和日志记录 ===
    raw_data_publisher_ =
        this->create_publisher<std_msgs::msg::String>("/barcode_data", 10);

    //=== 配置 zbar 扫描器，启用全部条码符号类型 ===
    code_scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

    RCLCPP_INFO(this->get_logger(), "码标解码器启动就绪");
  }

private:
  //=== 从 HbmMsg1080P 共享内存图像中提取 Y 平面灰度图 ===
  cv::Mat extract_y_plane(const hbm_img_msgs::msg::HbmMsg1080P::SharedPtr msg)
  {
    int height = msg->height;
    int width = msg->width;
    size_t step = msg->step;

    cv::Mat y_plane(height, width, CV_8UC1, msg->data.data(), step);

    cv::Mat gray;
    if (step == static_cast<size_t>(width) && y_plane.isContinuous())
    {
      gray = y_plane;
    }
    else
    {
      gray = y_plane.clone();
    }

    return gray;
  }

  //=== 图像数据到达时的回调处理：隔帧解码 + 去抖动 ===
  void subscription_callback(const hbm_img_msgs::msg::HbmMsg1080P::SharedPtr msg)
  {
    if (!msg)
      return;

    //=== 隔帧处理：仅对偶数帧执行解码，奇数帧直接丢弃以降低 CPU 负载 ===
    ++frame_count_;
    if (frame_count_ % 2 != 0)
      return;

    int height = msg->height;
    int width = msg->width;
    size_t step = msg->step;

    //=== 图像参数合法性校验 ===
    if (height <= 0 || width <= 0)
    {
      RCLCPP_WARN(
          this->get_logger(),
          "图像尺寸异常，已舍弃当前帧: 宽=%d, 高=%d",
          width, height);
      return;
    }

    if (step < static_cast<size_t>(width))
    {
      RCLCPP_WARN(
          this->get_logger(),
          "图像行步长不合法: step=%zu, 宽=%d",
          step, width);
      return;
    }

    if (msg->data.size() < step * static_cast<size_t>(height))
    {
      RCLCPP_WARN(this->get_logger(), "图像数据缓冲区长度不足，跳过该帧");
      return;
    }

    //=== 提取 Y 平面灰度数据 ===
    cv::Mat gray = extract_y_plane(msg);

    //=== 构造 zbar 扫描图像并执行识别 ===
    zbar::Image zbar_image(
        width,
        height,
        "Y800",
        gray.data,
        width * height);

    int result = code_scanner_.scan(zbar_image);

    if (result <= 0)
    {
      //=== 当前帧未识别到码标，重置去抖动状态 ===
      debounce_count_ = 0;
      last_number_ = 0;
      return;
    }

    //=== 遍历 zbar 扫描到的所有符号 ===
    for (zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin();
         symbol != zbar_image.symbol_end();
         ++symbol)
    {
      std::string qr_data = symbol->get_data();

      std_msgs::msg::Int32 direction_msg;
      std_msgs::msg::String raw_msg;

      bool parse_ok = true;
      int parsed_direction = 0;

      //=== 码值 -> 方向映射逻辑 ===
      if (qr_data == "ClockWise")
      {
        parsed_direction = 3;
      }
      else if (qr_data == "AntiClockWise")
      {
        parsed_direction = 4;
      }
      else
      {
        try
        {
          int number = std::stoi(qr_data);

          if (number >= 1 && number <= 9999)
          {
            //=== 奇数→顺时针(3)，偶数→逆时针(4) ===
            parsed_direction = (number % 2 == 0) ? 4 : 3;
          }
          else
          {
            RCLCPP_WARN(
                this->get_logger(),
                "码标数值超出有效范围[1,9999]: %d",
                number);

            parse_ok = false;
          }
        }
        catch (const std::invalid_argument &e)
        {
          RCLCPP_WARN(
              this->get_logger(),
              "码标内容无法解释为有效方向指令: %s",
              qr_data.c_str());

          parse_ok = false;
        }
        catch (const std::out_of_range &e)
        {
          RCLCPP_WARN(
              this->get_logger(),
              "码标数值超出 int 表达范围: %s",
              qr_data.c_str());

          parse_ok = false;
        }
      }

      if (!parse_ok)
        continue;

      //=== 去抖动机制：仅当连续两次读到相同方向值时才正式发布 ===
      direction_msg.data = parsed_direction;

      if (parsed_direction == last_direction_)
      {
        ++debounce_count_;
      }
      else
      {
        debounce_count_ = 1;
        last_direction_ = parsed_direction;
      }

      if (debounce_count_ < 2)
      {
        continue;
      }

      //=== 发布原始码值和转换后的方向指令 ===
      raw_msg.data = qr_data;
      raw_data_publisher_->publish(raw_msg);

      direction_publisher_->publish(direction_msg);

      RCLCPP_INFO(
          this->get_logger(),
          "码标识別结果: 原文=%s, 方向指令=%d",
          qr_data.c_str(),
          direction_msg.data);
    }
  }

  //=== 成员变量声明 ===
  rclcpp::Subscription<hbm_img_msgs::msg::HbmMsg1080P>::SharedPtr image_subscriber_;

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr direction_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr raw_data_publisher_;

  zbar::ImageScanner code_scanner_;

  int last_number_;
  uint64_t frame_count_;
  int debounce_count_;
  int last_direction_;
};


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CodeMarkDecoder>());
  rclcpp::shutdown();
  return 0;
}
