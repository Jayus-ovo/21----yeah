// -- Team 2 障碍物感知节点 — YOLOv5s 推理 + 检测框发布 --
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <cstring>
#include <fstream>
#include <sstream>

#include "ai_msgs/msg/perception_targets.hpp"
#include "dnn_node/dnn_node.h"
#include "dnn_node/util/image_proc.h"
#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"
#include "hobot_cv/hobotcv_imgproc.h"
#include "sensor_msgs/msg/image.hpp"

#include "rapidjson/document.h"
#include "rapidjson/istreamwrapper.h"
#include "rapidjson/writer.h"

#include "perception_yolo/yolo_parser.h"
#include "perception_yolo/img_converter.h"

int ResizeNV12Img(const char *in_img_data,
                  const int &in_img_height,
                  const int &in_img_width,
                  const int &scaled_img_height,
                  const int &scaled_img_width,
                  cv::Mat &out_img,
                  float &ratio)
{
  cv::Mat src(
      in_img_height * 3 / 2, in_img_width, CV_8UC1, (void *)(in_img_data));
  float ratio_w =
      static_cast<float>(in_img_width) / static_cast<float>(scaled_img_width);
  float ratio_h =
      static_cast<float>(in_img_height) / static_cast<float>(scaled_img_height);
  float dst_ratio = std::max(ratio_w, ratio_h);
  int resized_width, resized_height;
  if (dst_ratio == ratio_w)
  {
    resized_width = scaled_img_width;
    resized_height = static_cast<float>(in_img_height) / dst_ratio;
  }
  else if (dst_ratio == ratio_h)
  {
    resized_width = static_cast<float>(in_img_width) / dst_ratio;
    resized_height = scaled_img_height;
  }

  int remain = resized_width % 16;
  if (remain != 0)
  {
    resized_width -= remain;
    dst_ratio = static_cast<float>(in_img_width) / resized_width;
    resized_height = static_cast<float>(in_img_height) / dst_ratio;
  }
  resized_height =
      resized_height % 2 == 0 ? resized_height : resized_height - 1;
  ratio = dst_ratio;

  return hobot_cv::hobotcv_resize(
      src, in_img_height, in_img_width, out_img, resized_height, resized_width);
}

int LetterboxNV12Img(const char *in_img_data,
                     const int &in_img_height,
                     const int &in_img_width,
                     const int &model_img_height,
                     const int &model_img_width,
                     cv::Mat &out_img,
                     float &ratio,
                     int &pad_x,
                     int &pad_y)
{
  cv::Mat resized_img;
  if (ResizeNV12Img(in_img_data,
                    in_img_height,
                    in_img_width,
                    model_img_height,
                    model_img_width,
                    resized_img,
                    ratio) < 0)
  {
    return -1;
  }

  const int resized_width = resized_img.cols;
  const int resized_height = resized_img.rows * 2 / 3;
  if (resized_width > model_img_width || resized_height > model_img_height)
  {
    return -1;
  }

  pad_x = ((model_img_width - resized_width) / 2) & ~1;
  pad_y = ((model_img_height - resized_height) / 2) & ~1;

  out_img.create(model_img_height * 3 / 2, model_img_width, CV_8UC1);
  out_img.setTo(cv::Scalar(128));
  out_img(cv::Rect(0, 0, model_img_width, model_img_height))
      .setTo(cv::Scalar(114));

  for (int row = 0; row < resized_height; ++row)
  {
    std::memcpy(out_img.ptr<uint8_t>(pad_y + row) + pad_x,
                resized_img.ptr<uint8_t>(row),
                resized_width);
  }
  for (int row = 0; row < resized_height / 2; ++row)
  {
    std::memcpy(out_img.ptr<uint8_t>(model_img_height + pad_y / 2 + row) +
                    pad_x,
                resized_img.ptr<uint8_t>(resized_height + row),
                resized_width);
  }
  return 0;
}

int InitClassNames(const std::string &cls_name_file, hobot::dnn_node::perception_yolo::YoloModelConfig &yolo_config)
{
  std::ifstream fi(cls_name_file);
  if (fi)
  {
    yolo_config.class_names.clear();
    std::string line;
    while (std::getline(fi, line))
    {
      yolo_config.class_names.push_back(line);
    }
    int size = yolo_config.class_names.size();
    if (size != yolo_config.class_num)
    {
      RCLCPP_ERROR(rclcpp::get_logger("YoloDetectionParser"),
                   "class_names length %d is not equal to class_num %d",
                   size, yolo_config.class_num);
      return -1;
    }
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("YoloDetectionParser"),
                 "can not open cls name file: %s",
                 cls_name_file.c_str());
    return -1;
  }
  return 0;
}

int InitClassNum(const int &class_num, hobot::dnn_node::perception_yolo::YoloModelConfig &yolo_config)
{
  if (class_num > 0)
  {
    yolo_config.class_num = class_num;
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("YoloDetectionParser"),
                 "class_num = %d is not allowed, only support class_num > 0",
                 class_num);
    return -1;
  }
  return 0;
}

void LoadConfig(const std::string &config_file, hobot::dnn_node::perception_yolo::YoloModelConfig &yolo_config)
{
  if (config_file.empty())
  {
    RCLCPP_ERROR(rclcpp::get_logger("LoadModelConfig"),
                 "Config file [%s] is empty!",
                 config_file.data());
    return;
  }
  std::ifstream ifs(config_file.c_str());
  if (!ifs)
  {
    RCLCPP_ERROR(rclcpp::get_logger("LoadModelConfig"),
                 "Read config file [%s] fail!",
                 config_file.data());
    return;
  }
  rapidjson::IStreamWrapper isw(ifs);
  rapidjson::Document document;
  document.ParseStream(isw);
  if (document.HasParseError())
  {
    RCLCPP_ERROR(rclcpp::get_logger("LoadModelConfig"),
                 "Parsing config file %s failed",
                 config_file.data());
    return;
  }

  if (document.HasMember("class_num"))
  {
    int class_num = document["class_num"].GetInt();
    if (InitClassNum(class_num, yolo_config) < 0)
    {
      return;
    }
  }
  if (document.HasMember("cls_names_list"))
  {
    std::string cls_name_file = document["cls_names_list"].GetString();
    if (InitClassNames(cls_name_file, yolo_config) < 0)
    {
      return;
    }
  }
  if (document.HasMember("score_threshold"))
  {
    yolo_config.score_threshold = document["score_threshold"].GetFloat();
  }
  if (document.HasMember("nms_threshold"))
  {
    yolo_config.nms_threshold = document["nms_threshold"].GetFloat();
  }
  if (document.HasMember("nms_top_k"))
  {
    yolo_config.nms_top_k = document["nms_top_k"].GetInt();
  }
  return;
}

struct DetectionNodeOutput : public hobot::dnn_node::DnnNodeOutput
{
  float ratio = 1.0f;
  int pad_x = 0;
  int pad_y = 0;
  int source_width = 0;
  int source_height = 0;
};

class YoloDetectionNode : public hobot::dnn_node::DnnNode
{
public:
  YoloDetectionNode(const std::string &node_name = "YoloDetectionNode",
                        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

protected:
  int SetNodePara() override;
  int PostProcess(const std::shared_ptr<hobot::dnn_node::DnnNodeOutput> &
                      node_output) override;

private:
  int model_input_width_ = -1;
  int model_input_height_ = -1;

  rclcpp::Subscription<hbm_img_msgs::msg::HbmMsg1080P>::ConstSharedPtr
      hbm_img_sub_ = nullptr;
  rclcpp::Subscription<sensor_msgs::msg::Image>::ConstSharedPtr
      ros_img_sub_ = nullptr;
  rclcpp::Publisher<ai_msgs::msg::PerceptionTargets>::SharedPtr msg_pub_ =
      nullptr;

  std::string sub_img_topic_ = "/hbmem_img";
  std::string pub_ai_topic_ = "/perception_yolo_detection";
  std::string config_file_ = "config/yolov5sconfig_simulation.json";
  bool use_shared_mem_ = true;
  bool is_color_model_ = false;

  hobot::dnn_node::perception_yolo::YoloModelConfig yolo_config_ = {
      {8, 16, 32},
      {{{10, 13}, {16, 30}, {33, 23}},
       {{30, 61}, {62, 45}, {59, 119}},
       {{116, 90}, {156, 198}, {373, 326}}},
      1,
      {"construction_cone"},
      {},
      0.28f,
      0.68f,
      6000};

  void OnHbmImg(const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr msg);
  void OnRosImg(const sensor_msgs::msg::Image::ConstSharedPtr msg);
};

YoloDetectionNode::YoloDetectionNode(const std::string &node_name,
                                             const rclcpp::NodeOptions &options)
    : hobot::dnn_node::DnnNode(node_name, options)
{
  this->declare_parameter<std::string>("sub_img_topic", sub_img_topic_);
  this->declare_parameter<std::string>("pub_ai_topic", pub_ai_topic_);
  this->declare_parameter<std::string>("config_file", config_file_);
  this->declare_parameter<bool>("is_shared_mem_sub", use_shared_mem_);

  this->get_parameter<std::string>("sub_img_topic", sub_img_topic_);
  this->get_parameter<std::string>("pub_ai_topic", pub_ai_topic_);
  this->get_parameter<std::string>("config_file", config_file_);
  this->get_parameter<bool>("is_shared_mem_sub", use_shared_mem_);

  if (Init() != 0 ||
      GetModelInputSize(0, model_input_width_, model_input_height_) < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("YoloDetectionNode"), "Node init fail!");
    rclcpp::shutdown();
  }
  LoadConfig(config_file_, yolo_config_);
  is_color_model_ = yolo_config_.class_num == 1 &&
                    yolo_config_.class_names.size() == 1 &&
                    yolo_config_.class_names[0] == "tuWen";
  if (is_color_model_)
  {
    RCLCPP_WARN(this->get_logger(),
                "color model enabled: letterbox %dx%d, score=%.2f, nms=%.2f",
                model_input_width_,
                model_input_height_,
                yolo_config_.score_threshold,
                yolo_config_.nms_threshold);
  }

  rclcpp::QoS qos(1);
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

  if (use_shared_mem_ == true)
  {
    hbm_img_sub_ =
        this->create_subscription<hbm_img_msgs::msg::HbmMsg1080P>(
            sub_img_topic_,
            qos,
            std::bind(&YoloDetectionNode::OnHbmImg, this, std::placeholders::_1));
  }
  else
  {
    ros_img_sub_ =
        this->create_subscription<sensor_msgs::msg::Image>(
            sub_img_topic_,
            qos,
            std::bind(&YoloDetectionNode::OnRosImg, this, std::placeholders::_1));
  }
  msg_pub_ = this->create_publisher<ai_msgs::msg::PerceptionTargets>(
      pub_ai_topic_, 1);
}

int YoloDetectionNode::SetNodePara()
{
  if (!dnn_node_para_ptr_)
    return -1;
  std::ifstream ifs(config_file_.c_str());
  if (!ifs)
  {
    RCLCPP_ERROR(rclcpp::get_logger("SetModelPara"),
                 "Read config file [%s] fail!",
                 config_file_.data());
    return -1;
  }
  rapidjson::IStreamWrapper isw(ifs);
  rapidjson::Document document;
  document.ParseStream(isw);
  if (document.HasParseError())
  {
    RCLCPP_ERROR(rclcpp::get_logger("SetModelPara"),
                 "Parsing config file %s failed",
                 config_file_.data());
    return -1;
  }

  std::string model_file;
  if (document.HasMember("model_file"))
  {
    model_file = document["model_file"].GetString();
  }
  dnn_node_para_ptr_->model_file = model_file;
  dnn_node_para_ptr_->model_task_type =
      hobot::dnn_node::ModelTaskType::ModelInferType;
  dnn_node_para_ptr_->task_num = 4;
  return 0;
}

void YoloDetectionNode::OnHbmImg(
    const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr img_msg)
{
  if (!rclcpp::ok() || !img_msg)
  {
    return;
  }

  if ("nv12" !=
      std::string(reinterpret_cast<const char *>(img_msg->encoding.data())))
  {
    RCLCPP_ERROR(rclcpp::get_logger("YoloDetectionNode"),
                 "Only support nv12 img encoding!");
    return;
  }

  auto dnn_output = std::make_shared<DetectionNodeOutput>();
  dnn_output->msg_header = std::make_shared<std_msgs::msg::Header>();
  dnn_output->msg_header->set__frame_id(std::to_string(img_msg->index));
  dnn_output->msg_header->set__stamp(img_msg->time_stamp);
  dnn_output->source_width = img_msg->width;
  dnn_output->source_height = img_msg->height;

  std::shared_ptr<hobot::dnn_node::NV12PyramidInput> pyramid = nullptr;
  if (img_msg->height != static_cast<uint32_t>(model_input_height_) ||
      img_msg->width != static_cast<uint32_t>(model_input_width_))
  {
    cv::Mat out_img;
    int resize_result = 0;
    if (is_color_model_)
    {
      resize_result = LetterboxNV12Img(
          reinterpret_cast<const char *>(img_msg->data.data()),
          img_msg->height,
          img_msg->width,
          model_input_height_,
          model_input_width_,
          out_img,
          dnn_output->ratio,
          dnn_output->pad_x,
          dnn_output->pad_y);
    }
    else
    {
      resize_result = ResizeNV12Img(
          reinterpret_cast<const char *>(img_msg->data.data()),
          img_msg->height,
          img_msg->width,
          model_input_height_,
          model_input_width_,
          out_img,
          dnn_output->ratio);
    }
    if (resize_result < 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("YoloDetectionNode"),
                   "Resize or letterbox nv12 img fail!");
      return;
    }

    uint32_t out_img_width = out_img.cols;
    uint32_t out_img_height = out_img.rows * 2 / 3;
    pyramid = hobot::dnn_node::ImageProc::GetNV12PyramidFromNV12Img(
        reinterpret_cast<const char *>(out_img.data),
        out_img_height,
        out_img_width,
        model_input_height_,
        model_input_width_);
  }
  else
  {
    pyramid = hobot::dnn_node::ImageProc::GetNV12PyramidFromNV12Img(
        reinterpret_cast<const char *>(img_msg->data.data()),
        img_msg->height,
        img_msg->width,
        model_input_height_,
        model_input_width_);
  }
  if (!pyramid)
  {
    RCLCPP_ERROR(rclcpp::get_logger("YoloDetectionNode"), "Get pym fail");
    return;
  }
  auto inputs =
      std::vector<std::shared_ptr<hobot::dnn_node::DNNInput>>{pyramid};

  if (Run(inputs, dnn_output, nullptr, false) < 0)
  {
    RCLCPP_INFO(rclcpp::get_logger("YoloDetectionNode"), "Run predict fail!");
  }
}

void YoloDetectionNode::OnRosImg(
    const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
{
  if (!rclcpp::ok() || !img_msg)
  {
    return;
  }
  auto dnn_output = std::make_shared<DetectionNodeOutput>();
  dnn_output->msg_header = std::make_shared<std_msgs::msg::Header>();
  dnn_output->msg_header->set__frame_id(img_msg->header.frame_id);
  dnn_output->msg_header->set__stamp(img_msg->header.stamp);
  auto cv_img = cv_bridge::cvtColorForDisplay(cv_bridge::toCvShare(img_msg), "bgr8");
  dnn_output->source_width = cv_img->image.cols;
  dnn_output->source_height = cv_img->image.rows;
  std::shared_ptr<hobot::dnn_node::NV12PyramidInput> pyramid = nullptr;
  if (is_color_model_ &&
      (cv_img->image.cols != model_input_width_ ||
       cv_img->image.rows != model_input_height_))
  {
    const float scale = std::min(
        static_cast<float>(model_input_width_) / cv_img->image.cols,
        static_cast<float>(model_input_height_) / cv_img->image.rows);
    const int resized_width =
        static_cast<int>(std::round(cv_img->image.cols * scale));
    const int resized_height =
        static_cast<int>(std::round(cv_img->image.rows * scale));
    dnn_output->ratio = 1.0f / scale;
    dnn_output->pad_x = (model_input_width_ - resized_width) / 2;
    dnn_output->pad_y = (model_input_height_ - resized_height) / 2;

    cv::Mat model_img(model_input_height_,
                      model_input_width_,
                      cv_img->image.type(),
                      cv::Scalar(114, 114, 114));
    cv::Mat resized_img;
    cv::resize(cv_img->image,
               resized_img,
               cv::Size(resized_width, resized_height),
               0,
               0,
               cv::INTER_LINEAR);
    resized_img.copyTo(model_img(cv::Rect(dnn_output->pad_x,
                                          dnn_output->pad_y,
                                          resized_width,
                                          resized_height)));
    pyramid = ImgConverter::GetNV12Pyramid(
        model_img, model_input_height_, model_input_width_);
  }
  else
  {
    pyramid = ImgConverter::GetNV12Pyramid(
        cv_img->image, model_input_height_, model_input_width_);
  }
  if (!pyramid)
  {
    RCLCPP_ERROR(rclcpp::get_logger("YoloDetectionNode"), "Get pym fail");
    return;
  }
  auto inputs =
      std::vector<std::shared_ptr<hobot::dnn_node::DNNInput>>{pyramid};

  if (Run(inputs, dnn_output, nullptr, false) < 0)
  {
    RCLCPP_INFO(rclcpp::get_logger("YoloDetectionNode"), "Run predict fail!");
  }
}

int YoloDetectionNode::PostProcess(
    const std::shared_ptr<hobot::dnn_node::DnnNodeOutput> &node_output)
{
  if (!rclcpp::ok())
  {
    return 0;
  }

  auto tp_start = std::chrono::system_clock::now();

  ai_msgs::msg::PerceptionTargets::UniquePtr pub_data(
      new ai_msgs::msg::PerceptionTargets());

  pub_data->set__header(*node_output->msg_header);

  std::vector<std::shared_ptr<hobot::dnn_node::perception_yolo::DetectionResult>>
      results;

  if (hobot::dnn_node::perception_yolo::Parse(node_output, results, yolo_config_) < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("YoloDetectionNode"),
                 "Parse node_output fail!");
    return -1;
  }

  auto sample_node_output =
      std::dynamic_pointer_cast<DetectionNodeOutput>(node_output);
  if (!sample_node_output)
  {
    RCLCPP_ERROR(rclcpp::get_logger("YoloDetectionNode"),
                 "Cast dnn node output fail!");
    return -1;
  }

  for (auto &rect : results)
  {
    if (!rect)
      continue;

    float xmin = (rect->xmin - sample_node_output->pad_x) *
                 sample_node_output->ratio;
    float ymin = (rect->ymin - sample_node_output->pad_y) *
                 sample_node_output->ratio;
    float xmax = (rect->xmax - sample_node_output->pad_x) *
                 sample_node_output->ratio;
    float ymax = (rect->ymax - sample_node_output->pad_y) *
                 sample_node_output->ratio;
    xmin = std::max(0.0f, std::min(
        xmin, static_cast<float>(sample_node_output->source_width - 1)));
    ymin = std::max(0.0f, std::min(
        ymin, static_cast<float>(sample_node_output->source_height - 1)));
    xmax = std::max(0.0f, std::min(
        xmax, static_cast<float>(sample_node_output->source_width - 1)));
    ymax = std::max(0.0f, std::min(
        ymax, static_cast<float>(sample_node_output->source_height - 1)));
    if (xmax <= xmin || ymax <= ymin)
    {
      continue;
    }

    const int x1 = static_cast<int>(std::round(xmin));
    const int y1 = static_cast<int>(std::round(ymin));
    const int x2 = static_cast<int>(std::round(xmax));
    const int y2 = static_cast<int>(std::round(ymax));

    std::stringstream ss;
    ss << "det rect: " << x1 << " " << y1 << " " << x2
       << " " << y2 << ", det type: " << rect->class_name
       << ", score:" << rect->score;
    RCLCPP_INFO(rclcpp::get_logger("YoloDetectionNode"), "%s", ss.str().c_str());

    ai_msgs::msg::Roi roi;
    roi.rect.set__x_offset(x1);
    roi.rect.set__y_offset(y1);
    roi.rect.set__width(x2 - x1);
    roi.rect.set__height(y2 - y1);
    roi.set__confidence(rect->score);

    ai_msgs::msg::Target target;
    target.set__type(rect->class_name);
    target.rois.emplace_back(roi);
    pub_data->targets.emplace_back(std::move(target));
  }

  if (node_output->rt_stat)
  {
    pub_data->set__fps(round(node_output->rt_stat->output_fps));
    if (node_output->rt_stat->fps_updated)
    {
      auto tp_now = std::chrono::system_clock::now();
      auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                          tp_now - tp_start)
                          .count();
      RCLCPP_WARN(rclcpp::get_logger("YoloDetectionNode"),
                  "input fps: %.2f, out fps: %.2f, infer time ms: %d, "
                  "post process time ms: %d",
                  node_output->rt_stat->input_fps,
                  node_output->rt_stat->output_fps,
                  node_output->rt_stat->infer_time_ms,
                  interval);
    }
  }

  msg_pub_->publish(std::move(pub_data));

  return 0;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<YoloDetectionNode>());
  rclcpp::shutdown();
  return 0;
}