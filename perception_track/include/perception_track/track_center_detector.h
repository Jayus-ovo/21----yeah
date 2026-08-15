#ifndef PERCEPTION_TRACK_CENTER_DETECTOR_H_
#define PERCEPTION_TRACK_CENTER_DETECTOR_H_

#include <atomic>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "dnn_node/dnn_node.h"
#include "dnn_node/dnn_node_data.h"
#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "ai_msgs/msg/perception_targets.hpp"
#include "sensor_msgs/msg/image.hpp"

using rclcpp::NodeOptions;

using hobot::dnn_node::DNNInput;
using hobot::dnn_node::DnnNode;
using hobot::dnn_node::DnnNodeOutput;
using hobot::dnn_node::ModelTaskType;
using hobot::dnn_node::DNNTensor;

namespace perception_track {

class TrackPointResult {
 public:
  float x;
  float y;
  void Reset() {x = -1.0; y = -1.0;}
};

class TrackPointParser {
 public:
  TrackPointParser() {}
  ~TrackPointParser() {}
  int32_t Parse(
      std::shared_ptr<TrackPointResult>& output,
      std::shared_ptr<DNNTensor>& output_tensor);
};

class TrackCenterDetector : public DnnNode {
 public:
  TrackCenterDetector(const std::string& node_name,
                        const NodeOptions &options = NodeOptions());
  ~TrackCenterDetector() override;

 protected:
  int SetNodePara() override;
  int PostProcess(const std::shared_ptr<DnnNodeOutput> &outputs) override;

 private:
  int Predict(std::vector<std::shared_ptr<DNNInput>> &dnn_inputs,
              const std::shared_ptr<DnnNodeOutput> &output,
              const std::shared_ptr<std::vector<hbDNNRoi>> rois);
  void image_callback(
    const hbm_img_msgs::msg::HbmMsg1080P::SharedPtr msg);
  void mode_switch_callback(const std_msgs::msg::String::SharedPtr msg);
  bool GetParams();
  bool AssignParams(const std::vector<rclcpp::Parameter> & parameters);
  ModelTaskType model_task_type_ = ModelTaskType::ModelInferType;
  rclcpp::Subscription<hbm_img_msgs::msg::HbmMsg1080P>::SharedPtr
    hbmem_sub_ = nullptr;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
    mode_sub_ = nullptr;
  rclcpp::Publisher<ai_msgs::msg::PerceptionTargets>::SharedPtr publisher_ =
      nullptr;
  cv::Mat image_bgr_;
  std::string model_path_ = "config/bravo_centerline.bin";
  std::string sub_img_topic_ = "/hbmem_img";
  std::string mode_name_ = "disabled";
  std::atomic<bool> infer_enabled_{false};
};

}  // namespace perception_track

#endif  // PERCEPTION_TRACK_CENTER_DETECTOR_H_
