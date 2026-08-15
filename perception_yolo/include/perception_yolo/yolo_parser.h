#ifndef PERCEPTION_YOLO_PARSER_H
#define PERCEPTION_YOLO_PARSER_H

#include "dnn_node/dnn_node_data.h"

namespace hobot {
namespace dnn_node {
namespace perception_yolo {

struct DetectionResult {
  int id;
  float xmin;
  float ymin;
  float xmax;
  float ymax;
  float score;
  std::string class_name;

  DetectionResult(int id_,
               float xmin_,
               float ymin_,
               float xmax_,
               float ymax_,
               float score_,
               std::string class_name_)
      : id(id_),
        xmin(xmin_),
        ymin(ymin_),
        xmax(xmax_),
        ymax(ymax_),
        score(score_),
        class_name(class_name_) {}

  friend bool operator>(const DetectionResult &lhs, const DetectionResult &rhs) {
    return (lhs.score > rhs.score);
  }
};

struct YoloModelConfig {
  std::vector<int> strides;
  std::vector<std::vector<std::pair<double, double>>> anchors_table;
  int class_num;
  std::vector<std::string> class_names;
  std::vector<std::vector<float>> dequantize_scale;
  float score_threshold = 0.28f;
  float nms_threshold = 0.68f;
  int nms_top_k = 6000;
};

int32_t Parse(
    const std::shared_ptr<hobot::dnn_node::DnnNodeOutput> &node_output,
    std::vector<std::shared_ptr<DetectionResult>> &results,
    YoloModelConfig &yolo_config);

}  // namespace perception_yolo
}  // namespace dnn_node
}  // namespace hobot

#endif  // PERCEPTION_YOLO_PARSER_H