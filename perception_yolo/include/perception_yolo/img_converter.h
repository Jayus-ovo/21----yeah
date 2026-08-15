#ifndef PERCEPTION_YOLO_IMG_CONVERTER_H
#define PERCEPTION_YOLO_IMG_CONVERTER_H

#include <memory>
#include <string>
#include <vector>

#include "ai_msgs/msg/perception_targets.hpp"
#include "dnn_node/dnn_node_data.h"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

using hobot::dnn_node::DNNTensor;
using hobot::dnn_node::NV12PyramidInput;

#define ALIGNED_2E(w, alignment) \
  ((static_cast<uint32_t>(w) + (alignment - 1U)) & (~(alignment - 1U)))
#define ALIGN_4(w) ALIGNED_2E(w, 4U)
#define ALIGN_8(w) ALIGNED_2E(w, 8U)
#define ALIGN_16(w) ALIGNED_2E(w, 16U)
#define ALIGN_64(w) ALIGNED_2E(w, 64U)

static std::vector<cv::Scalar> colors{
    cv::Scalar(255, 0, 0),
    cv::Scalar(255, 255, 0),
    cv::Scalar(0, 255, 0),
    cv::Scalar(0, 0, 255),
};

enum class ImageType { BGR = 0, NV12 = 1, BIN = 2 };

class ImgConverter {
 public:

  static std::shared_ptr<NV12PyramidInput> GetNV12Pyramid(const cv::Mat &image,
                                                          int scaled_img_height,
                                                          int scaled_img_width);

};

#endif  // PERCEPTION_YOLO_IMG_CONVERTER_H