// -- Team 2 YOLO 解析器 — 推理结果 JSON 解析 + 后处理 --
#include "perception_yolo/yolo_parser.h"

#include "rapidjson/document.h"
#include "rapidjson/istreamwrapper.h"
#include "rapidjson/writer.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <functional>
#include <memory>

using hobot::dnn_node::DNNTensor;

namespace hobot
{
  namespace dnn_node
  {
    namespace perception_yolo
    {

      void ParseOutputTensor(std::shared_ptr<DNNTensor> tensor,
                       int layer,
                       std::vector<DetectionResult> &results,
                       YoloModelConfig &yolo_config);

      int ParseColorTensor(std::shared_ptr<DNNTensor> tensor,
                           int layer,
                           std::vector<DetectionResult> &results,
                           YoloModelConfig &yolo_config);

      void nms_suppression(std::vector<DetectionResult> &input,
                     float iou_threshold,
                     int top_k,
                     std::vector<std::shared_ptr<DetectionResult>> &result,
                     bool suppress);

      int get_tensor_hw(std::shared_ptr<DNNTensor> tensor, int *height, int *width);
      int get_tensor_shape(std::shared_ptr<DNNTensor> tensor,
                           int *height,
                           int *width,
                           int *valid_channel,
                           int *aligned_height,
                           int *aligned_width,
                           int *aligned_channel);

      int get_color_layer_by_hw(int height)
      {
        if (height == 80)
        {
          return 0;
        }
        if (height == 40)
        {
          return 1;
        }
        if (height == 20)
        {
          return 2;
        }
        return -1;
      }

      bool is_color_model(const YoloModelConfig &yolo_config)
      {
        return yolo_config.class_num == 1 &&
               yolo_config.class_names.size() == 1 &&
               yolo_config.class_names[0] == "tuWen";
      }

      template <class ForwardIterator>
      inline size_t argmax(ForwardIterator first, ForwardIterator last)
      {
        return std::distance(first, std::max_element(first, last));
      }

      void ParseOutputTensor(std::shared_ptr<DNNTensor> tensor,
                       int layer,
                       std::vector<DetectionResult> &results,
                       YoloModelConfig &yolo_config)
      {
        hbSysFlushMem(&(tensor->sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
        int num_classes = yolo_config.class_num;
        int stride = yolo_config.strides[layer];
        int num_pred = yolo_config.class_num + 4 + 1;

        std::vector<float> class_pred(yolo_config.class_num, 0.0);
        std::vector<std::pair<double, double>> &anchors =
            yolo_config.anchors_table[layer];

        int height, width;
        auto ret = get_tensor_hw(tensor, &height, &width);
        if (ret != 0)
        {
          RCLCPP_ERROR(rclcpp::get_logger("YoloDetectionParser"),
                       "get_tensor_hw failed");
        }

        int anchor_num = anchors.size();
        auto *data = reinterpret_cast<float *>(tensor->sysMem[0].virAddr);
        for (int h = 0; h < height; h++)
        {
          for (int w = 0; w < width; w++)
          {
            for (int k = 0; k < anchor_num; k++)
            {
              double anchor_x = anchors[k].first;
              double anchor_y = anchors[k].second;
              float *cur_data = data + k * num_pred;
              float objness = cur_data[4];

              int id = argmax(cur_data + 5, cur_data + 5 + num_classes);
              double x1 = 1 / (1 + std::exp(-objness)) * 1;
              double x2 = 1 / (1 + std::exp(-cur_data[id + 5]));
              double confidence = x1 * x2;

              if (confidence < 0.3f)
              {
                continue;
              }

              float center_x = cur_data[0];
              float center_y = cur_data[1];
              float scale_x = cur_data[2];
              float scale_y = cur_data[3];

              double box_center_x =
                  ((1.0 / (1.0 + std::exp(-center_x))) * 2 - 0.5 + w) * stride;
              double box_center_y =
                  ((1.0 / (1.0 + std::exp(-center_y))) * 2 - 0.5 + h) * stride;

              double box_scale_x =
                  std::pow((1.0 / (1.0 + std::exp(-scale_x))) * 2, 2) * anchor_x;
              double box_scale_y =
                  std::pow((1.0 / (1.0 + std::exp(-scale_y))) * 2, 2) * anchor_y;

              double xmin = (box_center_x - box_scale_x / 2.0);
              double ymin = (box_center_y - box_scale_y / 2.0);
              double xmax = (box_center_x + box_scale_x / 2.0);
              double ymax = (box_center_y + box_scale_y / 2.0);

              if (xmax <= 0 || ymax <= 0)
              {
                continue;
              }

              if (xmin > xmax || ymin > ymax)
              {
                continue;
              }
              results.emplace_back(
                  DetectionResult(static_cast<int>(id),
                               xmin,
                               ymin,
                               xmax,
                               ymax,
                               confidence,
                               yolo_config.class_names[static_cast<int>(id)]));
            }
            data = data + num_pred * anchors.size();
          }
        }
      }

      double sigmoid(float value)
      {
        const float clipped = std::max(-50.0f, std::min(50.0f, value));
        return 1.0 / (1.0 + std::exp(-clipped));
      }

      int ParseColorTensor(std::shared_ptr<DNNTensor> tensor,
                           int layer,
                           std::vector<DetectionResult> &results,
                           YoloModelConfig &yolo_config)
      {
        if (!tensor || layer < 0 ||
            layer >= static_cast<int>(yolo_config.strides.size()) ||
            layer >= static_cast<int>(yolo_config.anchors_table.size()))
        {
          return -1;
        }

        hbSysFlushMem(&(tensor->sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);

        int height = 0;
        int width = 0;
        int valid_channel = 0;
        int aligned_height = 0;
        int aligned_width = 0;
        int aligned_channel = 0;
        if (get_tensor_shape(tensor,
                             &height,
                             &width,
                             &valid_channel,
                             &aligned_height,
                             &aligned_width,
                             &aligned_channel) != 0)
        {
          RCLCPP_ERROR(rclcpp::get_logger("YoloDetectionParser"),
                       "Unsupported color tensor layout");
          return -1;
        }

        const int num_classes = yolo_config.class_num;
        const int num_pred = num_classes + 5;
        const int anchor_num = yolo_config.anchors_table[layer].size();
        const int required_channel = num_pred * anchor_num;
        if (valid_channel < required_channel)
        {
          RCLCPP_ERROR(rclcpp::get_logger("YoloDetectionParser"),
                       "Invalid color output channel: valid=%d, required=%d",
                       valid_channel,
                       required_channel);
          return -1;
        }

        const int stride = yolo_config.strides[layer];
        const auto &anchors = yolo_config.anchors_table[layer];
        const auto *data = reinterpret_cast<const float *>(tensor->sysMem[0].virAddr);
        if (!data)
        {
          RCLCPP_ERROR(rclcpp::get_logger("YoloDetectionParser"),
                       "Color output buffer is null");
          return -1;
        }
        const auto &valid_shape = tensor->properties.validShape;
        const bool channels_last =
            valid_shape.numDimensions == 4 &&
            valid_shape.dimensionSize[1] == height &&
            valid_shape.dimensionSize[2] == width &&
            valid_shape.dimensionSize[3] == valid_channel;

        auto tensor_value = [=](int h, int w, int channel) -> float
        {
          size_t index = 0;
          if (channels_last)
          {
            index = (static_cast<size_t>(h) * width + w) *
                        valid_channel +
                    channel;
          }
          else
          {
            index = (static_cast<size_t>(channel) * height + h) *
                        width +
                    w;
          }
          return data[index];
        };

        for (int h = 0; h < height; ++h)
        {
          for (int w = 0; w < width; ++w)
          {
            for (int anchor_index = 0; anchor_index < anchor_num; ++anchor_index)
            {
              const int channel = anchor_index * num_pred;
              int class_id = 0;
              float best_class_logit = tensor_value(h, w, channel + 5);
              for (int class_index = 1; class_index < num_classes; ++class_index)
              {
                const float class_logit =
                    tensor_value(h, w, channel + 5 + class_index);
                if (class_logit > best_class_logit)
                {
                  best_class_logit = class_logit;
                  class_id = class_index;
                }
              }

              const double confidence =
                  sigmoid(tensor_value(h, w, channel + 4)) *
                  sigmoid(best_class_logit);
              if (confidence < yolo_config.score_threshold)
              {
                continue;
              }

              const double center_x =
                  (sigmoid(tensor_value(h, w, channel)) * 2.0 - 0.5 + w) *
                  stride;
              const double center_y =
                  (sigmoid(tensor_value(h, w, channel + 1)) * 2.0 - 0.5 + h) *
                  stride;
              const double scale_x =
                  std::pow(sigmoid(tensor_value(h, w, channel + 2)) * 2.0, 2) *
                  anchors[anchor_index].first;
              const double scale_y =
                  std::pow(sigmoid(tensor_value(h, w, channel + 3)) * 2.0, 2) *
                  anchors[anchor_index].second;

              const double xmin = center_x - scale_x / 2.0;
              const double ymin = center_y - scale_y / 2.0;
              const double xmax = center_x + scale_x / 2.0;
              const double ymax = center_y + scale_y / 2.0;
              if (xmax <= 0 || ymax <= 0 || xmin > xmax || ymin > ymax)
              {
                continue;
              }

              results.emplace_back(DetectionResult(
                  class_id,
                  xmin,
                  ymin,
                  xmax,
                  ymax,
                  confidence,
                  yolo_config.class_names[class_id]));
            }
          }
        }
        return 0;
      }

      void nms_suppression(std::vector<DetectionResult> &input,
                     float iou_threshold,
                     int top_k,
                     std::vector<std::shared_ptr<DetectionResult>> &result,
                     bool suppress)
      {
        std::stable_sort(input.begin(), input.end(), std::greater<DetectionResult>());

        std::vector<bool> skip(input.size(), false);

        std::vector<float> areas;
        areas.reserve(input.size());
        for (size_t i = 0; i < input.size(); i++)
        {
          float width = input[i].xmax - input[i].xmin;
          float height = input[i].ymax - input[i].ymin;
          areas.push_back(width * height);
        }

        int count = 0;
        for (size_t i = 0; count < top_k && i < skip.size(); i++)
        {
          if (skip[i])
          {
            continue;
          }
          skip[i] = true;
          ++count;

          for (size_t j = i + 1; j < skip.size(); ++j)
          {
            if (skip[j])
            {
              continue;
            }
            if (suppress == false)
            {
              if (input[i].id != input[j].id)
              {
                continue;
              }
            }

            float xx1 = std::max(input[i].xmin, input[j].xmin);
            float yy1 = std::max(input[i].ymin, input[j].ymin);
            float xx2 = std::min(input[i].xmax, input[j].xmax);
            float yy2 = std::min(input[i].ymax, input[j].ymax);

            if (xx2 > xx1 && yy2 > yy1)
            {
              float area_intersection = (xx2 - xx1) * (yy2 - yy1);
              float iou_ratio =
                  area_intersection / (areas[j] + areas[i] - area_intersection);
              if (iou_ratio > iou_threshold)
              {
                skip[j] = true;
              }
            }
          }

          auto yolo_res = std::make_shared<DetectionResult>(input[i].id,
                                                         input[i].xmin,
                                                         input[i].ymin,
                                                         input[i].xmax,
                                                         input[i].ymax,
                                                         input[i].score,
                                                         input[i].class_name);
          if (!yolo_res)
          {
            RCLCPP_ERROR(rclcpp::get_logger("YoloDetectionParser"),
                         "invalid yolo_res");
          }

          result.push_back(yolo_res);
        }
      }

      int get_tensor_hw(std::shared_ptr<DNNTensor> tensor, int *height, int *width)
      {
        int h_index = 0;
        int w_index = 0;
        if (tensor->properties.tensorLayout == HB_DNN_LAYOUT_NHWC)
        {
          h_index = 1;
          w_index = 2;
        }
        else if (tensor->properties.tensorLayout == HB_DNN_LAYOUT_NCHW)
        {
          h_index = 2;
          w_index = 3;
        }
        else
        {
          return -1;
        }
        *height = tensor->properties.validShape.dimensionSize[h_index];
        *width = tensor->properties.validShape.dimensionSize[w_index];
        return 0;
      }

      int get_tensor_shape(std::shared_ptr<DNNTensor> tensor,
                           int *height,
                           int *width,
                           int *valid_channel,
                           int *aligned_height,
                           int *aligned_width,
                           int *aligned_channel)
      {
        const auto &valid_shape = tensor->properties.validShape;
        const auto &aligned_shape = tensor->properties.alignedShape;
        const bool channels_last =
            valid_shape.numDimensions == 4 &&
            (valid_shape.dimensionSize[1] == 20 ||
             valid_shape.dimensionSize[1] == 40 ||
             valid_shape.dimensionSize[1] == 80) &&
            valid_shape.dimensionSize[1] == valid_shape.dimensionSize[2];

        if (channels_last)
        {
          *height = valid_shape.dimensionSize[1];
          *width = valid_shape.dimensionSize[2];
          *valid_channel = valid_shape.dimensionSize[3];
          *aligned_height = aligned_shape.dimensionSize[1];
          *aligned_width = aligned_shape.dimensionSize[2];
          *aligned_channel = aligned_shape.dimensionSize[3];
        }
        else if (valid_shape.numDimensions == 4)
        {
          *valid_channel = valid_shape.dimensionSize[1];
          *height = valid_shape.dimensionSize[2];
          *width = valid_shape.dimensionSize[3];
          *aligned_channel = aligned_shape.dimensionSize[1];
          *aligned_height = aligned_shape.dimensionSize[2];
          *aligned_width = aligned_shape.dimensionSize[3];
        }
        else
        {
          return -1;
        }

        *aligned_height = std::max(*height, *aligned_height);
        *aligned_width = std::max(*width, *aligned_width);
        *aligned_channel = std::max(*valid_channel, *aligned_channel);
        return 0;
      }

      int32_t Parse(
          const std::shared_ptr<hobot::dnn_node::DnnNodeOutput> &node_output,
          std::vector<std::shared_ptr<DetectionResult>> &results,
          YoloModelConfig &yolo_config)
      {
        std::vector<DetectionResult> parse_results;
        const bool color_model = is_color_model(yolo_config);
        static std::atomic<bool> color_shape_logged(false);
        bool expected = false;
        if (color_model &&
            color_shape_logged.compare_exchange_strong(expected, true))
        {
          for (size_t i = 0; i < node_output->output_tensors.size(); ++i)
          {
            int height = 0;
            int width = 0;
            int valid_channel = 0;
            int aligned_height = 0;
            int aligned_width = 0;
            int aligned_channel = 0;
            auto tensor = node_output->output_tensors[i];
            if (get_tensor_shape(tensor,
                                 &height,
                                 &width,
                                 &valid_channel,
                                 &aligned_height,
                                 &aligned_width,
                                 &aligned_channel) == 0)
            {
              const char *runtime_layout =
                  tensor->properties.tensorLayout == HB_DNN_LAYOUT_NHWC
                      ? "NHWC"
                      : "NCHW";
              RCLCPP_WARN(rclcpp::get_logger("YoloDetectionParser"),
                          "color output[%zu] runtime=%s valid=%dx%dx%d aligned=%dx%dx%d",
                          i,
                          runtime_layout,
                          height,
                          width,
                          valid_channel,
                          aligned_height,
                          aligned_width,
                          aligned_channel);
            }
          }
        }
        for (size_t i = 0; i < node_output->output_tensors.size(); i++)
        {
          int layer = static_cast<int>(i);
          if (color_model)
          {
            int height = 0;
            int width = 0;
            if (get_tensor_hw(node_output->output_tensors[i], &height, &width) == 0)
            {
              int matched_layer = get_color_layer_by_hw(height);
              if (matched_layer >= 0)
              {
                layer = matched_layer;
              }
            }
          }
          if (color_model)
          {
            if (ParseColorTensor(node_output->output_tensors[i],
                                 layer,
                                 parse_results,
                                 yolo_config) != 0)
            {
              return -1;
            }
          }
          else
          {
            ParseOutputTensor(node_output->output_tensors[i],
                        layer,
                        parse_results,
                        yolo_config);
          }
        }
        nms_suppression(parse_results,
                  color_model ? yolo_config.nms_threshold : 0.65f,
                  color_model ? yolo_config.nms_top_k : 5000,
                  results,
                  false);

        return 0;
      }

    } // namespace perception_yolo
  } // namespace dnn_node
} // namespace hobot