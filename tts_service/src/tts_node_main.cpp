// -- Team 2 TTS 节点入口 — Hobot 语音合成主程序 --
#include "tts_service/tts_engine.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("tts_service");
  tts_service::TtsEngineNode engine(nh);
  rclcpp::spin(nh);
  rclcpp::shutdown();
}