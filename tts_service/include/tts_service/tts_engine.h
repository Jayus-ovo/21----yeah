#ifndef TTS_SERVICE_ENGINE_H_
#define TTS_SERVICE_ENGINE_H_

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tts_api.h"
#include "utils/alsa_device.h"

namespace tts_service {

class TtsEngineNode {
 public:
  TtsEngineNode(rclcpp::Node::SharedPtr& nh);
  ~TtsEngineNode();

  void OnGetText(const std_msgs::msg::String::SharedPtr msg);

 private:
  rclcpp::Node::SharedPtr nh_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr text_sub_;
  std::string sub_topic_name_ = "/tts_text";
  std::string device_name_ = "hw:0,1";
  double volume_gain_ = 1.0;

  void OnTextMessage(const std_msgs::msg::String::SharedPtr msg);
  void MessageLoop();
  void PlaybackLoop();
  void ShutdownPlayback();

  int TextToPCM(const std::string& text, std::unique_ptr<float[]>& pcm,
                int& pcm_len);
  int Synthesize(const std::string& text, std::unique_ptr<float[]>& pcm,
                 int& pcm_len);

  void WarmupQRCodeFragments();
  void WarmupCommonCharsCache();

  int CacheToMemory(const std::string& text);
  int EnsureDiskCache(const std::string& text, bool& generated);

  bool CreateCacheDirectory();
  std::string BuildCachePath(const std::string& text) const;
  bool LoadFromDisk(const std::string& text, std::vector<float>& data);
  bool SaveToDisk(const std::string& text, const std::vector<float>& data);

  bool AssembleQRAnnouncement(const std::string& text,
                              std::unique_ptr<float[]>& pcm, int& pcm_len);
  void QueuePCM(std::unique_ptr<float[]> pcm, int len);

  std::queue<std_msgs::msg::String::SharedPtr> message_queue_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;

  std::queue<std::pair<std::unique_ptr<float[]>, int>> playback_queue_;
  std::mutex playback_mutex_;
  std::condition_variable playback_cv_;

  std::atomic<bool> stopped_{false};
  std::thread worker_thread_;
  std::thread speaker_thread_;
  std::thread cache_thread_;

  static constexpr size_t kMaxQueueSize_ = 10;
  static constexpr size_t kMaxPlaybackSize_ = 5;

  void* tts_handle_ = nullptr;
  alsa_device_t* speaker_dev_ = nullptr;
  char* pcm_buffer_ = nullptr;

  bool qrcode_warmup_ = true;
  bool disk_cache_enabled_ = true;
  bool common_chars_enabled_ = false;
  std::string cache_dir_;
  std::string common_chars_path_;

  std::unordered_map<std::string, std::vector<float>> memory_cache_;
  std::mutex synth_mutex_;
  std::mutex disk_mutex_;
};

}  // namespace tts_service

#endif