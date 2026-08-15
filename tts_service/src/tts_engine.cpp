// -- Team 2 TTS引擎 — Hobot 语音合成 + ALSA 音频输出 --
#include "tts_service/tts_engine.h"

#include <alsa/asoundlib.h>

#include <algorithm>
#include <cerrno>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <sys/stat.h>
#include <sys/types.h>

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace tts_service {

namespace {

constexpr char kCacheMagic[] = "TTSMPC1";
constexpr size_t kMagicLen = sizeof(kCacheMagic) - 1;
constexpr uint32_t kMaxTextBytes = 1024 * 1024;
constexpr uint32_t kMaxPCMElements = 10 * 1000 * 1000;
constexpr size_t kBarWidth = 30;

std::string MakeProgressBar(size_t cur, size_t total) {
  const size_t bounded = std::min(cur, total);
  const size_t filled = total == 0 ? kBarWidth : bounded * kBarWidth / total;
  const size_t pct = total == 0 ? 100 : bounded * 100 / total;

  std::ostringstream oss;
  oss << "[" << std::string(filled, '#')
      << std::string(kBarWidth - filled, '-') << "] "
      << bounded << "/" << total << " (" << pct << "%)";
  return oss.str();
}

size_t Utf8CharLen(unsigned char byte) {
  if ((byte & 0x80) == 0) return 1;
  if ((byte & 0xE0) == 0xC0) return 2;
  if ((byte & 0xF0) == 0xE0) return 3;
  if ((byte & 0xF8) == 0xF0) return 4;
  return 0;
}

}  // namespace

TtsEngineNode::TtsEngineNode(rclcpp::Node::SharedPtr& nh) : nh_(nh) {
  nh_->declare_parameter<std::string>("playback_device", device_name_);
  nh_->get_parameter<std::string>("playback_device", device_name_);
  nh_->declare_parameter<double>("volume_gain", volume_gain_);
  nh_->get_parameter<double>("volume_gain", volume_gain_);

  speaker_dev_ = alsa_device_allocate();
  if (!speaker_dev_) {
    RCLCPP_ERROR(nh_->get_logger(), "Failed to allocate speaker device");
    throw std::runtime_error("Allocate alsa device failed");
  }
  speaker_dev_->name = const_cast<char*>(device_name_.c_str());
  speaker_dev_->format = SND_PCM_FORMAT_S16;
  speaker_dev_->direct = SND_PCM_STREAM_PLAYBACK;
  speaker_dev_->rate = 16000;
  speaker_dev_->channels = 2;
  speaker_dev_->buffer_time = 0;
  speaker_dev_->nperiods = 4;
  speaker_dev_->period_size = 512;

  auto ret = alsa_device_init(speaker_dev_);
  if (ret < 0) {
    alsa_device_free(speaker_dev_);
    speaker_dev_ = nullptr;
    RCLCPP_ERROR(nh_->get_logger(), "ALSA init failed, ret=%d", ret);
    throw std::runtime_error("ALSA device init failed");
  }

  nh_->declare_parameter<std::string>("topic_sub", sub_topic_name_);
  nh_->get_parameter<std::string>("topic_sub", sub_topic_name_);
  text_sub_ = nh_->create_subscription<std_msgs::msg::String>(
      sub_topic_name_, 10,
      std::bind(&TtsEngineNode::OnTextMessage, this, std::placeholders::_1));

  int err_code = 0;
  std::string tros_distro = std::string(std::getenv("TROS_DISTRO") ? std::getenv("TROS_DISTRO") : "");
  tts_handle_ =
      wetts_init(std::string("/opt/tros/" + tros_distro + "/lib/tts_service/tts_model").c_str(),
      "tts.flags", &err_code);
  if (!tts_handle_) {
    RCLCPP_ERROR(nh_->get_logger(), "wetts_init failed, err=%d", err_code);
    alsa_device_deinit(speaker_dev_);
    alsa_device_free(speaker_dev_);
    speaker_dev_ = nullptr;
    throw std::runtime_error("TTS model init failed");
  }

  struct audio_info info = wetts_audio_info(tts_handle_);
  pcm_buffer_ = new char[info.max_len];

  RCLCPP_INFO_STREAM(nh_->get_logger(), "Sample rate: " << info.sample_rate);
  RCLCPP_INFO_STREAM(nh_->get_logger(), "Bit depth: " << info.bit_depth);
  RCLCPP_INFO_STREAM(nh_->get_logger(), "Channels: " << info.num_channels);
  RCLCPP_INFO_STREAM(nh_->get_logger(), "Max audio seconds: " << info.max_dur_ms / 1000);

  cache_dir_ = std::string(TTS_SERVICE_SOURCE_DIR) + "/pcm_cache";
  try {
    const auto share_dir = ament_index_cpp::get_package_share_directory("tts_service");
    common_chars_path_ = share_dir + "/resources/common_3500_chars.txt";
  } catch (const std::exception& ex) {
    RCLCPP_WARN(nh_->get_logger(), "Package share dir error: %s", ex.what());
  }

  nh_->declare_parameter<bool>("disk_cache_enabled", disk_cache_enabled_);
  nh_->get_parameter<bool>("disk_cache_enabled", disk_cache_enabled_);
  nh_->declare_parameter<std::string>("pcm_cache_dir", cache_dir_);
  nh_->get_parameter<std::string>("pcm_cache_dir", cache_dir_);
  nh_->declare_parameter<bool>("common_chars_cache_enabled", common_chars_enabled_);
  nh_->get_parameter<bool>("common_chars_cache_enabled", common_chars_enabled_);
  nh_->declare_parameter<std::string>("common_chars_file", common_chars_path_);
  nh_->get_parameter<std::string>("common_chars_file", common_chars_path_);

  if (disk_cache_enabled_ && !CreateCacheDirectory()) {
    RCLCPP_ERROR(nh_->get_logger(), "Failed to create cache dir: %s", cache_dir_.c_str());
    disk_cache_enabled_ = false;
  }
  if (disk_cache_enabled_) {
    RCLCPP_INFO(nh_->get_logger(), "PCM cache dir: %s", cache_dir_.c_str());
  }

  nh_->declare_parameter<bool>("warmup_enabled", qrcode_warmup_);
  nh_->get_parameter<bool>("warmup_enabled", qrcode_warmup_);
  WarmupQRCodeFragments();

  worker_thread_ = std::thread(&TtsEngineNode::MessageLoop, this);
  speaker_thread_ = std::thread(&TtsEngineNode::PlaybackLoop, this);
  if (disk_cache_enabled_ && common_chars_enabled_) {
    cache_thread_ = std::thread(&TtsEngineNode::WarmupCommonCharsCache, this);
  } else {
    RCLCPP_INFO(nh_->get_logger(), "Common character cache disabled");
  }
}

TtsEngineNode::~TtsEngineNode() {
  ShutdownPlayback();

  if (pcm_buffer_) {
    delete[] pcm_buffer_;
  }
  if (tts_handle_) {
    wetts_free(tts_handle_);
  }
  if (speaker_dev_) {
    alsa_device_deinit(speaker_dev_);
    alsa_device_free(speaker_dev_);
    speaker_dev_ = nullptr;
  }
}

void TtsEngineNode::OnTextMessage(const std_msgs::msg::String::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(queue_mutex_);
  if (message_queue_.size() >= kMaxQueueSize_) {
    message_queue_.pop();
  }
  message_queue_.push(msg);
  queue_cv_.notify_one();
}

void TtsEngineNode::OnGetText(const std_msgs::msg::String::SharedPtr msg) {
  OnTextMessage(msg);
}

int TtsEngineNode::TextToPCM(const std::string& text,
                             std::unique_ptr<float[]>& pcm,
                             int& pcm_len) {
  auto it = memory_cache_.find(text);
  if (it != memory_cache_.end()) {
    pcm_len = static_cast<int>(it->second.size());
    pcm.reset(new float[pcm_len]);
    std::copy(it->second.begin(), it->second.end(), pcm.get());
    return 0;
  }

  std::vector<float> disk_data;
  if (LoadFromDisk(text, disk_data)) {
    pcm_len = static_cast<int>(disk_data.size());
    pcm.reset(new float[pcm_len]);
    std::copy(disk_data.begin(), disk_data.end(), pcm.get());
    RCLCPP_INFO(nh_->get_logger(), "Loaded disk cache: %s", text.c_str());
    return 0;
  }

  auto ret = Synthesize(text, pcm, pcm_len);
  if (ret != 0) {
    return ret;
  }

  if (disk_cache_enabled_) {
    std::vector<float> synth_data(pcm.get(), pcm.get() + pcm_len);
    if (!SaveToDisk(text, synth_data)) {
      RCLCPP_WARN(nh_->get_logger(), "Failed to save disk cache: %s", text.c_str());
    } else {
      RCLCPP_INFO(nh_->get_logger(), "Saved disk cache: %s", text.c_str());
    }
  }
  return 0;
}

int TtsEngineNode::Synthesize(const std::string& text,
                              std::unique_ptr<float[]>& pcm,
                              int& pcm_len) {
  std::lock_guard<std::mutex> lock(synth_mutex_);
  auto err = wetts_synthesis(tts_handle_, text.c_str(), 1, pcm_buffer_, &pcm_len);
  if (err != ERRCODE_TTS_SUCC) {
    RCLCPP_ERROR_STREAM(nh_->get_logger(), "Synthesis failed, code: " << err);
    return -1;
  }
  pcm.reset(new float[pcm_len]);
  memcpy(pcm.get(), pcm_buffer_, pcm_len * sizeof(float));
  return 0;
}

bool TtsEngineNode::CreateCacheDirectory() {
  if (cache_dir_.empty()) return false;

  size_t pos = cache_dir_[0] == '/' ? 1 : 0;
  while (pos <= cache_dir_.size()) {
    pos = cache_dir_.find('/', pos);
    auto dir = cache_dir_.substr(0, pos);
    if (!dir.empty()) {
      struct stat st {};
      if (stat(dir.c_str(), &st) == 0) {
        if (!S_ISDIR(st.st_mode)) return false;
      } else if (mkdir(dir.c_str(), 0755) != 0 && errno != EEXIST) {
        return false;
      }
    }
    if (pos == std::string::npos) break;
    ++pos;
  }
  return true;
}

std::string TtsEngineNode::BuildCachePath(const std::string& text) const {
  uint64_t hash = 1469598103934665603ULL;
  for (unsigned char c : text) {
    hash ^= c;
    hash *= 1099511628211ULL;
  }
  std::ostringstream oss;
  oss << cache_dir_ << "/" << std::hex << std::setw(16)
      << std::setfill('0') << hash << ".pcm";
  return oss.str();
}

bool TtsEngineNode::LoadFromDisk(const std::string& text,
                                 std::vector<float>& data) {
  if (!disk_cache_enabled_) return false;

  std::lock_guard<std::mutex> lock(disk_mutex_);
  std::ifstream in(BuildCachePath(text), std::ios::binary);
  if (!in) return false;

  char magic[kMagicLen] = {};
  uint32_t text_len = 0;
  uint32_t pcm_len = 0;
  in.read(magic, sizeof(magic));
  in.read(reinterpret_cast<char*>(&text_len), sizeof(text_len));
  if (!in || memcmp(magic, kCacheMagic, sizeof(magic)) != 0 ||
      text_len > kMaxTextBytes) {
    return false;
  }

  std::string stored_text(text_len, '\0');
  if (text_len > 0) {
    in.read(&stored_text[0], text_len);
  }
  in.read(reinterpret_cast<char*>(&pcm_len), sizeof(pcm_len));
  if (!in || stored_text != text || pcm_len > kMaxPCMElements) {
    return false;
  }

  data.resize(pcm_len);
  in.read(reinterpret_cast<char*>(data.data()), data.size() * sizeof(float));
  return static_cast<bool>(in);
}

bool TtsEngineNode::SaveToDisk(const std::string& text,
                               const std::vector<float>& data) {
  if (!disk_cache_enabled_ || text.size() > kMaxTextBytes ||
      data.size() > kMaxPCMElements) {
    return false;
  }

  std::lock_guard<std::mutex> lock(disk_mutex_);
  auto path = BuildCachePath(text);
  auto tmp_path = path + ".tmp";
  std::ofstream out(tmp_path, std::ios::binary | std::ios::trunc);
  if (!out) return false;

  auto text_len = static_cast<uint32_t>(text.size());
  auto pcm_len = static_cast<uint32_t>(data.size());
  out.write(kCacheMagic, kMagicLen);
  out.write(reinterpret_cast<const char*>(&text_len), sizeof(text_len));
  out.write(text.data(), text.size());
  out.write(reinterpret_cast<const char*>(&pcm_len), sizeof(pcm_len));
  out.write(reinterpret_cast<const char*>(data.data()), data.size() * sizeof(float));
  out.close();
  if (!out || std::rename(tmp_path.c_str(), path.c_str()) != 0) {
    std::remove(tmp_path.c_str());
    return false;
  }
  return true;
}

int TtsEngineNode::EnsureDiskCache(const std::string& text, bool& generated) {
  generated = false;
  std::vector<float> data;
  if (LoadFromDisk(text, data)) return 0;

  std::unique_ptr<float[]> pcm;
  int pcm_len = 0;
  auto ret = Synthesize(text, pcm, pcm_len);
  if (ret != 0) return ret;

  std::vector<float> synth_data(pcm.get(), pcm.get() + pcm_len);
  if (!SaveToDisk(text, synth_data)) return -1;
  generated = true;
  return 0;
}

int TtsEngineNode::CacheToMemory(const std::string& text) {
  std::unique_ptr<float[]> pcm;
  int pcm_len = 0;
  auto ret = TextToPCM(text, pcm, pcm_len);
  if (ret != 0) return ret;

  memory_cache_[text] = std::vector<float>(pcm.get(), pcm.get() + pcm_len);
  return 0;
}

void TtsEngineNode::WarmupQRCodeFragments() {
  if (!qrcode_warmup_) {
    RCLCPP_WARN(nh_->get_logger(), "QR warmup disabled");
    return;
  }

  RCLCPP_INFO(nh_->get_logger(), "Building QR PCM fragment cache");
  const std::vector<std::string> fragments = {
      "QR", "0", "1", "2", "3", "4", "5", "6", "7",
      "8", "9", "clockwise", "counter-clockwise", "ClockWise", "AntiClockWise"};
  size_t done = 0;
  for (const auto& frag : fragments) {
    if (CacheToMemory(frag) != 0) {
      throw std::runtime_error("QR PCM cache build failed");
    }
    ++done;
    auto bar = MakeProgressBar(done, fragments.size());
    RCLCPP_INFO(nh_->get_logger(), "QR cache progress: %s, frag=%s", bar.c_str(), frag.c_str());
  }
  RCLCPP_INFO(nh_->get_logger(), "QR PCM fragment cache ready");
}

void TtsEngineNode::WarmupCommonCharsCache() {
  std::ifstream in(common_chars_path_);
  if (!in) {
    RCLCPP_ERROR(nh_->get_logger(), "Cannot open chars file: %s", common_chars_path_.c_str());
    return;
  }

  const std::string content((std::istreambuf_iterator<char>(in)),
                             std::istreambuf_iterator<char>());
  size_t done = 0, generated = 0, reused = 0, failed = 0;

  auto bar = MakeProgressBar(0, 3500);
  RCLCPP_INFO(nh_->get_logger(), "Building common chars cache: %s", bar.c_str());
  size_t offset = 0;
  while (offset < content.size() && !stopped_) {
    auto byte = static_cast<unsigned char>(content[offset]);
    if (std::isspace(byte)) {
      ++offset;
      continue;
    }
    auto char_len = Utf8CharLen(byte);
    if (char_len == 0 || offset + char_len > content.size()) {
      RCLCPP_WARN(nh_->get_logger(), "Invalid UTF-8 in char list");
      ++offset;
      continue;
    }

    auto ch = content.substr(offset, char_len);
    bool was_gen = false;
    if (EnsureDiskCache(ch, was_gen) == 0) {
      if (was_gen) ++generated;
      else ++reused;
    } else {
      ++failed;
      RCLCPP_WARN(nh_->get_logger(), "Failed to cache: %s", ch.c_str());
    }
    ++done;
    offset += char_len;

    if (done % 100 == 0) {
      bar = MakeProgressBar(done, 3500);
      RCLCPP_INFO(nh_->get_logger(), "Chars cache: %s, gen=%zu, reuse=%zu, fail=%zu",
                  bar.c_str(), generated, reused, failed);
    }
  }

  if (stopped_) {
    bar = MakeProgressBar(done, 3500);
    RCLCPP_WARN(nh_->get_logger(), "Char cache stopped: %s", bar.c_str());
    return;
  }
  bar = MakeProgressBar(done, 3500);
  RCLCPP_INFO(nh_->get_logger(), "Char cache done: %s, gen=%zu, reuse=%zu, fail=%zu",
              bar.c_str(), generated, reused, failed);
}

bool TtsEngineNode::AssembleQRAnnouncement(const std::string& text,
                                           std::unique_ptr<float[]>& pcm,
                                           int& pcm_len) {
  if (!qrcode_warmup_) return false;

  const std::string prefix = "QR";
  std::string direction;
  if (text.size() >= std::string("clockwise").size() &&
      text.compare(text.size() - std::string("clockwise").size(),
                  std::string("clockwise").size(), "clockwise") == 0) {
    direction = "clockwise";
  } else if (text.size() >= std::string("counter-clockwise").size() &&
             text.compare(text.size() - std::string("counter-clockwise").size(),
                         std::string("counter-clockwise").size(), "counter-clockwise") == 0) {
    direction = "counter-clockwise";
  } else {
    return false;
  }

  if (text.rfind(prefix, 0) != 0 ||
      text.size() <= prefix.size() + direction.size()) {
    return false;
  }

  const std::string payload =
      text.substr(prefix.size(), text.size() - prefix.size() - direction.size());

  std::vector<std::string> fragments = {prefix};
  if (std::all_of(payload.begin(), payload.end(),
                  [](unsigned char c) { return std::isdigit(c); })) {
    size_t value = 0;
    for (char digit : payload) {
      value = value * 10 + static_cast<size_t>(digit - '0');
      if (value > 9999) return false;
      fragments.emplace_back(1, digit);
    }
    if (value < 1) return false;
  } else if (payload == "ClockWise") {
    fragments.push_back("clockwise");
  } else if (payload == "AntiClockWise") {
    fragments.push_back("counter-clockwise");
  } else {
    return false;
  }
  fragments.push_back(direction);

  size_t total = 0;
  for (const auto& frag : fragments) {
    auto it = memory_cache_.find(frag);
    if (it == memory_cache_.end()) return false;
    total += it->second.size();
  }

  pcm_len = static_cast<int>(total);
  pcm.reset(new float[pcm_len]);
  auto out = pcm.get();
  for (const auto& frag : fragments) {
    const auto& cached = memory_cache_.at(frag);
    out = std::copy(cached.begin(), cached.end(), out);
  }
  return true;
}

void TtsEngineNode::QueuePCM(std::unique_ptr<float[]> pcm, int len) {
  std::lock_guard<std::mutex> lock(playback_mutex_);
  if (playback_queue_.size() >= kMaxPlaybackSize_) {
    playback_queue_.pop();
  }
  playback_queue_.push(std::make_pair(std::move(pcm), len));
  playback_cv_.notify_one();
}

void TtsEngineNode::MessageLoop() {
  auto splitText = [](std::string& input, std::vector<std::string>& segments) {
    std::string seg;
    size_t start = 0;
    size_t idx = 0;

    auto isCjkPunc = [](const std::string& s, size_t i) {
      if (i + 2 >= s.size()) return false;
      return (s[i] == '\xEF' && s[i + 1] == '\xBC' &&
              (s[i + 2] == '\x8C' || s[i + 2] == '\x9F' ||
               s[i + 2] == '\x9A' || s[i + 2] == '\x81')) ||
             (s[i] == '\xE3' && s[i + 1] == '\x80' &&
              (s[i + 2] == '\x82' || s[i + 2] == '\x81'));
    };

    while (idx < input.length()) {
      if (isCjkPunc(input, idx)) {
        seg = input.substr(start, idx - start);
        if (!seg.empty()) {
          segments.push_back(seg);
          seg.clear();
        }
        start = idx + 3;
        idx += 3;
        continue;
      }
      auto ch = static_cast<unsigned char>(input[idx]);
      if (std::ispunct(ch)) {
        seg = input.substr(start, idx - start);
        if (!seg.empty()) {
          segments.push_back(seg);
          seg.clear();
        }
        start = idx + 1;
      }
      if (std::isspace(ch)) {
        seg = input.substr(start, idx - start);
        if (!seg.empty()) {
          segments.push_back(seg);
          seg.clear();
        }
        start = idx + 1;
      }
      if (std::isupper(ch)) {
        input[idx] = static_cast<char>(std::tolower(ch));
      }
      ++idx;
    }

    seg = input.substr(start);
    if (!seg.empty()) {
      segments.push_back(seg);
    }
  };

  while (rclcpp::ok()) {
    std_msgs::msg::String::SharedPtr msg;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait(lock, [this] { return !message_queue_.empty() || stopped_; });
      if (stopped_) break;
      msg = message_queue_.front();
      message_queue_.pop();
    }

    std::unique_ptr<float[]> qr_pcm;
    int qr_len = 0;
    if (AssembleQRAnnouncement(msg->data, qr_pcm, qr_len)) {
      QueuePCM(std::move(qr_pcm), qr_len);
      RCLCPP_INFO(nh_->get_logger(), "Queued QR: %s", msg->data.c_str());
      continue;
    }

    std::vector<std::string> parts;
    splitText(msg->data, parts);
    for (auto& part : parts) {
      std::unique_ptr<float[]> pcm;
      int pcm_len;
      auto ret = TextToPCM(part, pcm, pcm_len);
      if (!ret) {
        QueuePCM(std::move(pcm), pcm_len);
      }
    }
  }
}

void TtsEngineNode::PlaybackLoop() {
  while (rclcpp::ok()) {
    std::unique_lock<std::mutex> lock(playback_mutex_);
    playback_cv_.wait(lock, [this] { return !playback_queue_.empty() || stopped_; });
    if (stopped_ && playback_queue_.empty()) break;

    while (!playback_queue_.empty()) {
      auto pcm_data = std::move(playback_queue_.front().first);
      auto pcm_len = playback_queue_.front().second;
      playback_queue_.pop();

      std::vector<int16_t> stereo;
      auto pcm_float = pcm_data.get();
      for (int i = 0; i < pcm_len; i++) {
        stereo.push_back(*pcm_float);
        stereo.push_back(*pcm_float);
        pcm_float++;
      }

      if (speaker_dev_) {
        snd_pcm_sframes_t frames = snd_pcm_bytes_to_frames(
            speaker_dev_->handle, stereo.size() * sizeof(int16_t));
        snd_pcm_prepare(speaker_dev_->handle);
        alsa_device_write(speaker_dev_, stereo.data(), frames);
        snd_pcm_drop(speaker_dev_->handle);
      }
    }
  }
}

void TtsEngineNode::ShutdownPlayback() {
  if (!stopped_) {
    stopped_ = true;
    queue_cv_.notify_one();
    playback_cv_.notify_one();
    if (worker_thread_.joinable()) worker_thread_.join();
    if (speaker_thread_.joinable()) speaker_thread_.join();
    if (cache_thread_.joinable()) cache_thread_.join();
  }
}

}  // namespace tts_service