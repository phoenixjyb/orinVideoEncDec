#pragma once

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/logger.hpp>

namespace cr_h265_publisher {

struct H265Packet {
  std::vector<uint8_t> data;
  uint64_t pts_ns = 0;
  bool is_keyframe = false;
};

struct GstH265EncoderOptions {
  std::string device;

  // Must be a GStreamer raw video format string (e.g. "UYVY", "NV16", "YUY2").
  std::string input_format = "UYVY";

  int input_width = 1920;
  int input_height = 1536;
  int output_width = 1920;
  int output_height = 1536;
  int fps = 30;

  uint32_t bitrate = 8'000'000;
  uint32_t iframeinterval = 30;
  uint32_t idrinterval = 30;
  bool insert_sps_pps = true;
  bool insert_aud = false;
  bool maxperf_enable = true;
  int preset_level = 1;  // 1=UltraFastPreset
  int profile = 0;       // 0=Main
};

class GstH265Encoder {
public:
  using OnPacketCallback = std::function<void(H265Packet&&)>;

  GstH265Encoder(const GstH265EncoderOptions& options, rclcpp::Logger logger);
  ~GstH265Encoder();

  GstH265Encoder(const GstH265Encoder&) = delete;
  GstH265Encoder& operator=(const GstH265Encoder&) = delete;

  bool start(std::string* error);
  void stop();

  void set_on_packet(OnPacketCallback cb);

private:
  static GstFlowReturn on_new_sample_static(GstAppSink* sink, gpointer user_data);
  GstFlowReturn on_new_sample(GstAppSink* sink);

  void bus_loop();
  std::string build_pipeline(std::string* error) const;

  GstH265EncoderOptions options_;
  rclcpp::Logger logger_;
  OnPacketCallback on_packet_;

  std::atomic<bool> running_{false};

  GstElement* pipeline_{nullptr};
  GstAppSink* appsink_{nullptr};
  GstBus* bus_{nullptr};
  std::thread bus_thread_;
};

}  // namespace cr_h265_publisher
