#include "cr_h265_publisher/gst_h265_encoder.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace cr_h265_publisher {
namespace {

std::vector<std::string> default_topics_for_devices(const std::vector<std::string>& devices) {
  std::vector<std::string> topics;
  topics.reserve(devices.size());
  for (size_t i = 0; i < devices.size(); ++i) {
    topics.push_back("/cr/camera/h265/cam" + std::to_string(i));
  }
  return topics;
}

std::vector<std::string> default_frame_ids_for_devices(const std::vector<std::string>& devices) {
  std::vector<std::string> frame_ids;
  frame_ids.reserve(devices.size());
  for (size_t i = 0; i < devices.size(); ++i) {
    frame_ids.push_back("cam" + std::to_string(i));
  }
  return frame_ids;
}

}  // namespace

class CrH265PublisherNode final : public rclcpp::Node {
public:
  CrH265PublisherNode() : rclcpp::Node("cr_h265_publisher") {
    this->declare_parameter<std::vector<std::string>>("devices", {"/dev/video0"});
    this->declare_parameter<std::vector<std::string>>("topics", std::vector<std::string>{});
    this->declare_parameter<std::vector<std::string>>("frame_ids", std::vector<std::string>{});
    this->declare_parameter<std::string>("input_format", "UYVY");
    this->declare_parameter<std::vector<std::string>>("input_formats", std::vector<std::string>{});

    this->declare_parameter<int>("input_width", 1920);
    this->declare_parameter<int>("input_height", 1536);
    this->declare_parameter<int>("output_width", 1920);
    this->declare_parameter<int>("output_height", 1536);
    this->declare_parameter<int>("fps", 30);
    this->declare_parameter<int>("control_rate", 1);  // 0=VBR, 1=CBR
    this->declare_parameter<bool>("ratecontrol_enable", true);
    this->declare_parameter<bool>("enable_twopass_cbr", false);
    this->declare_parameter<int>("bitrate", 8'000'000);
    this->declare_parameter<int>("peak_bitrate", 0);  // 0 => encoder default
    this->declare_parameter<int>("vbv_size", 0);      // 0 => encoder default
    this->declare_parameter<int>("num_b_frames", 0);
    this->declare_parameter<int>("num_ref_frames", 1);
    this->declare_parameter<int>("iframeinterval", 30);
    this->declare_parameter<int>("idrinterval", 30);
    this->declare_parameter<bool>("insert_sps_pps", true);
    this->declare_parameter<bool>("insert_aud", false);
    this->declare_parameter<bool>("maxperf_enable", true);
    this->declare_parameter<int>("preset_level", 1);
    this->declare_parameter<int>("profile", 0);

    const auto devices = this->get_parameter("devices").as_string_array();
    if (devices.empty()) {
      throw std::runtime_error("Parameter 'devices' must be non-empty");
    }

    auto topics = this->get_parameter("topics").as_string_array();
    if (topics.empty()) {
      topics = default_topics_for_devices(devices);
    }
    if (topics.size() != devices.size()) {
      throw std::runtime_error("Parameter 'topics' must match 'devices' length");
    }

    auto frame_ids = this->get_parameter("frame_ids").as_string_array();
    if (frame_ids.empty()) {
      frame_ids = default_frame_ids_for_devices(devices);
    }
    if (frame_ids.size() != devices.size()) {
      throw std::runtime_error("Parameter 'frame_ids' must match 'devices' length");
    }

    const auto input_format = this->get_parameter("input_format").as_string();
    auto input_formats = this->get_parameter("input_formats").as_string_array();
    if (!input_formats.empty() && input_formats.size() != devices.size()) {
      throw std::runtime_error("Parameter 'input_formats' must match 'devices' length");
    }

    const auto in_w = this->get_parameter("input_width").as_int();
    const auto in_h = this->get_parameter("input_height").as_int();
    const auto out_w = this->get_parameter("output_width").as_int();
    const auto out_h = this->get_parameter("output_height").as_int();
    const auto fps = this->get_parameter("fps").as_int();

    const auto control_rate = this->get_parameter("control_rate").as_int();
    if (control_rate != 0 && control_rate != 1) {
      throw std::runtime_error("Parameter 'control_rate' must be 0 (VBR) or 1 (CBR)");
    }

    const auto num_b_frames = this->get_parameter("num_b_frames").as_int();
    if (num_b_frames < 0 || num_b_frames > 2) {
      throw std::runtime_error("Parameter 'num_b_frames' must be in range 0..2");
    }

    const auto num_ref_frames = this->get_parameter("num_ref_frames").as_int();
    if (num_ref_frames < 0 || num_ref_frames > 8) {
      throw std::runtime_error("Parameter 'num_ref_frames' must be in range 0..8");
    }

    const auto bitrate = this->get_parameter("bitrate").as_int();
    if (bitrate < 0) {
      throw std::runtime_error("Parameter 'bitrate' must be >= 0");
    }

    const auto peak_bitrate = this->get_parameter("peak_bitrate").as_int();
    if (peak_bitrate < 0) {
      throw std::runtime_error("Parameter 'peak_bitrate' must be >= 0");
    }

    const auto vbv_size = this->get_parameter("vbv_size").as_int();
    if (vbv_size < 0) {
      throw std::runtime_error("Parameter 'vbv_size' must be >= 0");
    }

    GstH265EncoderOptions opts;
    opts.input_format = input_format;
    opts.input_width = static_cast<int>(in_w);
    opts.input_height = static_cast<int>(in_h);
    opts.output_width = static_cast<int>(out_w);
    opts.output_height = static_cast<int>(out_h);
    opts.fps = static_cast<int>(fps);
    opts.control_rate = static_cast<int>(control_rate);
    opts.ratecontrol_enable = this->get_parameter("ratecontrol_enable").as_bool();
    opts.enable_twopass_cbr = this->get_parameter("enable_twopass_cbr").as_bool();
    opts.bitrate = static_cast<uint32_t>(bitrate);
    opts.peak_bitrate = static_cast<uint32_t>(peak_bitrate);
    opts.vbv_size = static_cast<uint32_t>(vbv_size);
    opts.num_b_frames = static_cast<int>(num_b_frames);
    opts.num_ref_frames = static_cast<int>(num_ref_frames);
    opts.iframeinterval = static_cast<uint32_t>(this->get_parameter("iframeinterval").as_int());
    opts.idrinterval = static_cast<uint32_t>(this->get_parameter("idrinterval").as_int());
    opts.insert_sps_pps = this->get_parameter("insert_sps_pps").as_bool();
    opts.insert_aud = this->get_parameter("insert_aud").as_bool();
    opts.maxperf_enable = this->get_parameter("maxperf_enable").as_bool();
    opts.preset_level = static_cast<int>(this->get_parameter("preset_level").as_int());
    opts.profile = static_cast<int>(this->get_parameter("profile").as_int());

    const auto qos = rclcpp::SensorDataQoS();
    for (size_t i = 0; i < devices.size(); ++i) {
      EncoderInstance instance;
      instance.topic = topics[i];
      instance.frame_id = frame_ids[i];
      instance.publisher = this->create_publisher<sensor_msgs::msg::CompressedImage>(instance.topic, qos);

      opts.device = devices[i];
      opts.input_format = input_formats.empty() ? input_format : input_formats[i];
      instance.encoder = std::make_unique<GstH265Encoder>(opts, this->get_logger());
      instance.encoder->set_on_packet([this, pub = instance.publisher, frame_id = instance.frame_id](H265Packet&& pkt) {
        sensor_msgs::msg::CompressedImage msg;
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = frame_id;
        msg.format = "h265";
        msg.data = std::move(pkt.data);
        pub->publish(std::move(msg));
      });

      std::string err;
      if (!instance.encoder->start(&err)) {
        throw std::runtime_error("Failed to start encoder for " + devices[i] + ": " + err);
      }

      encoders_.push_back(std::move(instance));
      RCLCPP_INFO(this->get_logger(), "Publishing %s -> %s", devices[i].c_str(), topics[i].c_str());
    }
  }

  ~CrH265PublisherNode() override {
    for (auto& instance : encoders_) {
      if (instance.encoder) {
        instance.encoder->stop();
      }
    }
  }

private:
  struct EncoderInstance {
    std::string topic;
    std::string frame_id;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher;
    std::unique_ptr<GstH265Encoder> encoder;
  };

  std::vector<EncoderInstance> encoders_;
};

}  // namespace cr_h265_publisher

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<cr_h265_publisher::CrH265PublisherNode>());
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("cr_h265_publisher"), "Fatal: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
