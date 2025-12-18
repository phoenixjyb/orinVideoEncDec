#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <string>
#include <utility>

namespace cr_h265_publisher {
namespace {

class CrH265DumpNode final : public rclcpp::Node {
public:
  CrH265DumpNode() : rclcpp::Node("cr_h265_dump") {
    this->declare_parameter<std::string>("topic", "/cr/camera/h265/test0");
    this->declare_parameter<std::string>("output", "/tmp/out.h265");
    this->declare_parameter<int>("max_messages", 300);
    this->declare_parameter<bool>("append", false);

    const auto topic = this->get_parameter("topic").as_string();
    const auto output = this->get_parameter("output").as_string();
    const auto max_messages = this->get_parameter("max_messages").as_int();
    const auto append = this->get_parameter("append").as_bool();

    if (topic.empty()) {
      throw std::runtime_error("Parameter 'topic' must be non-empty");
    }
    if (output.empty()) {
      throw std::runtime_error("Parameter 'output' must be non-empty");
    }

    max_messages_ = static_cast<int>(max_messages);
    output_path_ = output;

    std::ios::openmode mode = std::ios::binary | std::ios::out;
    mode |= append ? std::ios::app : std::ios::trunc;
    out_.open(output_path_, mode);
    if (!out_.is_open()) {
      throw std::runtime_error("Failed to open output file: " + output_path_);
    }

    sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) { on_msg(std::move(msg)); });

    RCLCPP_INFO(this->get_logger(), "Dumping %s -> %s (max_messages=%d, append=%s)",
                topic.c_str(), output_path_.c_str(), max_messages_, append ? "true" : "false");
  }

  ~CrH265DumpNode() override {
    if (out_.is_open()) {
      out_.close();
    }
  }

private:
  void on_msg(sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) {
    if (!out_.is_open()) {
      return;
    }
    if (!msg) {
      return;
    }

    const auto& data = msg->data;
    if (!data.empty()) {
      out_.write(reinterpret_cast<const char*>(data.data()),
                 static_cast<std::streamsize>(data.size()));
      bytes_written_ += data.size();
    }

    msgs_written_++;

    if (msgs_written_ == 1 || (msgs_written_ % 30 == 0)) {
      const char* fmt = msg->format.empty() ? "(empty)" : msg->format.c_str();
      RCLCPP_INFO(this->get_logger(), "Wrote msg=%d bytes=%zu format=%s last_chunk=%zu",
                  msgs_written_, bytes_written_, fmt, data.size());
    }

    if (max_messages_ > 0 && msgs_written_ >= max_messages_) {
      out_.flush();
      out_.close();
      RCLCPP_INFO(this->get_logger(), "Done. Wrote msg=%d total_bytes=%zu output=%s",
                  msgs_written_, bytes_written_, output_path_.c_str());
      rclcpp::shutdown();
    }
  }

  int max_messages_{0};
  int msgs_written_{0};
  size_t bytes_written_{0};
  std::string output_path_;
  std::ofstream out_;

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;
};

}  // namespace
}  // namespace cr_h265_publisher

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<cr_h265_publisher::CrH265DumpNode>());
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("cr_h265_dump"), "Fatal: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}

