#include "cr_h265_publisher/gst_h265_encoder.hpp"

#include <gst/app/gstappsink.h>
#include <rclcpp/logging.hpp>

#include <cctype>
#include <cstring>
#include <string>

namespace cr_h265_publisher {
namespace {

bool ensure_gstreamer_init() {
  if (gst_is_initialized()) {
    return true;
  }
  gst_init(nullptr, nullptr);
  return gst_is_initialized();
}

bool has_element_factory(const char* name) {
  GstElementFactory* factory = gst_element_factory_find(name);
  if (!factory) {
    return false;
  }
  gst_object_unref(factory);
  return true;
}

bool is_safe_caps_token(const std::string& token) {
  if (token.empty()) {
    return false;
  }
  for (unsigned char ch : token) {
    if (!(std::isalnum(ch) || ch == '_' || ch == '-')) {
      return false;
    }
  }
  return true;
}

std::string to_upper_ascii(std::string s) {
  for (char& ch : s) {
    if (ch >= 'a' && ch <= 'z') {
      ch = static_cast<char>(ch - 'a' + 'A');
    }
  }
  return s;
}

std::string normalize_gst_format(const std::string& format_in) {
  const std::string fmt = to_upper_ascii(format_in);
  if (fmt == "YUYV") {
    return "YUY2";
  }
  if (fmt == "YUY2") {
    return "YUY2";
  }
  if (fmt == "UYVY") {
    return "UYVY";
  }
  if (fmt == "VYUY") {
    return "VYUY";
  }
  if (fmt == "YVYU") {
    return "YVYU";
  }
  if (fmt == "NV16") {
    return "NV16";
  }
  if (fmt == "NV12") {
    return "NV12";
  }
  return format_in;
}

}  // namespace

GstH265Encoder::GstH265Encoder(const GstH265EncoderOptions& options, rclcpp::Logger logger)
    : options_(options), logger_(std::move(logger)) {}

GstH265Encoder::~GstH265Encoder() {
  stop();
}

void GstH265Encoder::set_on_packet(OnPacketCallback cb) {
  on_packet_ = std::move(cb);
}

bool GstH265Encoder::start(std::string* error) {
  if (running_.load()) {
    return true;
  }

  if (!ensure_gstreamer_init()) {
    if (error) {
      *error = "Failed to initialize GStreamer";
    }
    return false;
  }

  std::string pipeline_str = build_pipeline(error);
  if (pipeline_str.empty()) {
    return false;
  }

  GError* gst_error = nullptr;
  pipeline_ = gst_parse_launch(pipeline_str.c_str(), &gst_error);
  if (gst_error) {
    if (error) {
      *error = gst_error->message ? gst_error->message : "Unknown GStreamer parse error";
    }
    g_error_free(gst_error);
    gst_error = nullptr;
  }
  if (!pipeline_) {
    if (error && error->empty()) {
      *error = "Failed to create GStreamer pipeline";
    }
    return false;
  }

  GstElement* appsink_element = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
  if (!appsink_element) {
    if (error) {
      *error = "Failed to find appsink element named 'sink'";
    }
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
    return false;
  }
  appsink_ = GST_APP_SINK(appsink_element);

  gst_app_sink_set_emit_signals(appsink_, FALSE);
  gst_app_sink_set_drop(appsink_, TRUE);
  gst_app_sink_set_max_buffers(appsink_, 1);

  GstAppSinkCallbacks callbacks{};
  callbacks.new_sample = &GstH265Encoder::on_new_sample_static;
  gst_app_sink_set_callbacks(appsink_, &callbacks, this, nullptr);

  bus_ = gst_element_get_bus(pipeline_);

  running_ = true;
  GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
  if (ret == GST_STATE_CHANGE_FAILURE) {
    if (error) {
      *error = "Failed to set pipeline to PLAYING";
    }
    running_ = false;
    stop();
    return false;
  }

  bus_thread_ = std::thread(&GstH265Encoder::bus_loop, this);

  RCLCPP_INFO(logger_, "GStreamer encoder started: %s", options_.device.c_str());
  return true;
}

void GstH265Encoder::stop() {
  if (!running_.exchange(false)) {
    // Still need to cleanup partially-initialized state.
  }

  if (bus_thread_.joinable()) {
    bus_thread_.join();
  }

  if (pipeline_) {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
  }

  if (bus_) {
    gst_object_unref(bus_);
    bus_ = nullptr;
  }

  if (appsink_) {
    gst_object_unref(GST_OBJECT(appsink_));
    appsink_ = nullptr;
  }

  if (pipeline_) {
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
  }
}

GstFlowReturn GstH265Encoder::on_new_sample_static(GstAppSink* sink, gpointer user_data) {
  auto* self = static_cast<GstH265Encoder*>(user_data);
  return self->on_new_sample(sink);
}

GstFlowReturn GstH265Encoder::on_new_sample(GstAppSink* sink) {
  if (!running_.load()) {
    return GST_FLOW_FLUSHING;
  }

  GstSample* sample = gst_app_sink_pull_sample(sink);
  if (!sample) {
    return GST_FLOW_ERROR;
  }

  GstBuffer* buffer = gst_sample_get_buffer(sample);
  if (!buffer) {
    gst_sample_unref(sample);
    return GST_FLOW_ERROR;
  }

  GstMapInfo map{};
  if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
    gst_sample_unref(sample);
    return GST_FLOW_ERROR;
  }

  H265Packet packet;
  packet.data.resize(map.size);
  std::memcpy(packet.data.data(), map.data, map.size);

  GstClockTime pts = GST_BUFFER_PTS(buffer);
  if (pts != GST_CLOCK_TIME_NONE) {
    packet.pts_ns = static_cast<uint64_t>(pts);
  }

  packet.is_keyframe = !GST_BUFFER_FLAG_IS_SET(buffer, GST_BUFFER_FLAG_DELTA_UNIT);

  gst_buffer_unmap(buffer, &map);
  gst_sample_unref(sample);

  if (on_packet_) {
    on_packet_(std::move(packet));
  }

  return GST_FLOW_OK;
}

void GstH265Encoder::bus_loop() {
  if (!bus_) {
    return;
  }

  while (running_.load()) {
    GstMessage* msg = gst_bus_timed_pop_filtered(
        bus_, 100 * GST_MSECOND,
        static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS | GST_MESSAGE_WARNING));
    if (!msg) {
      continue;
    }

    switch (GST_MESSAGE_TYPE(msg)) {
      case GST_MESSAGE_ERROR: {
        GError* err = nullptr;
        gchar* debug = nullptr;
        gst_message_parse_error(msg, &err, &debug);
        RCLCPP_ERROR(logger_, "GStreamer ERROR: %s (debug=%s)",
                     err ? err->message : "unknown", debug ? debug : "none");
        if (err) {
          g_error_free(err);
        }
        if (debug) {
          g_free(debug);
        }
        running_ = false;
        if (pipeline_) {
          gst_element_set_state(pipeline_, GST_STATE_NULL);
        }
        break;
      }
      case GST_MESSAGE_WARNING: {
        GError* err = nullptr;
        gchar* debug = nullptr;
        gst_message_parse_warning(msg, &err, &debug);
        RCLCPP_WARN(logger_, "GStreamer WARNING: %s (debug=%s)",
                    err ? err->message : "unknown", debug ? debug : "none");
        if (err) {
          g_error_free(err);
        }
        if (debug) {
          g_free(debug);
        }
        break;
      }
      case GST_MESSAGE_EOS:
        RCLCPP_WARN(logger_, "GStreamer EOS received");
        running_ = false;
        if (pipeline_) {
          gst_element_set_state(pipeline_, GST_STATE_NULL);
        }
        break;
      default:
        break;
    }

    gst_message_unref(msg);
  }
}

std::string GstH265Encoder::build_pipeline(std::string* error) const {
  if (options_.device.empty()) {
    if (error) {
      *error = "Device is empty";
    }
    return {};
  }

  const std::string input_format_gst = normalize_gst_format(options_.input_format);
  if (!is_safe_caps_token(input_format_gst)) {
    if (error) {
      *error = "Invalid input_format: " + options_.input_format;
    }
    return {};
  }

  if (!has_element_factory("v4l2src")) {
    if (error) {
      *error = "Missing GStreamer element: v4l2src";
    }
    return {};
  }
  if (!has_element_factory("nvvidconv")) {
    if (error) {
      *error = "Missing GStreamer element: nvvidconv (Jetson plugin)";
    }
    return {};
  }
  if (!has_element_factory("nvv4l2h265enc")) {
    if (error) {
      *error = "Missing GStreamer element: nvv4l2h265enc (Jetson NVENC plugin)";
    }
    return {};
  }

  const int out_w = options_.output_width > 0 ? options_.output_width : options_.input_width;
  const int out_h = options_.output_height > 0 ? options_.output_height : options_.input_height;

  std::string pipeline;
  pipeline += "v4l2src device=" + options_.device + " do-timestamp=true ! ";
  pipeline += "video/x-raw,format=" + input_format_gst +
              ",width=" + std::to_string(options_.input_width) +
              ",height=" + std::to_string(options_.input_height) +
              ",framerate=" + std::to_string(options_.fps) + "/1 ! ";
  pipeline += "nvvidconv ! ";
  pipeline += "video/x-raw(memory:NVMM),format=NV12,width=" + std::to_string(out_w) +
              ",height=" + std::to_string(out_h) +
              ",framerate=" + std::to_string(options_.fps) + "/1 ! ";
  pipeline += "nvv4l2h265enc ";
  pipeline += "bitrate=" + std::to_string(options_.bitrate) + " ";
  pipeline += "iframeinterval=" + std::to_string(options_.iframeinterval) + " ";
  pipeline += "idrinterval=" + std::to_string(options_.idrinterval) + " ";
  pipeline += "insert-sps-pps=" + std::string(options_.insert_sps_pps ? "true" : "false") + " ";
  pipeline += "insert-aud=" + std::string(options_.insert_aud ? "true" : "false") + " ";
  pipeline += "maxperf-enable=" + std::string(options_.maxperf_enable ? "true" : "false") + " ";
  pipeline += "preset-level=" + std::to_string(options_.preset_level) + " ";
  pipeline += "profile=" + std::to_string(options_.profile) + " ";
  pipeline += "! appsink name=sink sync=false";

  return pipeline;
}

}  // namespace cr_h265_publisher
