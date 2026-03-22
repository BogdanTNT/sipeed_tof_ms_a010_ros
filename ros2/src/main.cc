#include <cv_bridge/cv_bridge.hpp>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/header.hpp>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "cJSON.h"
#include "frame_struct.h"
#include "serial.hh"

extern frame_t *handle_process(std::string s);
extern void reset_frame_parser();

using namespace std::chrono_literals;

class SipeedTOFMSA010Node : public rclcpp::Node {
 public:
  SipeedTOFMSA010Node() : Node("sipeed_tof_node") {
    declare_parameters();
    load_parameters();

    publisher_depth_ =
        this->create_publisher<sensor_msgs::msg::Image>("depth", 10);
    publisher_pointcloud_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 10);
    publisher_camera_info_ =
        this->create_publisher<sensor_msgs::msg::CameraInfo>("depth/camera_info",
                                                             10);

    serial_ = std::make_unique<Serial>(device_path_);
    if (!serial_->isOpen()) {
      throw std::runtime_error("Failed to open serial device: " + device_path_);
    }

    if (!initialize_camera()) {
      throw std::runtime_error("Failed to initialize MS-A010 camera");
    }

    parameter_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&SipeedTOFMSA010Node::on_parameters_set, this,
                  std::placeholders::_1));

    running_.store(true);
    reader_thread_ = std::thread(&SipeedTOFMSA010Node::reader_loop, this);

    RCLCPP_INFO(this->get_logger(),
                "MS-A010 driver ready on %s, frame_id=%s, using GitHub depth "
                "model z=pow(raw/5.1, 2)/1000",
                device_path_.c_str(), frame_id_.c_str());
  }

  ~SipeedTOFMSA010Node() override {
    running_.store(false);
    if (reader_thread_.joinable()) {
      reader_thread_.join();
    }

    std::lock_guard<std::recursive_mutex> lock(io_mutex_);
    if (serial_ && serial_->isOpen()) {
      serial_->writeString("AT+DISP=0\r");
    }
  }

 private:
  void declare_parameters() {
    this->declare_parameter("device", "/dev/ttyUSB0");
    this->declare_parameter("frame_id", "camera_optical_frame");
    this->declare_parameter("timer_period_ms", 30);
    this->declare_parameter("timestamp_offset_ms", 100);
    this->declare_parameter("apply_sensor_settings", false);
    this->declare_parameter("binning", 1);
    this->declare_parameter("display_mode", 3);
    this->declare_parameter("fps", 10);
    this->declare_parameter("quantization_unit", 0);
  }

  void load_parameters() {
    device_path_ = this->get_parameter("device").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();
    timer_period_ms_ = this->get_parameter("timer_period_ms").as_int();
    timestamp_offset_ms_ = this->get_parameter("timestamp_offset_ms").as_int();
    apply_sensor_settings_ =
        this->get_parameter("apply_sensor_settings").as_bool();
    binning_ = this->get_parameter("binning").as_int();
    display_mode_ = this->get_parameter("display_mode").as_int();
    fps_ = this->get_parameter("fps").as_int();
    quantization_unit_ = this->get_parameter("quantization_unit").as_int();
  }

  bool initialize_camera() {
    std::lock_guard<std::recursive_mutex> lock(io_mutex_);
    RCLCPP_INFO(this->get_logger(), "Configuring camera at %s...",
                device_path_.c_str());

    serial_->writeString("AT+ISP=0\r");
    drain_input();
    serial_->writeString("AT+DISP=1\r");
    drain_input();
    serial_->writeString("AT+ISP=1\r");
    drain_input();
    std::this_thread::sleep_for(1000ms);

    if (!serial_->sendCommand("AT\r", "OK\r\n", 1000)) {
      RCLCPP_ERROR(this->get_logger(), "Camera did not respond to AT.");
      return false;
    }

    if (!enforce_quantization_unit_locked(quantization_unit_)) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to enforce startup quantization_unit=%d.",
                   quantization_unit_);
      return false;
    }

    if (!get_camera_coefficients()) {
      return false;
    }

    if (!start_stream_and_confirm()) {
      return false;
    }

    if (apply_sensor_settings_ &&
        (binning_ != 1 || fps_ != 10 || quantization_unit_ != 0)) {
      RCLCPP_INFO(this->get_logger(),
                  "Applying requested startup sensor settings: binning=%d, "
                  "fps=%d, quantization_unit=%d",
                  binning_, fps_, quantization_unit_);
      if (!reconfigure_sensor(binning_, fps_, quantization_unit_, true)) {
        RCLCPP_WARN(this->get_logger(),
                    "Requested startup sensor settings could not be applied. "
                    "Keeping baseline stream so publishing continues.");
      }
    }

    return true;
  }

  bool apply_binning(int binning) {
    const std::string binning_cmd = "AT+BINN=" + std::to_string(binning) + "\r";
    if (!serial_->sendCommand(binning_cmd, "OK\r\n", 2000)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to apply %s",
                   binning_cmd.c_str());
      return false;
    }
    drain_input();
    std::this_thread::sleep_for(100ms);
    return true;
  }

  bool apply_fps(int fps) {
    const std::string fps_cmd = "AT+FPS=" + std::to_string(fps) + "\r";
    if (!serial_->sendCommand(fps_cmd, "OK\r\n", 2000)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to apply %s", fps_cmd.c_str());
      return false;
    }
    drain_input();
    std::this_thread::sleep_for(100ms);
    return true;
  }

  bool apply_quantization_unit(int quantization_unit) {
    const std::string unit_cmd =
        "AT+UNIT=" + std::to_string(quantization_unit) + "\r";
    if (!serial_->sendCommand(unit_cmd, "OK\r\n", 2000)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to apply %s",
                   unit_cmd.c_str());
      return false;
    }
    drain_input();
    std::this_thread::sleep_for(100ms);
    return true;
  }

  bool query_quantization_unit(int &unit) {
    std::lock_guard<std::recursive_mutex> lock(io_mutex_);
    serial_->drain();
    serial_->writeString("AT+UNIT?\r");

    std::string response;
    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now() - start_time)
               .count() < 1000) {
      response += serial_->readString();

      const size_t prefix = response.find("+UNIT=");
      if (prefix != std::string::npos) {
        size_t begin = prefix + 6;
        while (begin < response.size() &&
               std::isspace(static_cast<unsigned char>(response[begin]))) {
          ++begin;
        }

        size_t end = begin;
        while (end < response.size() &&
               std::isdigit(static_cast<unsigned char>(response[end]))) {
          ++end;
        }

        if (end > begin) {
          unit = std::stoi(response.substr(begin, end - begin));
          drain_input();
          return true;
        }
      }

      std::this_thread::sleep_for(10ms);
    }

    RCLCPP_WARN(this->get_logger(),
                "Timed out while querying AT+UNIT?.");
    return false;
  }

  bool enforce_quantization_unit_locked(int desired_quantization_unit) {
    if (desired_quantization_unit < 0 || desired_quantization_unit > 10) {
      RCLCPP_ERROR(this->get_logger(),
                   "Quantization unit %d is out of range. Expected 0..10.",
                   desired_quantization_unit);
      return false;
    }

    if (!apply_quantization_unit(desired_quantization_unit)) {
      if (desired_quantization_unit == 0) {
        RCLCPP_WARN(this->get_logger(),
                    "Direct AT+UNIT=0 failed. Trying the stronger ISP reset "
                    "sequence for default quantization.");
        return reset_quantization_unit_to_default_locked(false);
      }
      return false;
    }

    int active_quantization_unit = -1;
    if (!query_quantization_unit(active_quantization_unit)) {
      RCLCPP_ERROR(this->get_logger(),
                   "Could not verify the active quantization unit.");
      return false;
    }

    if (active_quantization_unit != desired_quantization_unit) {
      RCLCPP_ERROR(this->get_logger(),
                   "Requested UNIT=%d but camera reports UNIT=%d.",
                   desired_quantization_unit, active_quantization_unit);
      if (desired_quantization_unit == 0) {
        RCLCPP_WARN(this->get_logger(),
                    "Trying the stronger ISP reset sequence for UNIT=0.");
        return reset_quantization_unit_to_default_locked(false);
      }
      return false;
    }

    quantization_unit_ = desired_quantization_unit;
    return true;
  }

  bool reset_quantization_unit_to_default_locked(bool restore_runtime_state) {
    const int current_binning = binning_;
    const int current_fps = fps_;

    if (!serial_->sendCommand("AT+DISP=0\r", "OK\r\n", 1000)) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to stop stream before resetting UNIT=0.");
      return false;
    }

    drain_input();
    reset_frame_parser();
    serial_->writeString("AT+ISP=0\r");
    drain_input();
    serial_->writeString("AT+DISP=1\r");
    drain_input();
    serial_->writeString("AT+ISP=1\r");
    drain_input();
    std::this_thread::sleep_for(1000ms);

    if (!serial_->sendCommand("AT\r", "OK\r\n", 1000)) {
      RCLCPP_ERROR(this->get_logger(),
                   "Camera did not respond after UNIT=0 ISP reset.");
      return false;
    }

    if (!apply_quantization_unit(0)) {
      return false;
    }

    int active_quantization_unit = -1;
    if (!query_quantization_unit(active_quantization_unit)) {
      RCLCPP_ERROR(this->get_logger(),
                   "Could not verify UNIT=0 after ISP reset.");
      return false;
    }
    if (active_quantization_unit != 0) {
      RCLCPP_ERROR(this->get_logger(),
                   "Requested UNIT=0 but camera reports UNIT=%d.",
                   active_quantization_unit);
      return false;
    }

    if (restore_runtime_state) {
      if (current_binning != 1 && !apply_binning(current_binning)) {
        return false;
      }

      if (current_fps != 10 && !apply_fps(current_fps)) {
        return false;
      }

      if (!get_camera_coefficients()) {
        return false;
      }

      if (!start_stream_and_confirm()) {
        return false;
      }
    }

    quantization_unit_ = 0;
    RCLCPP_INFO(this->get_logger(),
                "Reset quantization_unit live to 0 and restored the default "
                "depth model z=(raw/5.1)^2/1000.");
    return true;
  }

  bool reset_quantization_unit_to_default() {
    std::lock_guard<std::recursive_mutex> lock(io_mutex_);
    return reset_quantization_unit_to_default_locked(true);
  }

  bool restart_stream() {
    return start_stream_and_confirm();
  }

  bool reconfigure_sensor(std::optional<int> new_binning,
                          std::optional<int> new_fps,
                          std::optional<int> new_quantization_unit,
                          bool refresh_coefficients) {
    std::lock_guard<std::recursive_mutex> lock(io_mutex_);
    if (!serial_ || !serial_->isOpen()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Cannot reconfigure sensor: serial device is closed.");
      return false;
    }

    if (!serial_->sendCommand("AT+DISP=0\r", "OK\r\n", 1000)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to stop stream before reconfigure.");
      return false;
    }

    drain_input();
    reset_frame_parser();

    auto recover_and_fail = [this]() {
      if (!recover_baseline_stream()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to recover the baseline camera stream.");
      }
      return false;
    };

    if (new_binning.has_value() && !apply_binning(new_binning.value())) {
      return recover_and_fail();
    }

    if (new_fps.has_value() && !apply_fps(new_fps.value())) {
      return recover_and_fail();
    }

    if (new_quantization_unit.has_value() &&
        !enforce_quantization_unit_locked(new_quantization_unit.value())) {
      return recover_and_fail();
    }

    if (refresh_coefficients && !get_camera_coefficients()) {
      return recover_and_fail();
    }

    if (!restart_stream()) {
      RCLCPP_WARN(this->get_logger(),
                  "The camera accepted the reconfigure commands but did not "
                  "resume publishing frames. Reverting to the baseline stream.");
      return recover_and_fail();
    }

    return true;
  }

  bool set_quantization_unit_live(int new_quantization_unit) {
    if (new_quantization_unit < 0 || new_quantization_unit > 10) {
      RCLCPP_ERROR(this->get_logger(),
                   "Quantization unit %d is out of range. Expected 0..10.",
                   new_quantization_unit);
      return false;
    }

    if (new_quantization_unit == 0) {
      return reset_quantization_unit_to_default();
    }

    if (!reconfigure_sensor(std::nullopt, std::nullopt, new_quantization_unit,
                            false)) {
      return false;
    }

    quantization_unit_ = new_quantization_unit;
    RCLCPP_INFO(this->get_logger(),
                "Changed quantization_unit live to %d and restarted stream. "
                "Depth model is now %s.",
                quantization_unit_,
                quantization_unit_ == 0
                    ? "z=(raw/5.1)^2/1000"
                    : "z=raw*UNIT/1000");
    return true;
  }

  bool set_binning_live(int new_binning) {
    if (new_binning != 1 && new_binning != 2 && new_binning != 4) {
      RCLCPP_ERROR(this->get_logger(),
                   "Binning %d is invalid. Expected one of: 1, 2, 4.",
                   new_binning);
      return false;
    }

    if (!reconfigure_sensor(new_binning, std::nullopt, std::nullopt, true)) {
      return false;
    }

    binning_ = new_binning;
    RCLCPP_INFO(this->get_logger(),
                "Changed binning live to %d and refreshed coefficients.",
                binning_);
    return true;
  }

  bool get_camera_coefficients() {
    serial_->drain();
    serial_->writeString("AT+COEFF?\r");

    std::string response;
    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now() - start_time)
               .count() < 2000) {
      response += serial_->readString();
      const size_t json_start = response.find('{');
      const size_t json_end = response.rfind('}');
      if (json_start != std::string::npos && json_end != std::string::npos &&
          json_end > json_start) {
        const std::string json =
            response.substr(json_start, json_end - json_start + 1);
        std::unique_ptr<cJSON, decltype(&cJSON_Delete)> parsed(
            cJSON_ParseWithLength(json.c_str(), json.length()), cJSON_Delete);
        if (!parsed) {
          RCLCPP_ERROR(this->get_logger(),
                       "Failed to parse camera coefficient JSON.");
          return false;
        }

        fx_ = static_cast<float>(
            cJSON_GetObjectItem(parsed.get(), "fx")->valueint / 262144.0f);
        fy_ = static_cast<float>(
            cJSON_GetObjectItem(parsed.get(), "fy")->valueint / 262144.0f);
        u0_ = static_cast<float>(
            cJSON_GetObjectItem(parsed.get(), "u0")->valueint / 262144.0f);
        v0_ = static_cast<float>(
            cJSON_GetObjectItem(parsed.get(), "v0")->valueint / 262144.0f);

        RCLCPP_INFO(this->get_logger(),
                    "Camera coefficients: fx=%.3f fy=%.3f u0=%.3f v0=%.3f",
                    fx_, fy_, u0_, v0_);
        drain_input();
        return true;
      }
      std::this_thread::sleep_for(20ms);
    }

    RCLCPP_ERROR(this->get_logger(),
                 "Timed out while waiting for camera coefficients.");
    return false;
  }

  bool wait_for_valid_frame(int timeout_ms) {
    auto start_time = std::chrono::steady_clock::now();
    reset_frame_parser();

    while (std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now() - start_time)
               .count() < timeout_ms) {
      std::string chunk = serial_->readString();
      if (chunk.empty()) {
        std::this_thread::sleep_for(10ms);
        continue;
      }

      frame_t *frame = handle_process(chunk);
      if (frame != NULL) {
        free(frame);
        reset_frame_parser();
        return true;
      }
    }

    reset_frame_parser();
    return false;
  }

  bool start_stream_and_confirm() {
    const std::string start_stream =
        "AT+DISP=" + std::to_string(display_mode_) + "\r";
    if (!serial_->sendCommand(start_stream, "OK\r\n", 1000)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to start stream with %s",
                   start_stream.c_str());
      return false;
    }

    if (!wait_for_valid_frame(2000)) {
      RCLCPP_ERROR(this->get_logger(),
                   "Camera acknowledged %s but no valid frames arrived.",
                   start_stream.c_str());
      return false;
    }

    return true;
  }

  bool recover_baseline_stream() {
    std::lock_guard<std::recursive_mutex> lock(io_mutex_);
    RCLCPP_WARN(this->get_logger(),
                "Recovering the MS-A010 using the baseline init sequence.");

    serial_->writeString("AT+ISP=0\r");
    drain_input();
    serial_->writeString("AT+DISP=1\r");
    drain_input();
    serial_->writeString("AT+ISP=1\r");
    drain_input();
    std::this_thread::sleep_for(1000ms);

    if (!serial_->sendCommand("AT\r", "OK\r\n", 1000)) {
      RCLCPP_ERROR(this->get_logger(),
                   "Baseline recovery failed: camera did not respond to AT.");
      return false;
    }

    if (!get_camera_coefficients()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Baseline recovery failed: could not refresh coefficients.");
      return false;
    }

    if (!start_stream_and_confirm()) {
      return false;
    }

    binning_ = 1;
    fps_ = 10;
    quantization_unit_ = 0;
    RCLCPP_WARN(this->get_logger(),
                "Recovered baseline stream with binning=1, fps=10, "
                "quantization_unit=0.");
    return true;
  }

  void drain_input() {
    std::string chunk;
    do {
      chunk = serial_->readString();
    } while (!chunk.empty());
  }

  rcl_interfaces::msg::SetParametersResult on_parameters_set(
      const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    std::optional<std::string> next_frame_id;
    std::optional<int> next_timestamp_offset_ms;
    std::optional<int> next_quantization_unit;
    std::optional<int> next_binning;

    for (const auto &parameter : parameters) {
      if (parameter.get_name() == "frame_id") {
        next_frame_id = parameter.as_string();
      } else if (parameter.get_name() == "timestamp_offset_ms") {
        next_timestamp_offset_ms = static_cast<int>(parameter.as_int());
      } else if (parameter.get_name() == "quantization_unit") {
        next_quantization_unit = static_cast<int>(parameter.as_int());
      } else if (parameter.get_name() == "binning") {
        next_binning = static_cast<int>(parameter.as_int());
      }
    }

    if (next_timestamp_offset_ms.has_value() &&
        next_timestamp_offset_ms.value() < 0) {
      result.successful = false;
      result.reason = "timestamp_offset_ms must be >= 0";
      return result;
    }

    if (next_quantization_unit.has_value() &&
        next_quantization_unit.value() != quantization_unit_ &&
        !set_quantization_unit_live(next_quantization_unit.value())) {
      result.successful = false;
      result.reason = "Failed to apply AT+UNIT live";
      return result;
    }

    if (next_binning.has_value() && next_binning.value() != binning_ &&
        !set_binning_live(next_binning.value())) {
      result.successful = false;
      result.reason = "Failed to apply AT+BINN live";
      return result;
    }

    if (next_frame_id.has_value()) {
      frame_id_ = next_frame_id.value();
      RCLCPP_INFO(this->get_logger(), "Updated frame_id to %s",
                  frame_id_.c_str());
    }

    if (next_timestamp_offset_ms.has_value()) {
      timestamp_offset_ms_ = next_timestamp_offset_ms.value();
      RCLCPP_INFO(this->get_logger(), "Updated timestamp_offset_ms to %d",
                  timestamp_offset_ms_);
    }

    return result;
  }

  void reader_loop() {
    while (running_.load() && rclcpp::ok()) {
      std::vector<frame_t *> ready_frames;

      {
        std::lock_guard<std::recursive_mutex> lock(io_mutex_);
        if (!serial_ || !serial_->isOpen()) {
          return;
        }

        std::string chunk = serial_->readString();
        if (!chunk.empty()) {
          frame_t *frame = handle_process(chunk);
          while (frame != NULL) {
            ready_frames.push_back(frame);
            frame = handle_process("");
          }
        }
      }

      for (frame_t *frame : ready_frames) {
        publish_frame(*frame);
        free(frame);
      }
    }
  }

  void publish_frame(const frame_t &frame) {
    const uint8_t rows = frame.frame_head.resolution_rows;
    const uint8_t cols = frame.frame_head.resolution_cols;
    const uint8_t *depth = frame.payload;

    std_msgs::msg::Header header;
    header.stamp = this->get_clock()->now() -
                   rclcpp::Duration::from_nanoseconds(
                       static_cast<int64_t>(timestamp_offset_ms_) * 1000000LL);
    header.frame_id = frame_id_;

    cv::Mat depth_image(rows, cols, CV_8UC1,
                        const_cast<uint8_t *>(depth));
    auto msg_depth = cv_bridge::CvImage(header, "mono8", depth_image).toImageMsg();
    publisher_depth_->publish(*msg_depth);

    publisher_camera_info_->publish(create_camera_info(header, rows, cols));
    publisher_pointcloud_->publish(create_pointcloud(header, rows, cols, depth));
  }

  float raw_depth_to_meters(uint8_t raw_depth) const {
    const float raw = static_cast<float>(raw_depth);
    if (quantization_unit_ > 0) {
      // For AT+UNIT=N, the camera reports depth as N millimeters per level.
      return (raw * static_cast<float>(quantization_unit_)) / 1000.0f;
    }

    // UNIT=0 is the sensor's default non-linear 8-bit quantization.
    return std::pow(raw / 5.1f, 2.0f) / 1000.0f;
  }

  sensor_msgs::msg::CameraInfo create_camera_info(
      const std_msgs::msg::Header &header, uint8_t rows, uint8_t cols) const {
    sensor_msgs::msg::CameraInfo info;
    info.header = header;
    info.width = cols;
    info.height = rows;
    info.distortion_model = "plumb_bob";
    info.d.clear();
    info.binning_x = static_cast<uint32_t>(std::max(1, binning_));
    info.binning_y = static_cast<uint32_t>(std::max(1, binning_));

    const double scale_x = static_cast<double>(cols) / 100.0;
    const double scale_y = static_cast<double>(rows) / 100.0;
    const double fx = fx_ * scale_x;
    const double fy = fy_ * scale_y;
    const double u0 = u0_ * scale_x;
    const double v0 = v0_ * scale_y;

    info.k[0] = fx;
    info.k[1] = 0.0;
    info.k[2] = u0;
    info.k[3] = 0.0;
    info.k[4] = fy;
    info.k[5] = v0;
    info.k[6] = 0.0;
    info.k[7] = 0.0;
    info.k[8] = 1.0;

    info.r[0] = 1.0;
    info.r[4] = 1.0;
    info.r[8] = 1.0;

    info.p[0] = fx;
    info.p[1] = 0.0;
    info.p[2] = u0;
    info.p[3] = 0.0;
    info.p[4] = 0.0;
    info.p[5] = fy;
    info.p[6] = v0;
    info.p[7] = 0.0;
    info.p[8] = 0.0;
    info.p[9] = 0.0;
    info.p[10] = 1.0;
    info.p[11] = 0.0;

    return info;
  }

  sensor_msgs::msg::PointCloud2 create_pointcloud(
      const std_msgs::msg::Header &header, uint8_t rows, uint8_t cols,
      const uint8_t *depth) const {
    sensor_msgs::msg::PointCloud2 msg;
    msg.header = header;
    msg.height = rows;
    msg.width = cols;
    msg.is_bigendian = false;
    msg.is_dense = false;
    msg.point_step = 16;
    msg.row_step = msg.point_step * cols;
    msg.fields.resize(4);

    msg.fields[0].name = "x";
    msg.fields[0].offset = 0;
    msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[0].count = 1;

    msg.fields[1].name = "y";
    msg.fields[1].offset = 4;
    msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[1].count = 1;

    msg.fields[2].name = "z";
    msg.fields[2].offset = 8;
    msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[2].count = 1;

    msg.fields[3].name = "rgb";
    msg.fields[3].offset = 12;
    msg.fields[3].datatype = sensor_msgs::msg::PointField::UINT32;
    msg.fields[3].count = 1;

    msg.data.resize(msg.height * msg.width * msg.point_step, 0x00);
    uint8_t *ptr = msg.data.data();
    const float scale_x = static_cast<float>(cols) / 100.0f;
    const float scale_y = static_cast<float>(rows) / 100.0f;
    const float fx = fx_ * scale_x;
    const float fy = fy_ * scale_y;
    const float u0 = u0_ * scale_x;
    const float v0 = v0_ * scale_y;

    for (uint8_t j = 0; j < rows; ++j) {
      for (uint8_t i = 0; i < cols; ++i) {
        const size_t index = static_cast<size_t>(j) * cols + i;
        const float cx = (static_cast<float>(i) - u0) / fx;
        const float cy = (static_cast<float>(j) - v0) / fy;
        const float z = raw_depth_to_meters(depth[index]);
        const float x = z * cx;
        const float y = z * cy;

        *reinterpret_cast<float *>(ptr + 0) = x;
        *reinterpret_cast<float *>(ptr + 4) = y;
        *reinterpret_cast<float *>(ptr + 8) = z;

        const uint8_t *color = color_lut_jet[depth[index]];
        const uint32_t rgb = (static_cast<uint32_t>(color[0]) << 16) |
                             (static_cast<uint32_t>(color[1]) << 8) |
                             static_cast<uint32_t>(color[2]);
        *reinterpret_cast<uint32_t *>(ptr + 12) = rgb;
        ptr += msg.point_step;
      }
    }

    return msg;
  }

  std::unique_ptr<Serial> serial_;
  float fx_ = 0.0f;
  float fy_ = 0.0f;
  float u0_ = 0.0f;
  float v0_ = 0.0f;

  std::string device_path_;
  std::string frame_id_;
  int timer_period_ms_ = 30;
  int timestamp_offset_ms_ = 100;
  bool apply_sensor_settings_ = false;
  int binning_ = 1;
  int display_mode_ = 3;
  int fps_ = 10;
  int quantization_unit_ = 0;

  std::atomic<bool> running_{false};
  std::thread reader_thread_;
  std::recursive_mutex io_mutex_;
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_depth_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      publisher_pointcloud_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr
      publisher_camera_info_;

  const uint8_t color_lut_jet[256][3] = {
      {128, 0, 0},     {132, 0, 0},     {136, 0, 0},     {140, 0, 0},
      {144, 0, 0},     {148, 0, 0},     {152, 0, 0},     {156, 0, 0},
      {160, 0, 0},     {164, 0, 0},     {168, 0, 0},     {172, 0, 0},
      {176, 0, 0},     {180, 0, 0},     {184, 0, 0},     {188, 0, 0},
      {192, 0, 0},     {196, 0, 0},     {200, 0, 0},     {204, 0, 0},
      {208, 0, 0},     {212, 0, 0},     {216, 0, 0},     {220, 0, 0},
      {224, 0, 0},     {228, 0, 0},     {232, 0, 0},     {236, 0, 0},
      {240, 0, 0},     {244, 0, 0},     {248, 0, 0},     {252, 0, 0},
      {255, 0, 0},     {255, 4, 0},     {255, 8, 0},     {255, 12, 0},
      {255, 16, 0},    {255, 20, 0},    {255, 24, 0},    {255, 28, 0},
      {255, 32, 0},    {255, 36, 0},    {255, 40, 0},    {255, 44, 0},
      {255, 48, 0},    {255, 52, 0},    {255, 56, 0},    {255, 60, 0},
      {255, 64, 0},    {255, 68, 0},    {255, 72, 0},    {255, 76, 0},
      {255, 80, 0},    {255, 84, 0},    {255, 88, 0},    {255, 92, 0},
      {255, 96, 0},    {255, 100, 0},   {255, 104, 0},   {255, 108, 0},
      {255, 112, 0},   {255, 116, 0},   {255, 120, 0},   {255, 124, 0},
      {255, 128, 0},   {255, 132, 0},   {255, 136, 0},   {255, 140, 0},
      {255, 144, 0},   {255, 148, 0},   {255, 152, 0},   {255, 156, 0},
      {255, 160, 0},   {255, 164, 0},   {255, 168, 0},   {255, 172, 0},
      {255, 176, 0},   {255, 180, 0},   {255, 184, 0},   {255, 188, 0},
      {255, 192, 0},   {255, 196, 0},   {255, 200, 0},   {255, 204, 0},
      {255, 208, 0},   {255, 212, 0},   {255, 216, 0},   {255, 220, 0},
      {255, 224, 0},   {255, 228, 0},   {255, 232, 0},   {255, 236, 0},
      {255, 240, 0},   {255, 244, 0},   {255, 248, 0},   {255, 252, 0},
      {254, 255, 1},   {250, 255, 6},   {246, 255, 10},  {242, 255, 14},
      {238, 255, 18},  {234, 255, 22},  {230, 255, 26},  {226, 255, 30},
      {222, 255, 34},  {218, 255, 38},  {214, 255, 42},  {210, 255, 46},
      {206, 255, 50},  {202, 255, 54},  {198, 255, 58},  {194, 255, 62},
      {190, 255, 66},  {186, 255, 70},  {182, 255, 74},  {178, 255, 78},
      {174, 255, 82},  {170, 255, 86},  {166, 255, 90},  {162, 255, 94},
      {158, 255, 98},  {154, 255, 102}, {150, 255, 106}, {146, 255, 110},
      {142, 255, 114}, {138, 255, 118}, {134, 255, 122}, {130, 255, 126},
      {126, 255, 130}, {122, 255, 134}, {118, 255, 138}, {114, 255, 142},
      {110, 255, 146}, {106, 255, 150}, {102, 255, 154}, {98, 255, 158},
      {94, 255, 162},  {90, 255, 166},  {86, 255, 170},  {82, 255, 174},
      {78, 255, 178},  {74, 255, 182},  {70, 255, 186},  {66, 255, 190},
      {62, 255, 194},  {58, 255, 198},  {54, 255, 202},  {50, 255, 206},
      {46, 255, 210},  {42, 255, 214},  {38, 255, 218},  {34, 255, 222},
      {30, 255, 226},  {26, 255, 230},  {22, 255, 234},  {18, 255, 238},
      {14, 255, 242},  {10, 255, 246},  {6, 255, 250},   {2, 255, 254},
      {0, 252, 255},   {0, 248, 255},   {0, 244, 255},   {0, 240, 255},
      {0, 236, 255},   {0, 232, 255},   {0, 228, 255},   {0, 224, 255},
      {0, 220, 255},   {0, 216, 255},   {0, 212, 255},   {0, 208, 255},
      {0, 204, 255},   {0, 200, 255},   {0, 196, 255},   {0, 192, 255},
      {0, 188, 255},   {0, 184, 255},   {0, 180, 255},   {0, 176, 255},
      {0, 172, 255},   {0, 168, 255},   {0, 164, 255},   {0, 160, 255},
      {0, 156, 255},   {0, 152, 255},   {0, 148, 255},   {0, 144, 255},
      {0, 140, 255},   {0, 136, 255},   {0, 132, 255},   {0, 128, 255},
      {0, 124, 255},   {0, 120, 255},   {0, 116, 255},   {0, 112, 255},
      {0, 108, 255},   {0, 104, 255},   {0, 100, 255},   {0, 96, 255},
      {0, 92, 255},    {0, 88, 255},    {0, 84, 255},    {0, 80, 255},
      {0, 76, 255},    {0, 72, 255},    {0, 68, 255},    {0, 64, 255},
      {0, 60, 255},    {0, 56, 255},    {0, 52, 255},    {0, 48, 255},
      {0, 44, 255},    {0, 40, 255},    {0, 36, 255},    {0, 32, 255},
      {0, 28, 255},    {0, 24, 255},    {0, 20, 255},    {0, 16, 255},
      {0, 12, 255},    {0, 8, 255},     {0, 4, 255},     {0, 0, 255},
      {0, 0, 252},     {0, 0, 248},     {0, 0, 244},     {0, 0, 240},
      {0, 0, 236},     {0, 0, 232},     {0, 0, 228},     {0, 0, 224},
      {0, 0, 220},     {0, 0, 216},     {0, 0, 212},     {0, 0, 208},
      {0, 0, 204},     {0, 0, 200},     {0, 0, 196},     {0, 0, 192},
      {0, 0, 188},     {0, 0, 184},     {0, 0, 180},     {0, 0, 176},
      {0, 0, 172},     {0, 0, 168},     {0, 0, 164},     {0, 0, 160},
      {0, 0, 156},     {0, 0, 152},     {0, 0, 148},     {0, 0, 144},
      {0, 0, 140},     {0, 0, 136},     {0, 0, 132},     {0, 0, 128}};
};

int main(int argc, char const *argv[]) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<SipeedTOFMSA010Node>());
  } catch (const std::exception &e) {
    std::cerr << "MS-A010 driver failed: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();

  return 0;
}
