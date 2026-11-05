// Copyright 2026 Riccardo Schiavo
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "kinect_v1_driver/kinect_aux_node.hpp"

#include <libfreenect_audio.h>
#include <sys/time.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "rclcpp_components/register_node_macro.hpp"

namespace kinect_v1_driver
{

namespace
{

constexpr int kLibusbErrorInterrupted = -10;
constexpr int kMicChannels = 4;
constexpr uint32_t kAudioSampleRate = 16000;  // fixed by the Kinect firmware
// Re-anchor the synthetic audio clock if it disagrees with arrival times by
// more than this (normal jitter is up to ~2 chunks = 32 ms).
constexpr double kAudioResyncS = 0.05;

void appendLE32(uint8_t * out, int32_t value)
{
  const auto v = static_cast<uint32_t>(value);
  out[0] = static_cast<uint8_t>(v);
  out[1] = static_cast<uint8_t>(v >> 8);
  out[2] = static_cast<uint8_t>(v >> 16);
  out[3] = static_cast<uint8_t>(v >> 24);
}

}  // namespace

std::optional<freenect_led_options> parseLedCommand(const std::string & text)
{
  // Values from freenect_led_options (libfreenect.h); 5 is not defined there.
  if (text == "off") {return LED_OFF;}
  if (text == "green") {return LED_GREEN;}
  if (text == "red") {return LED_RED;}
  if (text == "yellow") {return LED_YELLOW;}
  if (text == "blink_green") {return LED_BLINK_GREEN;}
  if (text == "blink_red_yellow") {return LED_BLINK_RED_YELLOW;}
  return std::nullopt;
}

void interleaveS32LE(
  const int32_t * const * planar, int channels, int num_samples, uint8_t * out)
{
  for (int i = 0; i < num_samples; ++i) {
    for (int c = 0; c < channels; ++c) {
      appendLE32(out, planar[c][i]);
      out += 4;
    }
  }
}

KinectAuxNode::KinectAuxNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("kinect_aux", options)
{
  device_index_ = declare_parameter<int>("device_index", 0);
  imu_rate_hz_ = declare_parameter<double>("imu_rate_hz", 30.0);
  imu_frame_id_ = declare_parameter<std::string>("imu_frame_id", "kinect_imu_frame");
  // Default measured at rest on a 1414 unit (96 samples @ 30 Hz):
  // std dev 0.073-0.084 m/s^2 per axis -> variance ~0.0065 (m/s^2)^2.
  accel_variance_ = declare_parameter<double>("linear_acceleration_variance", 0.0065);
  const auto qos_depth = declare_parameter<int>("qos_depth", 1);
  const auto initial_led = declare_parameter<std::string>("initial_led", "green");
  enable_audio_ = declare_parameter<bool>("enable_audio", true);
  publish_cancelled_ = declare_parameter<bool>("publish_cancelled", false);
  audio_frame_id_ = declare_parameter<std::string>("audio_frame_id", "kinect_mic_array_frame");
  const auto audio_qos_depth = declare_parameter<int>("audio_qos_depth", 100);
  const auto stats_period_s = declare_parameter<double>("stats_period_s", 10.0);

  if (imu_rate_hz_ <= 0.0 || imu_rate_hz_ > 200.0) {
    throw std::invalid_argument("imu_rate_hz must be in (0, 200]");
  }
  if (qos_depth < 1 || audio_qos_depth < 1) {
    throw std::invalid_argument("qos_depth and audio_qos_depth must be >= 1");
  }
  const auto led = parseLedCommand(initial_led);
  if (!led) {
    throw std::invalid_argument("initial_led: unknown LED state '" + initial_led + "'");
  }
  pending_led_ = led;

  const auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<size_t>(qos_depth)))
    .best_effort()
    .durability_volatile();
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu", sensor_qos);

  if (enable_audio_) {
    // Audio is a continuous stream: a lost chunk is an audible gap, and the
    // chunks are small (4 KB at 62.5 Hz). Reliable with a deep queue (about
    // 1.6 s by default) instead of the best-effort/depth-1 sensor profile.
    const auto audio_qos =
      rclcpp::QoS(rclcpp::KeepLast(static_cast<size_t>(audio_qos_depth))).reliable();
    audio_pub_ = create_publisher<audio_common_msgs::msg::AudioDataStamped>(
      "audio/data", audio_qos);
    if (publish_cancelled_) {
      cancelled_pub_ = create_publisher<audio_common_msgs::msg::AudioDataStamped>(
        "audio/cancelled", audio_qos);
    }
    // Stream description, latched for late subscribers.
    audio_info_pub_ = create_publisher<audio_common_msgs::msg::AudioInfo>(
      "audio/info", rclcpp::QoS(1).reliable().transient_local());
    audio_common_msgs::msg::AudioInfo info;
    info.channels = kMicChannels;
    info.sample_rate = kAudioSampleRate;
    info.sample_format = "S32LE";
    info.bitrate = kAudioSampleRate * 32U * kMicChannels;
    info.coding_format = "wave";  // raw interleaved PCM chunks, no container
    audio_info_pub_->publish(info);
  }

  // Commands must not be lost: reliable.
  led_sub_ = create_subscription<std_msgs::msg::String>(
    "led/cmd", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
    [this](const std_msgs::msg::String & msg) {onLedCommand(msg);});

  openDevice();

  stats_start_ = now();
  if (stats_period_s > 0.0) {
    stats_timer_ = create_wall_timer(
      std::chrono::duration<double>(stats_period_s), [this]() {reportStats();});
  }

  running_ = true;
  worker_ = std::thread([this]() {workerLoop();});
}

KinectAuxNode::~KinectAuxNode()
{
  running_ = false;
  if (worker_.joinable()) {
    worker_.join();
  }
  closeDevice();
}

void KinectAuxNode::openDevice()
{
  if (freenect_init(&ctx_, nullptr) < 0) {
    ctx_ = nullptr;
    throw std::runtime_error("freenect_init failed");
  }
  freenect_set_log_level(ctx_, FREENECT_LOG_WARNING);

  // MOTOR: LED + accelerometer on the 1414. AUDIO: the microphone array.
  // The camera is only enumerated by libfreenect to locate its siblings.
  int subdevices = FREENECT_DEVICE_MOTOR;
  if (enable_audio_) {
    subdevices |= FREENECT_DEVICE_AUDIO;
  }
  freenect_select_subdevices(ctx_, static_cast<freenect_device_flags>(subdevices));

  // With AUDIO, opening may upload audios.bin and wait for the device to
  // re-enumerate: this can block for a few seconds (usb_libusb10.c).
  const auto t_open = std::chrono::steady_clock::now();
  const int num_devices = freenect_num_devices(ctx_);
  if (num_devices <= device_index_ ||
    freenect_open_device(ctx_, &dev_, device_index_) < 0)
  {
    freenect_shutdown(ctx_);
    ctx_ = nullptr;
    dev_ = nullptr;
    throw std::runtime_error(
            "Could not open Kinect #" + std::to_string(device_index_) + " motor" +
            (enable_audio_ ? "/audio" : "") + " device (" + std::to_string(num_devices) +
            " camera(s) found; is it used by another process? is audios.bin installed?)");
  }
  const double open_s = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - t_open).count();

  if (enable_audio_) {
    freenect_set_user(dev_, this);
    freenect_set_audio_in_callback(dev_, &KinectAuxNode::audioCallback);
    if (freenect_start_audio(dev_) < 0) {
      closeDevice();
      throw std::runtime_error("freenect_start_audio failed");
    }
  }

  RCLCPP_INFO(
    get_logger(), "Kinect #%d aux opened in %.1f s: LED, IMU @ %.1f Hz, audio %s "
    "(tilt motor unused)", device_index_, open_s, imu_rate_hz_,
    enable_audio_ ? "4 ch @ 16 kHz" : "off");
}

void KinectAuxNode::closeDevice()
{
  if (dev_ != nullptr) {
    if (enable_audio_) {
      freenect_stop_audio(dev_);
    }
    freenect_set_led(dev_, LED_BLINK_GREEN);  // visible sign that the driver has stopped
    freenect_close_device(dev_);
    dev_ = nullptr;
  }
  if (ctx_ != nullptr) {
    freenect_shutdown(ctx_);
    ctx_ = nullptr;
  }
}

void KinectAuxNode::onLedCommand(const std_msgs::msg::String & msg)
{
  const auto led = parseLedCommand(msg.data);
  if (!led) {
    RCLCPP_WARN(
      get_logger(), "led/cmd: unknown state '%s' (use off, green, red, yellow, blink_green, "
      "blink_red_yellow)", msg.data.c_str());
    return;
  }
  std::lock_guard<std::mutex> lock(led_mutex_);
  pending_led_ = led;
}

void KinectAuxNode::workerLoop()
{
  using clock = std::chrono::steady_clock;
  const auto period = std::chrono::duration_cast<clock::duration>(
    std::chrono::duration<double>(1.0 / imu_rate_hz_));
  auto next_sample = clock::now();

  while (running_) {
    std::optional<freenect_led_options> led;
    {
      std::lock_guard<std::mutex> lock(led_mutex_);
      led.swap(pending_led_);
    }
    if (led) {
      const int res = freenect_set_led(dev_, *led);
      if (res < 0) {
        RCLCPP_ERROR(get_logger(), "freenect_set_led failed (%d)", res);
      } else {
        RCLCPP_INFO(get_logger(), "LED set to %d", static_cast<int>(*led));
      }
    }

    publishImu();

    // Fixed-rate schedule; if we fell behind (e.g. USB stall), resync
    // instead of bursting to catch up.
    next_sample += period;
    if (next_sample < clock::now()) {
      next_sample = clock::now();
    }

    // Wait for the next IMU sample. With audio, the wait is spent inside
    // libusb event handling, so audio callbacks run on this same thread.
    while (running_ && clock::now() < next_sample) {
      if (!enable_audio_) {
        std::this_thread::sleep_until(next_sample);
        break;
      }
      const auto remaining_us = std::chrono::duration_cast<std::chrono::microseconds>(
        next_sample - clock::now()).count();
      timeval timeout{0, static_cast<suseconds_t>(std::max<int64_t>(remaining_us, 0))};
      const int res = freenect_process_events_timeout(ctx_, &timeout);
      if (res < 0 && res != kLibusbErrorInterrupted) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 1000, "freenect_process_events failed (%d)", res);
        std::this_thread::sleep_until(next_sample);
      }
    }
  }
}

void KinectAuxNode::publishImu()
{
  // Synchronous control transfer (~1 ms) that refreshes dev->raw_state.
  const int res = freenect_update_tilt_state(dev_);
  const auto stamp = now();
  if (res < 0) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000, "freenect_update_tilt_state failed (%d)", res);
    return;
  }

  double fx = 0.0;
  double fy = 0.0;
  double fz = 0.0;
  freenect_get_mks_accel(freenect_get_tilt_state(dev_), &fx, &fy, &fz);

  auto msg = std::make_unique<sensor_msgs::msg::Imu>();
  msg->header.stamp = stamp;
  msg->header.frame_id = imu_frame_id_;

  // libfreenect accelerometer axes (measured at rest on a 1414): +y up,
  // +z forward (follows the tilt), x lateral. Mapped to REP 103
  // (x forward, y left, z up). The sign of the lateral axis is an assumption
  // (right-handed sensor frame) until verified by rolling the sensor.
  msg->linear_acceleration.x = fz;
  msg->linear_acceleration.y = fx;
  msg->linear_acceleration.z = fy;
  msg->linear_acceleration_covariance = {
    accel_variance_, 0.0, 0.0,
    0.0, accel_variance_, 0.0,
    0.0, 0.0, accel_variance_};

  // No orientation estimate and no gyroscope: REP 145 says -1 in element 0.
  msg->orientation_covariance[0] = -1.0;
  msg->angular_velocity_covariance[0] = -1.0;

  imu_pub_->publish(std::move(msg));

  std::lock_guard<std::mutex> lock(stats_mutex_);
  stats_.imu_samples++;
}

void KinectAuxNode::audioCallback(
  freenect_device * dev, int num_samples, int32_t * mic1, int32_t * mic2,
  int32_t * mic3, int32_t * mic4, int16_t * cancelled, void *)
{
  const int32_t * const mics[kMicChannels] = {mic1, mic2, mic3, mic4};
  static_cast<KinectAuxNode *>(freenect_get_user(dev))->onAudio(num_samples, mics, cancelled);
}

void KinectAuxNode::onAudio(
  int num_samples, const int32_t * const * mics, const int16_t * cancelled)
{
  // Runs on the worker thread (inside freenect_process_events).
  //
  // Timestamp = time of the FIRST sample of the chunk, from a synthetic
  // clock: stamp(k) = stamp(k-1) + num_samples / 16 kHz. Arrival times are
  // useless here: one isochronous transfer carries two chunks, so callbacks
  // come in pairs (~0 ms apart, then ~32 ms). The clock is anchored to the
  // estimated start of the chunk (arrival - chunk duration) and re-anchored
  // if it drifts by more than kAudioResyncS (lost data, USB stall).
  const auto arrival = now();
  const rclcpp::Duration chunk_duration = rclcpp::Duration::from_seconds(
    static_cast<double>(num_samples) / kAudioSampleRate);
  const rclcpp::Time estimated_start = arrival - chunk_duration;
  bool resynced = false;
  if (!audio_clock_valid_ ||
    std::abs((audio_next_stamp_ - estimated_start).seconds()) > kAudioResyncS)
  {
    resynced = audio_clock_valid_;  // the first anchoring is not a resync
    audio_next_stamp_ = estimated_start;
    audio_clock_valid_ = true;
  }
  const rclcpp::Time stamp = audio_next_stamp_;
  audio_next_stamp_ = audio_next_stamp_ + chunk_duration;

  // libfreenect keeps the samples in its own planar buffers (no user-buffer
  // API for audio): one copy is unavoidable, and it also interleaves.
  auto msg = std::make_unique<audio_common_msgs::msg::AudioDataStamped>();
  msg->header.stamp = stamp;
  msg->header.frame_id = audio_frame_id_;
  msg->audio.data.resize(static_cast<size_t>(num_samples) * kMicChannels * 4U);
  interleaveS32LE(mics, kMicChannels, num_samples, msg->audio.data.data());
  audio_pub_->publish(std::move(msg));

  uint64_t non_zero = 0;
  for (int i = 0; i < num_samples; ++i) {
    non_zero += cancelled[i] != 0;
  }
  if (cancelled_pub_) {
    auto c = std::make_unique<audio_common_msgs::msg::AudioDataStamped>();
    c->header.stamp = stamp;
    c->header.frame_id = audio_frame_id_;
    c->audio.data.resize(static_cast<size_t>(num_samples) * 2U);
    // Host byte order: little-endian on x86 and aarch64, i.e. S16LE.
    std::memcpy(c->audio.data.data(), cancelled, c->audio.data.size());
    cancelled_pub_->publish(std::move(c));
  }

  std::lock_guard<std::mutex> lock(stats_mutex_);
  stats_.audio_callbacks++;
  stats_.audio_samples += static_cast<uint64_t>(num_samples);
  stats_.cancelled_non_zero += non_zero;
  stats_.audio_resyncs += resynced ? 1U : 0U;
}

void KinectAuxNode::reportStats()
{
  Stats s;
  {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    s = stats_;
    stats_ = Stats{};
  }
  const auto t_now = now();
  const double period_s = (t_now - stats_start_).seconds();
  stats_start_ = t_now;
  if (!enable_audio_) {
    RCLCPP_INFO(get_logger(), "imu %.2f Hz", s.imu_samples / period_s);
    return;
  }
  const double rate = s.audio_samples / period_s;
  RCLCPP_INFO(
    get_logger(),
    "imu %.2f Hz | audio %.0f Hz (%+.2f %%), %.1f chunks/s, clock resyncs %lu | "
    "cancelled non-zero %.1f %%",
    s.imu_samples / period_s, rate, 100.0 * (rate - kAudioSampleRate) / kAudioSampleRate,
    s.audio_callbacks / period_s, static_cast<unsigned long>(s.audio_resyncs),  // NOLINT
    s.audio_samples ? 100.0 * s.cancelled_non_zero / s.audio_samples : 0.0);
}

}  // namespace kinect_v1_driver

RCLCPP_COMPONENTS_REGISTER_NODE(kinect_v1_driver::KinectAuxNode)
