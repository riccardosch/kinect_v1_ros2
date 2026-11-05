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

#ifndef KINECT_V1_DRIVER__KINECT_AUX_NODE_HPP_
#define KINECT_V1_DRIVER__KINECT_AUX_NODE_HPP_

#include <libfreenect.h>

#include <atomic>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "audio_common_msgs/msg/audio_data_stamped.hpp"
#include "audio_common_msgs/msg/audio_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"

namespace kinect_v1_driver
{

/// Parses a led/cmd string ("off", "green", ...). Returns std::nullopt if unknown.
std::optional<freenect_led_options> parseLedCommand(const std::string & text);

/// Interleaves `channels` planar int32 buffers of `num_samples` samples each
/// into `out` as little-endian bytes (c0 c1 c2 c3 c0 c1 ...).
/// `out` must hold num_samples * channels * 4 bytes.
void interleaveS32LE(
  const int32_t * const * planar, int channels, int num_samples, uint8_t * out);

/// Kinect v1 auxiliary component: LED, accelerometer (sensor_msgs/Imu) and
/// the 4-microphone array (audio_common_msgs).
///
/// Owns its own freenect context with the MOTOR (and optionally AUDIO)
/// subdevices, so it runs next to the camera component. The tilt motor is
/// never commanded.
///
/// Threading: one worker thread owns the device. It pumps libusb events
/// (audio callbacks run inside freenect_process_events, on this thread),
/// applies pending LED requests and samples the accelerometer at a fixed
/// rate. ROS callbacks only hand requests to it (libfreenect has no locking).
class KinectAuxNode : public rclcpp::Node
{
public:
  explicit KinectAuxNode(const rclcpp::NodeOptions & options);
  ~KinectAuxNode() override;

  KinectAuxNode(const KinectAuxNode &) = delete;
  KinectAuxNode & operator=(const KinectAuxNode &) = delete;

private:
  struct Stats
  {
    uint64_t imu_samples{0};
    uint64_t audio_callbacks{0};
    uint64_t audio_samples{0};
    uint64_t cancelled_non_zero{0};
    uint64_t audio_resyncs{0};
  };

  void openDevice();
  void closeDevice();

  void workerLoop();
  void publishImu();

  static void audioCallback(
    freenect_device * dev, int num_samples, int32_t * mic1, int32_t * mic2,
    int32_t * mic3, int32_t * mic4, int16_t * cancelled, void * unknown);
  void onAudio(int num_samples, const int32_t * const * mics, const int16_t * cancelled);

  void onLedCommand(const std_msgs::msg::String & msg);
  void reportStats();

  // Parameters
  int device_index_{0};
  double imu_rate_hz_{30.0};
  std::string imu_frame_id_;
  double accel_variance_{0.0};
  bool enable_audio_{true};
  bool publish_cancelled_{false};
  std::string audio_frame_id_;

  // libfreenect handles (owned by this node, used only by the worker thread
  // after construction)
  freenect_context * ctx_{nullptr};
  freenect_device * dev_{nullptr};

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<audio_common_msgs::msg::AudioDataStamped>::SharedPtr audio_pub_;
  rclcpp::Publisher<audio_common_msgs::msg::AudioDataStamped>::SharedPtr cancelled_pub_;
  rclcpp::Publisher<audio_common_msgs::msg::AudioInfo>::SharedPtr audio_info_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr led_sub_;

  // Synthetic audio clock (worker thread only): stamp of the next chunk's
  // first sample.
  rclcpp::Time audio_next_stamp_;
  bool audio_clock_valid_{false};

  std::mutex led_mutex_;
  std::optional<freenect_led_options> pending_led_;

  std::mutex stats_mutex_;
  Stats stats_;
  rclcpp::Time stats_start_;
  rclcpp::TimerBase::SharedPtr stats_timer_;

  std::thread worker_;
  std::atomic<bool> running_{false};
};

}  // namespace kinect_v1_driver

#endif  // KINECT_V1_DRIVER__KINECT_AUX_NODE_HPP_
