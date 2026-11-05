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

#ifndef KINECT_V1_DRIVER__KINECT_CAMERA_NODE_HPP_
#define KINECT_V1_DRIVER__KINECT_CAMERA_NODE_HPP_

#include <libfreenect.h>

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace kinect_v1_driver
{

/// Kinect v1 camera component (RGB stream).
///
/// Zero-copy design: libfreenect converts each frame (Bayer -> RGB) directly
/// into the `data` vector of a sensor_msgs::msg::Image owned by this node.
/// When the frame is complete, a fresh message is registered as the next
/// libfreenect buffer and the filled one is published as a std::unique_ptr.
/// With intra-process communication enabled and ConstSharedPtr subscribers,
/// rclcpp hands the very same buffer to the consumers without any memcpy.
class KinectCameraNode : public rclcpp::Node
{
public:
  explicit KinectCameraNode(const rclcpp::NodeOptions & options);
  ~KinectCameraNode() override;

  KinectCameraNode(const KinectCameraNode &) = delete;
  KinectCameraNode & operator=(const KinectCameraNode &) = delete;

private:
  /// Per-period counters, written by the libfreenect event thread and
  /// consumed by the stats timer.
  struct FrameStats
  {
    uint64_t frames{0};
    uint64_t alloc_ns_sum{0};
    uint64_t alloc_ns_max{0};
    uint64_t publish_ns_sum{0};
    uint64_t publish_ns_max{0};
  };

  void openDevice();
  void closeDevice();

  /// Allocates a message sized for the current video mode. Its `data`
  /// storage becomes the libfreenect user buffer.
  sensor_msgs::msg::Image::UniquePtr makeRgbMessage() const;

  /// Body of the dedicated thread that pumps libusb events.
  void eventLoop();

  /// libfreenect C callback trampoline -> onVideoFrame().
  static void videoCallback(freenect_device * dev, void * buffer, uint32_t timestamp);
  void onVideoFrame(void * buffer, uint32_t timestamp);

  void reportStats();

  // Parameters
  int device_index_{0};
  std::string rgb_frame_id_;
  bool log_buffer_address_{false};

  // libfreenect handles (owned by this node)
  freenect_context * ctx_{nullptr};
  freenect_device * dev_{nullptr};
  freenect_frame_mode video_mode_{};

  /// Message whose data buffer libfreenect is currently writing into.
  /// Touched only by the event thread once streaming has started.
  sensor_msgs::msg::Image::UniquePtr rgb_back_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;

  std::thread event_thread_;
  std::atomic<bool> running_{false};

  std::mutex stats_mutex_;
  FrameStats stats_;
  rclcpp::Time stats_start_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
};

}  // namespace kinect_v1_driver

#endif  // KINECT_V1_DRIVER__KINECT_CAMERA_NODE_HPP_
