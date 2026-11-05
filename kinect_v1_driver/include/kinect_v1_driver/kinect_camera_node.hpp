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
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "kinect_v1_driver/camera_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace kinect_v1_driver
{

/// Kinect v1 camera component: RGB and depth streams with camera_info.
///
/// Zero-copy design: libfreenect converts each frame (Bayer -> RGB, raw
/// disparity -> millimetres) directly into the `data` vector of a
/// sensor_msgs::msg::Image owned by this node. When a frame is complete, a
/// fresh message is registered as the next libfreenect buffer and the filled
/// one is published as a std::unique_ptr. With intra-process communication
/// enabled and ConstSharedPtr subscribers, rclcpp hands the very same buffer
/// to the consumers without any memcpy.
///
/// Topics follow the openni/freenect ROS convention:
///   rgb/image_raw, rgb/camera_info
///   depth/image_raw, depth/camera_info                       (depth_mode mm)
///   depth_registered/image_raw, depth_registered/camera_info (depth_mode registered)
class KinectCameraNode : public rclcpp::Node
{
public:
  explicit KinectCameraNode(const rclcpp::NodeOptions & options);
  ~KinectCameraNode() override;

  KinectCameraNode(const KinectCameraNode &) = delete;
  KinectCameraNode & operator=(const KinectCameraNode &) = delete;

private:
  /// Per-period counters, written by the libfreenect event thread and
  /// consumed by the stats timer (guarded by stats_mutex_).
  struct FrameStats
  {
    uint64_t frames{0};
    uint64_t alloc_ns_sum{0};
    uint64_t alloc_ns_max{0};
    uint64_t publish_ns_sum{0};
    uint64_t publish_ns_max{0};
  };

  /// libfreenect setter for the user buffer of one stream
  /// (freenect_set_video_buffer or freenect_set_depth_buffer).
  using SetBufferFn = int (*)(freenect_device *, void *);

  /// Everything needed to run one zero-copy image stream.
  struct Stream
  {
    const char * name{""};
    std::string topic_ns;  // "rgb", "depth" or "depth_registered"
    bool enabled{false};
    SetBufferFn set_buffer{nullptr};
    freenect_frame_mode mode{};
    std::string frame_id;
    std::string encoding;
    uint32_t bytes_per_pixel{0};

    /// Message whose data buffer libfreenect is currently writing into.
    /// Touched only by the event thread once streaming has started.
    sensor_msgs::msg::Image::UniquePtr back;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub;
    /// Calibration; shared between rgb and depth in registered mode.
    std::shared_ptr<CameraInfoSource> info;
    FrameStats stats;
  };

  void openDevice();
  void setupCameraInfo();
  void startStreams();
  void startStream(Stream & stream);
  void closeDevice();

  /// Allocates a message sized for the stream's frame mode. Its `data`
  /// storage becomes the libfreenect user buffer.
  sensor_msgs::msg::Image::UniquePtr makeMessage(const Stream & stream) const;

  /// Body of the dedicated thread that pumps libusb events.
  void eventLoop();

  /// libfreenect C callback trampolines -> onFrame().
  static void videoCallback(freenect_device * dev, void * buffer, uint32_t timestamp);
  static void depthCallback(freenect_device * dev, void * buffer, uint32_t timestamp);
  void onFrame(Stream & stream, void * buffer);

  void reportStats();

  // Parameters
  int device_index_{0};
  bool log_buffer_address_{false};
  std::string depth_mode_;
  std::string rgb_camera_info_url_;
  std::string depth_camera_info_url_;

  // Device identity, read at open time
  std::string serial_;

  // libfreenect handles (owned by this node)
  freenect_context * ctx_{nullptr};
  freenect_device * dev_{nullptr};

  Stream rgb_;
  Stream depth_;

  std::thread event_thread_;
  std::atomic<bool> running_{false};

  std::mutex stats_mutex_;
  rclcpp::Time stats_start_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
};

}  // namespace kinect_v1_driver

#endif  // KINECT_V1_DRIVER__KINECT_CAMERA_NODE_HPP_
