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

#ifndef KINECT_V1_DRIVER__COMPRESSED_RELAY_HPP_
#define KINECT_V1_DRIVER__COMPRESSED_RELAY_HPP_

#include <atomic>
#include <string>
#include <vector>

#include "image_transport/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace kinect_v1_driver
{

/// Republishes raw images as compressed transports for remote viewing.
///
/// For each configured stream it subscribes to `<ns>/image_raw` (best effort,
/// ConstSharedPtr: zero-copy when it runs in the driver's container) and
/// republishes on the SAME base topic through image_transport with only the
/// compressed plugin enabled (`<ns>/image_raw/compressed` or
/// `<ns>/image_raw/compressedDepth`). No duplicate raw topic is created.
///
/// Compression runs on the executor (one callback group per stream, so RGB
/// and depth compress in parallel on a multi-threaded executor), never on the
/// libfreenect event thread. image_transport only compresses when the
/// compressed topic has subscribers.
class CompressedRelay : public rclcpp::Node
{
public:
  explicit CompressedRelay(const rclcpp::NodeOptions & options);

private:
  struct Relay
  {
    std::string name;
    rclcpp::CallbackGroup::SharedPtr group;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub;
    image_transport::Publisher pub;
    std::atomic<uint64_t> frames{0};
    std::atomic<uint64_t> busy_ns{0};
  };

  void addRelay(
    Relay & relay, const std::string & topic_ns, const std::string & transport);
  void reportStats();

  Relay rgb_;
  Relay depth_;
  rclcpp::Time stats_start_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
};

}  // namespace kinect_v1_driver

#endif  // KINECT_V1_DRIVER__COMPRESSED_RELAY_HPP_
