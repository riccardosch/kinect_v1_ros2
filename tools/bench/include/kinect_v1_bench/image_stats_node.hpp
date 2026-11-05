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

#ifndef KINECT_V1_BENCH__IMAGE_STATS_NODE_HPP_
#define KINECT_V1_BENCH__IMAGE_STATS_NODE_HPP_

#include <mutex>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace kinect_v1_bench
{

/// Subscribes to an image topic (ConstSharedPtr, so intra-process delivery
/// stays zero-copy) and periodically logs:
///   - receive rate,
///   - latency now - header.stamp (mean, p95, max),
///   - address of msg->data.data() (to verify zero-copy against the driver).
class ImageStatsNode : public rclcpp::Node
{
public:
  explicit ImageStatsNode(const rclcpp::NodeOptions & options);

private:
  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr & msg, bool from_intra_process);
  void report();

  bool log_buffer_address_{false};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mutex_;
  std::vector<double> latencies_ms_;
  const void * last_address_{nullptr};
  size_t intra_count_{0};
  rclcpp::Time period_start_;
};

}  // namespace kinect_v1_bench

#endif  // KINECT_V1_BENCH__IMAGE_STATS_NODE_HPP_
