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

#include "kinect_v1_bench/image_stats_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <utility>
#include <vector>

#include "rclcpp_components/register_node_macro.hpp"

namespace kinect_v1_bench
{

ImageStatsNode::ImageStatsNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("image_stats", options)
{
  const auto qos_depth = declare_parameter<int>("qos_depth", 1);
  const auto report_period_s = declare_parameter<double>("report_period_s", 5.0);
  log_buffer_address_ = declare_parameter<bool>("log_buffer_address", false);
  if (qos_depth < 1 || report_period_s <= 0.0) {
    throw std::invalid_argument("qos_depth must be >= 1 and report_period_s > 0");
  }

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<size_t>(qos_depth)))
    .best_effort()
    .durability_volatile();
  // Remap "image" to the topic under test, e.g. -r image:=/kinect/rgb/image_raw
  // The MessageInfo variant tells whether the message came through the
  // intra-process path. Equal addresses alone are not a proof of zero-copy:
  // malloc may hand a freed publisher buffer back to the deserializer.
  sub_ = create_subscription<sensor_msgs::msg::Image>(
    "image", qos,
    [this](const sensor_msgs::msg::Image::ConstSharedPtr & msg, const rclcpp::MessageInfo & info) {
      onImage(msg, info.get_rmw_message_info().from_intra_process);
    });

  period_start_ = now();
  timer_ = create_wall_timer(
    std::chrono::duration<double>(report_period_s), [this]() {report();});

  RCLCPP_INFO(
    get_logger(), "Listening on %s (intra-process %s)", sub_->get_topic_name(),
    options.use_intra_process_comms() ? "enabled" : "disabled");
}

void ImageStatsNode::onImage(
  const sensor_msgs::msg::Image::ConstSharedPtr & msg, bool from_intra_process)
{
  const double latency_ms = (now() - rclcpp::Time(msg->header.stamp)).seconds() * 1e3;
  const void * address = msg->data.data();
  if (log_buffer_address_) {
    RCLCPP_INFO(
      get_logger(), "recv stamp=%d.%09u addr=%p %s", msg->header.stamp.sec,
      msg->header.stamp.nanosec, address, from_intra_process ? "intra" : "inter");
  }
  std::lock_guard<std::mutex> lock(mutex_);
  latencies_ms_.push_back(latency_ms);
  last_address_ = address;
  if (from_intra_process) {
    intra_count_++;
  }
}

void ImageStatsNode::report()
{
  std::vector<double> samples;
  const void * address = nullptr;
  size_t intra = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    samples.swap(latencies_ms_);
    address = last_address_;
    intra = intra_count_;
    intra_count_ = 0;
  }
  const auto t_now = now();
  const double period_s = (t_now - period_start_).seconds();
  period_start_ = t_now;

  if (samples.empty()) {
    RCLCPP_WARN(get_logger(), "No images in the last %.1f s", period_s);
    return;
  }
  std::sort(samples.begin(), samples.end());
  const double mean = std::accumulate(samples.begin(), samples.end(), 0.0) / samples.size();
  const size_t p95_index = static_cast<size_t>(std::ceil(0.95 * samples.size())) - 1;
  RCLCPP_INFO(
    get_logger(),
    "%.2f Hz | lag mean %.2f ms p95 %.2f ms max %.2f ms | intra-process %zu/%zu | last addr %p",
    samples.size() / period_s, mean, samples[p95_index], samples.back(), intra, samples.size(),
    address);
}

}  // namespace kinect_v1_bench

RCLCPP_COMPONENTS_REGISTER_NODE(kinect_v1_bench::ImageStatsNode)
