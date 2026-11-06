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

#include "kinect_v1_driver/compressed_relay.hpp"

#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <string>
#include <vector>

#include "image_transport/image_transport.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace kinect_v1_driver
{

namespace
{

/// image_transport derives parameter names from the base topic relative to
/// the node namespace, with '/' replaced by '.' and the leading '.' removed
/// (publisher.cpp): used for `enable_pub_plugins`.
std::string paramBase(const std::string & topic)
{
  std::string base = topic;
  std::replace(base.begin(), base.end(), '/', '.');
  return base;
}

/// The compressed plugins (image_transport_plugins 4.0.7) build their own
/// parameter names WITHOUT removing the leading '.', e.g.
/// ".rgb.image_raw.compressed.jpeg_quality" (checked with `ros2 param list`).
std::string pluginParamBase(const std::string & topic)
{
  return "." + paramBase(topic);
}

}  // namespace

CompressedRelay::CompressedRelay(const rclcpp::NodeOptions & options)
: rclcpp::Node("compressed_relay", options)
{
  const auto rgb_ns = declare_parameter<std::string>("rgb_ns", "rgb");
  // "depth" (depth_mode mm) or "depth_registered"; empty disables the stream.
  const auto depth_ns = declare_parameter<std::string>("depth_ns", "depth");
  const auto jpeg_quality = declare_parameter<int>("jpeg_quality", 80);
  const auto depth_format = declare_parameter<std::string>("depth_format", "rvl");
  const auto png_level = declare_parameter<int>("png_level", 1);
  const auto stats_period_s = declare_parameter<double>("stats_period_s", 10.0);

  if (jpeg_quality < 1 || jpeg_quality > 100) {
    throw std::invalid_argument("jpeg_quality must be in [1, 100]");
  }
  if (png_level < 1 || png_level > 9) {
    throw std::invalid_argument("png_level must be in [1, 9]");
  }
  if (depth_format != "rvl" && depth_format != "png") {
    throw std::invalid_argument("depth_format must be 'rvl' or 'png'");
  }

  // Plugin parameters must exist before the image_transport publisher is
  // created: the plugins read them (or declare their defaults) in advertise().
  if (!rgb_ns.empty()) {
    const auto base = pluginParamBase(rgb_ns + "/image_raw");
    declare_parameter(base + ".compressed.format", "jpeg");
    declare_parameter(base + ".compressed.jpeg_quality", jpeg_quality);
    addRelay(rgb_, rgb_ns, "compressed");
  }
  if (!depth_ns.empty()) {
    const auto base = pluginParamBase(depth_ns + "/image_raw");
    declare_parameter(base + ".compressedDepth.format", depth_format);
    declare_parameter(base + ".compressedDepth.png_level", png_level);
    addRelay(depth_, depth_ns, "compressedDepth");
  }
  if (rgb_ns.empty() && depth_ns.empty()) {
    throw std::invalid_argument("rgb_ns and depth_ns are both empty: nothing to relay");
  }

  RCLCPP_INFO(
    get_logger(), "Relaying %s%s%s (jpeg_quality %ld, depth %s%s)",
    rgb_ns.empty() ? "" : (rgb_ns + "/image_raw -> compressed").c_str(),
    (!rgb_ns.empty() && !depth_ns.empty()) ? ", " : "",
    depth_ns.empty() ? "" : (depth_ns + "/image_raw -> compressedDepth").c_str(),
    static_cast<long>(jpeg_quality), depth_format.c_str(),  // NOLINT(runtime/int)
    depth_format == "png" ? (" level " + std::to_string(png_level)).c_str() : "");

  stats_start_ = now();
  if (stats_period_s > 0.0) {
    stats_timer_ = create_wall_timer(
      std::chrono::duration<double>(stats_period_s), [this]() {reportStats();});
  }
}

void CompressedRelay::addRelay(
  Relay & relay, const std::string & topic_ns, const std::string & transport)
{
  relay.name = topic_ns;
  const std::string topic = topic_ns + "/image_raw";

  // Only the compressed plugin: no duplicate raw publisher on the driver's topic.
  declare_parameter(
    paramBase(topic) + ".enable_pub_plugins",
    std::vector<std::string>{"image_transport/" + transport});

  // Remote viewing: latest frame only, never block or retransmit.
  rmw_qos_profile_t out_qos = rmw_qos_profile_sensor_data;
  out_qos.depth = 1;
  relay.pub = image_transport::create_publisher(this, topic, out_qos);

  relay.group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = relay.group;
  relay.sub = create_subscription<sensor_msgs::msg::Image>(
    topic, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
    [this, &relay](const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
      if (relay.pub.getNumSubscribers() == 0) {
        return;  // nobody is watching: skip compression entirely
      }
      const auto t0 = std::chrono::steady_clock::now();
      relay.pub.publish(msg);
      relay.busy_ns += static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now() - t0).count());
      relay.frames++;
    },
    sub_options);
}

void CompressedRelay::reportStats()
{
  const auto t_now = now();
  const double period_s = (t_now - stats_start_).seconds();
  stats_start_ = t_now;
  for (Relay * r : {&rgb_, &depth_}) {
    if (!r->sub) {
      continue;
    }
    const uint64_t frames = r->frames.exchange(0);
    const uint64_t busy_ns = r->busy_ns.exchange(0);
    if (frames == 0) {
      continue;  // no remote subscriber: nothing to report
    }
    RCLCPP_INFO(
      get_logger(), "%-16s %.2f Hz | compress+publish mean %.1f ms (%.0f %% of one core)",
      r->name.c_str(), frames / period_s, busy_ns / 1e6 / frames, 100.0 * busy_ns / 1e9 / period_s);
  }
}

}  // namespace kinect_v1_driver

RCLCPP_COMPONENTS_REGISTER_NODE(kinect_v1_driver::CompressedRelay)
