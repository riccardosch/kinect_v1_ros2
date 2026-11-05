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

#include "kinect_v1_driver/kinect_camera_node.hpp"

#include <sys/time.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "rclcpp_components/register_node_macro.hpp"

namespace kinect_v1_driver
{

namespace
{

// libusb error code returned when a signal interrupts event handling.
// Kept as a literal to avoid depending on libusb headers directly.
constexpr int kLibusbErrorInterrupted = -10;

// Upper bound on how long the event thread blocks inside libusb, so that
// shutdown is noticed promptly.
constexpr suseconds_t kEventTimeoutUs = 100000;

int64_t elapsedNs(std::chrono::steady_clock::time_point since)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now() - since).count();
}

// libfreenect has no user pointer in its log callback, so a named logger is used.
void freenectLogCallback(freenect_context *, freenect_loglevel level, const char * msg)
{
  const auto logger = rclcpp::get_logger("libfreenect");
  std::string text(msg);
  if (!text.empty() && text.back() == '\n') {
    text.pop_back();
  }
  if (level <= FREENECT_LOG_ERROR) {
    RCLCPP_ERROR(logger, "%s", text.c_str());
  } else if (level <= FREENECT_LOG_WARNING) {
    RCLCPP_WARN(logger, "%s", text.c_str());
  } else if (level <= FREENECT_LOG_INFO) {
    RCLCPP_INFO(logger, "%s", text.c_str());
  } else {
    RCLCPP_DEBUG(logger, "%s", text.c_str());
  }
}

}  // namespace

KinectCameraNode::KinectCameraNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("kinect_camera", options)
{
  device_index_ = declare_parameter<int>("device_index", 0);
  rgb_frame_id_ = declare_parameter<std::string>("rgb_frame_id", "kinect_rgb_optical_frame");
  const auto qos_depth = declare_parameter<int>("qos_depth", 1);
  const auto stats_period_s = declare_parameter<double>("stats_period_s", 5.0);
  log_buffer_address_ = declare_parameter<bool>("log_buffer_address", false);

  if (qos_depth < 1) {
    throw std::invalid_argument("qos_depth must be >= 1");
  }

  // Sensor QoS: never block or retransmit; keep_last + volatile are also the
  // requirements for intra-process communication.
  const auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<size_t>(qos_depth)))
    .best_effort()
    .durability_volatile();
  rgb_pub_ = create_publisher<sensor_msgs::msg::Image>("rgb/image_raw", sensor_qos);

  RCLCPP_INFO(
    get_logger(), "Intra-process communication: %s",
    options.use_intra_process_comms() ? "enabled" : "DISABLED (memcpy/serialization will occur)");

  openDevice();

  stats_start_ = now();
  if (stats_period_s > 0.0) {
    stats_timer_ = create_wall_timer(
      std::chrono::duration<double>(stats_period_s), [this]() {reportStats();});
  }

  running_ = true;
  event_thread_ = std::thread([this]() {eventLoop();});
}

KinectCameraNode::~KinectCameraNode()
{
  running_ = false;
  if (event_thread_.joinable()) {
    event_thread_.join();
  }
  closeDevice();
}

void KinectCameraNode::openDevice()
{
  if (freenect_init(&ctx_, nullptr) < 0) {
    ctx_ = nullptr;
    throw std::runtime_error("freenect_init failed");
  }
  freenect_set_log_callback(ctx_, &freenectLogCallback);
  freenect_set_log_level(ctx_, FREENECT_LOG_WARNING);

  // This context only owns the camera USB device; motor and audio are left
  // free for another context/process.
  freenect_select_subdevices(ctx_, FREENECT_DEVICE_CAMERA);

  const int num_devices = freenect_num_devices(ctx_);
  if (num_devices <= device_index_) {
    freenect_shutdown(ctx_);
    ctx_ = nullptr;
    throw std::runtime_error(
            "Kinect #" + std::to_string(device_index_) + " not found (" +
            std::to_string(num_devices) + " connected)");
  }
  if (freenect_open_device(ctx_, &dev_, device_index_) < 0) {
    freenect_shutdown(ctx_);
    ctx_ = nullptr;
    dev_ = nullptr;
    throw std::runtime_error("freenect_open_device failed (is the device busy or not permitted?)");
  }

  video_mode_ = freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB);
  if (!video_mode_.is_valid || freenect_set_video_mode(dev_, video_mode_) < 0) {
    closeDevice();
    throw std::runtime_error("Could not set RGB video mode");
  }

  // The user buffer must be registered BEFORE start_video, otherwise
  // libfreenect allocates its own internal buffer (cameras.c, stream_init).
  rgb_back_ = makeRgbMessage();
  freenect_set_video_buffer(dev_, rgb_back_->data.data());

  freenect_set_user(dev_, this);
  freenect_set_video_callback(dev_, &KinectCameraNode::videoCallback);

  if (freenect_start_video(dev_) < 0) {
    closeDevice();
    throw std::runtime_error("freenect_start_video failed");
  }

  RCLCPP_INFO(
    get_logger(), "Kinect #%d RGB streaming %dx%d @ %d Hz (%d bytes/frame), libfreenect %s",
    device_index_, video_mode_.width, video_mode_.height, video_mode_.framerate,
    video_mode_.bytes, KINECT_V1_LIBFREENECT_VERSION);
}

void KinectCameraNode::closeDevice()
{
  // Order matters: stop the stream (no more writes into our buffer), then
  // detach the user buffer, then release the device and finally the message.
  if (dev_ != nullptr) {
    freenect_stop_video(dev_);
    freenect_set_video_buffer(dev_, nullptr);
    freenect_close_device(dev_);
    dev_ = nullptr;
  }
  if (ctx_ != nullptr) {
    freenect_shutdown(ctx_);
    ctx_ = nullptr;
  }
  rgb_back_.reset();
}

sensor_msgs::msg::Image::UniquePtr KinectCameraNode::makeRgbMessage() const
{
  auto msg = std::make_unique<sensor_msgs::msg::Image>();
  msg->header.frame_id = rgb_frame_id_;
  msg->height = static_cast<uint32_t>(video_mode_.height);
  msg->width = static_cast<uint32_t>(video_mode_.width);
  msg->encoding = "rgb8";
  msg->is_bigendian = 0;
  msg->step = msg->width * 3U;
  // NOTE: resize() value-initialises (zeroes) every byte. Its cost is
  // measured in the stats (alloc_us) to decide whether a pool is needed.
  msg->data.resize(static_cast<size_t>(video_mode_.bytes));
  return msg;
}

void KinectCameraNode::eventLoop()
{
  while (running_) {
    timeval timeout{0, kEventTimeoutUs};
    const int res = freenect_process_events_timeout(ctx_, &timeout);
    if (res < 0 && res != kLibusbErrorInterrupted) {
      RCLCPP_ERROR(get_logger(), "freenect_process_events failed (%d); event loop stopped", res);
      break;
    }
  }
}

void KinectCameraNode::videoCallback(freenect_device * dev, void * buffer, uint32_t timestamp)
{
  static_cast<KinectCameraNode *>(freenect_get_user(dev))->onVideoFrame(buffer, timestamp);
}

void KinectCameraNode::onVideoFrame(void * buffer, uint32_t /*timestamp*/)
{
  // Stamp first: this is the closest point to the frame arrival we can observe.
  const auto stamp = now();

  if (buffer != rgb_back_->data.data()) {
    // Should never happen: libfreenect writes only into the registered buffer.
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000, "Unexpected video buffer %p (expected %p); frame dropped",
      buffer, static_cast<void *>(rgb_back_->data.data()));
    return;
  }

  // Swap: take ownership of the filled message and register a new one.
  // We are on the libfreenect event thread, the only thread allowed to call
  // freenect_set_video_buffer while streaming (no locking inside libfreenect).
  const auto t_alloc = std::chrono::steady_clock::now();
  auto filled = std::move(rgb_back_);
  rgb_back_ = makeRgbMessage();
  freenect_set_video_buffer(dev_, rgb_back_->data.data());
  const auto alloc_ns = static_cast<uint64_t>(elapsedNs(t_alloc));

  filled->header.stamp = stamp;
  if (log_buffer_address_) {
    RCLCPP_INFO(
      get_logger(), "rgb publish stamp=%d.%09u addr=%p", filled->header.stamp.sec,
      filled->header.stamp.nanosec, static_cast<const void *>(filled->data.data()));
  }

  const auto t_pub = std::chrono::steady_clock::now();
  rgb_pub_->publish(std::move(filled));
  const auto publish_ns = static_cast<uint64_t>(elapsedNs(t_pub));

  std::lock_guard<std::mutex> lock(stats_mutex_);
  stats_.frames++;
  stats_.alloc_ns_sum += alloc_ns;
  stats_.alloc_ns_max = std::max(stats_.alloc_ns_max, alloc_ns);
  stats_.publish_ns_sum += publish_ns;
  stats_.publish_ns_max = std::max(stats_.publish_ns_max, publish_ns);
}

void KinectCameraNode::reportStats()
{
  FrameStats snapshot;
  {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    snapshot = stats_;
    stats_ = FrameStats{};
  }
  const auto t_now = now();
  const double period_s = (t_now - stats_start_).seconds();
  stats_start_ = t_now;
  if (snapshot.frames == 0) {
    RCLCPP_WARN(get_logger(), "No RGB frames in the last %.1f s", period_s);
    return;
  }
  const double n = static_cast<double>(snapshot.frames);
  RCLCPP_INFO(
    get_logger(),
    "rgb %.2f Hz | alloc+zero mean %.1f us max %.1f us | publish mean %.1f us max %.1f us | "
    "subs %zu (intra %zu)",
    n / period_s, snapshot.alloc_ns_sum / n / 1e3, snapshot.alloc_ns_max / 1e3,
    snapshot.publish_ns_sum / n / 1e3, snapshot.publish_ns_max / 1e3,
    rgb_pub_->get_subscription_count(), rgb_pub_->get_intra_process_subscription_count());
}

}  // namespace kinect_v1_driver

RCLCPP_COMPONENTS_REGISTER_NODE(kinect_v1_driver::KinectCameraNode)
