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
  const auto enable_rgb = declare_parameter<bool>("enable_rgb", true);
  depth_mode_ = declare_parameter<std::string>("depth_mode", "mm");
  const auto rgb_frame_id =
    declare_parameter<std::string>("rgb_frame_id", "kinect_rgb_optical_frame");
  const auto depth_frame_id =
    declare_parameter<std::string>("depth_frame_id", "kinect_depth_optical_frame");
  const auto qos_depth = declare_parameter<int>("qos_depth", 1);
  const auto stats_period_s = declare_parameter<double>("stats_period_s", 5.0);
  log_buffer_address_ = declare_parameter<bool>("log_buffer_address", false);

  if (qos_depth < 1) {
    throw std::invalid_argument("qos_depth must be >= 1");
  }
  if (depth_mode_ != "mm" && depth_mode_ != "registered" && depth_mode_ != "off") {
    throw std::invalid_argument("depth_mode must be 'mm', 'registered' or 'off'");
  }
  if (!enable_rgb && depth_mode_ == "off") {
    throw std::invalid_argument("Both RGB and depth are disabled: nothing to do");
  }

  rgb_.name = "rgb";
  rgb_.enabled = enable_rgb;
  rgb_.set_buffer = &freenect_set_video_buffer;
  rgb_.mode = freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB);
  rgb_.frame_id = rgb_frame_id;
  rgb_.encoding = "rgb8";
  rgb_.bytes_per_pixel = 3;

  // DEPTH_MM: millimetres in the IR (depth) camera geometry.
  // DEPTH_REGISTERED: millimetres re-projected onto the RGB camera, so the
  // image shares the RGB optical frame and pixel grid.
  depth_.name = "depth";
  depth_.enabled = depth_mode_ != "off";
  depth_.set_buffer = &freenect_set_depth_buffer;
  depth_.mode = freenect_find_depth_mode(
    FREENECT_RESOLUTION_MEDIUM,
    depth_mode_ == "registered" ? FREENECT_DEPTH_REGISTERED : FREENECT_DEPTH_MM);
  depth_.frame_id = depth_mode_ == "registered" ? rgb_frame_id : depth_frame_id;
  depth_.encoding = "16UC1";
  depth_.bytes_per_pixel = 2;

  // Sensor QoS: never block or retransmit; keep_last + volatile are also the
  // requirements for intra-process communication.
  const auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<size_t>(qos_depth)))
    .best_effort()
    .durability_volatile();
  if (rgb_.enabled) {
    rgb_.pub = create_publisher<sensor_msgs::msg::Image>("rgb/image_raw", sensor_qos);
  }
  if (depth_.enabled) {
    depth_.pub = create_publisher<sensor_msgs::msg::Image>("depth/image_raw", sensor_qos);
  }

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
  // free for another context (the aux component).
  freenect_select_subdevices(ctx_, FREENECT_DEVICE_CAMERA);

  const int num_devices = freenect_num_devices(ctx_);
  if (num_devices <= device_index_) {
    freenect_shutdown(ctx_);
    ctx_ = nullptr;
    throw std::runtime_error(
            "Kinect #" + std::to_string(device_index_) + " not found (" +
            std::to_string(num_devices) + " connected; is the 12 V adapter plugged in?)");
  }
  if (freenect_open_device(ctx_, &dev_, device_index_) < 0) {
    freenect_shutdown(ctx_);
    ctx_ = nullptr;
    dev_ = nullptr;
    throw std::runtime_error("freenect_open_device failed (is the device busy or not permitted?)");
  }

  freenect_set_user(dev_, this);

  if (rgb_.enabled) {
    if (!rgb_.mode.is_valid || freenect_set_video_mode(dev_, rgb_.mode) < 0) {
      closeDevice();
      throw std::runtime_error("Could not set RGB video mode");
    }
    freenect_set_video_callback(dev_, &KinectCameraNode::videoCallback);
  }
  if (depth_.enabled) {
    if (!depth_.mode.is_valid || freenect_set_depth_mode(dev_, depth_.mode) < 0) {
      closeDevice();
      throw std::runtime_error("Could not set depth mode");
    }
    freenect_set_depth_callback(dev_, &KinectCameraNode::depthCallback);
  }

  try {
    startStream(rgb_);
    startStream(depth_);
  } catch (...) {
    closeDevice();
    throw;
  }

  RCLCPP_INFO(
    get_logger(), "Kinect #%d streaming: rgb %s, depth %s (frame '%s'), libfreenect %s",
    device_index_, rgb_.enabled ? "640x480@30" : "off", depth_mode_.c_str(),
    depth_.frame_id.c_str(), KINECT_V1_LIBFREENECT_VERSION);
}

void KinectCameraNode::startStream(Stream & stream)
{
  if (!stream.enabled) {
    return;
  }
  // The user buffer must be registered BEFORE start_*, otherwise libfreenect
  // allocates its own internal buffer (cameras.c, stream_init).
  stream.back = makeMessage(stream);
  stream.set_buffer(dev_, stream.back->data.data());
  const int res = (&stream == &rgb_) ? freenect_start_video(dev_) : freenect_start_depth(dev_);
  if (res < 0) {
    throw std::runtime_error(std::string("Could not start ") + stream.name + " stream");
  }
}

void KinectCameraNode::closeDevice()
{
  // Order matters: stop the streams (no more writes into our buffers), then
  // detach the user buffers, then release the device and finally the messages.
  if (dev_ != nullptr) {
    freenect_stop_video(dev_);
    freenect_stop_depth(dev_);
    freenect_set_video_buffer(dev_, nullptr);
    freenect_set_depth_buffer(dev_, nullptr);
    freenect_close_device(dev_);
    dev_ = nullptr;
  }
  if (ctx_ != nullptr) {
    freenect_shutdown(ctx_);
    ctx_ = nullptr;
  }
  rgb_.back.reset();
  depth_.back.reset();
}

sensor_msgs::msg::Image::UniquePtr KinectCameraNode::makeMessage(const Stream & stream) const
{
  auto msg = std::make_unique<sensor_msgs::msg::Image>();
  msg->header.frame_id = stream.frame_id;
  msg->height = static_cast<uint32_t>(stream.mode.height);
  msg->width = static_cast<uint32_t>(stream.mode.width);
  msg->encoding = stream.encoding;
  // libfreenect writes 16-bit depth in host byte order (little-endian on
  // x86 and aarch64).
  msg->is_bigendian = 0;
  msg->step = msg->width * stream.bytes_per_pixel;
  // NOTE: resize() value-initialises (zeroes) every byte. Its cost is
  // measured in the stats (alloc) to decide whether a pool is needed.
  msg->data.resize(static_cast<size_t>(stream.mode.bytes));
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

void KinectCameraNode::videoCallback(freenect_device * dev, void * buffer, uint32_t)
{
  auto * self = static_cast<KinectCameraNode *>(freenect_get_user(dev));
  self->onFrame(self->rgb_, buffer);
}

void KinectCameraNode::depthCallback(freenect_device * dev, void * buffer, uint32_t)
{
  auto * self = static_cast<KinectCameraNode *>(freenect_get_user(dev));
  self->onFrame(self->depth_, buffer);
}

void KinectCameraNode::onFrame(Stream & stream, void * buffer)
{
  // Stamp first: this is the closest point to the frame arrival we can observe.
  const auto stamp = now();

  if (buffer != stream.back->data.data()) {
    // Should never happen: libfreenect writes only into the registered buffer.
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000, "Unexpected %s buffer %p (expected %p); frame dropped",
      stream.name, buffer, static_cast<void *>(stream.back->data.data()));
    return;
  }

  // Swap: take ownership of the filled message and register a new one.
  // We are on the libfreenect event thread, the only thread allowed to call
  // freenect_set_*_buffer while streaming (no locking inside libfreenect).
  const auto t_alloc = std::chrono::steady_clock::now();
  auto filled = std::move(stream.back);
  stream.back = makeMessage(stream);
  stream.set_buffer(dev_, stream.back->data.data());
  const auto alloc_ns = static_cast<uint64_t>(elapsedNs(t_alloc));

  filled->header.stamp = stamp;
  if (log_buffer_address_) {
    RCLCPP_INFO(
      get_logger(), "%s publish stamp=%d.%09u addr=%p", stream.name, filled->header.stamp.sec,
      filled->header.stamp.nanosec, static_cast<const void *>(filled->data.data()));
  }

  const auto t_pub = std::chrono::steady_clock::now();
  stream.pub->publish(std::move(filled));
  const auto publish_ns = static_cast<uint64_t>(elapsedNs(t_pub));

  std::lock_guard<std::mutex> lock(stats_mutex_);
  stream.stats.frames++;
  stream.stats.alloc_ns_sum += alloc_ns;
  stream.stats.alloc_ns_max = std::max(stream.stats.alloc_ns_max, alloc_ns);
  stream.stats.publish_ns_sum += publish_ns;
  stream.stats.publish_ns_max = std::max(stream.stats.publish_ns_max, publish_ns);
}

void KinectCameraNode::reportStats()
{
  FrameStats rgb_stats;
  FrameStats depth_stats;
  {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    rgb_stats = rgb_.stats;
    depth_stats = depth_.stats;
    rgb_.stats = FrameStats{};
    depth_.stats = FrameStats{};
  }
  const auto t_now = now();
  const double period_s = (t_now - stats_start_).seconds();
  stats_start_ = t_now;

  for (const auto & [stream, s] : {std::pair<const Stream *, FrameStats>{&rgb_, rgb_stats},
      std::pair<const Stream *, FrameStats>{&depth_, depth_stats}})
  {
    if (!stream->enabled) {
      continue;
    }
    if (s.frames == 0) {
      RCLCPP_WARN(get_logger(), "No %s frames in the last %.1f s", stream->name, period_s);
      continue;
    }
    const double n = static_cast<double>(s.frames);
    RCLCPP_INFO(
      get_logger(),
      "%-5s %.2f Hz | alloc+zero mean %.1f us max %.1f us | publish mean %.1f us max %.1f us | "
      "subs %zu (intra %zu)",
      stream->name, n / period_s, s.alloc_ns_sum / n / 1e3, s.alloc_ns_max / 1e3,
      s.publish_ns_sum / n / 1e3, s.publish_ns_max / 1e3, stream->pub->get_subscription_count(),
      stream->pub->get_intra_process_subscription_count());
  }
}

}  // namespace kinect_v1_driver

RCLCPP_COMPONENTS_REGISTER_NODE(kinect_v1_driver::KinectCameraNode)
