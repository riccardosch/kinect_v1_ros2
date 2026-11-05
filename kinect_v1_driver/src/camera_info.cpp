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

#include "kinect_v1_driver/camera_info.hpp"

#include <cstdlib>
#include <filesystem>
#include <memory>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "camera_calibration_parsers/parse.hpp"

namespace kinect_v1_driver
{

namespace
{

void replaceAll(std::string & text, const std::string & from, const std::string & to)
{
  for (size_t pos = text.find(from); pos != std::string::npos;
    pos = text.find(from, pos + to.size()))
  {
    text.replace(pos, from.size(), to);
  }
}

std::string rosHome()
{
  if (const char * ros_home = std::getenv("ROS_HOME"); ros_home && *ros_home) {
    return ros_home;
  }
  const char * home = std::getenv("HOME");
  return std::string(home ? home : "") + "/.ros";
}

}  // namespace

std::string resolveCameraInfoUrl(const std::string & url, const std::string & camera_name)
{
  std::string resolved = url.empty() ? kDefaultCameraInfoUrl : url;
  replaceAll(resolved, "${NAME}", camera_name);
  replaceAll(resolved, "${ROS_HOME}", rosHome());
  return resolved;
}

std::optional<std::string> cameraInfoUrlToPath(const std::string & resolved_url)
{
  const std::string file_scheme = "file://";
  const std::string package_scheme = "package://";
  if (resolved_url.rfind(file_scheme, 0) == 0) {
    const std::string path = resolved_url.substr(file_scheme.size());
    if (path.empty() || path.front() != '/') {
      return std::nullopt;  // only absolute paths, as in camera_info_manager
    }
    return path;
  }
  if (resolved_url.rfind(package_scheme, 0) == 0) {
    const std::string rest = resolved_url.substr(package_scheme.size());
    const size_t slash = rest.find('/');
    if (slash == std::string::npos || slash == 0 || slash + 1 >= rest.size()) {
      return std::nullopt;
    }
    try {
      return ament_index_cpp::get_package_share_directory(rest.substr(0, slash)) +
             rest.substr(slash);
    } catch (const std::exception &) {
      return std::nullopt;  // package not found
    }
  }
  return std::nullopt;
}

sensor_msgs::msg::CameraInfo makePinholeCameraInfo(
  uint32_t width, uint32_t height, double fx, double fy, double cx, double cy)
{
  sensor_msgs::msg::CameraInfo info;
  info.width = width;
  info.height = height;
  info.distortion_model = "plumb_bob";
  info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
  info.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
  info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  info.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
  return info;
}

double depthFocalLengthPx(double reference_distance_mm, double reference_pixel_size_mm)
{
  if (reference_pixel_size_mm <= 0.0) {
    return 0.0;
  }
  return reference_distance_mm / (2.0 * reference_pixel_size_mm);
}

CameraInfoSource::CameraInfoSource(
  rclcpp::Node & node, const std::string & ns, const std::string & camera_name,
  const std::string & url, const sensor_msgs::msg::CameraInfo & fallback)
: logger_(node.get_logger().get_child(ns)), camera_name_(camera_name), info_(fallback)
{
  const std::string resolved = resolveCameraInfoUrl(url, camera_name);
  const auto path = cameraInfoUrlToPath(resolved);
  if (!path) {
    RCLCPP_WARN(
      logger_, "Unsupported camera_info URL '%s' (use file:// or package://); "
      "using the default model, set_camera_info disabled", resolved.c_str());
  } else {
    path_ = *path;
    std::string file_camera_name;
    sensor_msgs::msg::CameraInfo loaded;
    if (!std::filesystem::exists(path_)) {
      RCLCPP_WARN(
        logger_, "No calibration at %s: using the default (uncalibrated) model. "
        "Calibrate with camera_calibration; set_camera_info will save it there.",
        path_.c_str());
    } else if (!camera_calibration_parsers::readCalibration(path_, file_camera_name, loaded)) {
      RCLCPP_ERROR(logger_, "Could not parse %s: using the default model", path_.c_str());
    } else if (loaded.width != fallback.width || loaded.height != fallback.height) {
      RCLCPP_ERROR(
        logger_, "%s is for %ux%u images, stream is %ux%u: using the default model",
        path_.c_str(), loaded.width, loaded.height, fallback.width, fallback.height);
    } else {
      info_ = loaded;
      calibrated_ = true;
      RCLCPP_INFO(
        logger_, "Loaded calibration '%s' from %s", file_camera_name.c_str(), path_.c_str());
    }
  }

  service_ = node.create_service<sensor_msgs::srv::SetCameraInfo>(
    ns + "/set_camera_info",
    [this](
      const std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Request> request,
      std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Response> response) {
      onSetCameraInfo(request, response);
    });
}

sensor_msgs::msg::CameraInfo CameraInfoSource::get() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return info_;
}

bool CameraInfoSource::isCalibrated() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return calibrated_;
}

void CameraInfoSource::onSetCameraInfo(
  const std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Request> request,
  std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Response> response)
{
  const auto & info = request->camera_info;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (info.width != info_.width || info.height != info_.height) {
      response->success = false;
      response->status_message = "Image size does not match the stream";
      return;
    }
  }
  if (path_.empty()) {
    response->success = false;
    response->status_message = "camera_info URL is not writable (use file:// or package://)";
    return;
  }
  std::error_code ec;
  std::filesystem::create_directories(std::filesystem::path(path_).parent_path(), ec);
  if (!camera_calibration_parsers::writeCalibration(path_, camera_name_, info)) {
    response->success = false;
    response->status_message = "Could not write " + path_;
    RCLCPP_ERROR(logger_, "%s", response->status_message.c_str());
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    info_ = info;
    info_.header = std_msgs::msg::Header();
    calibrated_ = true;
  }
  response->success = true;
  response->status_message = "Saved to " + path_;
  RCLCPP_INFO(logger_, "New calibration saved to %s", path_.c_str());
}

}  // namespace kinect_v1_driver
