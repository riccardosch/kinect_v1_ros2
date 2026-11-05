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

#ifndef KINECT_V1_DRIVER__CAMERA_INFO_HPP_
#define KINECT_V1_DRIVER__CAMERA_INFO_HPP_

#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/srv/set_camera_info.hpp"

namespace kinect_v1_driver
{

/// Default calibration URL, same convention as camera_info_manager.
constexpr const char * kDefaultCameraInfoUrl = "file://${ROS_HOME}/camera_info/${NAME}.yaml";

/// Expands ${NAME} and ${ROS_HOME} (ROS_HOME env var, else $HOME/.ros).
/// An empty URL resolves to kDefaultCameraInfoUrl.
std::string resolveCameraInfoUrl(const std::string & url, const std::string & camera_name);

/// Maps a resolved URL to a local file path:
///   file:///abs/path.yaml          -> /abs/path.yaml
///   package://pkg/rel/path.yaml    -> <share dir of pkg>/rel/path.yaml
/// Returns std::nullopt for unsupported schemes or unknown packages.
std::optional<std::string> cameraInfoUrlToPath(const std::string & resolved_url);

/// Ideal pinhole model (no distortion, plumb_bob with D = 0).
sensor_msgs::msg::CameraInfo makePinholeCameraInfo(
  uint32_t width, uint32_t height, double fx, double fy, double cx, double cy);

/// Focal length, in pixels of the 640x480 depth image, from the factory
/// zero-plane data. libfreenect's own camera->world mapping
/// (freenect_camera_to_world, registration.c) uses
///   x = (u - 320) * 2 * reference_pixel_size * z / reference_distance
/// i.e. f = reference_distance / (2 * reference_pixel_size).
double depthFocalLengthPx(double reference_distance_mm, double reference_pixel_size_mm);

/// Calibration of one camera: loads it from a URL, falls back to a default
/// model, and serves <ns>/set_camera_info (the service camera_calibration
/// calls) by writing the new calibration back to the same URL.
/// Thread-safe: get() can be called from the libfreenect event thread while
/// the service runs on an executor thread.
class CameraInfoSource
{
public:
  CameraInfoSource(
    rclcpp::Node & node, const std::string & ns, const std::string & camera_name,
    const std::string & url, const sensor_msgs::msg::CameraInfo & fallback);

  /// Copy of the current calibration (header not set).
  sensor_msgs::msg::CameraInfo get() const;
  bool isCalibrated() const;
  const std::string & path() const {return path_;}

private:
  void onSetCameraInfo(
    const std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Request> request,
    std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Response> response);

  rclcpp::Logger logger_;
  std::string camera_name_;
  std::string path_;  // empty if the URL is not writable/supported
  mutable std::mutex mutex_;
  sensor_msgs::msg::CameraInfo info_;
  bool calibrated_{false};
  rclcpp::Service<sensor_msgs::srv::SetCameraInfo>::SharedPtr service_;
};

}  // namespace kinect_v1_driver

#endif  // KINECT_V1_DRIVER__CAMERA_INFO_HPP_
