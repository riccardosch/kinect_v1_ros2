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

#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <string>

#include "camera_calibration_parsers/parse.hpp"
#include "kinect_v1_driver/camera_info.hpp"

using kinect_v1_driver::cameraInfoUrlToPath;
using kinect_v1_driver::depthFocalLengthPx;
using kinect_v1_driver::makePinholeCameraInfo;
using kinect_v1_driver::resolveCameraInfoUrl;

TEST(CameraInfoUrl, EmptyUrlUsesRosHomeAndName)
{
  setenv("ROS_HOME", "/tmp/kinect_test_ros_home", 1);
  EXPECT_EQ(
    resolveCameraInfoUrl("", "rgb_A00362A02228049A"),
    "file:///tmp/kinect_test_ros_home/camera_info/rgb_A00362A02228049A.yaml");
}

TEST(CameraInfoUrl, RosHomeFallsBackToHomeDotRos)
{
  unsetenv("ROS_HOME");
  setenv("HOME", "/home/someone", 1);
  EXPECT_EQ(
    resolveCameraInfoUrl("file://${ROS_HOME}/x.yaml", "n"),
    "file:///home/someone/.ros/x.yaml");
}

TEST(CameraInfoUrl, NameIsSubstitutedEverywhere)
{
  EXPECT_EQ(
    resolveCameraInfoUrl("file:///c/${NAME}/${NAME}.yaml", "depth_1"),
    "file:///c/depth_1/depth_1.yaml");
}

TEST(CameraInfoUrl, FileAndPackageSchemes)
{
  EXPECT_EQ(cameraInfoUrlToPath("file:///abs/cal.yaml"), "/abs/cal.yaml");
  EXPECT_FALSE(cameraInfoUrlToPath("file://relative/cal.yaml").has_value());
  EXPECT_FALSE(cameraInfoUrlToPath("http://example.com/cal.yaml").has_value());
  EXPECT_FALSE(cameraInfoUrlToPath("package://no_such_package_xyz/cal.yaml").has_value());
  EXPECT_FALSE(cameraInfoUrlToPath("package://rclcpp").has_value());

  // rclcpp is always installed in a ROS environment.
  const auto path = cameraInfoUrlToPath("package://rclcpp/calib/cam.yaml");
  ASSERT_TRUE(path.has_value());
  EXPECT_NE(path->find("/share/rclcpp/calib/cam.yaml"), std::string::npos);
}

TEST(CameraInfoModel, PinholeMatrices)
{
  const auto info = makePinholeCameraInfo(640, 480, 575.8, 575.8, 320.0, 240.0);
  EXPECT_EQ(info.width, 640u);
  EXPECT_EQ(info.height, 480u);
  EXPECT_EQ(info.distortion_model, "plumb_bob");
  ASSERT_EQ(info.d.size(), 5u);
  for (double d : info.d) {
    EXPECT_EQ(d, 0.0);
  }
  EXPECT_DOUBLE_EQ(info.k[0], 575.8);  // fx
  EXPECT_DOUBLE_EQ(info.k[2], 320.0);  // cx
  EXPECT_DOUBLE_EQ(info.k[4], 575.8);  // fy
  EXPECT_DOUBLE_EQ(info.k[5], 240.0);  // cy
  EXPECT_DOUBLE_EQ(info.k[8], 1.0);
  EXPECT_DOUBLE_EQ(info.p[0], 575.8);
  EXPECT_DOUBLE_EQ(info.p[3], 0.0);  // monocular: no baseline term
  EXPECT_DOUBLE_EQ(info.r[0] * info.r[4] * info.r[8], 1.0);
}

TEST(CameraInfoModel, DepthFocalFromZeroPlane)
{
  // Typical Kinect v1 factory data: 120 mm, 0.1042 mm -> about 575.8 px.
  EXPECT_NEAR(depthFocalLengthPx(120.0, 0.1042), 575.8, 0.1);
  EXPECT_EQ(depthFocalLengthPx(120.0, 0.0), 0.0);  // invalid data
}

TEST(CameraInfoFile, WriteThenReadRoundTrip)
{
  const auto dir = std::filesystem::temp_directory_path() / "kinect_v1_driver_test";
  std::filesystem::create_directories(dir);
  const auto file = (dir / "rgb_test.yaml").string();
  auto info = makePinholeCameraInfo(640, 480, 520.0, 521.0, 318.0, 242.0);
  info.d = {0.1, -0.2, 0.001, 0.002, 0.0};
  ASSERT_TRUE(camera_calibration_parsers::writeCalibration(file, "rgb_test", info));

  std::string name;
  sensor_msgs::msg::CameraInfo loaded;
  ASSERT_TRUE(camera_calibration_parsers::readCalibration(file, name, loaded));
  EXPECT_EQ(name, "rgb_test");
  EXPECT_EQ(loaded.width, 640u);
  EXPECT_DOUBLE_EQ(loaded.k[0], 520.0);
  EXPECT_DOUBLE_EQ(loaded.k[4], 521.0);
  EXPECT_DOUBLE_EQ(loaded.d[1], -0.2);
  std::filesystem::remove_all(dir);
}
