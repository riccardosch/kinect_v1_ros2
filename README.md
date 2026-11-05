# kinect_v1_ros2

ROS 2 Jazzy driver for the **Microsoft Kinect v1** (Xbox 360, model 1414), written from scratch on top of [libfreenect](https://github.com/OpenKinect/libfreenect).

It uses all the hardware (RGB, depth, LED, accelerometer, 4 microphones), publishes standard ROS messages, and moves images to consumers in the same process **without copying them**.

> **Status: work in progress.** Working: RGB, depth, `camera_info` and calibration, LED, IMU, microphones. Coming next: point cloud, compressed topics for remote viewing. The tilt motor is intentionally not driven.

---

## Quick start

### 1. Requirements

- **Kinect v1, model 1414**, powered by its **12 V adapter**. With USB power alone only the motor enumerates.
- **Ubuntu 24.04 + ROS 2 Jazzy** (tested on a Raspberry Pi 5).
- **libfreenect** built from source. `BUILD_REDIST_PACKAGE=OFF` downloads and installs the audio firmware:

  ```bash
  git clone https://github.com/OpenKinect/libfreenect.git ~/src/libfreenect
  cd ~/src/libfreenect && mkdir build && cd build
  cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_REDIST_PACKAGE=OFF
  make -j4 && sudo make install && sudo ldconfig
  ```

### 2. Install

```bash
git clone https://github.com/riccardosch/kinect_v1_ros2.git ~/kinect && cd ~/kinect

# ROS dependencies, from the package.xml files (or apt directly:
#   ros-jazzy-camera-calibration-parsers ros-jazzy-audio-common-msgs ros-jazzy-depth-image-proc)
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths . --ignore-src -y

# USB permissions (udev rule for the plugdev group) and kernel webcam driver off
sudo cp scripts/51-kinect.rules /etc/udev/rules.d/ && sudo udevadm control --reload-rules && sudo udevadm trigger
sudo cp scripts/blacklist-gspca-kinect.conf /etc/modprobe.d/ && sudo modprobe -r gspca_kinect
groups | grep -q plugdev || sudo usermod -aG plugdev $USER   # then log out and back in

# Build
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Check the hardware: `lsusb | grep 045e` must list **three** devices (`02b0` motor, `02ad` audio, `02ae` camera).

### 3. Run

```bash
cd ~/kinect && source install/setup.bash
ros2 launch kinect_v1_bringup kinect.launch.py
```

This one command starts everything in a single multi-threaded container with intra-process communication.

| Variant | Command |
|---|---|
| Depth aligned to the RGB image | `ros2 launch kinect_v1_bringup kinect.launch.py depth_mode:=registered` |
| Camera only (no LED / IMU / audio) | `ros2 launch kinect_v1_bringup kinect.launch.py aux:=false` |
| LED + IMU + audio only | `ros2 launch kinect_v1_bringup kinect.launch.py camera:=false` |
| No microphones | `ros2 launch kinect_v1_bringup kinect.launch.py enable_audio:=false` |

All launch arguments: `camera`, `aux`, `enable_rgb`, `depth_mode` (`mm` \| `registered` \| `off`), `imu_rate_hz`, `initial_led`, `enable_audio`, `publish_cancelled`, `qos_depth`.

### 4. Topics

All topics are in the `/kinect` namespace.

| Topic | Type | Rate |
|---|---|---|
| `rgb/image_raw` + `rgb/camera_info` | `sensor_msgs/Image` `rgb8` 640×480 + `CameraInfo` | 30 Hz |
| `depth/image_raw` + `depth/camera_info` | `sensor_msgs/Image` `16UC1` (mm, 0 = no reading) + `CameraInfo` | 30 Hz |
| `depth_registered/image_raw` + `depth_registered/camera_info` | same, aligned to RGB (instead of `depth/`, with `depth_mode:=registered`) | 30 Hz |
| `imu` | `sensor_msgs/Imu` (linear acceleration only) | 30 Hz |
| `audio/data` + `audio/info` | `audio_common_msgs/AudioDataStamped` 4 ch `S32LE` 16 kHz + `AudioInfo` (latched) | 62.5 msg/s |
| `led/cmd` (input) | `std_msgs/String`: `off` `green` `red` `yellow` `blink_green` `blink_red_yellow` | — |
| `rgb/set_camera_info`, `depth/set_camera_info` (services) | `sensor_msgs/srv/SetCameraInfo` | — |

Image topics are **best effort**. With the CLI, use `--qos-reliability best_effort`, otherwise you receive nothing.

### 5. Common tasks

```bash
ros2 topic pub --once /kinect/led/cmd std_msgs/msg/String "{data: red}"                 # LED
ros2 topic echo /kinect/imu --qos-reliability best_effort --field linear_acceleration    # accelerometer
ros2 run kinect_v1_bench audio_stats.py 5 /tmp/mic.wav                                   # record 5 s of audio
ros2 run rqt_image_view rqt_image_view /kinect/rgb/image_raw                             # view images
```

To calibrate the RGB camera with `camera_calibration`, see [docs/camera.md](docs/camera.md#calibrating-the-rgb-camera).

---

## Features

| | Feature | Details |
|---|---|---|
| ⚡ | **Zero-copy images.** libfreenect writes each frame straight into the ROS message, which reaches consumers in the same container without any copy. Verified frame by frame, with a negative control. | [docs/zero-copy.md](docs/zero-copy.md) |
| 📷 | **Camera.** RGB and depth (millimetres, or registered to RGB); `camera_info` on every frame with the same stamp; per-device calibration files; `set_camera_info` for `camera_calibration`; depth intrinsics read from the device's factory data. | [docs/camera.md](docs/camera.md) |
| 💡 | **LED, IMU, microphones.** LED commands; accelerometer as `sensor_msgs/Imu` (REP 103 / 145); 4 microphones at 16 kHz with sample-accurate timestamps. All device calls are owned by one thread. | [docs/aux.md](docs/aux.md) |
| 📊 | **Measured performance.** On a Pi 5: 30 Hz on every stream; image latency p95 < 0.6 ms in the container and ≈ 1.7 ms across processes; CPU ≈ 8 % of one core for RGB. | [docs/benchmark.md](docs/benchmark.md) |
| ✅ | **Tests.** Lint and unit tests (no hardware), plus 13 hardware tests, each with a numeric pass criterion and a status table. | [docs/testing.md](docs/testing.md) |
| 🛠 | **Troubleshooting.** No device found, nothing received, QoS mismatches, and more. | [docs/troubleshooting.md](docs/troubleshooting.md) |

## Repository layout

| Path | Package | Content |
|---|---|---|
| `kinect_v1_driver/` | `kinect_v1_driver` | Components `KinectCameraNode` (RGB, depth, camera_info) and `KinectAuxNode` (LED, IMU, audio) |
| `kinect_v1_bringup/` | `kinect_v1_bringup` | `kinect.launch.py` (everything) and `camera.launch.py` (camera + benchmark options) |
| `tools/bench/` | `kinect_v1_bench` | Measurement node (`image_stats_node`) and check scripts used by the tests |
| `scripts/` | — | udev rule and kernel-module blacklist |
| `docs/` | — | Detailed documentation |

## License

Apache-2.0, see [LICENSE](LICENSE).
