# kinect_v1_ros2

ROS 2 Jazzy driver for the Microsoft Kinect v1 (Xbox 360, model 1414), written from scratch on top of [libfreenect](https://github.com/OpenKinect/libfreenect).

> **Status: work in progress.** RGB and depth (mm / registered) are implemented. LED, audio, tilt and IMU are coming.

## Design goals

1. **Minimal-cost internal pipeline.** Images move with zero copies between components in the same container.
2. **Low-lag remote visualisation.** Compressed topics, best effort, queue depth 1 *(planned)*.

### How the RGB zero-copy works

- libfreenect converts each frame (Bayer → RGB) **directly into the `data` vector of a `sensor_msgs/Image`** owned by the driver (`freenect_set_video_buffer`).
- In the frame callback, which runs on the dedicated libfreenect event thread, the driver:
  1. stamps the filled message,
  2. registers a freshly allocated message as the next buffer,
  3. publishes the filled one as a `std::unique_ptr`.
- With `use_intra_process_comms` enabled and `ConstSharedPtr` subscribers, rclcpp passes the same buffer to the consumers.

What this does **not** cover:
- The Bayer → RGB conversion inside libfreenect is still one pass over the frame.
- Subscribers in other processes always receive a serialized copy.
- Each new message is allocated and zero-filled. The driver reports the cost of this as `alloc+zero` in its periodic stats.

## Packages

| Package | Path | Content |
|---|---|---|
| `kinect_v1_driver` | `kinect_v1_driver/` | `kinect_v1_driver::KinectCameraNode` component (+ `kinect_camera_node` executable) |
| `kinect_v1_bench` | `tools/bench/` | `kinect_v1_bench::ImageStatsNode`: rate, latency (now − stamp), buffer address, intra-process delivery |
| `kinect_v1_bringup` | `kinect_v1_bringup/` | `camera.launch.py`: `component_container_mt` with intra-process comms |

## Requirements

- Ubuntu 24.04, ROS 2 Jazzy.
- libfreenect built from source and installed in `/usr/local` (provides `libfreenectConfig.cmake`).
- Kinect powered by its **12 V adapter**. With USB power alone only the motor device (`045e:02b0`) enumerates.
- User in the `plugdev` group, with a udev rule granting access to `045e:02ae/02ad/02b0`.

## Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Run

```bash
# Driver + in-container consumer (benchmark scenario A)
ros2 launch kinect_v1_bringup camera.launch.py bench:=true

# Zero-copy check: per-frame buffer addresses from publisher and consumer
ros2 launch kinect_v1_bringup camera.launch.py bench:=true log_buffer_address:=true

# Consumer in a separate process (benchmark scenario B)
ros2 run kinect_v1_bench image_stats_node --ros-args -r image:=/kinect/rgb/image_raw

# CPU of the container, once per second for 60 s
install/kinect_v1_bench/lib/kinect_v1_bench/cpu_usage.sh kinect_container 60
```

### Topics (namespace `kinect`)

| Topic | Type | QoS |
|---|---|---|
| `rgb/image_raw` | `sensor_msgs/Image` (`rgb8`, 640×480) | best effort, keep last `qos_depth`, volatile |
| `depth/image_raw` | `sensor_msgs/Image` (`16UC1`, millimetres, 0 = no reading, 640×480) | best effort, keep last `qos_depth`, volatile |

`header.stamp` is taken when the frame callback fires (host clock). RGB and depth are **not** hardware-synchronised.

### Parameters (`kinect_camera`)

| Name | Default | Description |
|---|---|---|
| `device_index` | `0` | Index of the Kinect camera on the bus |
| `enable_rgb` | `true` | Stream RGB |
| `depth_mode` | `mm` | `mm`: depth in the IR camera geometry; `registered`: re-projected onto the RGB camera (same frame and pixel grid as RGB, more holes, more CPU); `off` |
| `rgb_frame_id` | `kinect_rgb_optical_frame` | `header.frame_id` of RGB images (and of depth in `registered` mode) |
| `depth_frame_id` | `kinect_depth_optical_frame` | `header.frame_id` of depth images in `mm` mode |
| `qos_depth` | `1` | History depth of sensor topics |
| `stats_period_s` | `5.0` | Period of the rate/alloc/publish stats log (0 = off) |
| `log_buffer_address` | `false` | Log the data address of every published frame |

## License

Apache-2.0, see [LICENSE](LICENSE).
