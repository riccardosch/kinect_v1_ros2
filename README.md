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
| `kinect_v1_bench` | `tools/bench/` | `kinect_v1_bench::ImageStatsNode`: rate, latency (now − stamp), buffer address, intra-process delivery. Scripts: `check_zero_copy.py`, `depth_stats.py`, `cpu_usage.sh` |
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

## Testing

The tests below need the Kinect connected. Except for step 1, run every command in a terminal set up with:

```bash
cd ~/kinect && source install/setup.bash
B_RUN=install/kinect_v1_bench/lib/kinect_v1_bench   # shorthand for the bench scripts
```

Only one process at a time can own the camera. Stop the driver (Ctrl-C) before starting another one.

### 0. Hardware check

```bash
lsusb | grep 045e
```

Expected: **three** devices, `02b0` (Motor), `02ad` (Audio) and `02ae` (Camera). If only `02b0` is listed, the 12 V adapter is not powering the sensor (see [Troubleshooting](#troubleshooting)).

### 1. Build and static checks (no hardware needed)

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test && colcon test-result --verbose
```

Pass: the build has no warnings, and `colcon test-result` reports `0 errors, 0 failures` (cpplint, uncrustify, copyright, flake8, xmllint, ...).

### 2. Smoke test

```bash
ros2 launch kinect_v1_bringup camera.launch.py
```

Expected log, within about 1 s:

```
[kinect.kinect_camera]: Intra-process communication: enabled
[kinect.kinect_camera]: Kinect #0 streaming: rgb 640x480@30, depth mm (frame 'kinect_depth_optical_frame'), libfreenect 0.7.5
```

Then, every 5 s, one line per stream:

```
rgb   30.00 Hz | alloc+zero mean ... us | publish mean ... us | subs 0 (intra 0)
depth 30.00 Hz | ...
```

In a second terminal:

```bash
ros2 topic list | grep kinect                    # /kinect/rgb/image_raw, /kinect/depth/image_raw
ros2 topic info -v /kinect/rgb/image_raw         # Reliability: BEST_EFFORT, History: KEEP_LAST (1), Durability: VOLATILE
ros2 topic echo /kinect/depth/image_raw --field header --qos-reliability best_effort
ros2 param dump /kinect/kinect_camera
```

Pass:
- both streams run at 29.5–30.5 Hz;
- `header.frame_id` is `kinect_depth_optical_frame` for depth in `mm` mode;
- `header.stamp` increases by about 33 ms between frames.

> `ros2 topic echo` needs `--qos-reliability best_effort`. A reliable subscriber receives **nothing** from a best-effort publisher, and no error is shown.

### 3. Parameters and modes

| Command | Expected |
|---|---|
| `ros2 launch kinect_v1_bringup camera.launch.py depth_mode:=registered` | depth `frame_id` = `kinect_rgb_optical_frame` |
| `ros2 launch kinect_v1_bringup camera.launch.py depth_mode:=off` | only `rgb` stats lines; no depth topic |
| `ros2 launch kinect_v1_bringup camera.launch.py enable_rgb:=false` | only `depth` stats lines; no RGB topic |
| `ros2 launch kinect_v1_bringup camera.launch.py depth_mode:=foo` | load fails: `depth_mode must be 'mm', 'registered' or 'off'` |
| `ros2 launch kinect_v1_bringup camera.launch.py enable_rgb:=false depth_mode:=off` | load fails: `Both RGB and depth are disabled` |

### 4. Zero-copy (benchmark scenario A)

Launch the driver with one consumer per stream in the same container, logging every buffer address. Let it run for about 10 s, then stop it with Ctrl-C:

```bash
ros2 launch kinect_v1_bringup camera.launch.py bench:=true log_buffer_address:=true 2>&1 | tee /tmp/zc.log
ros2 run kinect_v1_bench check_zero_copy.py /tmp/zc.log
```

Pass: `PASS` for both streams, and exit code 0. Every received frame must have the **same buffer address** as the one the driver registered with libfreenect, **and** must have been delivered through the intra-process path.

```
depth  published   260  received   257  zero-copy   257  -> PASS
rgb    published   265  received   258  zero-copy   258  -> PASS
```

A few frames more on the published side is normal: they were published before the consumer subscribed or after shutdown started.

**Negative control.** The same procedure with `intra_process:=false` must report `FAIL`. With intra-process disabled, malloc often recycles the publisher's freed buffer for the deserialized message, so the addresses can match by coincidence. The `intra` flag (`rclcpp::MessageInfo::from_intra_process`) is what tells the two cases apart.

### 5. Rate and latency

**Scenario A** (same container), without per-frame logging:

```bash
ros2 launch kinect_v1_bringup camera.launch.py bench:=true
```

**Scenario B** (separate process). Run the driver as in step 2, then in another terminal:

```bash
ros2 run kinect_v1_bench image_stats_node --ros-args -r image:=/kinect/rgb/image_raw
ros2 run kinect_v1_bench image_stats_node --ros-args -r image:=/kinect/depth/image_raw
```

Every 5 s the bench reports: rate, lag mean, p95 and max (`now − header.stamp`), how many messages arrived intra-process, and the last buffer address.

Pass, with reference values measured on the Raspberry Pi 5 (see [docs/benchmark.md](docs/benchmark.md)):

| | Scenario A | Scenario B |
|---|---|---|
| Rate | 29.5–30.5 Hz | 29.5–30.5 Hz |
| `intra-process` | N/N (all) | 0/N |
| Lag p95 | < 1 ms (measured ≈ 0.3–0.55 ms) | < 3 ms (measured ≈ 1.6–1.8 ms) |
| Driver `publish mean` | < 50 µs | hundreds of µs (serialization) |

### 6. CPU usage

While the driver runs (any scenario):

```bash
$B_RUN/cpu_usage.sh component_container 30 | tee /tmp/cpu.txt
```

The script prints `pidstat` output once per second for the process and each of its threads. The process line (TID `-`) gives the total. Reference: about 8 % of one core for RGB in scenario A, almost all of it in the libfreenect event thread (Bayer → RGB conversion).

### 7. Depth content and accuracy

Point the centre of the image at a flat, matte surface (a wall or a box) at a **tape-measured** distance between 0.8 and 4 m. Run the driver (step 2), then:

```bash
ros2 run kinect_v1_bench depth_stats.py /kinect/depth/image_raw 60
```

Expected output:

```
frames 60  640x480  frame_id 'kinect_depth_optical_frame'
no-reading pixels: mean 8.9 %  (min 8.5, max 9.4)
centre 10x10 median: 2496 mm  (std 9.4 mm over 60 frames)
```

Pass:
- the centre median is within **±5 %** of the measured distance;
- the frame-to-frame standard deviation stays below about 1 % of the distance.

Repeat with `depth_mode:=registered`: the median should agree with `mm` within about 2 %, while the no-reading fraction is higher (re-projection shadows and borders, ≈ 20 %).

Things to know:
- 0 means *no reading*, not zero distance. The Kinect cannot measure closer than about 0.5 m, nor on black or shiny surfaces.
- The very first depth frame after start can be incomplete (centre 0, more holes).

### 8. Visual check

With a display available (needs `sudo apt install ros-jazzy-rqt-image-view`):

```bash
ros2 run rqt_image_view rqt_image_view
```

Select `/kinect/rgb/image_raw` and `/kinect/depth/image_raw`.

Pass:
- the RGB image has correct colours (red objects look red, so R and B are not swapped);
- the depth image shows the scene shapes, with black holes at the object edges.

### 9. Endurance

Run scenario A for 10 minutes:

```bash
timeout -s INT 600 ros2 launch kinect_v1_bringup camera.launch.py bench:=true 2>&1 | tee /tmp/endurance.log
grep -cE 'WARN|ERROR' /tmp/endurance.log                    # expected 0
grep -oE '[0-9]+\.[0-9]+ Hz' /tmp/endurance.log | sort | uniq -c | sort -rn   # rate histogram
```

Pass:
- no `WARN` or `ERROR` lines (no `No ... frames` warning, no `Unexpected buffer`, no libfreenect errors);
- every stats line between 29.5 and 30.5 Hz;
- clean shutdown (`process has finished cleanly`).

### Test status

| # | Test | Status |
|---|---|---|
| 1 | Build + lint | ✅ 2026-11-05 |
| 2 | Smoke test | ✅ 2026-11-05 |
| 3 | Parameters and modes | ✅ 2026-11-05 |
| 4 | Zero-copy, both streams + negative control | ✅ 2026-11-05 |
| 5 | Rate and latency A/B | ✅ 2026-11-05 (see docs/benchmark.md) |
| 6 | CPU | ✅ RGB only, scenario A; ⏳ RGB+depth and scenario B |
| 7 | Depth accuracy against a tape-measured distance | ⏳ to do |
| 8 | Visual check | ⏳ to do (RGB looked very dark in a dark room; to recheck in daylight) |
| 9 | Endurance, 10 min | ⏳ to do |

### Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `Kinect #0 not found (0 connected ...)` and `lsusb` shows only `02b0` | The camera and audio are powered by the **12 V adapter**: with USB power alone only the motor enumerates. Reseat the orange Kinect connector and check the adapter. Camera and audio appear in `journalctl -k -f` as soon as 12 V arrives. |
| `freenect_open_device failed` | Another process already owns the camera (another driver instance, a `learn/` example, `freenect-glview`, ...), or the user is not in `plugdev` / the udev rule is missing. |
| Topic listed but `echo` or another tool receives nothing | QoS mismatch: the subscriber is RELIABLE and the publisher is BEST_EFFORT. Use `--qos-reliability best_effort` or a sensor-data QoS. |
| `parameter 'depth_mode' has invalid type ... bool` | Launch files parse arguments as YAML, where `off` means `false`. Fixed in `camera.launch.py` (`value_type=str`). Keep this in mind when you pass `on`, `off`, `yes` or `no` as strings. |
| Zero-copy check reports `FAIL` | Check the `Intra-process communication: enabled` log line, and that consumers are in the **same container**, use `ConstSharedPtr` and compatible QoS (`KEEP_LAST`, `VOLATILE`). |

## License

Apache-2.0, see [LICENSE](LICENSE).
