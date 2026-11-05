# kinect_v1_ros2

ROS 2 Jazzy driver for the Microsoft Kinect v1 (Xbox 360, model 1414), written from scratch on top of [libfreenect](https://github.com/OpenKinect/libfreenect).

> **Status: work in progress.** RGB, depth (mm / registered), LED, IMU (accelerometer) and the 4-microphone array are implemented. Point cloud and `camera_info` are coming. The tilt motor is intentionally not driven.

## Quick start: everything in one command

```bash
cd ~/kinect && source install/setup.bash
ros2 launch kinect_v1_bringup kinect.launch.py
```

This starts one multi-threaded container (intra-process) with:
- `kinect_camera`: `/kinect/rgb/image_raw` and `/kinect/depth/image_raw`;
- `kinect_aux`: `/kinect/imu`, `/kinect/led/cmd`, `/kinect/audio/data` and `/kinect/audio/info`.

Launch arguments:

| Argument | Default | Description |
|---|---|---|
| `camera` | `true` | Load the camera component |
| `aux` | `true` | Load the aux component |
| `enable_rgb` | `true` | Stream RGB |
| `depth_mode` | `mm` | `mm`, `registered` or `off` |
| `imu_rate_hz` | `30.0` | IMU publish rate |
| `initial_led` | `green` | LED state at startup |
| `enable_audio` | `true` | Publish the microphone array |
| `publish_cancelled` | `false` | Also publish the Kinect's noise-cancelled channel |
| `qos_depth` | `1` | History depth of sensor topics |

```bash
ros2 topic pub --once /kinect/led/cmd std_msgs/msg/String "{data: red}"
ros2 topic echo /kinect/imu --qos-reliability best_effort --field linear_acceleration
ros2 run kinect_v1_bench audio_stats.py 5 /tmp/mic.wav     # record 5 s of the 4 microphones
```

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
| `kinect_v1_driver` | `kinect_v1_driver/` | `kinect_v1_driver::KinectCameraNode` (RGB + depth) and `kinect_v1_driver::KinectAuxNode` (LED + IMU) components, plus `kinect_camera_node` / `kinect_aux_node` executables |
| `kinect_v1_bench` | `tools/bench/` | `kinect_v1_bench::ImageStatsNode`: rate, latency (now − stamp), buffer address, intra-process delivery. Scripts: `check_zero_copy.py`, `depth_stats.py`, `imu_stats.py`, `audio_stats.py`, `cpu_usage.sh` |
| `kinect_v1_bringup` | `kinect_v1_bringup/` | `kinect.launch.py` (everything) and `camera.launch.py` (camera only, with bench options): `component_container_mt` with intra-process comms |

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
| `imu` | `sensor_msgs/Imu` (linear acceleration only) | best effort, keep last `qos_depth`, volatile |
| `led/cmd` (sub) | `std_msgs/String`: `off`, `green`, `red`, `yellow`, `blink_green`, `blink_red_yellow` | reliable, keep last 10 |
| `audio/data` | `audio_common_msgs/AudioDataStamped`: 256 frames per message, 4 channels interleaved, `S32LE`, 16 kHz (about 62.5 msg/s) | reliable, keep last `audio_qos_depth` |
| `audio/info` | `audio_common_msgs/AudioInfo`: `channels 4`, `sample_rate 16000`, `sample_format S32LE`, `coding_format wave` | reliable, transient local (latched) |
| `audio/cancelled` | `audio_common_msgs/AudioDataStamped`: mono `S16LE`, 16 kHz; only with `publish_cancelled:=true` | reliable, keep last `audio_qos_depth` |

`header.stamp` is taken when the frame callback fires (host clock). RGB and depth are **not** hardware-synchronised.

About `imu`:
- Only `linear_acceleration` is valid. The Kinect has no gyroscope and no orientation estimate, so `orientation_covariance[0]` and `angular_velocity_covariance[0]` are −1 (REP 145).
- The axes are mapped to REP 103 in `imu_frame_id`: x forward, y left, z up. They were measured at rest: with the sensor pitched about 8.5° up, the reading is x ≈ +1.4, z ≈ +9.5 m/s². **The sign of y (lateral) is not verified yet.**
- The value is the specific force, so at rest it reads about +9.8 m/s² along up.

About the audio topics:
- Channels are in libfreenect order: mic1 leftmost … mic4 rightmost. Which side "left" is, seen from the front or from behind, is not verified yet.
- `header.stamp` is the time of the **first sample** of the chunk, taken from a synthetic clock that advances by exactly 256/16000 s per chunk. Arrival times are unusable: one USB transfer carries two chunks, so they arrive in pairs. The clock is anchored to the arrival time and re-anchored if the two disagree by more than 50 ms; the stats log counts these as `clock resyncs`.
- This is the only stream that is **not zero-copy**. libfreenect has no user-buffer API for audio and delivers each channel in its own array, so the driver copies once to interleave (4 KB per chunk).
- QoS is reliable with a deep queue, unlike the other sensor topics. A lost chunk is an audible gap, and the stream is small (256 KB/s).
- `audio/cancelled` is off by default. In our tests the Kinect's noise-cancelled channel was almost entirely zeros (≤ 1.3 % non-zero samples).

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

### Parameters (`kinect_aux`)

| Name | Default | Description |
|---|---|---|
| `device_index` | `0` | Index of the Kinect on the bus |
| `imu_rate_hz` | `30.0` | Accelerometer sampling / publish rate, (0, 200] |
| `imu_frame_id` | `kinect_imu_frame` | `header.frame_id` of `imu` |
| `linear_acceleration_variance` | `0.0065` | Diagonal of `linear_acceleration_covariance` (m/s²)², measured at rest |
| `initial_led` | `green` | LED state at startup. On shutdown the LED is set to `blink_green`. |
| `qos_depth` | `1` | History depth of `imu` |
| `enable_audio` | `true` | Open the audio device and publish `audio/*` |
| `publish_cancelled` | `false` | Publish `audio/cancelled` |
| `audio_frame_id` | `kinect_mic_array_frame` | `header.frame_id` of audio messages |
| `audio_qos_depth` | `100` | History depth of `audio/data` (100 chunks ≈ 1.6 s) |
| `stats_period_s` | `10.0` | Period of the IMU/audio rate log (0 = off) |

The aux component opens the **motor and audio USB devices** (never the camera), with its own libfreenect context, so it runs next to the camera without conflicts.

One worker thread owns the device and does everything on it:
- it applies LED commands;
- it reads the accelerometer at `imu_rate_hz`;
- in the time between, it pumps libusb events, which is where the audio callbacks run.

ROS callbacks only queue requests for it, because libfreenect has no locking.

Opening the audio device the first time after power-up uploads the firmware (`audios.bin`) and waits for the device to re-enumerate: the component then takes about 2 s to load.

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

### 10. LED

```bash
ros2 launch kinect_v1_bringup kinect.launch.py camera:=false
# second terminal:
for c in red yellow blink_red_yellow blink_green off green; do
  ros2 topic pub --once /kinect/led/cmd std_msgs/msg/String "{data: $c}"; sleep 2; done
ros2 topic pub --once /kinect/led/cmd std_msgs/msg/String "{data: purple}"
```

Pass:
- the LED visibly follows the sequence, and the log prints `LED set to <n>` for each command;
- `purple` produces the warning `led/cmd: unknown state 'purple'`;
- on Ctrl-C the LED starts blinking green.

### 11. IMU

```bash
ros2 launch kinect_v1_bringup kinect.launch.py camera:=false
# second terminal (ros2 topic hz on Jazzy subscribes RELIABLE and would show nothing):
ros2 run kinect_v1_bench imu_stats.py /kinect/imu 6
ros2 topic echo /kinect/imu --qos-reliability best_effort --field linear_acceleration
```

Expected output, measured with the sensor at rest and pitched about 8.5° up:

```
rate  29.99 Hz over 96 messages
mean  x +1.43  y -0.56  z +9.52 m/s^2   |a| 9.64
std   x 0.084  y 0.081  z 0.073 m/s^2
pitch (nose up +) +8.6 deg   roll (left side down +) -3.4 deg
```

Pass:
1. The rate is `imu_rate_hz` ±2 %.
2. At rest, |a| = 9.81 ± 0.3 m/s².
3. The pitch printed by the script matches the tilt angle the motor reports (`freenect_get_tilt_degs`) within ±2°, and becomes more positive when you raise the front of the sensor.
4. Lower the sensor's **left** side (seen from behind the sensor): the roll becomes **more positive** (REP 103: y = left). Not verified yet.

### 12. Microphones

```bash
ros2 launch kinect_v1_bringup kinect.launch.py
# second terminal: record 8 s; clap your hands a few times, first on one side, then on the other
ros2 run kinect_v1_bench audio_stats.py 8 /tmp/mic.wav
```

Expected output (quiet room, no clapping):

```
format 4 ch S32LE @ 16000 Hz, 256 frames/chunk
chunks 501 (62.6/s)  effective rate 15999 Hz (-0.00 %)
stamp step mean 16.0 ms, max 16.0 ms, gaps > 1.5 chunks: 0
mic1: peak   0.06 %FS ( -64.2 dBFS)  rms  -77.3 dBFS
...
```

The driver also logs, every `stats_period_s`:

```
imu 30.00 Hz | audio 16026 Hz (+0.16 %), 62.6 chunks/s, clock resyncs 0 | cancelled non-zero 0.0 %
```

Pass:
1. The effective rate is 16 kHz ±1 %.
2. `gaps > 1.5 chunks: 0`, and `clock resyncs 0` after the first stats period (one resync while the stream starts is normal).
3. Claps raise the peak of **all four** channels well above the quiet level (tens of dB). A clap on one side gives the highest peak on that side's outer microphone (mic1 or mic4).
4. `/tmp/mic.wav` plays back the recorded sound. Most players play only the first two channels; `sox /tmp/mic.wav -n stat` or Audacity show all four.
5. No drops in 5 minutes: `audio_stats.py 300` reports `gaps 0` and a rate within ±1 %.

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
| 10 | LED commands and invalid input | ✅ 2026-11-05 (log); ⏳ visual confirmation of the colours |
| 11 | IMU rate, \|a\|, pitch | ✅ 2026-11-05: 29.99 Hz, \|a\| 9.64, pitch 8.6° against 8.5° from the motor; ⏳ roll sign (y) |
| 12 | Microphones | ✅ 2026-11-05: 15999 Hz effective, 0 gaps over 8 s, received in another process; ⏳ clap test (channel order), 5-minute run, listening to the WAV |

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
