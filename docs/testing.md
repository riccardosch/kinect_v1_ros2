# Testing

The tests below need the Kinect connected. Except for step 1, run every command in a terminal set up with:

```bash
cd ~/kinect && source install/setup.bash
B_RUN=install/kinect_v1_bench/lib/kinect_v1_bench   # shorthand for the bench scripts
```

Only one process at a time can own the camera. Stop the driver (Ctrl-C) before starting another one.

## 0. Hardware check

```bash
lsusb | grep 045e
```

Expected: **three** devices, `02b0` (Motor), `02ad` (Audio) and `02ae` (Camera). If only `02b0` is listed, the 12 V adapter is not powering the sensor (see [troubleshooting.md](troubleshooting.md)).

## 1. Build and static checks (no hardware needed)

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test && colcon test-result --verbose
```

Pass: the build has no warnings, and `colcon test-result` reports `0 errors, 0 failures` (cpplint, uncrustify, copyright, flake8, xmllint, ...).

## 2. Smoke test

```bash
ros2 launch kinect_v1_bringup camera.launch.py
```

Expected log, within about 1 s:

```
[kinect.kinect_camera]: Intra-process communication: enabled
[kinect.kinect_camera]: Kinect A00362A02228049A: factory depth focal 575.8 px (zero plane 120.00 mm / 0.1042 mm); rgb nominal, depth factory default
[kinect.kinect_camera]: Kinect #0 streaming: rgb 640x480@30, depth mm -> depth/ (frame 'kinect_depth_optical_frame'), libfreenect 0.7.5
```

Before calibration you will also see two `No calibration at ~/.ros/camera_info/...` warnings. They are expected (see test 13).

Then, every 5 s, one line per stream:

```
rgb   30.00 Hz | alloc+zero mean ... us | publish mean ... us | subs 0 (intra 0)
depth 30.00 Hz | ...
```

In a second terminal:

```bash
ros2 topic list | grep kinect                    # rgb/ and depth/ image_raw + camera_info
ros2 topic info -v /kinect/rgb/image_raw         # Reliability: BEST_EFFORT, History: KEEP_LAST (1), Durability: VOLATILE
ros2 topic echo /kinect/depth/image_raw --field header --qos-reliability best_effort
ros2 param dump /kinect/kinect_camera
```

Pass:
- both streams run at 29.5–30.5 Hz;
- `header.frame_id` is `kinect_depth_optical_frame` for depth in `mm` mode;
- `header.stamp` increases by about 33 ms between frames.

> `ros2 topic echo` needs `--qos-reliability best_effort`. A reliable subscriber receives **nothing** from a best-effort publisher, and no error is shown.

## 3. Parameters and modes

| Command | Expected |
|---|---|
| `ros2 launch kinect_v1_bringup camera.launch.py depth_mode:=registered` | depth on `depth_registered/image_raw` + `depth_registered/camera_info`, `frame_id` = `kinect_rgb_optical_frame`; no `depth/` topics |
| `ros2 launch kinect_v1_bringup camera.launch.py depth_mode:=off` | only `rgb` stats lines; no depth topic |
| `ros2 launch kinect_v1_bringup camera.launch.py enable_rgb:=false` | only `depth` stats lines; no RGB topic |
| `ros2 launch kinect_v1_bringup camera.launch.py depth_mode:=foo` | load fails: `depth_mode must be 'mm', 'registered' or 'off'` |
| `ros2 launch kinect_v1_bringup camera.launch.py enable_rgb:=false depth_mode:=off` | load fails: `Both RGB and depth are disabled` |

## 4. Zero-copy (benchmark scenario A)

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

## 5. Rate and latency

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

Pass, with reference values measured on the Raspberry Pi 5 (see [benchmark.md](benchmark.md)):

| | Scenario A | Scenario B |
|---|---|---|
| Rate | 29.5–30.5 Hz | 29.5–30.5 Hz |
| `intra-process` | N/N (all) | 0/N |
| Lag p95 | < 1 ms (measured ≈ 0.3–0.55 ms) | < 3 ms (measured ≈ 1.6–1.8 ms) |
| Driver `publish mean` | < 50 µs | hundreds of µs (serialization) |

## 6. CPU usage

While the driver runs (any scenario):

```bash
$B_RUN/cpu_usage.sh component_container 30 | tee /tmp/cpu.txt
```

The script prints `pidstat` output once per second for the process and each of its threads. The process line (TID `-`) gives the total. Reference: about 8 % of one core for RGB in scenario A, almost all of it in the libfreenect event thread (Bayer → RGB conversion).

## 7. Depth content and accuracy

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

## 8. Visual check

With a display available (needs `sudo apt install ros-jazzy-rqt-image-view`):

```bash
ros2 run rqt_image_view rqt_image_view
```

Select `/kinect/rgb/image_raw` and `/kinect/depth/image_raw`.

Pass:
- the RGB image has correct colours (red objects look red, so R and B are not swapped);
- the depth image shows the scene shapes, with black holes at the object edges.

## 9. Endurance

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

## 10. LED

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

## 11. IMU

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

## 12. Microphones

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

## 13. camera_info and calibration

To keep your real calibration files out of the way, run this test with a temporary `ROS_HOME`, in **both** terminals:

```bash
export ROS_HOME=/tmp/kinect_ros_home
ros2 launch kinect_v1_bringup kinect.launch.py aux:=false
# second terminal (same ROS_HOME):
ros2 run kinect_v1_bench check_camera_info.py /kinect/rgb 4
ros2 run kinect_v1_bench check_camera_info.py /kinect/depth 4
ros2 service list | grep set_camera_info        # /kinect/rgb/... and /kinect/depth/...
```

Expected, before any calibration:

```
images 118, with matching camera_info 118
frame_id 'kinect_rgb_optical_frame'  640x480  model plumb_bob  D [0.0, 0.0, 0.0, 0.0, 0.0]
fx 525.0  fy 525.0  cx 319.5  cy 239.5
PASS
images 118, with matching camera_info 118
frame_id 'kinect_depth_optical_frame'  640x480  model plumb_bob  D [0.0, 0.0, 0.0, 0.0, 0.0]
fx 575.8  fy 575.8  cx 320.0  cy 240.0
PASS
```

Then store a calibration through the service, as `camera_calibration` does:

```bash
ros2 service call /kinect/rgb/set_camera_info sensor_msgs/srv/SetCameraInfo "{camera_info: {width: 640, height: 480, distortion_model: plumb_bob, d: [0.05, 0.0, 0.0, 0.0, 0.0], k: [530.0, 0.0, 320.0, 0.0, 530.0, 240.0, 0.0, 0.0, 1.0], r: [1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0], p: [530.0,0.0,320.0,0.0,0.0,530.0,240.0,0.0,0.0,0.0,1.0,0.0]}}"
ros2 service call /kinect/rgb/set_camera_info sensor_msgs/srv/SetCameraInfo "{camera_info: {width: 320, height: 240}}"
ros2 run kinect_v1_bench check_camera_info.py /kinect/rgb 2
```

Restart the launch afterwards.

Pass:
1. Every image has a `camera_info` with the same stamp and `frame_id` (`PASS`), on both cameras.
2. The defaults are as above. The depth focal is computed from the factory zero-plane data of **your** device (≈ 575.8 px on our unit).
3. The first call answers `success=True, 'Saved to /tmp/kinect_ros_home/camera_info/rgb_<serial>.yaml'`, and `check_camera_info.py` shows `fx 530.0` and `D [0.05, ...]` right away, without a restart.
4. The second call (wrong size) answers `success=False, 'Image size does not match the stream'`.
5. After the restart the log shows `Loaded calibration 'rgb_<serial>' from ...` and `rgb calibrated`.
6. With `depth_mode:=registered`, only `/kinect/rgb/set_camera_info` exists, and `depth_registered/camera_info` equals `rgb/camera_info` (same `frame_id` and K).

## Test status

| # | Test | Status |
|---|---|---|
| 1 | Build + lint | ✅ 2026-11-05 |
| 2 | Smoke test | ✅ 2026-11-05 |
| 3 | Parameters and modes | ✅ 2026-11-05 |
| 4 | Zero-copy, both streams + negative control | ✅ 2026-11-05 |
| 5 | Rate and latency A/B | ✅ 2026-11-05 (see [benchmark.md](benchmark.md)) |
| 6 | CPU | ✅ RGB only, scenario A; ⏳ RGB+depth and scenario B |
| 7 | Depth accuracy against a tape-measured distance | ⏳ to do |
| 8 | Visual check | ⏳ to do (RGB looked very dark in a dark room; to recheck in daylight) |
| 9 | Endurance, 10 min | ⏳ to do |
| 10 | LED commands and invalid input | ✅ 2026-11-05 (log); ⏳ visual confirmation of the colours |
| 11 | IMU rate, \|a\|, pitch | ✅ 2026-11-05: 29.99 Hz, \|a\| 9.64, pitch 8.6° against 8.5° from the motor; ⏳ roll sign (y) |
| 12 | Microphones | ✅ 2026-11-05: 15999 Hz effective, 0 gaps over 8 s, received in another process; ⏳ clap test (channel order), 5-minute run, listening to the WAV |
| 13 | camera_info and calibration | ✅ 2026-11-06: stamps matched on all 117–118 images (mm and registered); set_camera_info saves, applies live, rejects the wrong size, reloads after restart |
