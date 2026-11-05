# Aux: LED, IMU and microphones

Component `kinect_v1_driver::KinectAuxNode` (node name `kinect_aux`). It opens the **motor and audio USB devices** (never the camera) with its own libfreenect context, so it runs next to the [camera component](camera.md). The tilt motor is **never commanded**.

## Threading

libfreenect has no locking: `freenect_update_tilt_state` writes `dev->raw_state` field by field (`tilt.c`). One worker thread therefore owns the device and does everything on it:

1. applies the pending LED command, if any;
2. reads the accelerometer and publishes `imu`, at `imu_rate_hz`;
3. in the time until the next IMU sample, pumps libusb events (`freenect_process_events_timeout`). The audio callbacks run in there.

ROS callbacks (for example `led/cmd`) only leave a request under a mutex. The mutex protects the hand-off (nanoseconds), never a USB transfer (milliseconds).

## LED

| Topic | Type | QoS |
|---|---|---|
| `led/cmd` (subscribed) | `std_msgs/String`: `off`, `green`, `red`, `yellow`, `blink_green`, `blink_red_yellow` | reliable, keep last 10 |

- Unknown values are rejected with a warning.
- The LED is set to `initial_led` at startup and to `blink_green` on shutdown.
- On the 1414 the LED is a control transfer on the motor device (`freenect_set_led`, `tilt.c`).

## IMU

| Topic | Type | QoS |
|---|---|---|
| `imu` | `sensor_msgs/Imu`, frame `kinect_imu_frame` | best effort, keep last `qos_depth`, volatile |

- Only `linear_acceleration` is valid. The Kinect has no gyroscope and no orientation estimate, so `orientation_covariance[0]` and `angular_velocity_covariance[0]` are −1 (REP 145).
- The value is the **specific force**: at rest it reads about +9.8 m/s² pointing up.
- The axes are mapped to REP 103 (x forward, y left, z up). This was measured at rest: with the sensor pitched 8.5° up, the reading is x ≈ +1.4, z ≈ +9.5 m/s², and the pitch computed from it (8.5°) matches the angle the motor reports. **The sign of y (lateral) is not verified yet.**
- `linear_acceleration_covariance` defaults to 0.0065 (m/s²)² per axis, the variance measured at rest (standard deviation ≈ 0.08 m/s²; 1 count = 0.012 m/s²).

## Microphones

| Topic | Type | QoS |
|---|---|---|
| `audio/data` | `audio_common_msgs/AudioDataStamped`: 256 frames per message, 4 channels interleaved, `S32LE`, 16 kHz (≈ 62.5 msg/s) | reliable, keep last `audio_qos_depth` (100 ≈ 1.6 s) |
| `audio/info` | `audio_common_msgs/AudioInfo`: `channels 4`, `sample_rate 16000`, `sample_format S32LE`, `coding_format wave` | reliable, transient local (latched) |
| `audio/cancelled` | `audio_common_msgs/AudioDataStamped`: mono `S16LE`, 16 kHz; only with `publish_cancelled:=true` | reliable, keep last `audio_qos_depth` |

- **Firmware.** After power-up the audio device is in bootloader mode (one USB interface). The first time it is opened, libfreenect uploads `audios.bin` and waits for the device to re-enumerate (`usb_libusb10.c`), so the component takes about 2 s to load.
- **Channel order** is libfreenect's: mic1 leftmost … mic4 rightmost. Whether "left" is seen from the front or from behind is not verified yet.
- **Timestamps.** `header.stamp` is the time of the **first sample** of the chunk, from a synthetic clock that advances by exactly 256 / 16000 s per chunk. Arrival times cannot be used: one USB transfer carries two chunks, so they arrive in pairs (≈ 0 ms apart, then ≈ 32 ms). The clock is anchored to the arrival time and re-anchored if the two disagree by more than 50 ms; the stats log counts these as `clock resyncs`.
- **Not zero-copy.** libfreenect has no user-buffer API for audio and delivers each channel in its own array, so the driver copies once while interleaving (4 KB per chunk, ≈ 256 KB/s).
- **QoS** is reliable with a deep queue, unlike the other sensor topics: a lost chunk is an audible gap, and the stream is small.
- `audio/cancelled` (the Kinect's noise-cancelled channel) is off by default. In our tests it was almost entirely zeros (≤ 1.3 % non-zero samples).

Measured: 15999 Hz effective rate, 0 gaps over 8 s, received in another process. See [testing.md](testing.md), test 12.

## Parameters

| Name | Default | Description |
|---|---|---|
| `device_index` | `0` | Index of the Kinect on the bus |
| `initial_led` | `green` | LED state at startup |
| `imu_rate_hz` | `30.0` | Accelerometer sampling / publish rate, in (0, 200] |
| `imu_frame_id` | `kinect_imu_frame` | `frame_id` of `imu` |
| `linear_acceleration_variance` | `0.0065` | Diagonal of `linear_acceleration_covariance`, (m/s²)² |
| `qos_depth` | `1` | History depth of `imu` |
| `enable_audio` | `true` | Open the audio device and publish `audio/*` |
| `publish_cancelled` | `false` | Publish `audio/cancelled` |
| `audio_frame_id` | `kinect_mic_array_frame` | `frame_id` of audio messages |
| `audio_qos_depth` | `100` | History depth of `audio/*` |
| `stats_period_s` | `10.0` | Period of the IMU / audio rate log (0 = off) |
