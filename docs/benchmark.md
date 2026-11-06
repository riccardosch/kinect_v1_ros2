# Benchmark

All numbers are measured, not estimated. Raspberry Pi 5 (4 cores), Ubuntu 24.04, ROS 2 Jazzy, default RMW (Fast DDS), Kinect 1414, RGB 640×480 @ 30 Hz, `qos_depth` 1, Release build.

Tools:
- `kinect_v1_bench/ImageStatsNode` measures latency (`now − header.stamp`), rate and buffer address, and checks `MessageInfo.from_intra_process`.
- The driver's own stats log reports `alloc+zero` and `publish` times. Both are measured inside the libfreenect frame callback.
- `tools/bench/scripts/cpu_usage.sh` wraps `pidstat -t -p <container>` at 1 Hz.

## RGB, zero-copy driver (2026-11-05)

| Scenario | Consumer | Rate | Lag mean / p95 / max | `publish()` in callback, mean | Container CPU (1 core = 100 %) |
|---|---|---|---|---|---|
| idle | none | 30.00 Hz | — | 7 µs | 7.4 % |
| **A** | same container, intra-process | 30.00 Hz | **0.23 / 0.28 / 1.51 ms** | 17 µs | 8.1 % |
| A, `intra_process:=false` | same container, through DDS | 30.00 Hz | 0.81 / 0.91 / 1.03 ms | 268 µs | 10.3 % |
| **B** | separate process on the Pi | 29.4–30.0 Hz | 1.35 / 1.56–1.82 / 2.30 ms | 427–786 µs | not measured |

Measurement windows: 5 s for latency and rate, 12 s for CPU.

Zero-copy check (scenario A, `log_buffer_address:=true`, about 6 s):
- 170/170 frames received with the **same `data()` address the driver registered with libfreenect** and `from_intra_process == true`;
- 13 distinct buffer addresses in total, because malloc recycles freed messages.

`alloc+zero` (new `Image` plus `data.resize(921600)`, which zero-fills) costs **116–162 µs** per frame on average, with occasional spikes up to about 0.95 ms. That is roughly 0.4 % of one core at 30 Hz.

### Observations

- Most of the container CPU in every scenario (about 7.5 % of one core) is one thread, almost certainly the libfreenect event thread. It runs the Bayer → RGB conversion, which is unavoidable. Delivering intra-process adds about 0.7 %; going through DDS adds about 2–3 %.
- Without intra-process, `publish()` (serialization plus DDS write) costs 0.27–0.8 ms **inside the libfreenect event thread**. This is the main argument for keeping consumers in the same container.
- **Equal addresses alone do not prove zero-copy.** With intra-process disabled, malloc often hands the freed publisher buffer to the deserialized message, so the addresses match anyway. The bench therefore also checks `from_intra_process`.

## RGB + depth, zero-copy driver (2026-11-05)

Scenario A, both streams at the same time, with one consumer per stream in the container (5 s windows):

| `depth_mode` | Stream | Rate | Lag mean / p95 / max | alloc+zero mean | publish mean | Zero-copy (same address + intra) |
|---|---|---|---|---|---|---|
| `mm` | rgb | 30.00 Hz | 0.40 / 0.55 / 1.20 ms | 188 µs | 23 µs | 438/438 |
| `mm` | depth | 30.00 Hz | 0.29 / 0.40 / 0.84 ms | 101 µs | 22 µs | 437/437 |
| `registered` | rgb | 29.8–30.0 Hz | 0.25 / 0.29 / 0.31 ms | 133 µs | 15 µs | 440/440 |
| `registered` | depth | 29.8–30.0 Hz | 0.30 / 0.39 / 0.57 ms | 85 µs | 22 µs | 439/439 |

The driver published a few more frames than the consumers received (for example 446 against 438): those are frames from before the consumer subscribed or after shutdown began.

Depth content over 60 frames, same scene:

| `depth_mode` | frame_id | Zero (no reading) pixels | Centre 10×10 median | Max value |
|---|---|---|---|---|
| `mm` | `kinect_depth_optical_frame` | 8.9 % | 2478 mm | 10000 mm |
| `registered` | `kinect_rgb_optical_frame` | 21.3 % | 2443 mm | 4243 mm |

- The extra holes in `registered` come from the re-projection: occlusions, plus RGB-image borders that the depth camera does not see.
- The centre spread reached 0 in some frames of both modes. Confirmed later (`learn/04`): the **first** depth frame after `start_depth` is incomplete (centre 0, 15.5 % holes); from the second frame on it is stable.
- **Not verified yet:** the ±5 % accuracy check against a tape-measured distance.

## Point cloud (2026-11-06)

`kinect.launch.py aux:=false point_cloud:=true`, RGB + depth streaming. `cloud_stats.py` runs in **another process** (so the cloud is serialized through DDS). CPU measured with `pidstat` over 8–10 s.

| `depth_mode` | Component | Cloud size | Rate | Lag mean / p95 / max | Valid points | Container CPU without → with cloud subscriber |
|---|---|---|---|---|---|---|
| `mm` | `PointCloudXyzNode` | 4.9 MB | 29.7 Hz | 14.2 / 19.6 / 31.8 ms | 91.2 % | 13.5 % → 47.7 % |
| `registered` | `PointCloudXyzrgbNode` | 9.8 MB | 25.4 Hz | 122 / 173 / 181 ms | 78.7 % | 18.9 % → 152 % |

- The depth (and, for XYZRGB, the RGB) reaches `depth_image_proc` intra-process: the driver reports `subs 1 (intra 1)` while the cloud is subscribed, and `subs 0` otherwise (lazy subscription).
- The cloud centre agrees with the depth image: 2.496 m against 2496 mm.
- Without a consumer, the loaded point cloud component costs nothing. 13.5 % and 18.9 % are the camera alone; `registered` is higher because of the registration.
- **Hypothesis, not measured:** most of the CPU jump comes from serializing 147–245 MB/s to the external consumer, not from computing the cloud. Checking it needs a consumer inside the container.
- XYZRGB at 25 Hz with 120+ ms of lag across processes confirms the design rule: point clouds are consumed inside the container and never sent over the network.

## Remote viewing: compression (2026-11-06)

`kinect.launch.py aux:=false depth_mode:=registered remote_view:=true`. `remote_stats.py` runs in another process **on the robot** (loopback): this measures compression only, not the Wi-Fi. Dark room.

| `depth_format` | RGB JPEG q80 | Depth | Total | Relay time per frame (RGB / depth) | Container CPU, no viewer → viewer |
|---|---|---|---|---|---|
| `rvl` | 7 KB, 1.6 Mbit/s, lag 4.1 ms | 175 KB, 42.0 Mbit/s, lag 5.7 ms | 43.6 Mbit/s | 2.9 / 4.6 ms | 21.0 % → 47.2 % |
| `png` level 1 | 7 KB, 1.6 Mbit/s, lag 3.8 ms | 125 KB, 29.9 Mbit/s, lag 18.3 ms | 31.5 Mbit/s | 3.1 / 17.5 ms | 20.3 % → 86.0 % |

- On this scene RVL is ~4× faster than PNG but 40 % larger. My earlier expectation that RVL would also be smaller was **wrong**.
- The JPEG size depends heavily on the scene: 7 KB in a dark room. To be repeated in daylight.
- **Still to do:** scenario C (laptop over Wi-Fi, Fast DDS) and scenario D (Zenoh). The robot's Wi-Fi link was −68 dBm (quality 42/70) during these tests.

### Open points

- **No memcpy baseline yet.** The driver went straight to zero-copy. Design point 4 (does the cost of allocating and zeroing cancel the gain?) needs the same measurement on a memcpy version. Hypothesis, not verified: zero-filling 900 KB costs about as much as copying them, so a message pool (no allocation, no zeroing) would be the real win.
- Scenario B CPU and long runs (10 min without corrupted frames) are still to do.
