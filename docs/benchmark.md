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
- The centre spread reached 0 in some frames of both modes. Not investigated yet. Hypothesis: a few frames right after `start_depth`.
- **Not verified yet:** the ±5 % accuracy check against a tape-measured distance, and the CPU cost of `registered` compared with `mm`.

### Open points

- **No memcpy baseline yet.** The driver went straight to zero-copy. Design point 4 (does the cost of allocating and zeroing cancel the gain?) needs the same measurement on a memcpy version. Hypothesis, not verified: zero-filling 900 KB costs about as much as copying them, so a message pool (no allocation, no zeroing) would be the real win.
- Scenario B CPU and long runs (10 min without corrupted frames) are still to do.
