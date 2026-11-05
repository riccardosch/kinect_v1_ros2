# Zero-copy image pipeline

Goal: inside the robot, images go from the USB driver to every consumer in the same container **without being copied**.

## How it works

1. **libfreenect writes into the ROS message.** Before streaming starts, the driver registers `msg->data.data()` of a `sensor_msgs/Image` as the libfreenect user buffer (`freenect_set_video_buffer` / `freenect_set_depth_buffer`).
   - For RGB, `MM` and `REGISTERED` depth, the USB packets go into an internal raw buffer (`split_bufs = 1` in `stream_init`, `cameras.c`).
   - The user buffer is written **once per frame**, by the conversion that runs just before the frame callback (Bayer → RGB, disparity → mm).
2. **Swap in the callback.** The frame callback runs on the driver's libfreenect event thread, the only thread allowed to change the buffer while streaming (libfreenect has no locking). It:
   1. takes ownership of the filled message;
   2. registers a newly allocated one for the next frame (`stream_setbuf` only changes `proc_buf`);
   3. publishes the filled one with `publish(std::unique_ptr)`.
3. **Intra-process delivery.** With `use_intra_process_comms(true)` (set by the launch files on every component) and `ConstSharedPtr` subscribers, rclcpp promotes the `unique_ptr` to a `shared_ptr` and hands **the same buffer** to every consumer (`intra_process_manager.hpp`). This requires `KEEP_LAST` + `VOLATILE` QoS.

On shutdown the order is: stop the streams → `set_*_buffer(NULL)` → close the device → free the messages.

## What it does not cover

- The conversion inside libfreenect (Bayer → RGB, disparity → mm) is still one pass over the frame. "Zero-copy" means no copy **after** it.
- Consumers in **another process** always get a serialized copy (DDS).
- A subscriber that asks for `UniquePtr` while others share the message forces rclcpp to copy for it.
- **Audio** cannot be zero-copy: libfreenect has no user-buffer API for it (see [aux.md](aux.md)).
- Each new message is allocated and zero-filled (`vector::resize`), at a cost of 80–190 µs per frame on the Pi 5. The driver reports it as `alloc+zero` in its stats log. A message pool would remove it; to be decided with the numbers ([benchmark.md](benchmark.md)).

## How it is verified

**Equal addresses are not enough.** With intra-process disabled, malloc often hands the publisher's freed buffer to the deserialized message, so the addresses can match by coincidence.

The bench therefore checks two things for **every** frame:
1. the address the consumer receives equals the one the driver registered with libfreenect;
2. `rclcpp::MessageInfo::from_intra_process` is true.

`check_zero_copy.py` matches the two logs frame by frame.

| Run | Result |
|---|---|
| RGB + depth, intra-process on | 257/257 and 258/258 frames zero-copy → `PASS` |
| Same, `intra_process:=false` (negative control) | 0 frames → `FAIL`, as expected |

Procedure: [testing.md](testing.md), test 4. Latency and CPU numbers: [benchmark.md](benchmark.md).
