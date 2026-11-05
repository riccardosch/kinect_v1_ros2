# Camera: RGB, depth and calibration

Component `kinect_v1_driver::KinectCameraNode` (node name `kinect_camera`). It opens **only the camera USB device**, so the [aux component](aux.md) can use the motor and microphones at the same time.

## Streams

| Stream | Format | Notes |
|---|---|---|
| RGB | `rgb8`, 640×480, 30 Hz | libfreenect demosaics the Bayer pattern into the message buffer |
| Depth | `16UC1`, millimetres, 640×480, 30 Hz | 0 = no reading; maximum 10000 |

RGB and depth are two independent USB streams, **not synchronised in hardware**: matching frames arrive 25–60 ms apart. Consumers that need both must use approximate synchronisation (`message_filters` `ApproximateTime`; `depth_image_proc` does this by default).

`header.stamp` is the host time at which the frame callback fires, which is when the frame has completely arrived.

## Depth modes (`depth_mode`)

| Mode | Topics | Geometry | Frame | Holes |
|---|---|---|---|---|
| `mm` (default) | `depth/image_raw`, `depth/camera_info` | IR camera | `kinect_depth_optical_frame` | ≈ 9 % |
| `registered` | `depth_registered/image_raw`, `depth_registered/camera_info` | re-projected onto the RGB camera, so pixel (u, v) of the depth image is pixel (u, v) of the RGB image | `kinect_rgb_optical_frame` | ≈ 21 % (re-projection shadows, borders) |
| `off` | none | | | |

The two modes are mutually exclusive: they are two output formats of the same libfreenect stream (`depth_process`, `cameras.c`). The `depth_registered/` name follows the openni/freenect convention used in ROS, and it is what `depth_image_proc::PointCloudXyzrgbNode` expects.

Measured on our unit (same scene): `mm` 2478 mm against `registered` 2443 mm at the image centre. Details are in [benchmark.md](benchmark.md).

## camera_info and calibration

Every image is published together with a `camera_info` that has the **same stamp and `frame_id`**, on the sibling topic (`rgb/camera_info`, `depth/camera_info`, `depth_registered/camera_info`). This is what `image_transport::CameraSubscriber`, `depth_image_proc` and RViz `DepthCloud` expect.

### Where the calibration comes from

At startup each camera loads its calibration from a URL, using the same convention as `camera_info_manager`:

| Parameter | Default |
|---|---|
| `rgb_camera_info_url` | `file://${ROS_HOME}/camera_info/rgb_<serial>.yaml` |
| `depth_camera_info_url` | `file://${ROS_HOME}/camera_info/depth_<serial>.yaml` |

- `${ROS_HOME}` is the `ROS_HOME` environment variable, or `~/.ros` if it is not set.
- `<serial>` is the camera serial number, so each physical Kinect keeps its own calibration.
- `file:///absolute/path.yaml` and `package://<package>/<path>.yaml` URLs are accepted.

If the file does not exist, a default model is used:

| Camera | Default intrinsics | Source |
|---|---|---|
| RGB | fx = fy = 525, cx = 319.5, cy = 239.5, no distortion | nominal Kinect v1 value (as in the ROS 1 openni/freenect drivers). **Calibrate for real accuracy.** |
| Depth (IR) | fx = fy = `reference_distance / (2 · reference_pixel_size)`, cx = 320, cy = 240, no distortion | **factory data read from the device** at startup: 120.00 mm / 0.1042 mm → 575.8 px on our unit. It is the same model libfreenect uses in `freenect_camera_to_world`. |
| Depth, `registered` | same as RGB | the registered depth lives in the RGB camera |

The startup log says which source is in use:

```
Kinect A00362A02228049A: factory depth focal 575.8 px (...); rgb nominal, depth factory default
```

### Calibrating the RGB camera

The driver provides `rgb/set_camera_info` and `depth/set_camera_info` (`sensor_msgs/srv/SetCameraInfo`). A successful call stores the calibration at the camera's URL and applies it immediately, without a restart. The standard ROS calibration tool works directly:

```bash
sudo apt install ros-jazzy-camera-calibration
ros2 launch kinect_v1_bringup kinect.launch.py aux:=false
# 8x6 inner corners, 24 mm squares: adapt to your checkerboard
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.024 \
  --ros-args -r image:=/kinect/rgb/image_raw -r camera/set_camera_info:=/kinect/rgb/set_camera_info
```

Move the board until **CALIBRATE** turns on, then press **COMMIT**: the file `~/.ros/camera_info/rgb_<serial>.yaml` is written. `camera_calibration` matches the publisher's QoS, so it receives the best-effort images.

Not verified yet: a full calibration session with a real checkerboard. The service path itself is tested in [testing.md](testing.md) (test 13).

The depth camera cannot be calibrated this way: a checkerboard is not visible in a depth image, and the driver does not publish the IR image yet. Its factory model is usually good enough.

### Rectified images

`image_raw` is not rectified. With the default models the distortion is zero, so `image_raw` can be used wherever `image_rect` is expected; the launch files do this for the point cloud. After a calibration with real distortion coefficients, rectify with `image_proc` first.

## Parameters

| Name | Default | Description |
|---|---|---|
| `device_index` | `0` | Index of the Kinect camera on the bus |
| `enable_rgb` | `true` | Stream RGB |
| `depth_mode` | `mm` | `mm`, `registered` or `off` (see above) |
| `rgb_frame_id` | `kinect_rgb_optical_frame` | `frame_id` of RGB (and of registered depth) |
| `depth_frame_id` | `kinect_depth_optical_frame` | `frame_id` of depth in `mm` mode |
| `rgb_camera_info_url` | `""` (= default above) | RGB calibration URL |
| `depth_camera_info_url` | `""` (= default above) | Depth calibration URL |
| `qos_depth` | `1` | History depth of image and camera_info topics |
| `stats_period_s` | `5.0` | Period of the rate / alloc / publish stats log (0 = off) |
| `log_buffer_address` | `false` | Log the buffer address of every frame (zero-copy check) |

## QoS

| Topics | QoS | Why |
|---|---|---|
| `*/image_raw` | best effort, keep last `qos_depth` (1), volatile | sensor data: a late frame is useless, retransmissions only add lag. Keep last + volatile is also required by intra-process. |
| `*/camera_info` | **reliable**, keep last `qos_depth`, volatile | small message; a reliable publisher matches both reliable and best-effort subscribers. `PointCloudXyzrgbNode` subscribes `camera_info` with a fixed RELIABLE QoS. |
