# Remote viewing (RViz on another machine)

Goal: watch the Kinect live from a laptop over Wi-Fi, with as little lag as possible and without loading the robot.

## What goes over the network

| Sent | Bandwidth (640×480 @ 30 Hz) |
|---|---|
| RGB raw | 221 Mbit/s: too much for Wi-Fi ❌ |
| Depth raw | 147 Mbit/s ❌ |
| Point cloud (XYZ / XYZRGB) | 1.2–2 Gbit/s ❌❌ |
| **RGB JPEG** (`rgb/image_raw/compressed`, quality 80) | **≈ 1.6 Mbit/s** in a dark room; more in daylight |
| **Depth RVL** (`…/image_raw/compressedDepth`, default) | **≈ 42 Mbit/s** |
| **Depth PNG level 1** (`depth_format:=png`) | **≈ 30 Mbit/s** |

So the robot sends **compressed images plus `camera_info`**, and RViz on the laptop rebuilds the 3D view (the `DepthCloud` display). Point clouds never travel over the network.

The compression runs in `kinect_v1_driver::CompressedRelay`, inside the driver's container:
- it receives the raw images intra-process (zero-copy);
- it republishes them **on the same base topic** with only the compressed plugin, so there is no duplicate raw topic;
- it compresses **only while someone subscribes** to the compressed topics;
- RGB and depth compress in parallel on executor threads, never on the libfreenect thread.

Output QoS: best effort, keep last 1. A frame lost on the Wi-Fi is skipped and never delays the next one.

### RVL or PNG for the depth?

Measured on the Pi 5 with a consumer in another process on the robot (loopback, no Wi-Fi):

| `depth_format` | Size | Bandwidth | Compression time | Container CPU with a viewer | Lag (loopback) |
|---|---|---|---|---|---|
| `rvl` (default) | 175 KB/frame | 42 Mbit/s | **4.6 ms** (13 % of a core) | 47 % | **5.7 ms** |
| `png` (`png_level:=1`) | **125 KB/frame** | **30 Mbit/s** | 17.5 ms (49 % of a core) | 86 % | 18.3 ms |

RVL is 4× faster but produces 40 % more data on this scene. Keep **RVL on a good link**; switch to **PNG when the Wi-Fi is the bottleneck**. The RGB JPEG costs 2.9 ms per frame.

## 1. Robot (Raspberry Pi)

```bash
ros2 launch kinect_v1_bringup kinect.launch.py depth_mode:=registered remote_view:=true
# options: jpeg_quality:=80  depth_format:=rvl|png  png_level:=1
```

`depth_mode:=registered` puts the depth in the RGB camera's pixel grid, so RViz can colour the cloud without extra calibration. The launch also publishes the static TF `kinect_link → optical frames` (`publish_tf:=true`, default).

## 2. Laptop

You need **ROS 2 Jazzy**, the same as the robot (no Humble ↔ Jazzy mixing), on the same network and with the same `ROS_DOMAIN_ID`.

**Option A: native Ubuntu 24.04**

```bash
sudo apt install ros-jazzy-desktop ros-jazzy-image-transport-plugins ros-jazzy-rmw-zenoh-cpp
git clone https://github.com/riccardosch/kinect_v1_ros2.git ~/kinect && cd ~/kinect
source /opt/ros/jazzy/setup.bash
colcon build --packages-ignore kinect_v1_driver
```

The laptop does not need libfreenect: `--packages-ignore kinect_v1_driver` builds only the bringup (RViz config) and the bench scripts. `--packages-select` alone would fail, because it expects the driver (a run dependency of the bringup) to be installed. If conda is installed, run `conda deactivate` first, because its Python breaks `colcon` and `rclpy`. `image-transport-plugins` is needed **on the laptop too**, to decompress.

**Option B: Docker** (any Linux with X11)

```bash
xhost +local:
docker run -it --rm --net=host -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix osrf/ros:jazzy-desktop
# then inside: apt update && apt install -y ros-jazzy-image-transport-plugins, and clone/build as above
```

**View:**

```bash
source ~/kinect/install/setup.bash
ros2 launch kinect_v1_bringup view.launch.py
```

The config (`kinect_v1_bringup/rviz/remote.rviz`) shows:
- **RGB (JPEG)**: `Image` display on `/kinect/rgb/image_raw/compressed`;
- **DepthCloud (registered)**: depth `/kinect/depth_registered/image_raw` with transport `compressedDepth`, colour `/kinect/rgb/image_raw` with transport `compressed`, best effort, queue 1;
- Grid and TF, with Fixed Frame `kinect_link`.

> rviz detail (v14.1.24, `depth_cloud_display.cpp`): the drop-down of *Color Transport Hint* lists the transports of the **depth** topic. The value `compressed` from the config file works, but you may not see it in the list.

**Not verified yet:** the robot is headless, so this RViz config has not been opened yet. It is valid YAML, and the property names were taken from the rviz 14.1.24 sources.

## 3. Measure (benchmark scenarios C and D)

On the laptop, while RViz is open (or instead of RViz):

```bash
ros2 run kinect_v1_bench remote_stats.py 20
```

For each topic it prints the rate, KB/frame, Mbit/s and lag (`now − header.stamp`). The lag across machines is only meaningful if the clocks are synchronised: install `chrony` on both and check `chronyc tracking`.

On the robot, the relay logs its own work every 10 s:

```
rgb              29.00 Hz | compress+publish mean 2.9 ms (9 % of one core)
depth_registered 29.10 Hz | compress+publish mean 4.6 ms (13 % of one core)
```

**Scenario D, Fast DDS against Zenoh.** Same commands, with the Zenoh RMW (no code change: the driver does not depend on the RMW):

```bash
# robot, one terminal: the Zenoh router
export RMW_IMPLEMENTATION=rmw_zenoh_cpp && ros2 run rmw_zenoh_cpp rmw_zenohd
# robot, the driver
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 launch kinect_v1_bringup kinect.launch.py depth_mode:=registered remote_view:=true
# laptop: connect to the robot's router (replace the IP)
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/192.168.1.50:7447"]'
ros2 launch kinect_v1_bringup view.launch.py
```

`ZENOH_CONFIG_OVERRIDE` is supported by the installed `rmw_zenoh_cpp` 0.2.11. The exact override syntax is still to be confirmed during the test.

## Tips for a smooth view

- **Wi-Fi signal first.** On our robot the link measured −68 dBm (quality 42/70): a weak link. Moving closer to the access point, or using 5 GHz, helps more than any parameter.
- If the view stutters, in order:
  1. `depth_format:=png` (−30 % bandwidth);
  2. lower `jpeg_quality`;
  3. view only RGB (disable the DepthCloud display: the depth is then not even compressed).
- Never subscribe to `*/points` or `*/image_raw` from the laptop: raw data does not fit on Wi-Fi.
