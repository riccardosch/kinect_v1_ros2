#!/usr/bin/env python3
# Copyright 2026 Riccardo Schiavo
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Summarise a PointCloud2 topic: rate, latency, valid points, centre point.

Subscribing also wakes up depth_image_proc, which only processes depth while
someone listens to its output.

Usage: cloud_stats.py [topic] [seconds]
  default topic /kinect/depth/points, default 6 s
"""

import sys
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


def xyz(msg):
    """Return an (height, width, 3) float array of x, y, z."""
    offsets = {f.name: f.offset for f in msg.fields}
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, msg.point_step)
    out = np.empty((msg.height, msg.width, 3), dtype=np.float32)
    for i, name in enumerate('xyz'):
        o = offsets[name]
        out[..., i] = raw[..., o:o + 4].copy().view('<f4')[..., 0]
    return out


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else '/kinect/depth/points'
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 6.0
    samples = []
    last = {}

    rclpy.init()
    node = rclpy.create_node('cloud_stats')

    def on_cloud(msg):
        now = node.get_clock().now().nanoseconds * 1e-9
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        samples.append((time.monotonic(), (now - stamp) * 1e3))
        last['msg'] = msg

    node.create_subscription(PointCloud2, topic, on_cloud, qos_profile_sensor_data)
    print(f'Collecting {topic} for {seconds:.0f} s ...')
    end = time.monotonic() + seconds
    try:
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

    if len(samples) < 3:
        print(f'Too few clouds ({len(samples)}): is point_cloud:=true and the camera running?')
        return 1
    rate = (len(samples) - 1) / (samples[-1][0] - samples[0][0])
    lags = np.array([s[1] for s in samples])
    msg = last['msg']
    pts = xyz(msg)
    valid = np.isfinite(pts[..., 2])
    cy, cx = msg.height // 2, msg.width // 2
    centre = pts[cy - 5:cy + 5, cx - 5:cx + 5].reshape(-1, 3)
    centre = centre[np.isfinite(centre[:, 2])]
    fields = ','.join(f.name for f in msg.fields)
    print(f"frame_id '{msg.header.frame_id}'  {msg.width}x{msg.height}  fields {fields}  "
          f'{len(msg.data) / 1e6:.1f} MB/cloud')
    print(f'rate {rate:.2f} Hz  lag mean {lags.mean():.1f} ms  '
          f'p95 {np.percentile(lags, 95):.1f} ms  max {lags.max():.1f} ms')
    print(f'valid points {100 * valid.mean():.1f} %')
    if centre.size:
        m = np.median(centre, axis=0)
        print(f'centre 10x10 median: x {m[0]:+.3f}  y {m[1]:+.3f}  z {m[2]:.3f} m')
    return 0


if __name__ == '__main__':
    sys.exit(main())
