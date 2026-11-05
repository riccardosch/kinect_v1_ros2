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
Summarise the content of N depth frames (16UC1, millimetres).

Reports encoding, frame_id, the fraction of pixels with no reading (0), and
the median depth of a 10x10 window at the image centre. Use it to check depth
accuracy against a tape-measured distance.

Usage: depth_stats.py [topic] [frames]
  default topic /kinect/depth/image_raw, default 60 frames
"""

import sys

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else '/kinect/depth/image_raw'
    count = int(sys.argv[2]) if len(sys.argv) > 2 else 60
    samples = []
    meta = {}

    def on_image(msg):
        if msg.encoding != '16UC1':
            raise RuntimeError(f'Expected 16UC1, got {msg.encoding}')
        dtype = '>u2' if msg.is_bigendian else '<u2'
        depth = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width)
        cy, cx = msg.height // 2, msg.width // 2
        centre = depth[cy - 5:cy + 5, cx - 5:cx + 5]
        valid = centre[centre > 0]
        samples.append((float((depth == 0).mean()),
                        float(np.median(valid)) if valid.size else 0.0))
        meta.update(frame_id=msg.header.frame_id, size=f'{msg.width}x{msg.height}')

    rclpy.init()
    node = rclpy.create_node('depth_stats')
    node.create_subscription(Image, topic, on_image, qos_profile_sensor_data)
    print(f'Collecting {count} frames from {topic} ...')
    try:
        while rclpy.ok() and len(samples) < count:
            rclpy.spin_once(node, timeout_sec=2.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

    if not samples:
        print('No frames received (is the driver running? is depth_mode != off?)')
        return 1
    zeros = np.array([s[0] for s in samples]) * 100
    centres = np.array([s[1] for s in samples if s[1] > 0])
    print(f"frames {len(samples)}  {meta['size']}  frame_id '{meta['frame_id']}'")
    print(f'no-reading pixels: mean {zeros.mean():.1f} %  (min {zeros.min():.1f}, '
          f'max {zeros.max():.1f})')
    if centres.size:
        print(f'centre 10x10 median: {np.median(centres):.0f} mm  '
              f'(std {centres.std():.1f} mm over {centres.size} frames)')
    else:
        print('centre 10x10: no valid reading (too close, too far, or absorbing surface)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
