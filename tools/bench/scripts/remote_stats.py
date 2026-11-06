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
Measure compressed image topics as a remote viewer receives them.

Run it on the viewing machine (or on the robot, for a loopback baseline). For
each topic it reports the rate, the mean message size, the bandwidth and the
latency now - header.stamp (meaningful across machines only if their clocks
are synchronised, e.g. with chrony).

Usage: remote_stats.py [seconds] [topic ...]
  default 10 s on /kinect/rgb/image_raw/compressed and
  /kinect/depth/image_raw/compressedDepth (+ depth_registered variant)
"""

import sys
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

DEFAULT_TOPICS = [
    '/kinect/rgb/image_raw/compressed',
    '/kinect/depth/image_raw/compressedDepth',
    '/kinect/depth_registered/image_raw/compressedDepth',
]


def main():
    args = sys.argv[1:]
    seconds = float(args.pop(0)) if args and args[0].replace('.', '', 1).isdigit() else 10.0
    topics = args or DEFAULT_TOPICS
    data = {t: [] for t in topics}

    rclpy.init()
    node = rclpy.create_node('remote_stats')
    clock = node.get_clock()
    for topic in topics:
        def on_msg(msg, topic=topic):
            now = clock.now().nanoseconds * 1e-9
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            data[topic].append((time.monotonic(), len(msg.data), (now - stamp) * 1e3, msg.format))
        node.create_subscription(CompressedImage, topic, on_msg, qos_profile_sensor_data)

    print(f'Measuring {len(topics)} topic(s) for {seconds:.0f} s ...')
    end = time.monotonic() + seconds
    try:
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

    total_mbit = 0.0
    for topic, s in data.items():
        if len(s) < 3:
            if s or topic not in DEFAULT_TOPICS:
                print(f'{topic}: {len(s)} messages')
            continue
        duration = s[-1][0] - s[0][0]
        rate = (len(s) - 1) / duration
        sizes = np.array([x[1] for x in s])
        lags = np.array([x[2] for x in s])
        mbit = sizes.mean() * 8 * rate / 1e6
        total_mbit += mbit
        print(f'{topic}\n  format "{s[-1][3]}"  {rate:.2f} Hz  {sizes.mean() / 1e3:.0f} KB/frame  '
              f'{mbit:.1f} Mbit/s  lag mean {lags.mean():.1f} ms  '
              f'p95 {np.percentile(lags, 95):.1f} ms  max {lags.max():.1f} ms')
    print(f'total {total_mbit:.1f} Mbit/s')
    return 0


if __name__ == '__main__':
    sys.exit(main())
