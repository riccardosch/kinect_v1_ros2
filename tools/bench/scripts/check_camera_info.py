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
Check that a camera publishes image_raw + camera_info as consumers expect.

For each received image there must be a camera_info with the same stamp and
frame_id (image_transport CameraSubscriber and depth_image_proc pair them
exactly), and the camera_info size must match the image.

Usage: check_camera_info.py <camera namespace> [seconds]
  e.g. check_camera_info.py /kinect/rgb 5
Exit code 0 only if every image has a matching camera_info.
"""

import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


def stamp_key(header):
    return (header.stamp.sec, header.stamp.nanosec)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    ns = sys.argv[1].rstrip('/')
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
    images = {}
    infos = {}

    rclpy.init()
    node = rclpy.create_node('check_camera_info')
    node.create_subscription(
        Image, f'{ns}/image_raw',
        lambda m: images.setdefault(stamp_key(m.header), (m.header.frame_id, m.width, m.height)),
        qos_profile_sensor_data)
    node.create_subscription(
        CameraInfo, f'{ns}/camera_info',
        lambda m: infos.setdefault(stamp_key(m.header), m),
        QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
    print(f'Collecting {ns}/image_raw and {ns}/camera_info for {seconds:.0f} s ...')
    end = time.monotonic() + seconds
    try:
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

    if not images or not infos:
        print(f'No data (images {len(images)}, camera_info {len(infos)})')
        return 1
    # The last image may still be waiting for its info: ignore it.
    keys = sorted(images)[:-1]
    matched = 0
    problems = set()
    for k in keys:
        info = infos.get(k)
        if info is None:
            continue
        frame_id, w, h = images[k]
        if info.header.frame_id != frame_id:
            problems.add(f'frame_id mismatch: image {frame_id}, info {info.header.frame_id}')
        elif (info.width, info.height) != (w, h):
            problems.add(f'size mismatch: image {w}x{h}, info {info.width}x{info.height}')
        else:
            matched += 1
    info = infos[max(infos)]
    k = info.k
    print(f'images {len(keys)}, with matching camera_info {matched}')
    print(f"frame_id '{info.header.frame_id}'  {info.width}x{info.height}  "
          f'model {info.distortion_model}  D {[round(d, 4) for d in info.d]}')
    print(f'fx {k[0]:.1f}  fy {k[4]:.1f}  cx {k[2]:.1f}  cy {k[5]:.1f}')
    for p in sorted(problems):
        print('PROBLEM:', p)
    ok = matched == len(keys) and not problems
    print('PASS' if ok else 'FAIL')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
