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
Summarise an IMU topic (best-effort QoS): rate, mean, noise, |a|, pitch and roll.

`ros2 topic hz` on Jazzy subscribes RELIABLE and receives nothing from the
best-effort driver topic; this script uses the sensor-data QoS instead.

Usage: imu_stats.py [topic] [seconds]
  default topic /kinect/imu, default 6 s
"""

import math
import statistics
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else '/kinect/imu'
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 6.0
    samples = []

    def on_imu(msg):
        a = msg.linear_acceleration
        samples.append((time.monotonic(), a.x, a.y, a.z))

    rclpy.init()
    node = rclpy.create_node('imu_stats')
    node.create_subscription(Imu, topic, on_imu, qos_profile_sensor_data)
    print(f'Collecting {topic} for {seconds:.0f} s ...')
    end = time.monotonic() + seconds
    try:
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

    if len(samples) < 3:
        print('Too few messages received (is the aux component running?)')
        return 1
    duration = samples[-1][0] - samples[0][0]
    ax, ay, az = ([s[i] for s in samples] for i in (1, 2, 3))
    norm = [math.sqrt(x * x + y * y + z * z) for x, y, z in zip(ax, ay, az)]
    mx, my, mz = statistics.mean(ax), statistics.mean(ay), statistics.mean(az)
    print(f'rate  {(len(samples) - 1) / duration:.2f} Hz over {len(samples)} messages')
    print(f'mean  x {mx:+.2f}  y {my:+.2f}  z {mz:+.2f} m/s^2   |a| {statistics.mean(norm):.2f}')
    print(f'std   x {statistics.stdev(ax):.3f}  y {statistics.stdev(ay):.3f}  '
          f'z {statistics.stdev(az):.3f} m/s^2')
    # REP 103 frame (x forward, y left, z up): nose-up pitch -> +x, left roll -> +y.
    print(f'pitch (nose up +) {math.degrees(math.atan2(mx, mz)):+.1f} deg   '
          f'roll (left side down +) {math.degrees(math.atan2(my, mz)):+.1f} deg')
    return 0


if __name__ == '__main__':
    sys.exit(main())
