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
Check the Kinect microphone stream and optionally record it to a WAV file.

Reads audio/info (format) and audio/data (interleaved PCM chunks), then reports:
- the chunk rate and the effective sample rate (wall clock),
- gaps between chunk stamps (a gap > 1.5 chunks means data was lost),
- peak and RMS per channel.

Usage: audio_stats.py [seconds] [out.wav] [--namespace /kinect]
"""

import argparse
import math
import sys
import time
import wave

from audio_common_msgs.msg import AudioDataStamped, AudioInfo
import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

FORMATS = {'S32LE': '<i4', 'S16LE': '<i2'}


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument('seconds', type=float, nargs='?', default=5.0)
    parser.add_argument('wav', nargs='?', default=None, help='optional output WAV file')
    parser.add_argument('--namespace', default='/kinect')
    args = parser.parse_args()

    info = {}
    chunks = []

    rclpy.init()
    node = rclpy.create_node('audio_stats')
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(AudioInfo, f'{args.namespace}/audio/info',
                             lambda m: info.update(msg=m), latched)
    node.create_subscription(
        AudioDataStamped, f'{args.namespace}/audio/data',
        lambda m: chunks.append((time.monotonic(),
                                 m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                                 bytes(m.audio.data))),
        QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE))

    print(f'Recording {args.namespace}/audio/data for {args.seconds:.0f} s ...')
    end = time.monotonic() + args.seconds
    try:
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

    if 'msg' not in info or len(chunks) < 3:
        print(f"No data (audio/info received: {'msg' in info}, chunks: {len(chunks)})")
        return 1
    ai = info['msg']
    dtype = FORMATS.get(ai.sample_format)
    if dtype is None:
        print(f'Unsupported sample_format {ai.sample_format}')
        return 1
    samples = np.frombuffer(b''.join(c[2] for c in chunks), dtype=dtype)
    samples = samples.reshape(-1, ai.channels)
    frames_per_chunk = len(chunks[0][2]) // (np.dtype(dtype).itemsize * ai.channels)

    wall = chunks[-1][0] - chunks[0][0]
    # Rate over the chunks after the first one (the first marks t=0).
    rate = (len(samples) - frames_per_chunk) / wall
    stamp_steps = np.diff([c[1] for c in chunks])
    chunk_s = frames_per_chunk / ai.sample_rate
    gaps = int(np.sum(stamp_steps > 1.5 * chunk_s))
    full_scale = float(np.iinfo(dtype).max)

    print(f'format {ai.channels} ch {ai.sample_format} @ {ai.sample_rate} Hz, '
          f'{frames_per_chunk} frames/chunk')
    print(f'chunks {len(chunks)} ({len(chunks) / wall:.1f}/s)  effective rate {rate:.0f} Hz '
          f'({100 * (rate - ai.sample_rate) / ai.sample_rate:+.2f} %)')
    print(f'stamp step mean {1e3 * stamp_steps.mean():.1f} ms, '
          f'max {1e3 * stamp_steps.max():.1f} ms, gaps > 1.5 chunks: {gaps}')
    for ch in range(ai.channels):
        x = samples[:, ch].astype(np.float64)
        peak = np.abs(x).max() / full_scale
        rms = math.sqrt(np.mean(x * x)) / full_scale
        peak_db = 20 * math.log10(peak + 1e-12)
        rms_db = 20 * math.log10(rms + 1e-12)
        print(f'mic{ch + 1}: peak {100 * peak:6.2f} %FS ({peak_db:6.1f} dBFS)  '
              f'rms {rms_db:6.1f} dBFS')

    if args.wav:
        with wave.open(args.wav, 'wb') as w:
            w.setnchannels(ai.channels)
            w.setsampwidth(np.dtype(dtype).itemsize)
            w.setframerate(ai.sample_rate)
            w.writeframes(samples.astype(dtype).tobytes())
        print(f'wrote {args.wav} ({len(samples) / ai.sample_rate:.1f} s)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
