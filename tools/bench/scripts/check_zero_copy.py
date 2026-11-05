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
Verify zero-copy delivery from a launch log.

Run the driver with `bench:=true log_buffer_address:=true` and save the output,
then pass the log to this script. For every received frame, the publish and
receive lines with the same stamp must report the same buffer address, and the
receive line must be marked `intra` (delivered through the intra-process path).

Usage: check_zero_copy.py <launch.log>
Exit code 0 only if every received frame of every stream is zero-copy.
"""

import re
import sys

PUBLISH_RE = re.compile(r'\]: (\w+) publish stamp=(\S+) addr=(\S+)')
RECEIVE_RE = re.compile(r'\[[\w.]*?(\w+)_stats\]: recv stamp=(\S+) addr=(\S+) (\w+)')


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        return 2
    published = {}
    received = {}
    with open(sys.argv[1], encoding='utf-8', errors='replace') as log:
        for line in log:
            match = PUBLISH_RE.search(line)
            if match:
                published.setdefault(match[1], {})[match[2]] = match[3]
                continue
            match = RECEIVE_RE.search(line)
            if match:
                received.setdefault(match[1], {})[match[2]] = (match[3], match[4])

    if not received:
        print('No "recv" lines found: did you launch with bench:=true log_buffer_address:=true?')
        return 1

    all_ok = True
    for stream in sorted(received):
        pub = published.get(stream, {})
        rec = received[stream]
        zero_copy = sum(1 for stamp, (addr, path) in rec.items()
                        if pub.get(stamp) == addr and path == 'intra')
        ok = zero_copy == len(rec)
        all_ok &= ok
        print(f'{stream:6s} published {len(pub):5d}  received {len(rec):5d}  '
              f'zero-copy {zero_copy:5d}  -> {"PASS" if ok else "FAIL"}')
    return 0 if all_ok else 1


if __name__ == '__main__':
    sys.exit(main())
