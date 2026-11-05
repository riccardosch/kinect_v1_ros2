// Copyright 2026 Riccardo Schiavo
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include "kinect_v1_driver/kinect_aux_node.hpp"

using kinect_v1_driver::parseLedCommand;

TEST(LedCommand, KnownStatesMapToLibfreenectValues)
{
  EXPECT_EQ(parseLedCommand("off"), LED_OFF);
  EXPECT_EQ(parseLedCommand("green"), LED_GREEN);
  EXPECT_EQ(parseLedCommand("red"), LED_RED);
  EXPECT_EQ(parseLedCommand("yellow"), LED_YELLOW);
  EXPECT_EQ(parseLedCommand("blink_green"), LED_BLINK_GREEN);
  EXPECT_EQ(parseLedCommand("blink_red_yellow"), LED_BLINK_RED_YELLOW);
}

TEST(AudioInterleave, PlanarToInterleavedLittleEndian)
{
  const int32_t c0[2] = {0x01020304, -1};
  const int32_t c1[2] = {0x11121314, 0};
  const int32_t c2[2] = {0x21222324, 0x7fffffff};
  const int32_t c3[2] = {0x31323334, static_cast<int32_t>(0x80000000)};
  const int32_t * const planar[4] = {c0, c1, c2, c3};
  uint8_t out[2 * 4 * 4];
  kinect_v1_driver::interleaveS32LE(planar, 4, 2, out);

  const uint8_t expected[2 * 4 * 4] = {
    // sample 0: c0 c1 c2 c3, each little-endian
    0x04, 0x03, 0x02, 0x01, 0x14, 0x13, 0x12, 0x11,
    0x24, 0x23, 0x22, 0x21, 0x34, 0x33, 0x32, 0x31,
    // sample 1
    0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,
    0xff, 0xff, 0xff, 0x7f, 0x00, 0x00, 0x00, 0x80};
  for (size_t i = 0; i < sizeof(out); ++i) {
    EXPECT_EQ(out[i], expected[i]) << "byte " << i;
  }
}

TEST(LedCommand, UnknownOrMalformedStatesAreRejected)
{
  EXPECT_FALSE(parseLedCommand("").has_value());
  EXPECT_FALSE(parseLedCommand("blue").has_value());
  EXPECT_FALSE(parseLedCommand("Green").has_value());   // case-sensitive
  EXPECT_FALSE(parseLedCommand(" green").has_value());
  EXPECT_FALSE(parseLedCommand("5").has_value());       // value 5 does not exist
}
