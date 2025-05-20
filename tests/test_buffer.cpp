#include "sensor_fuse/buffer.hpp"
#include <gtest/gtest.h>

TEST(BufferTest, PushInterpolate) {
  sensor_fuse::LinearBuffer<int> buf;
  buf.push(0.0, 1);
  EXPECT_EQ(buf.interpolate(0.0), 1);
  return 0;
}

int main() { return BufferTest_PushInterpolate(); }
