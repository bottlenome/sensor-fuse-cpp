#include "sensor_fuse/kalman.hpp"
#include <gtest/gtest.h>

TEST(KalmanTest, DefaultPose) {
  sensor_fuse::ImuKalman kf;
  auto pose = kf.update(0.1, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  EXPECT_EQ(pose.position, Eigen::Vector3d::Zero());
  return 0;
}

int main() { return KalmanTest_DefaultPose(); }
