#include "sensor_fuse/imu_service.hpp"
#include <gtest/gtest.h>

struct PoseEstimator {
  sensor_fuse::ImuService &imu;
  Eigen::Vector3d estimate(double t) { return imu.get(t).position; }
};

struct ObjectRecognizer {
  int camera_count;
  bool lidar_available;
  bool recognized{false};
  void process() {
    if (camera_count == 4 && lidar_available)
      recognized = true;
  }
};

struct Planner {
  bool plan_ready{false};
  void create(const ObjectRecognizer &obj) {
    if (obj.recognized)
      plan_ready = true;
  }
};

struct Control {
  bool command{false};
  void apply(const Planner &p) {
    if (p.plan_ready)
      command = true;
  }
};

TEST(Integration, Pipeline) {
  sensor_fuse::LinearBuffer<Eigen::Vector3d> buf;
  sensor_fuse::ImuKalman kf;
  sensor_fuse::ImuService imu(buf, kf);
  imu.set(0.0, Eigen::Vector3d{0, 0, 9.8}, Eigen::Vector3d{0, 0, 0});

  PoseEstimator pose_est{imu};
  auto pose = pose_est.estimate(0.0);

  ObjectRecognizer recog{4, true};
  recog.process();

  Planner planner;
  planner.create(recog);

  Control ctrl;
  ctrl.apply(planner);

  EXPECT_EQ(pose, Eigen::Vector3d::Zero());
  EXPECT_EQ(ctrl.command, true);
  return 0;
}

int main() { return Integration_Pipeline(); }
