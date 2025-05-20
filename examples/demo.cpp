#include "sensor_fuse/imu_service.hpp"
#include <iostream>

int main() {
  sensor_fuse::LinearBuffer<Eigen::Vector3d> buf;
  sensor_fuse::ImuKalman kf;
  sensor_fuse::ImuService imu(buf, kf);

  imu.set(0.0, Eigen::Vector3d{0, 0, 9.8}, Eigen::Vector3d{0, 0, 0});
  imu.set(1.0, Eigen::Vector3d{0, 0, 9.6}, Eigen::Vector3d{0, 0, 0});

  auto pose = imu.get(0.25);
  std::cout << "Position: " << pose.position.transpose() << "\n";
  return 0;
}
