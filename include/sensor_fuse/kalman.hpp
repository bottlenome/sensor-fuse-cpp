#ifndef SENSOR_FUSE_KALMAN_HPP
#define SENSOR_FUSE_KALMAN_HPP

#include <Eigen/Dense>

namespace sensor_fuse {

/**
 * @brief Simple IMU Kalman filter.
 */
class ImuKalman {
public:
  struct Pose {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d bias{Eigen::Vector3d::Zero()};
  };

  /**
   * @brief Update filter state.
   */
  Pose update(double /*dt*/, const Eigen::Vector3d & /*accel*/,
              const Eigen::Vector3d & /*gyro*/) {
    return Pose{};
  }
};

} // namespace sensor_fuse

#endif // SENSOR_FUSE_KALMAN_HPP
