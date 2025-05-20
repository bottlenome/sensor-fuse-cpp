#ifndef SENSOR_FUSE_IMU_SERVICE_HPP
#define SENSOR_FUSE_IMU_SERVICE_HPP

#include "buffer.hpp"
#include "kalman.hpp"

namespace sensor_fuse {

/**
 * @brief Service combining buffer and Kalman filter.
 */
class ImuService {
public:
  ImuService(LinearBuffer<Eigen::Vector3d> &accel_buf, ImuKalman &filter)
      : accel_buf_(accel_buf), filter_(filter) {}

  /**
   * @brief Feed IMU data.
   */
  void set(double timestamp, const Eigen::Vector3d &accel,
           const Eigen::Vector3d &gyro) {
    accel_buf_.push(timestamp, accel);
    last_time_ = timestamp;
  }

  /**
   * @brief Get filtered pose at timestamp.
   */
  ImuKalman::Pose get(double timestamp) {
    double dt = timestamp - last_time_;
    return filter_.update(dt, accel_buf_.interpolate(timestamp), gyro_zero_);
  }

private:
  LinearBuffer<Eigen::Vector3d> &accel_buf_;
  ImuKalman &filter_;
  double last_time_{0.0};
  Eigen::Vector3d gyro_zero_{Eigen::Vector3d::Zero()};
};

} // namespace sensor_fuse

#endif // SENSOR_FUSE_IMU_SERVICE_HPP
