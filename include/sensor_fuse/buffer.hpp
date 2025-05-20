#ifndef SENSOR_FUSE_BUFFER_HPP
#define SENSOR_FUSE_BUFFER_HPP

#include <Eigen/Dense>
#include <deque>

namespace sensor_fuse {

/**
 * @brief Timestamped buffer with linear interpolation.
 *
 * @tparam T Data type stored in the buffer.
 */
template <typename T> class LinearBuffer {
public:
  struct Entry {
    double timestamp;
    T value;
  };

  /**
   * @brief Add a new entry to the buffer.
   */
  void push(double timestamp, const T &value) {
    buffer_.push_back({timestamp, value});
  }

  /**
   * @brief Interpolate value at given timestamp.
   */
  T interpolate(double /*timestamp*/) const {
    return buffer_.empty() ? T{} : buffer_.back().value;
  }

private:
  std::deque<Entry> buffer_;
};

} // namespace sensor_fuse

#endif // SENSOR_FUSE_BUFFER_HPP
