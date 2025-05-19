# sensor-fuse-cpp

## Overview

`sensor-fuse-cpp` is a C++ library for sensor data synchronization and fusion, specializing in IMU data interpolation and Kalman filtering. Designed with Dependency Injection (DI), it maximizes modularity, testability, and ease of extension.

## Features

* **LinearBuffer**: Timestamped data buffer with linear interpolation
* **ImuKalman**: Simple Kalman filter for state estimation (position, velocity, bias)
* **ImuService**: DI-based service combining buffer and filter with unified `set`/`get` API
* **Header-only interfaces**: Minimal external dependencies (STL, Eigen)
* **Examples & Tests**: Demo application and unit tests included

## Prerequisites

* C++14-compatible compiler (GCC ≥ 5.0, Clang ≥ 3.8, MSVC 2015+)
* CMake ≥ 3.10
* [Eigen3](http://eigen.tuxfamily.org/) for linear algebra
* [GoogleTest](https://github.com/google/googletest) (optional, for unit tests)

## Building

```bash
# Clone repository
git clone https://github.com/your-org/sensor-fuse-cpp.git
cd sensor-fuse-cpp

# Create build directory
mkdir build && cd build

# Configure and build
cmake ..
make
```

### CMake Options

* `-DBUILD_EXAMPLES=ON/OFF` (default: ON) — Build demo applications
* `-DBUILD_TESTS=ON/OFF`    (default: ON) — Build unit tests

## Usage

```cpp
#include <sensor_fuse/buffer.hpp>
#include <sensor_fuse/kalman.hpp>
#include <sensor_fuse/imu_service.hpp>

int main() {
    // DI setup
    LinearBuffer<Eigen::Vector3d> accelBuf;
    ImuKalman kf;
    ImuService imu(accelBuf, kf);

    // Feed IMU data
    imu.set(0.0, Eigen::Vector3d{0,0,9.8}, Eigen::Vector3d{0,0,0});
    imu.set(1.0, Eigen::Vector3d{0,0,9.6}, Eigen::Vector3d{0,0,0});

    // Get interpolated & filtered pose at t = 0.25s
    auto pose = imu.get(0.25);
    std::cout << "Position: "
              << pose.position.transpose() << "\n";
    return 0;
}
```

## Project Structure

```
sensor-fuse-cpp/
├── CMakeLists.txt        # Build configuration
├── README.md             # Project overview and instructions
├── include/
│   └── sensor_fuse/
│       ├── buffer.hpp    # IBuffer & LinearBuffer
│       ├── kalman.hpp    # IKalmanFilter & ImuKalman
│       └── imu_service.hpp # IImuService & ImuService
├── src/
│   ├── buffer.cpp        # LinearBuffer implementation (if any)
│   ├── kalman.cpp        # ImuKalman implementation
│   └── imu_service.cpp   # ImuService implementation
├── examples/
│   └── demo.cpp          # Simple usage demonstration
└── tests/
    ├── test_buffer.cpp   # Unit tests for buffer
    └── test_kalman.cpp   # Unit tests for Kalman filter
```

## Contributing

Contributions are welcome! Please:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/YourFeature`)
3. Commit your changes (`git commit -m "Add YourFeature"`)
4. Push to the branch (`git push origin feature/YourFeature`)
5. Open a Pull Request

Please ensure all new code is covered by unit tests and follows existing style conventions.

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.
