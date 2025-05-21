# Tests

This directory contains unit and integration tests for **sensor-fuse-cpp**.

## Integration Test Requirements (Integration testの要件)

- **Sensors**
  - IMU: 100 Hz
  - 4 cameras: 2k YUV at 30 Hz each
  - Lidar: 20 Hz
- **Data Flow**
  - IMU → PoseEstimator → DeltaPose
  - Cameras & Lidar → Object Recognizer → Environment model
  - Environment model → Planner → Trajectory
  - Trajectory → Control → Control signal
  - DeltaPose is referenced by all processing stages (DeltaPose はすべての処理から参照されます)

The integration test verifies that data can pass through this simplified
pipeline and produce a valid control command.
