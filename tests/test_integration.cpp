#include "sensor_fuse/imu_service.hpp"
#include <chrono>
#include <condition_variable>
#include <gtest/gtest.h>
#include <mutex>
#include <queue>
#include <thread>

template <typename T> class ConcurrentQueue {
public:
  void push(const T &value) {
    std::lock_guard<std::mutex> lock(m_);
    q_.push(value);
    cv_.notify_one();
  }

  void wait_and_pop(T &value) {
    std::unique_lock<std::mutex> lock(m_);
    cv_.wait(lock, [this] { return !q_.empty(); });
    value = q_.front();
    q_.pop();
  }

  bool pop(T &value) {
    std::lock_guard<std::mutex> lock(m_);
    if (q_.empty())
      return false;
    value = q_.front();
    q_.pop();
    return true;
  }

private:
  std::queue<T> q_;
  std::mutex m_;
  std::condition_variable cv_;
};

struct PoseEstimator {
  sensor_fuse::ImuService &imu;
  Eigen::Vector3d estimate(double t) { return imu.get(t).position; }
};

struct DeltaPose {
  Eigen::Vector3d value{Eigen::Vector3d::Zero()};
  DeltaPose() = default;
  explicit DeltaPose(const Eigen::Vector3d &v) : value(v) {}
};

struct ObjectRecognizer {
  int camera_count;
  bool lidar_available;
  bool recognized{false};
  void process(const DeltaPose &delta) {
    if (camera_count == 4 && lidar_available &&
        delta.value == Eigen::Vector3d::Zero())
      recognized = true;
  }
};

struct SensorData {
  double timestamp{0.0};
  int seq{0};
};

struct ImuData : SensorData {
  Eigen::Vector3d accel;
  ImuData() = default;
  ImuData(double ts, int s, const Eigen::Vector3d &a) {
    timestamp = ts;
    seq = s;
    accel = a;
  }
};

struct CamData : SensorData {
  int id{0};
  CamData() = default;
  CamData(double ts, int s, int cam_id) {
    timestamp = ts;
    seq = s;
    id = cam_id;
  }
};

struct LidarData : SensorData {
  bool available{false};
  LidarData() = default;
  LidarData(double ts, int s, bool avail) {
    timestamp = ts;
    seq = s;
    available = avail;
  }
};

struct EnvironmentModel {
  bool objects{false};
};

struct Trajectory {
  bool valid{false};
};

struct Planner {
  bool plan_ready{false};
  void create(const EnvironmentModel &env, const DeltaPose &delta) {
    if (env.objects && delta.value == Eigen::Vector3d::Zero())
      plan_ready = true;
  }
};

struct Control {
  bool command{false};
  void apply(const Trajectory &traj, const DeltaPose &delta) {
    if (traj.valid && delta.value == Eigen::Vector3d::Zero())
      command = true;
  }
};

TEST(Integration, Pipeline) {
  ConcurrentQueue<ImuData> imu_queue;
  ConcurrentQueue<CamData> cam_queue;
  ConcurrentQueue<LidarData> lidar_queue;
  ConcurrentQueue<DeltaPose> delta_queue;
  ConcurrentQueue<EnvironmentModel> env_queue;
  ConcurrentQueue<Trajectory> traj_queue;

  DeltaPose shared_delta;
  std::mutex delta_mutex;

  auto imu_thread = std::thread([&] {
    double t = 0.0;
    int seq = 0;
    for (int i = 0; i < 1; ++i) {
      imu_queue.push(ImuData(t, seq++, Eigen::Vector3d{0, 0, 9.8}));
      t += 0.01;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  });

  auto cam_worker = [&](int id) {
    cam_queue.push(CamData(0.0, id, id));
    std::this_thread::sleep_for(std::chrono::milliseconds(33));
  };
  std::thread cam_threads[4];
  for (int i = 0; i < 4; ++i) {
    cam_threads[i] = std::thread(cam_worker, i);
  }

  auto lidar_thread = std::thread([&] {
    lidar_queue.push(LidarData(0.0, 0, true));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  });

  sensor_fuse::LinearBuffer<Eigen::Vector3d> buf;
  sensor_fuse::ImuKalman kf;
  sensor_fuse::ImuService imu(buf, kf);

  auto pose_thread = std::thread([&] {
    ImuData imu_data;
    imu_queue.wait_and_pop(imu_data);
    imu.set(imu_data.timestamp, imu_data.accel, Eigen::Vector3d::Zero());
    PoseEstimator pose_est{imu};
    auto pose = pose_est.estimate(imu_data.timestamp);
    {
      std::lock_guard<std::mutex> lock(delta_mutex);
      shared_delta = DeltaPose(pose);
    }
    delta_queue.push(shared_delta);
  });

  for (auto &th : cam_threads)
    th.join();
  imu_thread.join();
  lidar_thread.join();
  pose_thread.join();

  auto recog_thread = std::thread([&] {
    CamData cam;
    int cam_count = 0;
    for (int i = 0; i < 4; ++i) {
      cam_queue.wait_and_pop(cam);
      ++cam_count;
    }
    LidarData lidar;
    lidar_queue.wait_and_pop(lidar);
    ObjectRecognizer recog{cam_count, lidar.available};
    DeltaPose local_delta;
    {
      std::lock_guard<std::mutex> lock(delta_mutex);
      local_delta = shared_delta;
    }
    recog.process(local_delta);
    env_queue.push(EnvironmentModel{recog.recognized});
  });

  auto planner_thread = std::thread([&] {
    EnvironmentModel env;
    env_queue.wait_and_pop(env);
    Planner planner;
    DeltaPose local_delta;
    {
      std::lock_guard<std::mutex> lock(delta_mutex);
      local_delta = shared_delta;
    }
    planner.create(env, local_delta);
    traj_queue.push(Trajectory{planner.plan_ready});
  });

  Control ctrl;
  auto control_thread = std::thread([&] {
    Trajectory traj;
    traj_queue.wait_and_pop(traj);
    DeltaPose local_delta;
    {
      std::lock_guard<std::mutex> lock(delta_mutex);
      local_delta = shared_delta;
    }
    ctrl.apply(traj, local_delta);
  });

  recog_thread.join();
  planner_thread.join();
  control_thread.join();

  DeltaPose delta;
  delta_queue.wait_and_pop(delta);

  EXPECT_EQ(delta.value, Eigen::Vector3d::Zero());
  EXPECT_EQ(ctrl.command, true);
  return 0;
}

int main() { return Integration_Pipeline(); }
