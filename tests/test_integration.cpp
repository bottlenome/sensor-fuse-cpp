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

struct ImuData {
  double timestamp;
  Eigen::Vector3d accel;
};

TEST(Integration, Pipeline) {
  ConcurrentQueue<ImuData> imu_queue;
  ConcurrentQueue<int> cam_queue;
  ConcurrentQueue<int> lidar_queue;

  auto imu_thread = std::thread([&] {
    double t = 0.0;
    for (int i = 0; i < 1; ++i) {
      imu_queue.push(ImuData{t, Eigen::Vector3d{0, 0, 9.8}});
      t += 0.01;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  });

  auto cam_worker = [&](int id) {
    cam_queue.push(id);
    std::this_thread::sleep_for(std::chrono::milliseconds(33));
  };
  std::thread cam_threads[4];
  for (int i = 0; i < 4; ++i) {
    cam_threads[i] = std::thread(cam_worker, i);
  }

  auto lidar_thread = std::thread([&] {
    lidar_queue.push(0);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  });

  sensor_fuse::LinearBuffer<Eigen::Vector3d> buf;
  sensor_fuse::ImuKalman kf;
  sensor_fuse::ImuService imu(buf, kf);

  ImuData imu_data;
  imu_queue.wait_and_pop(imu_data);
  imu.set(imu_data.timestamp, imu_data.accel, Eigen::Vector3d::Zero());

  PoseEstimator pose_est{imu};
  auto pose = pose_est.estimate(imu_data.timestamp);

  for (auto &th : cam_threads)
    th.join();
  imu_thread.join();
  lidar_thread.join();

  int cam_id;
  int cam_count = 0;
  while (cam_queue.pop(cam_id))
    ++cam_count;
  int lidar_dummy;
  lidar_queue.wait_and_pop(lidar_dummy);

  ObjectRecognizer recog{cam_count, true};
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
