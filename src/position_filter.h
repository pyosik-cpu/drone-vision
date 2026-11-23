#pragma once

#include <array>
#include <optional>

struct PositionMeasurement {
  double x_m = 0.0;
  double y_m = 0.0;
  double z_m = 0.0;
};

enum class PositionFilterState {
  kUninitialized,
  kInitialized,
  kUpdated,
  kPredicted,
  kRejected,
  kReset,
};

const char* PositionFilterStateName(PositionFilterState state);

struct PositionFilterResult {
  bool valid = false;
  PositionFilterState state = PositionFilterState::kUninitialized;
  double x_m = 0.0;
  double y_m = 0.0;
  double z_m = 0.0;
  double vx_mps = 0.0;
  double vy_mps = 0.0;
  double vz_mps = 0.0;
  double nis = 0.0;
};

// 三维恒速卡尔曼滤波器。状态为[x,y,z,vx,vy,vz]，测量为PnP的[x,y,z]。
// 线性模型本质上是KF；保留独立模块，后续接入世界坐标和旋转模型时再扩展为EKF。
class PositionFilter {
 public:
  PositionFilterResult Step(const std::optional<PositionMeasurement>& measurement,
                            double dt_seconds);
  void Reset();
  bool initialized() const { return initialized_; }

 private:
  using Matrix6 = std::array<std::array<double, 6>, 6>;

  PositionFilterResult Current(PositionFilterState state, double nis) const;
  void Initialize(const PositionMeasurement& measurement);
  void Predict(double dt_seconds);
  bool Update(const PositionMeasurement& measurement, double& nis);

  bool initialized_ = false;
  std::array<double, 6> state_{};
  Matrix6 covariance_{};
};
