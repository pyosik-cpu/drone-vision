#include "position_filter.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {

using Matrix3 = std::array<std::array<double, 3>, 3>;

constexpr double kAccelerationStandardDeviation = 4.0;  // m/s^2
constexpr std::array<double, 3> kMeasurementVariance{
    0.02 * 0.02, 0.02 * 0.02, 0.04 * 0.04};
// 三维测量卡方分布99%分位数；超限观测不写入状态。
constexpr double kNisThreshold = 11.345;
constexpr double kMaximumDtSeconds = 0.3;

bool Invert3x3(const Matrix3& matrix, Matrix3& inverse) {
  const double determinant =
      matrix[0][0] * (matrix[1][1] * matrix[2][2] -
                      matrix[1][2] * matrix[2][1]) -
      matrix[0][1] * (matrix[1][0] * matrix[2][2] -
                      matrix[1][2] * matrix[2][0]) +
      matrix[0][2] * (matrix[1][0] * matrix[2][1] -
                      matrix[1][1] * matrix[2][0]);
  if (!std::isfinite(determinant) || std::abs(determinant) < 1e-15) {
    return false;
  }
  const double scale = 1.0 / determinant;
  inverse[0][0] = (matrix[1][1] * matrix[2][2] -
                   matrix[1][2] * matrix[2][1]) * scale;
  inverse[0][1] = (matrix[0][2] * matrix[2][1] -
                   matrix[0][1] * matrix[2][2]) * scale;
  inverse[0][2] = (matrix[0][1] * matrix[1][2] -
                   matrix[0][2] * matrix[1][1]) * scale;
  inverse[1][0] = (matrix[1][2] * matrix[2][0] -
                   matrix[1][0] * matrix[2][2]) * scale;
  inverse[1][1] = (matrix[0][0] * matrix[2][2] -
                   matrix[0][2] * matrix[2][0]) * scale;
  inverse[1][2] = (matrix[0][2] * matrix[1][0] -
                   matrix[0][0] * matrix[1][2]) * scale;
  inverse[2][0] = (matrix[1][0] * matrix[2][1] -
                   matrix[1][1] * matrix[2][0]) * scale;
  inverse[2][1] = (matrix[0][1] * matrix[2][0] -
                   matrix[0][0] * matrix[2][1]) * scale;
  inverse[2][2] = (matrix[0][0] * matrix[1][1] -
                   matrix[0][1] * matrix[1][0]) * scale;
  return true;
}

}  // namespace

const char* PositionFilterStateName(PositionFilterState state) {
  switch (state) {
    case PositionFilterState::kUninitialized: return "uninitialized";
    case PositionFilterState::kInitialized: return "initialized";
    case PositionFilterState::kUpdated: return "updated";
    case PositionFilterState::kPredicted: return "predicted";
    case PositionFilterState::kRejected: return "rejected";
    case PositionFilterState::kReset: return "reset";
  }
  return "unknown";
}

void PositionFilter::Reset() {
  initialized_ = false;
  state_.fill(0.0);
  for (auto& row : covariance_) row.fill(0.0);
}

void PositionFilter::Initialize(const PositionMeasurement& measurement) {
  Reset();
  initialized_ = true;
  state_[0] = measurement.x_m;
  state_[1] = measurement.y_m;
  state_[2] = measurement.z_m;
  covariance_[0][0] = kMeasurementVariance[0];
  covariance_[1][1] = kMeasurementVariance[1];
  covariance_[2][2] = kMeasurementVariance[2];
  covariance_[3][3] = 4.0;
  covariance_[4][4] = 4.0;
  covariance_[5][5] = 4.0;
}

void PositionFilter::Predict(double dt_seconds) {
  std::array<std::array<double, 6>, 6> transition{};
  for (int index = 0; index < 6; ++index) transition[index][index] = 1.0;
  for (int axis = 0; axis < 3; ++axis) transition[axis][axis + 3] = dt_seconds;

  std::array<double, 6> predicted_state{};
  for (int row = 0; row < 6; ++row) {
    for (int column = 0; column < 6; ++column) {
      predicted_state[row] += transition[row][column] * state_[column];
    }
  }

  Matrix6 intermediate{};
  Matrix6 predicted_covariance{};
  for (int row = 0; row < 6; ++row) {
    for (int column = 0; column < 6; ++column) {
      for (int item = 0; item < 6; ++item) {
        intermediate[row][column] +=
            transition[row][item] * covariance_[item][column];
      }
    }
  }
  for (int row = 0; row < 6; ++row) {
    for (int column = 0; column < 6; ++column) {
      for (int item = 0; item < 6; ++item) {
        predicted_covariance[row][column] +=
            intermediate[row][item] * transition[column][item];
      }
    }
  }

  const double acceleration_variance =
      kAccelerationStandardDeviation * kAccelerationStandardDeviation;
  const double dt2 = dt_seconds * dt_seconds;
  const double dt3 = dt2 * dt_seconds;
  const double dt4 = dt2 * dt2;
  for (int axis = 0; axis < 3; ++axis) {
    predicted_covariance[axis][axis] += dt4 * 0.25 * acceleration_variance;
    predicted_covariance[axis][axis + 3] += dt3 * 0.5 * acceleration_variance;
    predicted_covariance[axis + 3][axis] += dt3 * 0.5 * acceleration_variance;
    predicted_covariance[axis + 3][axis + 3] += dt2 * acceleration_variance;
  }
  state_ = predicted_state;
  covariance_ = predicted_covariance;
}

bool PositionFilter::Update(const PositionMeasurement& measurement, double& nis) {
  const std::array<double, 3> innovation{
      measurement.x_m - state_[0], measurement.y_m - state_[1],
      measurement.z_m - state_[2]};
  Matrix3 innovation_covariance{};
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      innovation_covariance[row][column] = covariance_[row][column];
    }
    innovation_covariance[row][row] += kMeasurementVariance[row];
  }
  Matrix3 inverse{};
  if (!Invert3x3(innovation_covariance, inverse)) return false;
  nis = 0.0;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      nis += innovation[row] * inverse[row][column] * innovation[column];
    }
  }
  if (!std::isfinite(nis) || nis > kNisThreshold) return false;

  std::array<std::array<double, 3>, 6> gain{};
  for (int row = 0; row < 6; ++row) {
    for (int column = 0; column < 3; ++column) {
      for (int item = 0; item < 3; ++item) {
        gain[row][column] += covariance_[row][item] * inverse[item][column];
      }
    }
  }
  for (int row = 0; row < 6; ++row) {
    for (int item = 0; item < 3; ++item) {
      state_[row] += gain[row][item] * innovation[item];
    }
  }

  // Joseph形式保证有限精度下协方差仍保持半正定。
  Matrix6 residual_transform{};
  for (int index = 0; index < 6; ++index) residual_transform[index][index] = 1.0;
  for (int row = 0; row < 6; ++row) {
    for (int column = 0; column < 3; ++column) {
      residual_transform[row][column] -= gain[row][column];
    }
  }
  Matrix6 intermediate{};
  Matrix6 posterior{};
  for (int row = 0; row < 6; ++row) {
    for (int column = 0; column < 6; ++column) {
      for (int item = 0; item < 6; ++item) {
        intermediate[row][column] +=
            residual_transform[row][item] * covariance_[item][column];
      }
    }
  }
  for (int row = 0; row < 6; ++row) {
    for (int column = 0; column < 6; ++column) {
      for (int item = 0; item < 6; ++item) {
        posterior[row][column] +=
            intermediate[row][item] * residual_transform[column][item];
      }
      for (int item = 0; item < 3; ++item) {
        posterior[row][column] +=
            gain[row][item] * kMeasurementVariance[item] * gain[column][item];
      }
    }
  }
  covariance_ = posterior;
  return true;
}

PositionFilterResult PositionFilter::Current(PositionFilterState state,
                                             double nis) const {
  PositionFilterResult result;
  result.valid = initialized_;
  result.state = state;
  result.nis = nis;
  if (initialized_) {
    result.x_m = state_[0];
    result.y_m = state_[1];
    result.z_m = state_[2];
    result.vx_mps = state_[3];
    result.vy_mps = state_[4];
    result.vz_mps = state_[5];
  }
  return result;
}

PositionFilterResult PositionFilter::Step(
    const std::optional<PositionMeasurement>& measurement,
    double dt_seconds) {
  if (initialized_ && (!std::isfinite(dt_seconds) || dt_seconds <= 0.0 ||
                       dt_seconds > kMaximumDtSeconds)) {
    Reset();
    if (measurement) {
      Initialize(*measurement);
      return Current(PositionFilterState::kInitialized, 0.0);
    }
    return Current(PositionFilterState::kReset, 0.0);
  }
  if (!initialized_) {
    if (!measurement) return Current(PositionFilterState::kUninitialized, 0.0);
    Initialize(*measurement);
    return Current(PositionFilterState::kInitialized, 0.0);
  }

  Predict(dt_seconds);
  if (!measurement) return Current(PositionFilterState::kPredicted, 0.0);
  double nis = std::numeric_limits<double>::infinity();
  if (!Update(*measurement, nis)) {
    return Current(PositionFilterState::kRejected, nis);
  }
  return Current(PositionFilterState::kUpdated, nis);
}
