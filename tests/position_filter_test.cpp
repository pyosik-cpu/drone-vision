#include "position_filter.h"

#include <cmath>
#include <iostream>
#include <optional>

int main() {
  PositionFilter filter;
  PositionFilterResult result;
  for (int frame = 0; frame < 120; ++frame) {
    const double noise = static_cast<double>((frame % 7) - 3) * 0.005;
    result = filter.Step(PositionMeasurement{0.1 + noise, -0.05, 1.0 + noise},
                         1.0 / 30.0);
  }
  std::cout << "steady state=" << PositionFilterStateName(result.state)
            << " xyz=" << result.x_m << ',' << result.y_m << ',' << result.z_m
            << " velocity=" << result.vx_mps << ',' << result.vy_mps << ','
            << result.vz_mps << " nis=" << result.nis << '\n';
  if (!result.valid || std::abs(result.z_m - 1.0) > 0.03) return 1;

  const PositionFilterResult rejected =
      filter.Step(PositionMeasurement{2.0, -2.0, 4.0}, 1.0 / 30.0);
  std::cout << "outlier state=" << PositionFilterStateName(rejected.state)
            << " z=" << rejected.z_m << " nis=" << rejected.nis << '\n';
  if (rejected.state != PositionFilterState::kRejected || rejected.z_m > 1.2) {
    return 2;
  }

  const PositionFilterResult predicted = filter.Step(std::nullopt, 1.0 / 30.0);
  std::cout << "missing state=" << PositionFilterStateName(predicted.state)
            << " z=" << predicted.z_m << '\n';
  if (predicted.state != PositionFilterState::kPredicted) return 3;
  return 0;
}
