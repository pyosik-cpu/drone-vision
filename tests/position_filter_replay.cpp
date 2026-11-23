#include "position_filter.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <regex>
#include <string>
#include <vector>

double StandardDeviation(const std::vector<double>& values) {
  double sum = 0.0;
  double squared_sum = 0.0;
  for (double value : values) {
    sum += value;
    squared_sum += value * value;
  }
  const double mean = sum / values.size();
  return std::sqrt(std::max(0.0, squared_sum / values.size() - mean * mean));
}

int main(int argc, char** argv) {
  if (argc != 2) return 2;
  std::ifstream input(argv[1]);
  if (!input) return 3;
  const std::regex pattern(
      R"(^frame_id=.*dt_us=([0-9]+).*pose_valid=1 x_m=([-0-9.]+) y_m=([-0-9.]+) z_m=([-0-9.]+))");
  PositionFilter filter;
  std::vector<double> raw_z;
  std::vector<double> filtered_z;
  int rejected = 0;
  std::string line;
  while (std::getline(input, line)) {
    std::smatch match;
    if (!std::regex_search(line, match, pattern)) continue;
    const double dt = std::stod(match[1]) * 1e-6;
    const PositionMeasurement measurement{
        std::stod(match[2]), std::stod(match[3]), std::stod(match[4])};
    const PositionFilterResult result = filter.Step(measurement, dt);
    raw_z.push_back(measurement.z_m);
    if (result.valid) filtered_z.push_back(result.z_m);
    if (result.state == PositionFilterState::kRejected) ++rejected;
  }
  if (raw_z.empty() || filtered_z.empty()) return 4;
  std::cout << "samples=" << raw_z.size()
            << " raw_z_std_m=" << StandardDeviation(raw_z)
            << " filtered_z_std_m=" << StandardDeviation(filtered_z)
            << " rejected=" << rejected << '\n';
  return 0;
}
