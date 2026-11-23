#include "pose_estimator.h"

#include <array>
#include <cmath>
#include <iostream>

namespace {

using Mat3 = std::array<std::array<double, 3>, 3>;

Mat3 Rodrigues(const std::array<double, 3>& vector) {
  const double angle = std::sqrt(vector[0] * vector[0] +
                                 vector[1] * vector[1] +
                                 vector[2] * vector[2]);
  Mat3 result{{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}};
  const std::array<double, 3> axis{vector[0] / angle, vector[1] / angle,
                                   vector[2] / angle};
  const double cosine = std::cos(angle);
  const double sine = std::sin(angle);
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      result[row][column] = cosine * (row == column ? 1.0 : 0.0) +
          (1.0 - cosine) * axis[row] * axis[column];
    }
  }
  result[0][1] -= sine * axis[2]; result[0][2] += sine * axis[1];
  result[1][0] += sine * axis[2]; result[1][2] -= sine * axis[0];
  result[2][0] -= sine * axis[1]; result[2][1] += sine * axis[0];
  return result;
}

std::array<float, 8> MakeImagePoints(const CameraCalibration& calibration) {
  const Mat3 rotation = Rodrigues({0.12, -0.18, 0.04});
  const std::array<double, 3> translation{0.035, -0.018, 1.42};
  const double half_width = calibration.armor_width_m * 0.5;
  const double half_height = calibration.armor_height_m * 0.5;
  const std::array<std::array<double, 2>, 4> object_points{{
      {-half_width, -half_height}, {half_width, -half_height},
      {half_width, half_height}, {-half_width, half_height}}};
  std::array<float, 8> image_points{};
  for (int point = 0; point < 4; ++point) {
    const double object_x = object_points[point][0];
    const double object_y = object_points[point][1];
    const double camera_x = rotation[0][0] * object_x +
                            rotation[0][1] * object_y + translation[0];
    const double camera_y = rotation[1][0] * object_x +
                            rotation[1][1] * object_y + translation[1];
    const double camera_z = rotation[2][0] * object_x +
                            rotation[2][1] * object_y + translation[2];
    const double x = camera_x / camera_z;
    const double y = camera_y / camera_z;
    const double r2 = x * x + y * y;
    const double radial = 1.0 + calibration.distortion[0] * r2 +
        calibration.distortion[1] * r2 * r2 +
        calibration.distortion[4] * r2 * r2 * r2;
    const double xd = x * radial + 2.0 * calibration.distortion[2] * x * y +
        calibration.distortion[3] * (r2 + 2.0 * x * x);
    const double yd = y * radial + calibration.distortion[2] *
        (r2 + 2.0 * y * y) + 2.0 * calibration.distortion[3] * x * y;
    image_points[point * 2] =
        static_cast<float>(calibration.fx * xd + calibration.cx);
    image_points[point * 2 + 1] =
        static_cast<float>(calibration.fy * yd + calibration.cy);
  }
  return image_points;
}

}  // namespace

int main() {
  CameraCalibration calibration;
  std::string error;
  if (!LoadCameraCalibration("calibration/camera_calibration.ini", calibration,
                             error)) {
    std::cerr << error << '\n';
    return 1;
  }
  PoseEstimator estimator(calibration, 1920, 1080);
  const PoseResult result = estimator.Solve(MakeImagePoints(calibration));
  std::cout << "valid=" << result.valid << " x=" << result.x_m
            << " y=" << result.y_m << " z=" << result.z_m
            << " error_px=" << result.reprojection_error_px << '\n';
  if (!result.valid || std::abs(result.x_m - 0.035) > 1e-3 ||
      std::abs(result.y_m + 0.018) > 1e-3 ||
      std::abs(result.z_m - 1.42) > 2e-3) {
    std::cerr << "PnP合成数据验证失败: " << result.error << '\n';
    return 2;
  }

  // 来自GC2083实时检测日志的角点，用于覆盖近距离、大透视四边形。
  const std::array<std::array<float, 8>, 3> recorded_points{{
      {510.0F, 255.0F, 1170.0F, 249.0F, 1200.0F, 504.0F, 534.0F, 504.0F},
      {648.0F, 300.0F, 1284.0F, 246.0F, 1308.0F, 480.0F, 666.0F, 534.0F},
      {714.0F, 336.0F, 1332.0F, 282.0F, 1362.0F, 534.0F, 738.0F, 594.0F},
  }};
  for (const auto& points : recorded_points) {
    const PoseResult recorded = estimator.Solve(points);
    std::cout << "recorded valid=" << recorded.valid << " x=" << recorded.x_m
              << " y=" << recorded.y_m << " z=" << recorded.z_m
              << " error_px=" << recorded.reprojection_error_px
              << " reason=" << recorded.error << '\n';
  }
  calibration.max_reprojection_error_px = 20.0;
  PoseEstimator relaxed_estimator(calibration, 1920, 1080);
  for (const auto& points : recorded_points) {
    const PoseResult recorded = relaxed_estimator.Solve(points);
    std::cout << "relaxed valid=" << recorded.valid << " x=" << recorded.x_m
              << " y=" << recorded.y_m << " z=" << recorded.z_m
              << " error_px=" << recorded.reprojection_error_px << '\n';
  }
  return 0;
}
