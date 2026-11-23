#include "pose_estimator.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <unordered_map>

namespace {

std::string Trim(std::string text) {
  const auto is_space = [](unsigned char value) { return std::isspace(value); };
  text.erase(text.begin(), std::find_if_not(text.begin(), text.end(), is_space));
  text.erase(std::find_if_not(text.rbegin(), text.rend(), is_space).base(),
             text.end());
  return text;
}

bool ParseDouble(const std::unordered_map<std::string, std::string>& values,
                 const char* key, double& output, std::string& error) {
  const auto iterator = values.find(key);
  if (iterator == values.end()) {
    error = std::string("缺少配置项: ") + key;
    return false;
  }
  char* end = nullptr;
  output = std::strtod(iterator->second.c_str(), &end);
  if (end == iterator->second.c_str() || *end != '\0' ||
      !std::isfinite(output)) {
    error = std::string("配置项不是有效数字: ") + key;
    return false;
  }
  return true;
}

using Vec3 = std::array<double, 3>;
using Mat3 = std::array<std::array<double, 3>, 3>;

double Norm(const Vec3& value) {
  return std::sqrt(value[0] * value[0] + value[1] * value[1] +
                   value[2] * value[2]);
}

Vec3 Cross(const Vec3& lhs, const Vec3& rhs) {
  return {lhs[1] * rhs[2] - lhs[2] * rhs[1],
          lhs[2] * rhs[0] - lhs[0] * rhs[2],
          lhs[0] * rhs[1] - lhs[1] * rhs[0]};
}

double Dot(const Vec3& lhs, const Vec3& rhs) {
  return lhs[0] * rhs[0] + lhs[1] * rhs[1] + lhs[2] * rhs[2];
}

bool Normalize(Vec3& value) {
  const double norm = Norm(value);
  if (!std::isfinite(norm) || norm < 1e-12) return false;
  for (double& item : value) item /= norm;
  return true;
}

// 带部分选主元的高斯消元。矩阵很小，避免为PnP引入额外线性代数库。
template <std::size_t N>
bool SolveLinear(std::array<std::array<double, N>, N> matrix,
                 std::array<double, N> values,
                 std::array<double, N>& solution) {
  for (std::size_t column = 0; column < N; ++column) {
    std::size_t pivot = column;
    for (std::size_t row = column + 1; row < N; ++row) {
      if (std::abs(matrix[row][column]) >
          std::abs(matrix[pivot][column])) {
        pivot = row;
      }
    }
    if (!std::isfinite(matrix[pivot][column]) ||
        std::abs(matrix[pivot][column]) < 1e-12) {
      return false;
    }
    std::swap(matrix[pivot], matrix[column]);
    std::swap(values[pivot], values[column]);
    const double divisor = matrix[column][column];
    for (std::size_t item = column; item < N; ++item) {
      matrix[column][item] /= divisor;
    }
    values[column] /= divisor;
    for (std::size_t row = 0; row < N; ++row) {
      if (row == column) continue;
      const double factor = matrix[row][column];
      for (std::size_t item = column; item < N; ++item) {
        matrix[row][item] -= factor * matrix[column][item];
      }
      values[row] -= factor * values[column];
    }
  }
  solution = values;
  return true;
}

// 从像素坐标迭代消除Brown-Conrady畸变，返回归一化相机坐标。
bool UndistortPoint(double u, double v, const CameraCalibration& calibration,
                    double& x, double& y) {
  const double distorted_x = (u - calibration.cx) / calibration.fx;
  const double distorted_y = (v - calibration.cy) / calibration.fy;
  x = distorted_x;
  y = distorted_y;
  const double k1 = calibration.distortion[0];
  const double k2 = calibration.distortion[1];
  const double p1 = calibration.distortion[2];
  const double p2 = calibration.distortion[3];
  const double k3 = calibration.distortion[4];
  for (int iteration = 0; iteration < 10; ++iteration) {
    const double r2 = x * x + y * y;
    const double radial = 1.0 + k1 * r2 + k2 * r2 * r2 +
                          k3 * r2 * r2 * r2;
    if (!std::isfinite(radial) || std::abs(radial) < 1e-9) return false;
    const double tangential_x = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
    const double tangential_y = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
    x = (distorted_x - tangential_x) / radial;
    y = (distorted_y - tangential_y) / radial;
  }
  return std::isfinite(x) && std::isfinite(y);
}

Mat3 Rodrigues(const Vec3& rotation) {
  const double angle = Norm(rotation);
  Mat3 result{{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}};
  if (angle < 1e-12) return result;
  const Vec3 axis{rotation[0] / angle, rotation[1] / angle,
                  rotation[2] / angle};
  const double cosine = std::cos(angle);
  const double sine = std::sin(angle);
  const double one_minus_cosine = 1.0 - cosine;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      result[row][column] =
          cosine * (row == column ? 1.0 : 0.0) +
          one_minus_cosine * axis[row] * axis[column];
    }
  }
  result[0][1] -= sine * axis[2];
  result[0][2] += sine * axis[1];
  result[1][0] += sine * axis[2];
  result[1][2] -= sine * axis[0];
  result[2][0] -= sine * axis[1];
  result[2][1] += sine * axis[0];
  return result;
}

Vec3 RotationVector(const Mat3& matrix) {
  const double cosine = std::clamp(
      (matrix[0][0] + matrix[1][1] + matrix[2][2] - 1.0) * 0.5,
      -1.0, 1.0);
  const double angle = std::acos(cosine);
  if (angle < 1e-9) {
    return {(matrix[2][1] - matrix[1][2]) * 0.5,
            (matrix[0][2] - matrix[2][0]) * 0.5,
            (matrix[1][0] - matrix[0][1]) * 0.5};
  }
  const double scale = angle / (2.0 * std::sin(angle));
  return {(matrix[2][1] - matrix[1][2]) * scale,
          (matrix[0][2] - matrix[2][0]) * scale,
          (matrix[1][0] - matrix[0][1]) * scale};
}

bool Project(const std::array<double, 6>& pose, double object_x,
             double object_y, const CameraCalibration& calibration,
             double& u, double& v) {
  const Mat3 rotation = Rodrigues({pose[0], pose[1], pose[2]});
  const double camera_x = rotation[0][0] * object_x +
                          rotation[0][1] * object_y + pose[3];
  const double camera_y = rotation[1][0] * object_x +
                          rotation[1][1] * object_y + pose[4];
  const double camera_z = rotation[2][0] * object_x +
                          rotation[2][1] * object_y + pose[5];
  if (!std::isfinite(camera_z) || camera_z <= 1e-6) return false;
  const double x = camera_x / camera_z;
  const double y = camera_y / camera_z;
  const double r2 = x * x + y * y;
  const double radial = 1.0 + calibration.distortion[0] * r2 +
                        calibration.distortion[1] * r2 * r2 +
                        calibration.distortion[4] * r2 * r2 * r2;
  const double distorted_x =
      x * radial + 2.0 * calibration.distortion[2] * x * y +
      calibration.distortion[3] * (r2 + 2.0 * x * x);
  const double distorted_y =
      y * radial + calibration.distortion[2] * (r2 + 2.0 * y * y) +
      2.0 * calibration.distortion[3] * x * y;
  u = calibration.fx * distorted_x + calibration.cx;
  v = calibration.fy * distorted_y + calibration.cy;
  return std::isfinite(u) && std::isfinite(v);
}

bool Residuals(const std::array<double, 6>& pose,
               const std::array<std::array<double, 2>, 4>& object_points,
               const std::array<float, 8>& image_points,
               const CameraCalibration& calibration,
               std::array<double, 8>& residuals) {
  for (int point = 0; point < 4; ++point) {
    double u = 0.0;
    double v = 0.0;
    if (!Project(pose, object_points[point][0], object_points[point][1],
                 calibration, u, v)) {
      return false;
    }
    residuals[point * 2] = static_cast<double>(image_points[point * 2]) - u;
    residuals[point * 2 + 1] =
        static_cast<double>(image_points[point * 2 + 1]) - v;
  }
  return true;
}

double SquaredCost(const std::array<double, 8>& residuals) {
  double cost = 0.0;
  for (double residual : residuals) cost += residual * residual;
  return cost;
}

// 阻尼Gauss-Newton细化位姿。只有误差下降时才接受更新，避免近正视平面
// 的病态初值把解推向较差的局部极小值。
bool RefinePose(const std::array<std::array<double, 2>, 4>& object_points,
                const std::array<float, 8>& image_points,
                const CameraCalibration& calibration,
                std::array<double, 6>& pose, double& squared_cost) {
  std::array<double, 8> residuals{};
  if (!Residuals(pose, object_points, image_points, calibration, residuals)) {
    return false;
  }
  squared_cost = SquaredCost(residuals);
  double damping = 1e-4;
  for (int iteration = 0; iteration < 30; ++iteration) {
    std::array<std::array<double, 6>, 8> jacobian{};
    for (int parameter = 0; parameter < 6; ++parameter) {
      std::array<double, 6> shifted = pose;
      const double step = parameter < 3 ? 1e-6 : 1e-6;
      shifted[parameter] += step;
      std::array<double, 8> shifted_residuals{};
      if (!Residuals(shifted, object_points, image_points, calibration,
                     shifted_residuals)) {
        return false;
      }
      for (int residual = 0; residual < 8; ++residual) {
        jacobian[residual][parameter] =
            (shifted_residuals[residual] - residuals[residual]) / step;
      }
    }
    std::array<std::array<double, 6>, 6> normal_matrix{};
    std::array<double, 6> normal_values{};
    for (int row = 0; row < 6; ++row) {
      for (int column = 0; column < 6; ++column) {
        for (int residual = 0; residual < 8; ++residual) {
          normal_matrix[row][column] +=
              jacobian[residual][row] * jacobian[residual][column];
        }
      }
      normal_matrix[row][row] += damping;
      for (int residual = 0; residual < 8; ++residual) {
        normal_values[row] -= jacobian[residual][row] * residuals[residual];
      }
    }
    std::array<double, 6> update{};
    if (!SolveLinear(normal_matrix, normal_values, update)) return false;
    std::array<double, 6> candidate = pose;
    double update_norm = 0.0;
    for (int parameter = 0; parameter < 6; ++parameter) {
      candidate[parameter] += update[parameter];
      update_norm += update[parameter] * update[parameter];
    }
    std::array<double, 8> candidate_residuals{};
    if (!Residuals(candidate, object_points, image_points, calibration,
                   candidate_residuals)) {
      damping *= 10.0;
      continue;
    }
    const double candidate_cost = SquaredCost(candidate_residuals);
    if (candidate_cost < squared_cost) {
      pose = candidate;
      residuals = candidate_residuals;
      squared_cost = candidate_cost;
      damping = std::max(1e-12, damping * 0.3);
      if (std::sqrt(update_norm) < 1e-9) break;
    } else {
      damping *= 10.0;
    }
  }
  return std::isfinite(squared_cost) && pose[5] > 0.0;
}

}  // namespace

bool LoadCameraCalibration(const std::string& path,
                           CameraCalibration& calibration,
                           std::string& error) {
  std::ifstream input(path);
  if (!input) {
    error = "无法打开标定文件: " + path;
    return false;
  }

  std::unordered_map<std::string, std::string> values;
  std::string line;
  int line_number = 0;
  while (std::getline(input, line)) {
    ++line_number;
    const std::size_t comment = line.find('#');
    if (comment != std::string::npos) line.erase(comment);
    line = Trim(line);
    if (line.empty()) continue;
    const std::size_t equal = line.find('=');
    if (equal == std::string::npos) {
      error = "标定文件第" + std::to_string(line_number) + "行缺少等号";
      return false;
    }
    const std::string key = Trim(line.substr(0, equal));
    const std::string value = Trim(line.substr(equal + 1));
    if (key.empty() || value.empty()) {
      error = "标定文件第" + std::to_string(line_number) + "行为空键或空值";
      return false;
    }
    values[key] = value;
  }

  const auto enabled = values.find("enabled");
  if (enabled == values.end() ||
      (enabled->second != "true" && enabled->second != "false")) {
    error = "enabled必须是true或false";
    return false;
  }
  calibration.enabled = enabled->second == "true";
  if (!calibration.enabled) return true;

  double width = 0.0;
  double height = 0.0;
  if (!ParseDouble(values, "image_width", width, error) ||
      !ParseDouble(values, "image_height", height, error) ||
      !ParseDouble(values, "fx", calibration.fx, error) ||
      !ParseDouble(values, "fy", calibration.fy, error) ||
      !ParseDouble(values, "cx", calibration.cx, error) ||
      !ParseDouble(values, "cy", calibration.cy, error) ||
      !ParseDouble(values, "k1", calibration.distortion[0], error) ||
      !ParseDouble(values, "k2", calibration.distortion[1], error) ||
      !ParseDouble(values, "p1", calibration.distortion[2], error) ||
      !ParseDouble(values, "p2", calibration.distortion[3], error) ||
      !ParseDouble(values, "k3", calibration.distortion[4], error) ||
      !ParseDouble(values, "armor_width_m", calibration.armor_width_m, error) ||
      !ParseDouble(values, "armor_height_m", calibration.armor_height_m, error) ||
      !ParseDouble(values, "max_reprojection_error_px",
                   calibration.max_reprojection_error_px, error)) {
    return false;
  }
  calibration.image_width = static_cast<int>(width);
  calibration.image_height = static_cast<int>(height);
  if (width != calibration.image_width || height != calibration.image_height ||
      calibration.image_width <= 0 || calibration.image_height <= 0 ||
      calibration.fx <= 0.0 || calibration.fy <= 0.0 ||
      calibration.armor_width_m <= 0.0 || calibration.armor_height_m <= 0.0 ||
      calibration.max_reprojection_error_px <= 0.0) {
    error = "标定尺寸、焦距、装甲板尺寸和重投影阈值必须为有效正数";
    return false;
  }
  return true;
}

PoseEstimator::PoseEstimator(CameraCalibration calibration, int frame_width,
                             int frame_height)
    : calibration_(calibration) {
  if (!calibration_.enabled) {
    status_ = "配置已禁用";
    return;
  }
  if (calibration_.image_width != frame_width ||
      calibration_.image_height != frame_height) {
    status_ = "标定分辨率与传感器输出分辨率不一致";
    return;
  }

  enabled_ = true;
  status_ = "已启用内置平面PnP后端";
}

PoseResult PoseEstimator::Solve(
    const std::array<float, 8>& image_points) const {
  PoseResult result;
  if (!enabled_) {
    result.error = status_;
    return result;
  }
  for (float coordinate : image_points) {
    if (!std::isfinite(coordinate)) {
      result.error = "角点包含无效数值";
      return result;
    }
  }

  const double half_width = calibration_.armor_width_m * 0.5;
  const double half_height = calibration_.armor_height_m * 0.5;
  // 坐标约定：相机X向右、Y向下、Z向前；物点顺序对应左上、右上、右下、左下。
  const std::array<std::array<double, 2>, 4> object_points{{
      {-half_width, -half_height},
      {half_width, -half_height},
      {half_width, half_height},
      {-half_width, half_height},
  }};
  std::array<std::array<double, 2>, 4> normalized_points{};
  for (int point = 0; point < 4; ++point) {
    if (!UndistortPoint(image_points[point * 2], image_points[point * 2 + 1],
                        calibration_, normalized_points[point][0],
                        normalized_points[point][1])) {
      result.error = "角点去畸变失败";
      return result;
    }
  }

  // 解物平面(X,Y,1)到归一化像平面(x,y,1)的单应矩阵，h33固定为1。
  std::array<std::array<double, 8>, 8> homography_matrix{};
  std::array<double, 8> homography_values{};
  for (int point = 0; point < 4; ++point) {
    const double object_x = object_points[point][0];
    const double object_y = object_points[point][1];
    const double x = normalized_points[point][0];
    const double y = normalized_points[point][1];
    homography_matrix[point * 2] =
        {object_x, object_y, 1.0, 0.0, 0.0, 0.0,
         -x * object_x, -x * object_y};
    homography_values[point * 2] = x;
    homography_matrix[point * 2 + 1] =
        {0.0, 0.0, 0.0, object_x, object_y, 1.0,
         -y * object_x, -y * object_y};
    homography_values[point * 2 + 1] = y;
  }
  std::array<double, 8> homography{};
  if (!SolveLinear(homography_matrix, homography_values, homography)) {
    result.error = "四角点退化，无法计算单应矩阵";
    return result;
  }

  Vec3 first_column{homography[0], homography[3], homography[6]};
  Vec3 second_column{homography[1], homography[4], homography[7]};
  Vec3 translation{homography[2], homography[5], 1.0};
  const double scale = 2.0 / (Norm(first_column) + Norm(second_column));
  if (!std::isfinite(scale)) {
    result.error = "单应矩阵尺度无效";
    return result;
  }
  for (double& value : first_column) value *= scale;
  for (double& value : second_column) value *= scale;
  for (double& value : translation) value *= scale;
  if (translation[2] < 0.0) {
    for (double& value : first_column) value = -value;
    for (double& value : second_column) value = -value;
    for (double& value : translation) value = -value;
  }

  // 单应矩阵受噪声影响不完全正交，使用Gram-Schmidt恢复合法旋转矩阵。
  if (!Normalize(first_column)) {
    result.error = "旋转第一列无效";
    return result;
  }
  const double projection = Dot(first_column, second_column);
  for (int item = 0; item < 3; ++item) {
    second_column[item] -= projection * first_column[item];
  }
  if (!Normalize(second_column)) {
    result.error = "旋转第二列无效";
    return result;
  }
  Vec3 third_column = Cross(first_column, second_column);
  if (!Normalize(third_column)) {
    result.error = "旋转第三列无效";
    return result;
  }
  const Mat3 initial_rotation{{
      {first_column[0], second_column[0], third_column[0]},
      {first_column[1], second_column[1], third_column[1]},
      {first_column[2], second_column[2], third_column[2]},
  }};
  const Vec3 rotation_vector = RotationVector(initial_rotation);
  const std::array<double, 6> homography_pose{
      rotation_vector[0], rotation_vector[1], rotation_vector[2],
      translation[0], translation[1], translation[2]};
  // 平面目标存在两个相近的姿态分支。对倾斜符号和正视姿态进行确定性的
  // 多初值优化，最后选择原始像素重投影误差最小的前向解。
  std::array<std::array<double, 6>, 5> seeds{};
  seeds[0] = homography_pose;
  seeds[1] = homography_pose;
  seeds[1][0] = -seeds[1][0];
  seeds[2] = homography_pose;
  seeds[2][1] = -seeds[2][1];
  seeds[3] = homography_pose;
  seeds[3][0] = -seeds[3][0];
  seeds[3][1] = -seeds[3][1];
  seeds[4] = homography_pose;
  seeds[4][0] = 0.0;
  seeds[4][1] = 0.0;

  std::array<double, 6> pose{};
  double squared_error = std::numeric_limits<double>::infinity();
  bool refined = false;
  for (auto seed : seeds) {
    double candidate_error = 0.0;
    if (RefinePose(object_points, image_points, calibration_, seed,
                   candidate_error) && candidate_error < squared_error) {
      pose = seed;
      squared_error = candidate_error;
      refined = true;
    }
  }
  if (!refined) {
    result.error = "PnP迭代未得到相机前方的有效解";
    return result;
  }
  result.reprojection_error_px = std::sqrt(squared_error / 4.0);
  if (!std::isfinite(result.reprojection_error_px) ||
      result.reprojection_error_px > calibration_.max_reprojection_error_px) {
    result.error = "重投影误差超过阈值";
    return result;
  }
  result.valid = true;
  result.x_m = pose[3];
  result.y_m = pose[4];
  result.z_m = pose[5];
  return result;
}
