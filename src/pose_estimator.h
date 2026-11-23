#pragma once

#include <array>
#include <string>

// 相机标定参数必须与实际相机、分辨率和镜头一一对应，不能跨相机复用。
struct CameraCalibration {
  bool enabled = false;
  int image_width = 0;
  int image_height = 0;
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  std::array<double, 5> distortion{};  // k1, k2, p1, p2, k3
  double armor_width_m = 0.0;
  double armor_height_m = 0.0;
  double max_reprojection_error_px = 0.0;
};

struct PoseResult {
  bool valid = false;
  double x_m = 0.0;
  double y_m = 0.0;
  double z_m = 0.0;
  double reprojection_error_px = 0.0;
  std::string error;
};

// 读取简单的key=value标定文件；失败时返回false并说明具体原因。
bool LoadCameraCalibration(const std::string& path,
                           CameraCalibration& calibration,
                           std::string& error);

class PoseEstimator {
 public:
  PoseEstimator(CameraCalibration calibration, int frame_width,
                int frame_height);

  bool enabled() const { return enabled_; }
  const std::string& status() const { return status_; }

  // image_points顺序固定为左上、右上、右下、左下，单位为原图像素。
  PoseResult Solve(const std::array<float, 8>& image_points) const;

 private:
  CameraCalibration calibration_;
  bool enabled_ = false;
  std::string status_;
};
