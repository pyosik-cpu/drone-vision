#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "cviruntime.h"
#include "pose_estimator.h"
#include "position_filter.h"
#include "serial_transport.h"

extern "C" {
#include "cvi_sys.h"
#include "middleware_utils.h"
#include "sample_comm.h"
#include "vpss_helper.h"
}

namespace {

constexpr int kInputWidth = 320;
constexpr int kInputHeight = 320;
constexpr int kValuesPerCandidate = 22;
constexpr int kNotArmorClass = 8;
constexpr VPSS_GRP kVpssGroup = 0;
constexpr VPSS_CHN kStreamChannel = VPSS_CHN0;
constexpr VPSS_CHN kInferenceChannel = VPSS_CHN1;
constexpr CVI_U32 kStreamWidth = 1280;
constexpr CVI_U32 kStreamHeight = 720;

std::atomic_bool g_running{true};
std::atomic_uint64_t g_stream_frames{0};

struct Detection {
  // 四角点顺序：左上、右上、右下、左下。
  std::array<float, 8> points{};
  int color_id = -1;
  int number_id = -1;
  float confidence = 0.0F;
  bool border_clipped = false;
};

// 在识别线程和推流线程之间传递映射到1280x720画面的四角点。
class StreamTargetOverlay {
 public:
  StreamTargetOverlay() = default;
  StreamTargetOverlay(const StreamTargetOverlay&) = delete;
  StreamTargetOverlay& operator=(const StreamTargetOverlay&) = delete;

  void Initialize(int source_width, int source_height) {
    source_width_ = source_width;
    source_height_ = source_height;
    ready_ = source_width > 0 && source_height > 0;
  }

  void Update(const std::optional<Detection>& target) {
    std::optional<std::array<int, 8>> mapped_points;
    if (target) {
      mapped_points.emplace();
      const float scale =
          std::min(static_cast<float>(kStreamWidth) / source_width_,
                   static_cast<float>(kStreamHeight) / source_height_);
      const float pad_x = (kStreamWidth - source_width_ * scale) * 0.5F;
      const float pad_y = (kStreamHeight - source_height_ * scale) * 0.5F;
      for (int point = 0; point < 4; ++point) {
        int x = static_cast<int>(std::lround(
            target->points[point * 2] * scale + pad_x));
        int y = static_cast<int>(std::lround(
            target->points[point * 2 + 1] * scale + pad_y));
        x = std::clamp(x, 0, static_cast<int>(kStreamWidth) - 1);
        y = std::clamp(y, 0, static_cast<int>(kStreamHeight) - 1);
        (*mapped_points)[point * 2] = x;
        (*mapped_points)[point * 2 + 1] = y;
      }
    }
    std::lock_guard<std::mutex> lock(mutex_);
    points_ = mapped_points;
  }

  std::optional<std::array<int, 8>> Snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return points_;
  }

  void Destroy() {
    std::lock_guard<std::mutex> lock(mutex_);
    points_.reset();
    ready_ = false;
  }

  bool ready() const { return ready_; }

 private:
  mutable std::mutex mutex_;
  std::optional<std::array<int, 8>> points_;
  int source_width_ = 0;
  int source_height_ = 0;
  bool ready_ = false;
};

// 这是模型输出顺序，不等同于Hz项目内部Color枚举的数值顺序。
enum class ModelColor : int {
  kBlue = 0,
  kRed = 1,
  kExtinguish = 2,
  kPurple = 3,
};

const char* ModelColorName(int color_id) {
  switch (static_cast<ModelColor>(color_id)) {
    case ModelColor::kBlue: return "blue";
    case ModelColor::kRed: return "red";
    case ModelColor::kExtinguish: return "extinguish";
    case ModelColor::kPurple: return "purple";
  }
  return "unknown";
}

bool ParseEnemyColor(const std::string& name, int& color_id) {
  if (name == "any") color_id = -1;
  else if (name == "blue") color_id = static_cast<int>(ModelColor::kBlue);
  else if (name == "red") color_id = static_cast<int>(ModelColor::kRed);
  else if (name == "extinguish") color_id = static_cast<int>(ModelColor::kExtinguish);
  else return false;
  return true;
}

struct FrameTiming {
  std::uint64_t frame_id = 0;
  std::int64_t capture_pts_us = 0;
  std::int64_t received_time_us = 0;
  std::int64_t pipeline_latency_us = -1;
  std::int64_t dt_us = 0;
  bool uses_hardware_pts = false;
};

struct FrameResult {
  FrameTiming timing;
  std::vector<Detection> detections;
};

enum class TrackingState { kLost, kDetecting, kTracking, kTempLost };

const char* TrackingStateName(TrackingState state) {
  switch (state) {
    case TrackingState::kLost: return "lost";
    case TrackingState::kDetecting: return "detecting";
    case TrackingState::kTracking: return "tracking";
    case TrackingState::kTempLost: return "temp_lost";
  }
  return "unknown";
}

struct SelectionResult {
  std::optional<Detection> target;
  std::size_t valid_count = 0;
  TrackingState state = TrackingState::kLost;
  bool state_changed = false;
};

struct LetterboxInfo {
  float scale;
  float pad_x;
  float pad_y;
};

// VPSS使用固定VB缓冲池，物理地址会循环复用。缓存非缓存映射，避免每帧重复
// mmap/munmap产生约数十毫秒的系统调用开销；必须在CVI_SYS_Exit前清理。
class FrameMappingCache {
 public:
  FrameMappingCache() = default;
  FrameMappingCache(const FrameMappingCache&) = delete;
  FrameMappingCache& operator=(const FrameMappingCache&) = delete;

  std::uint8_t* Get(CVI_U64 physical_address, CVI_U32 length) {
    for (const Mapping& mapping : mappings_) {
      if (mapping.physical_address == physical_address &&
          mapping.length >= length) {
        return mapping.virtual_address;
      }
    }
    auto* address = static_cast<std::uint8_t*>(
        CVI_SYS_MmapCache(physical_address, length));
    if (address == nullptr) return nullptr;
    mappings_.push_back({physical_address, length, address});
    return address;
  }

  bool Clear() {
    bool success = true;
    for (const Mapping& mapping : mappings_) {
      if (CVI_SYS_Munmap(mapping.virtual_address, mapping.length) != CVI_SUCCESS) {
        success = false;
      }
    }
    mappings_.clear();
    return success;
  }

  std::size_t size() const { return mappings_.size(); }

 private:
  struct Mapping {
    CVI_U64 physical_address;
    CVI_U32 length;
    std::uint8_t* virtual_address;
  };
  std::vector<Mapping> mappings_;
};

using Clock = std::chrono::steady_clock;

double Milliseconds(Clock::time_point begin, Clock::time_point end) {
  return std::chrono::duration<double, std::milli>(end - begin).count();
}

void HandleSignal(int) { g_running.store(false); }

void DrawLumaPoint(std::uint8_t* luma, int stride, int width, int height,
                   int center_x, int center_y) {
  constexpr int kRadius = 2;
  for (int y = std::max(0, center_y - kRadius);
       y <= std::min(height - 1, center_y + kRadius); ++y) {
    for (int x = std::max(0, center_x - kRadius);
         x <= std::min(width - 1, center_x + kRadius); ++x) {
      luma[y * stride + x] = 235;
    }
  }
}

void DrawLumaLine(std::uint8_t* luma, int stride, int width, int height,
                  int x0, int y0, int x1, int y1) {
  const int delta_x = std::abs(x1 - x0);
  const int step_x = x0 < x1 ? 1 : -1;
  const int delta_y = -std::abs(y1 - y0);
  const int step_y = y0 < y1 ? 1 : -1;
  int error = delta_x + delta_y;
  while (true) {
    DrawLumaPoint(luma, stride, width, height, x0, y0);
    if (x0 == x1 && y0 == y1) break;
    const int doubled_error = error * 2;
    if (doubled_error >= delta_y) {
      error += delta_y;
      x0 += step_x;
    }
    if (doubled_error <= delta_x) {
      error += delta_x;
      y0 += step_y;
    }
  }
}

bool DrawTargetOnStreamFrame(VIDEO_FRAME_INFO_S& frame,
                             const StreamTargetOverlay& overlay,
                             FrameMappingCache& mapping_cache) {
  const auto points = overlay.Snapshot();
  if (!points) return true;

  VIDEO_FRAME_S& video = frame.stVFrame;
  if (video.enPixelFormat != PIXEL_FORMAT_NV21 &&
      video.enPixelFormat != PIXEL_FORMAT_NV12) {
    return false;
  }
  const CVI_U32 luma_length = video.u32Length[0];
  auto* luma = mapping_cache.Get(video.u64PhyAddr[0], luma_length);
  if (luma == nullptr) return false;
  if (CVI_SYS_IonInvalidateCache(video.u64PhyAddr[0], luma, luma_length) !=
      CVI_SUCCESS) {
    return false;
  }

  const int width = static_cast<int>(video.u32Width);
  const int height = static_cast<int>(video.u32Height);
  const int stride = static_cast<int>(video.u32Stride[0]);
  for (int point = 0; point < 4; ++point) {
    const int next = (point + 1) % 4;
    DrawLumaLine(luma, stride, width, height,
                 (*points)[point * 2], (*points)[point * 2 + 1],
                 (*points)[next * 2], (*points)[next * 2 + 1]);
  }
  return CVI_SYS_IonFlushCache(video.u64PhyAddr[0], luma, luma_length) ==
         CVI_SUCCESS;
}

// 推流线程独立消费VPSS CH0，使用SG2002硬件VENC编码并写入RTSP。
void StreamLoop(SAMPLE_TDL_MW_CONTEXT* context,
                const StreamTargetOverlay* overlay) {
  FrameMappingCache stream_mapping_cache;
  bool drawing_enabled = true;
  while (g_running.load()) {
    VIDEO_FRAME_INFO_S frame{};
    CVI_S32 result =
        CVI_VPSS_GetChnFrame(kVpssGroup, kStreamChannel, &frame, 2000);
    if (result != CVI_SUCCESS) {
      if (g_running.load()) {
        std::cerr << "推流通道取帧失败, CVI_S32=" << result << '\n';
        g_running.store(false);
      }
      break;
    }

    if (drawing_enabled &&
        !DrawTargetOnStreamFrame(frame, *overlay, stream_mapping_cache)) {
      std::cerr << "推流画框写入NV21帧失败，已停用画框。\n";
      drawing_enabled = false;
    }

    const CVI_S32 send_result = SAMPLE_TDL_Send_Frame_RTSP(&frame, context);
    const CVI_S32 release_result =
        CVI_VPSS_ReleaseChnFrame(kVpssGroup, kStreamChannel, &frame);
    if (send_result != CVI_SUCCESS || release_result != CVI_SUCCESS) {
      std::cerr << "H.264/RTSP推流失败, send=" << send_result
                << ", release=" << release_result << '\n';
      g_running.store(false);
      break;
    }
    g_stream_frames.fetch_add(1, std::memory_order_relaxed);
  }
  if (!stream_mapping_cache.Clear()) {
    std::cerr << "推流画框缓存映射解除失败。\n";
  }
}

float Sigmoid(float value) { return 1.0F / (1.0F + std::exp(-value)); }

void GetBounds(const Detection& detection, float& left, float& top,
               float& right, float& bottom) {
  left = right = detection.points[0];
  top = bottom = detection.points[1];
  for (int index = 1; index < 4; ++index) {
    left = std::min(left, detection.points[index * 2]);
    right = std::max(right, detection.points[index * 2]);
    top = std::min(top, detection.points[index * 2 + 1]);
    bottom = std::max(bottom, detection.points[index * 2 + 1]);
  }
}

std::array<float, 2> DetectionCenter(const Detection& detection) {
  std::array<float, 2> center{0.0F, 0.0F};
  for (int point = 0; point < 4; ++point) {
    center[0] += detection.points[point * 2] * 0.25F;
    center[1] += detection.points[point * 2 + 1] * 0.25F;
  }
  return center;
}

float DetectionArea(const Detection& detection) {
  float twice_area = 0.0F;
  for (int point = 0; point < 4; ++point) {
    const int next = (point + 1) % 4;
    twice_area += detection.points[point * 2] *
                      detection.points[next * 2 + 1] -
                  detection.points[next * 2] *
                      detection.points[point * 2 + 1];
  }
  return std::abs(twice_area) * 0.5F;
}

bool IsConvex(const Detection& detection) {
  float previous_cross = 0.0F;
  for (int point = 0; point < 4; ++point) {
    const int next = (point + 1) % 4;
    const int after = (point + 2) % 4;
    const float ax = detection.points[next * 2] - detection.points[point * 2];
    const float ay = detection.points[next * 2 + 1] - detection.points[point * 2 + 1];
    const float bx = detection.points[after * 2] - detection.points[next * 2];
    const float by = detection.points[after * 2 + 1] - detection.points[next * 2 + 1];
    const float cross = ax * by - ay * bx;
    if (std::abs(cross) < 1.0F) return false;
    if (previous_cross != 0.0F && cross * previous_cross < 0.0F) return false;
    previous_cross = cross;
  }
  return true;
}

// 在PnP之前完成轻量目标过滤、连续确认和短时丢失处理。
class TargetSelector {
 public:
  TargetSelector(int image_width, int image_height, int enemy_color_id)
      : image_width_(image_width),
        image_height_(image_height),
        enemy_color_id_(enemy_color_id) {}

  SelectionResult Update(const std::vector<Detection>& detections) {
    const TrackingState old_state = state_;
    std::vector<Detection> valid;
    valid.reserve(detections.size());
    for (const Detection& detection : detections) {
      if (IsValid(detection)) valid.push_back(detection);
    }

    std::optional<Detection> selected;
    switch (state_) {
      case TrackingState::kLost: {
        const auto best = PickBest(valid);
        if (best) {
          pending_ = *best;
          confirmation_count_ = 1;
          state_ = TrackingState::kDetecting;
        }
        break;
      }
      case TrackingState::kDetecting: {
        const auto associated = FindAssociated(valid, pending_, kAssociationPixels);
        if (associated) {
          pending_ = *associated;
          ++confirmation_count_;
          if (confirmation_count_ >= kConfirmationFrames) {
            last_target_ = pending_;
            missed_frames_ = 0;
            state_ = TrackingState::kTracking;
            selected = last_target_;
          }
        } else {
          const auto best = PickBest(valid);
          if (best) {
            pending_ = *best;
            confirmation_count_ = 1;
          } else {
            confirmation_count_ = 0;
            state_ = TrackingState::kLost;
          }
        }
        break;
      }
      case TrackingState::kTracking: {
        const auto associated = FindAssociated(valid, last_target_, kAssociationPixels);
        if (associated) {
          last_target_ = *associated;
          missed_frames_ = 0;
          selected = last_target_;
        } else {
          missed_frames_ = 1;
          state_ = TrackingState::kTempLost;
        }
        break;
      }
      case TrackingState::kTempLost: {
        const float expanded_gate =
            kAssociationPixels + 80.0F * static_cast<float>(missed_frames_);
        const auto associated = FindAssociated(valid, last_target_, expanded_gate);
        if (associated) {
          last_target_ = *associated;
          missed_frames_ = 0;
          state_ = TrackingState::kTracking;
          selected = last_target_;
        } else {
          ++missed_frames_;
          if (missed_frames_ > kMaxTempLostFrames) {
            state_ = TrackingState::kLost;
            confirmation_count_ = 0;
          }
        }
        break;
      }
    }

    return {selected, valid.size(), state_, state_ != old_state};
  }

  TrackingState state() const { return state_; }

 private:
  bool IsValid(const Detection& detection) const {
    if (detection.border_clipped) return false;
    if (enemy_color_id_ >= 0 && detection.color_id != enemy_color_id_) return false;
    if (!IsConvex(detection) || DetectionArea(detection) < kMinimumAreaPixels) return false;
    return true;
  }

  std::optional<Detection> PickBest(
      const std::vector<Detection>& detections) const {
    if (detections.empty()) return std::nullopt;
    const float image_center_x = image_width_ * 0.5F;
    const float image_center_y = image_height_ * 0.5F;
    const float diagonal = std::hypot(static_cast<float>(image_width_),
                                      static_cast<float>(image_height_));
    const Detection* best = nullptr;
    float best_score = -1.0e9F;
    for (const Detection& detection : detections) {
      const auto center = DetectionCenter(detection);
      const float center_distance =
          std::hypot(center[0] - image_center_x, center[1] - image_center_y);
      const float score = detection.confidence - 0.15F * center_distance / diagonal;
      if (score > best_score) {
        best_score = score;
        best = &detection;
      }
    }
    return best == nullptr ? std::nullopt : std::optional<Detection>(*best);
  }

  std::optional<Detection> FindAssociated(
      const std::vector<Detection>& detections, const Detection& reference,
      float maximum_distance) const {
    const auto reference_center = DetectionCenter(reference);
    const Detection* best = nullptr;
    float best_cost = 1.0e9F;
    for (const Detection& detection : detections) {
      if (detection.color_id != reference.color_id) continue;
      const auto center = DetectionCenter(detection);
      const float distance = std::hypot(center[0] - reference_center[0],
                                        center[1] - reference_center[1]);
      if (distance > maximum_distance) continue;
      // 数字偶发跳变时不中断跟踪，只增加关联代价。
      const float number_penalty =
          detection.number_id == reference.number_id ? 0.0F : 60.0F;
      const float cost = distance + number_penalty - detection.confidence * 20.0F;
      if (cost < best_cost) {
        best_cost = cost;
        best = &detection;
      }
    }
    return best == nullptr ? std::nullopt : std::optional<Detection>(*best);
  }

  static constexpr int kConfirmationFrames = 3;
  static constexpr int kMaxTempLostFrames = 5;
  static constexpr float kAssociationPixels = 300.0F;
  static constexpr float kMinimumAreaPixels = 64.0F;

  int image_width_;
  int image_height_;
  int enemy_color_id_;
  TrackingState state_ = TrackingState::kLost;
  Detection pending_{};
  Detection last_target_{};
  int confirmation_count_ = 0;
  int missed_frames_ = 0;
};

float BoxIou(const Detection& lhs, const Detection& rhs) {
  float l1, t1, r1, b1, l2, t2, r2, b2;
  GetBounds(lhs, l1, t1, r1, b1);
  GetBounds(rhs, l2, t2, r2, b2);
  const float intersection =
      std::max(0.0F, std::min(r1, r2) - std::max(l1, l2)) *
      std::max(0.0F, std::min(b1, b2) - std::max(t1, t2));
  const float area1 = std::max(0.0F, r1 - l1) * std::max(0.0F, b1 - t1);
  const float area2 = std::max(0.0F, r2 - l2) * std::max(0.0F, b2 - t2);
  const float union_area = area1 + area2 - intersection;
  return union_area > 0.0F ? intersection / union_area : 0.0F;
}

std::vector<Detection> Decode(const float* data, std::size_t value_count,
                              const LetterboxInfo& letterbox,
                              int source_width, int source_height,
                              float confidence_threshold,
                              float nms_threshold) {
  std::vector<Detection> candidates;
  candidates.reserve(64);
  const std::size_t candidate_count = value_count / kValuesPerCandidate;
  for (std::size_t index = 0; index < candidate_count; ++index) {
    const float* row = data + index * kValuesPerCandidate;
    const float confidence = Sigmoid(row[8]);
    if (confidence < confidence_threshold) continue;

    const int color_id =
        static_cast<int>(std::max_element(row + 9, row + 13) - (row + 9));
    const int number_id =
        static_cast<int>(std::max_element(row + 13, row + 22) - (row + 13));
    if (number_id == kNotArmorClass) continue;

    Detection detection;
    detection.color_id = color_id;
    detection.number_id = number_id;
    detection.confidence = confidence;
    constexpr int kPointOrder[4] = {0, 3, 2, 1};
    for (int point = 0; point < 4; ++point) {
      const int source = kPointOrder[point];
      const float raw_x =
          (row[source * 2] - letterbox.pad_x) / letterbox.scale;
      const float raw_y =
          (row[source * 2 + 1] - letterbox.pad_y) / letterbox.scale;
      if (raw_x <= 0.0F || raw_x >= static_cast<float>(source_width - 1) ||
          raw_y <= 0.0F || raw_y >= static_cast<float>(source_height - 1)) {
        detection.border_clipped = true;
      }
      detection.points[point * 2] = std::clamp(
          raw_x, 0.0F, static_cast<float>(source_width - 1));
      detection.points[point * 2 + 1] = std::clamp(
          raw_y, 0.0F, static_cast<float>(source_height - 1));
    }
    candidates.push_back(detection);
  }

  std::sort(candidates.begin(), candidates.end(),
            [](const Detection& lhs, const Detection& rhs) {
              return lhs.confidence > rhs.confidence;
            });
  std::vector<Detection> results;
  for (const Detection& candidate : candidates) {
    const bool suppressed = std::any_of(
        results.begin(), results.end(), [&](const Detection& kept) {
          return BoxIou(candidate, kept) >= nms_threshold;
        });
    if (!suppressed) results.push_back(candidate);
  }
  return results;
}

bool HasShape(const CVI_TENSOR& tensor,
              std::initializer_list<int> dimensions) {
  if (tensor.shape.dim_size != dimensions.size()) return false;
  std::size_t index = 0;
  for (const int dimension : dimensions) {
    if (tensor.shape.dim[index++] != dimension) return false;
  }
  return true;
}

// 将VPSS输出的RGB planar字节按stride转换成模型要求的FP32/NCHW [0,1]。
bool CopyFrameToTensor(const VIDEO_FRAME_INFO_S& frame, float* destination,
                       FrameMappingCache& mapping_cache) {
  const VIDEO_FRAME_S& video = frame.stVFrame;
  if (video.u32Width != kInputWidth || video.u32Height != kInputHeight ||
      video.enPixelFormat != PIXEL_FORMAT_RGB_888_PLANAR) {
    std::cerr << "VPSS帧格式不符合320x320 RGB planar预期。\n";
    return false;
  }

  constexpr std::size_t kPlanePixels = kInputWidth * kInputHeight;
  for (int channel = 0; channel < 3; ++channel) {
    if (video.u64PhyAddr[channel] == 0 || video.u32Length[channel] == 0 ||
        video.u32Stride[channel] < kInputWidth) {
      std::cerr << "VPSS第" << channel << "平面参数无效。\n";
      return false;
    }
    std::uint8_t* plane = mapping_cache.Get(video.u64PhyAddr[channel],
                                            video.u32Length[channel]);
    if (plane == nullptr) {
      std::cerr << "映射VPSS第" << channel << "平面失败。\n";
      return false;
    }
    // VPSS由硬件写入VB，CPU读取缓存映射前必须使对应缓存失效。
    const CVI_S32 invalidate_result = CVI_SYS_IonInvalidateCache(
        video.u64PhyAddr[channel], plane, video.u32Length[channel]);
    if (invalidate_result != CVI_SUCCESS) {
      std::cerr << "刷新VPSS第" << channel
                << "平面CPU缓存失败, CVI_S32=" << invalidate_result << '\n';
      return false;
    }
    float* output = destination + channel * kPlanePixels;
    constexpr float kPixelScale = 1.0F / 255.0F;
    for (int y = 0; y < kInputHeight; ++y) {
      const std::uint8_t* row = plane + y * video.u32Stride[channel];
      for (int x = 0; x < kInputWidth; ++x) {
        output[y * kInputWidth + x] = static_cast<float>(row[x]) * kPixelScale;
      }
    }
  }
  return true;
}

LetterboxInfo MakeLetterbox(int source_width, int source_height) {
  const float scale =
      std::min(static_cast<float>(kInputWidth) / source_width,
               static_cast<float>(kInputHeight) / source_height);
  const float resized_width = std::round(source_width * scale);
  const float resized_height = std::round(source_height * scale);
  return {scale, (kInputWidth - resized_width) / 2.0F,
          (kInputHeight - resized_height) / 2.0F};
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2 || argc > 9) {
    std::cerr << "用法: " << argv[0]
              << " <模型.cvimodel> [置信度=0.5] [NMS=0.45]"
                 " [帧数=0(持续)] [敌方颜色=any] [相机标定.ini|-]"
                 " [串口设备=off] [波特率=115200]\n";
    return 64;
  }
  const float confidence_threshold =
      argc >= 3 ? std::strtof(argv[2], nullptr) : 0.5F;
  const float nms_threshold = argc >= 4 ? std::strtof(argv[3], nullptr) : 0.45F;
  const long frame_limit = argc >= 5 ? std::strtol(argv[4], nullptr, 10) : 0;
  const std::string enemy_color_name = argc >= 6 ? argv[5] : "any";
  const std::string calibration_path = argc >= 7 ? argv[6] : "";
  const std::string serial_device = argc >= 8 ? argv[7] : "off";
  const long serial_baud = argc >= 9 ? std::strtol(argv[8], nullptr, 10) : 115200;
  int enemy_color_id = -1;
  if (confidence_threshold < 0.0F || confidence_threshold > 1.0F ||
      nms_threshold < 0.0F || nms_threshold > 1.0F || frame_limit < 0 ||
      !ParseEnemyColor(enemy_color_name, enemy_color_id) || serial_baud <= 0) {
    std::cerr << "阈值必须位于0到1之间，帧数不能为负数；敌方颜色必须是"
                 " any/blue/red/extinguish/purple。\n";
    return 64;
  }

  std::signal(SIGINT, HandleSignal);
  std::signal(SIGTERM, HandleSignal);

  if (CVI_MSG_Init() != CVI_SUCCESS) {
    std::cerr << "CVI_MSG_Init失败。\n";
    return 1;
  }

  int exit_code = 0;
  SAMPLE_TDL_MW_CONFIG_S config{};
  SAMPLE_TDL_MW_CONTEXT context{};
  bool middleware_ready = false;
  CVI_MODEL_HANDLE model = nullptr;
  FrameMappingCache mapping_cache;
  StreamTargetOverlay target_overlay;
  SerialTransport serial;
  std::thread stream_thread;

  CVI_S32 media_result = SAMPLE_TDL_Get_VI_Config(&config.stViConfig);
  SIZE_S sensor_size{};
  PIC_SIZE_E picture_size = PIC_BUTT;
  if (media_result != CVI_SUCCESS || config.stViConfig.s32WorkingViNum <= 0) {
    std::cerr << "读取/mnt/data/sensor_cfg.ini失败。\n";
    exit_code = 2;
  } else {
    media_result = SAMPLE_COMM_VI_GetSizeBySensor(
        config.stViConfig.astViInfo[0].stSnsInfo.enSnsType, &picture_size);
    if (media_result != CVI_SUCCESS ||
        SAMPLE_COMM_SYS_GetPicSize(picture_size, &sensor_size) != CVI_SUCCESS) {
      std::cerr << "获取传感器尺寸失败。\n";
      exit_code = 3;
    }
  }

  if (exit_code == 0) {
    config.stVBPoolConfig.u32VBPoolCount = 2;
    auto& sensor_pool = config.stVBPoolConfig.astVBPoolSetup[0];
    sensor_pool.enFormat = VI_PIXEL_FORMAT;
    sensor_pool.u32BlkCount = 5;
    sensor_pool.u32Width = sensor_size.u32Width;
    sensor_pool.u32Height = sensor_size.u32Height;
    sensor_pool.bBind = true;
    sensor_pool.u32VpssGrpBinding = kVpssGroup;
    sensor_pool.u32VpssChnBinding = kStreamChannel;

    auto& inference_pool = config.stVBPoolConfig.astVBPoolSetup[1];
    inference_pool.enFormat = PIXEL_FORMAT_RGB_888_PLANAR;
    inference_pool.u32BlkCount = 4;
    inference_pool.u32Width = kInputWidth;
    inference_pool.u32Height = kInputHeight;
    inference_pool.bBind = true;
    inference_pool.u32VpssGrpBinding = kVpssGroup;
    inference_pool.u32VpssChnBinding = kInferenceChannel;

    config.stVPSSPoolConfig.u32VpssGrpCount = 1;
    config.stVPSSPoolConfig.stVpssMode.aenInput[0] = VPSS_INPUT_MEM;
    config.stVPSSPoolConfig.stVpssMode.enMode = VPSS_MODE_DUAL;
    config.stVPSSPoolConfig.stVpssMode.ViPipe[0] = 0;
    config.stVPSSPoolConfig.stVpssMode.aenInput[1] = VPSS_INPUT_ISP;
    config.stVPSSPoolConfig.stVpssMode.ViPipe[1] = 0;

    auto& vpss = config.stVPSSPoolConfig.astVpssConfig[0];
    vpss.bBindVI = true;
    vpss.u32ChnCount = 2;
    vpss.u32ChnBindVI = 0;
    VPSS_GRP_DEFAULT_HELPER2(&vpss.stVpssGrpAttr, sensor_size.u32Width,
                             sensor_size.u32Height, VI_PIXEL_FORMAT, 1);
    // CH0保持宽高比输出NV21画面，交给硬件VENC进行H.264编码。
    VPSS_CHN_DEFAULT_HELPER(&vpss.astVpssChnAttr[kStreamChannel],
                            kStreamWidth, kStreamHeight, VI_PIXEL_FORMAT, true);
    // CH1保持宽高比并居中补黑边，和离线图片版本的letterbox一致。
    VPSS_CHN_DEFAULT_HELPER(&vpss.astVpssChnAttr[kInferenceChannel],
                            kInputWidth, kInputHeight,
                            PIXEL_FORMAT_RGB_888_PLANAR, true);

    // 初始化SG2002硬件VENC和RTSP服务，实际送帧由独立线程完成。
    SAMPLE_TDL_Get_Input_Config(&config.stVencConfig.stChnInputCfg);
    config.stVencConfig.u32FrameWidth = kStreamWidth;
    config.stVencConfig.u32FrameHeight = kStreamHeight;
    SAMPLE_TDL_Get_RTSP_Config(&config.stRTSPConfig.stRTSPConfig);

    media_result = SAMPLE_TDL_Init_WM(&config, &context);
    if (media_result != CVI_SUCCESS) {
      std::cerr << "middleware初始化失败, CVI_S32=" << media_result << '\n';
      exit_code = 4;
    } else {
      middleware_ready = true;
    }
  }

  CVI_TENSOR* inputs = nullptr;
  CVI_TENSOR* outputs = nullptr;
  std::int32_t input_count = 0;
  std::int32_t output_count = 0;
  if (exit_code == 0) {
    target_overlay.Initialize(sensor_size.u32Width, sensor_size.u32Height);
  }

  if (exit_code == 0) {
    CVI_RC runtime_result = CVI_NN_RegisterModel(argv[1], &model);
    if (runtime_result != 0) {
      std::cerr << "模型加载失败, CVI_RC=" << runtime_result << '\n';
      exit_code = 5;
    } else {
      runtime_result = CVI_NN_GetInputOutputTensors(
          model, &inputs, &input_count, &outputs, &output_count);
      if (runtime_result != 0) {
        std::cerr << "获取模型张量失败, CVI_RC=" << runtime_result << '\n';
        exit_code = 6;
      } else if (input_count != 1 || output_count != 1 ||
                 inputs[0].fmt != CVI_FMT_FP32 ||
                 outputs[0].fmt != CVI_FMT_FP32 ||
                 !HasShape(inputs[0], {1, 3, 320, 320}) ||
                 CVI_NN_TensorCount(&outputs[0]) % kValuesPerCandidate != 0) {
        std::cerr << "模型张量格式与装甲板模型预期不一致。\n";
        exit_code = 7;
      }
    }
  }

  if (exit_code == 0) {
    auto* input_data = static_cast<float*>(CVI_NN_TensorPtr(&inputs[0]));
    const auto* output_data =
        static_cast<const float*>(CVI_NN_TensorPtr(&outputs[0]));
    if (input_data == nullptr || output_data == nullptr) {
      std::cerr << "模型输入或输出缓冲区为空。\n";
      exit_code = 8;
    } else {
      const LetterboxInfo letterbox =
          MakeLetterbox(sensor_size.u32Width, sensor_size.u32Height);
      TargetSelector target_selector(sensor_size.u32Width, sensor_size.u32Height,
                                     enemy_color_id);
      PositionFilter position_filter;
      CameraCalibration calibration;
      std::string calibration_error;
      if (!calibration_path.empty() && calibration_path != "-" &&
          !LoadCameraCalibration(calibration_path, calibration,
                                 calibration_error)) {
        std::cerr << "读取PnP标定配置失败: " << calibration_error << '\n';
        exit_code = 14;
      }
      PoseEstimator pose_estimator(calibration, sensor_size.u32Width,
                                   sensor_size.u32Height);
      if (exit_code == 0 && serial_device != "off") {
        std::string serial_error;
        if (!serial.Open(serial_device, static_cast<int>(serial_baud),
                         serial_error)) {
          std::cerr << "串口初始化失败: " << serial_error << '\n';
          exit_code = 15;
        }
      }
      std::cout << "实时装甲板识别启动：传感器=" << sensor_size.u32Width << 'x'
                << sensor_size.u32Height
                << ", 模型=320x320，敌方颜色=" << enemy_color_name
                << "，RTSP=rtsp://192.168.42.1/h264，按Ctrl+C退出。\n";
      std::cout << "PnP状态: " << pose_estimator.status() << "。\n";
      std::cout << "串口状态: "
                << (serial.is_open() ? serial_device + " @ " +
                                           std::to_string(serial_baud)
                                     : "关闭")
                << "。\n";
      std::cout << "推流画框: "
                << (target_overlay.ready() ? "已启用" : "未启用") << "。\n";

      if (exit_code != 0) {
        g_running.store(false);
      }

      // middleware、模型和张量全部就绪后再启动推流线程。
      if (exit_code == 0) {
        stream_thread = std::thread(StreamLoop, &context, &target_overlay);
      }

      std::uint64_t total_frames = 0;
      std::uint64_t interval_frames = 0;
      double get_ms = 0.0;
      double convert_ms = 0.0;
      double inference_ms = 0.0;
      double postprocess_ms = 0.0;
      auto interval_begin = Clock::now();
      // 仿照Hz_Rm_Vision：取帧返回后立即记录steady_clock时间点，并将其
      // 作为这一帧后续检测、PnP和Tracker/EKF统一使用的采集时间戳。
      std::int64_t previous_received_time_us = 0;
      std::uint64_t previous_hardware_pts = 0;
      std::uint64_t previous_stream_frames = 0;

      while (g_running.load() &&
             (frame_limit == 0 || total_frames < static_cast<std::uint64_t>(frame_limit))) {
        VIDEO_FRAME_INFO_S frame{};
        const auto frame_begin = Clock::now();
        media_result = CVI_VPSS_GetChnFrame(
            kVpssGroup, kInferenceChannel, &frame, 2000);
        const auto get_end = Clock::now();
        if (media_result != CVI_SUCCESS) {
          std::cerr << "VPSS取帧失败, CVI_S32=" << media_result << '\n';
          exit_code = 9;
          break;
        }
        const std::int64_t received_time_us =
            std::chrono::duration_cast<std::chrono::microseconds>(
                get_end.time_since_epoch()).count();
        const CVI_U64 frame_pts = frame.stVFrame.u64PTS;
        // PTS为0或不递增时自动退回steady_clock，保证EKF时间严格单调。
        const bool hardware_pts_valid =
            frame_pts != 0 &&
            (previous_hardware_pts == 0 || frame_pts > previous_hardware_pts);
        const std::int64_t capture_us = hardware_pts_valid
            ? static_cast<std::int64_t>(frame_pts)
            : received_time_us;
        const std::int64_t capture_dt_us =
            hardware_pts_valid && previous_hardware_pts != 0
                ? static_cast<std::int64_t>(frame_pts - previous_hardware_pts)
                : (previous_received_time_us == 0
                       ? 0
                       : received_time_us - previous_received_time_us);
        if (hardware_pts_valid) previous_hardware_pts = frame_pts;
        previous_received_time_us = received_time_us;
        const std::int64_t pipeline_latency_us =
            hardware_pts_valid && received_time_us >= capture_us
                ? received_time_us - capture_us
                : -1;

        const bool copied = CopyFrameToTensor(frame, input_data, mapping_cache);
        const auto convert_end = Clock::now();
        const CVI_S32 release_result = CVI_VPSS_ReleaseChnFrame(
            kVpssGroup, kInferenceChannel, &frame);
        if (!copied || release_result != CVI_SUCCESS) {
          if (release_result != CVI_SUCCESS) {
            std::cerr << "VPSS释放帧失败, CVI_S32=" << release_result << '\n';
          }
          exit_code = 10;
          break;
        }

        const auto inference_begin = Clock::now();
        const CVI_RC runtime_result =
            CVI_NN_Forward(model, inputs, input_count, outputs, output_count);
        const auto inference_end = Clock::now();
        if (runtime_result != 0) {
          std::cerr << "TPU推理失败, CVI_RC=" << runtime_result << '\n';
          exit_code = 11;
          break;
        }

        FrameResult frame_result;
        frame_result.detections = Decode(
            output_data, CVI_NN_TensorCount(&outputs[0]), letterbox,
            sensor_size.u32Width, sensor_size.u32Height,
            confidence_threshold, nms_threshold);
        const SelectionResult selection =
            target_selector.Update(frame_result.detections);
        if (target_overlay.ready()) target_overlay.Update(selection.target);
        const auto postprocess_end = Clock::now();
        ++total_frames;
        ++interval_frames;
        frame_result.timing = {
            total_frames, capture_us, received_time_us, pipeline_latency_us,
            capture_dt_us, hardware_pts_valid};
        get_ms += Milliseconds(frame_begin, get_end);
        convert_ms += Milliseconds(get_end, convert_end);
        inference_ms += Milliseconds(inference_begin, inference_end);
        postprocess_ms += Milliseconds(inference_end, postprocess_end);

        if (selection.state_changed) {
          std::cout << "[状态] frame_id=" << total_frames
                    << " state=" << TrackingStateName(selection.state)
                    << " raw=" << frame_result.detections.size()
                    << " valid=" << selection.valid_count << '\n';
        }

        PoseResult pose;
        if (selection.target) pose = pose_estimator.Solve(selection.target->points);
        std::optional<PositionMeasurement> position_measurement;
        if (pose.valid) {
          position_measurement =
              PositionMeasurement{pose.x_m, pose.y_m, pose.z_m};
        }
        const double filter_dt_seconds =
            capture_dt_us > 0 ? static_cast<double>(capture_dt_us) * 1e-6
                              : 1.0 / 30.0;
        PositionFilterResult filtered_position;
        if (selection.state == TrackingState::kLost) {
          position_filter.Reset();
          filtered_position = position_filter.Step(std::nullopt, filter_dt_seconds);
        } else {
          filtered_position =
              position_filter.Step(position_measurement, filter_dt_seconds);
        }

        if (selection.target) {
          const Detection& target = *selection.target;
          const auto target_center = DetectionCenter(target);
          std::cout << std::fixed << std::setprecision(2)
                    << "frame_id=" << frame_result.timing.frame_id
                    << " capture_pts_us=" << frame_result.timing.capture_pts_us
                    << " received_time_us=" << frame_result.timing.received_time_us
                    << " pipeline_latency_us="
                    << frame_result.timing.pipeline_latency_us
                    << " dt_us=" << frame_result.timing.dt_us
                    << " timestamp_source="
                    << (frame_result.timing.uses_hardware_pts ? "pts" : "steady")
                    << " state=" << TrackingStateName(selection.state)
                    << " raw=" << frame_result.detections.size()
                    << " valid=" << selection.valid_count
                    << " target_valid=1"
                    << " 置信度=" << target.confidence
                    << " color=" << ModelColorName(target.color_id)
                    << " color_id=" << target.color_id
                    << " 数字=" << target.number_id
                    << " border_clipped=" << (target.border_clipped ? 1 : 0)
                    << " center_x_px=" << target_center[0]
                    << " center_y_px=" << target_center[1]
                    << " pose_valid=" << (pose.valid ? 1 : 0);
          if (pose.valid) {
            std::cout << std::setprecision(4)
                      << " x_m=" << pose.x_m << " y_m=" << pose.y_m
                      << " z_m=" << pose.z_m
                      << std::setprecision(2)
                      << " reprojection_error_px="
                      << pose.reprojection_error_px;
          } else if (pose_estimator.enabled()) {
            std::cout << " pose_error=" << pose.error;
            if (pose.reprojection_error_px > 0.0) {
              std::cout << " reprojection_error_px="
                        << pose.reprojection_error_px;
            }
          }
          std::cout << " filter_state="
                    << PositionFilterStateName(filtered_position.state);
          if (filtered_position.valid) {
            std::cout << std::setprecision(4)
                      << " filtered_x_m=" << filtered_position.x_m
                      << " filtered_y_m=" << filtered_position.y_m
                      << " filtered_z_m=" << filtered_position.z_m
                      << " vx_mps=" << filtered_position.vx_mps
                      << " vy_mps=" << filtered_position.vy_mps
                      << " vz_mps=" << filtered_position.vz_mps
                      << std::setprecision(2)
                      << " nis=" << filtered_position.nis;
          }
          std::cout
                    << " 四角点=";
          for (int point = 0; point < 4; ++point) {
            std::cout << (point == 0 ? "(" : " (")
                      << target.points[point * 2] << ','
                      << target.points[point * 2 + 1] << ')';
          }
          std::cout << '\n';
        }

        // 仅在状态机确认主目标后发送一行纯数字。滤波器就绪时发送滤波XYZ；
        // PnP未启用时保留全零XYZ以兼容已有串口联调流程。
        if (serial.is_open() && selection.target) {
          SerialArmorData serial_data;
          const auto center = DetectionCenter(*selection.target);
          serial_data.center_x_px = center[0];
          serial_data.center_y_px = center[1];
          if (filtered_position.valid) {
            serial_data.x_m = static_cast<float>(filtered_position.x_m);
            serial_data.y_m = static_cast<float>(filtered_position.y_m);
            serial_data.z_m = static_cast<float>(filtered_position.z_m);
          }
          std::string serial_error;
          if (!serial.Send(serial_data, serial_error)) {
            std::cerr << "串口发送停止: " << serial_error << '\n';
            serial.Close();
          }
        }

        const double interval_elapsed =
            Milliseconds(interval_begin, postprocess_end);
        if (interval_elapsed >= 1000.0) {
          std::cout << std::fixed << std::setprecision(2)
                    << "[性能] 帧=" << total_frames
                    << " received_time_us=" << received_time_us
                    << " pipeline_latency_us=" << pipeline_latency_us
                    << " dt_us=" << capture_dt_us
                    << " timestamp_source="
                    << (hardware_pts_valid ? "pts" : "steady")
                    << " state=" << TrackingStateName(selection.state)
                    << " raw=" << frame_result.detections.size()
                    << " valid=" << selection.valid_count
                    << " target_valid=" << (selection.target ? 1 : 0)
                    << " filter_state="
                    << PositionFilterStateName(filtered_position.state)
                    << " FPS=" << interval_frames * 1000.0 / interval_elapsed
                    << " 推流FPS="
                    << (g_stream_frames.load(std::memory_order_relaxed) -
                        previous_stream_frames) * 1000.0 / interval_elapsed
                    << " 取帧=" << get_ms / interval_frames << "ms"
                    << " 转换=" << convert_ms / interval_frames << "ms"
                    << " TPU=" << inference_ms / interval_frames << "ms"
                    << " 后处理=" << postprocess_ms / interval_frames << "ms\n";
          previous_stream_frames =
              g_stream_frames.load(std::memory_order_relaxed);
          interval_frames = 0;
          get_ms = convert_ms = inference_ms = postprocess_ms = 0.0;
          interval_begin = postprocess_end;
        }
      }
      g_running.store(false);
      if (stream_thread.joinable()) stream_thread.join();
      std::cout << "实时识别结束，总处理帧数=" << total_frames << "。\n";
      std::cout << "RTSP推流总帧数="
                << g_stream_frames.load(std::memory_order_relaxed) << "。\n";
      std::cout << "VPSS缓存映射数量=" << mapping_cache.size() << "。\n";
    }
  }

  g_running.store(false);
  if (stream_thread.joinable()) stream_thread.join();

  if (model != nullptr) {
    const CVI_RC cleanup_result = CVI_NN_CleanupModel(model);
    if (cleanup_result != 0) {
      std::cerr << "模型清理失败, CVI_RC=" << cleanup_result << '\n';
      if (exit_code == 0) exit_code = 12;
    }
  }
  if (!mapping_cache.Clear()) {
    std::cerr << "部分VPSS缓存映射解除失败。\n";
    if (exit_code == 0) exit_code = 13;
  }
  target_overlay.Destroy();
  if (middleware_ready) SAMPLE_TDL_Destroy_MW(&context);
  CVI_MSG_Deinit();
  return exit_code;
}
