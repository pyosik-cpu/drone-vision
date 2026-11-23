#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "cviruntime.h"

#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_JPEG
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

namespace {

constexpr int kInputWidth = 320;
constexpr int kInputHeight = 320;
constexpr int kValuesPerCandidate = 22;
constexpr int kNotArmorClass = 8;

struct Detection {
  // 顺序为左上、右上、右下、左下。
  std::array<float, 8> points{};
  int color_id = -1;
  int number_id = -1;
  float confidence = 0.0F;
};

struct LetterboxInfo {
  float scale = 1.0F;
  float pad_x = 0.0F;
  float pad_y = 0.0F;
};

struct RgbColor {
  std::uint8_t red;
  std::uint8_t green;
  std::uint8_t blue;
};

float Sigmoid(float value) {
  return 1.0F / (1.0F + std::exp(-value));
}

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

// 使用双线性插值完成黑边 letterbox，并直接生成模型需要的 FP32/NCHW 数据。
LetterboxInfo PrepareInput(const std::uint8_t* rgb, int source_width,
                           int source_height, float* destination) {
  const float scale = std::min(static_cast<float>(kInputWidth) / source_width,
                               static_cast<float>(kInputHeight) / source_height);
  const int resized_width = std::max(1, static_cast<int>(std::round(source_width * scale)));
  const int resized_height = std::max(1, static_cast<int>(std::round(source_height * scale)));
  const int offset_x = (kInputWidth - resized_width) / 2;
  const int offset_y = (kInputHeight - resized_height) / 2;
  std::fill(destination, destination + 3 * kInputWidth * kInputHeight, 0.0F);

  for (int y = 0; y < resized_height; ++y) {
    const float source_y = (y + 0.5F) / scale - 0.5F;
    const int y0 = std::clamp(static_cast<int>(std::floor(source_y)), 0, source_height - 1);
    const int y1 = std::min(y0 + 1, source_height - 1);
    const float wy = std::clamp(source_y - std::floor(source_y), 0.0F, 1.0F);
    for (int x = 0; x < resized_width; ++x) {
      const float source_x = (x + 0.5F) / scale - 0.5F;
      const int x0 = std::clamp(static_cast<int>(std::floor(source_x)), 0, source_width - 1);
      const int x1 = std::min(x0 + 1, source_width - 1);
      const float wx = std::clamp(source_x - std::floor(source_x), 0.0F, 1.0F);
      const int output_index = (y + offset_y) * kInputWidth + x + offset_x;

      for (int channel = 0; channel < 3; ++channel) {
        const float top = rgb[(y0 * source_width + x0) * 3 + channel] * (1.0F - wx) +
                          rgb[(y0 * source_width + x1) * 3 + channel] * wx;
        const float bottom = rgb[(y1 * source_width + x0) * 3 + channel] * (1.0F - wx) +
                             rgb[(y1 * source_width + x1) * 3 + channel] * wx;
        destination[channel * kInputWidth * kInputHeight + output_index] =
            (top * (1.0F - wy) + bottom * wy) / 255.0F;
      }
    }
  }
  return {scale, static_cast<float>(offset_x), static_cast<float>(offset_y)};
}

std::vector<Detection> Decode(const float* data, std::size_t value_count,
                              const LetterboxInfo& letterbox, int image_width,
                              int image_height, float confidence_threshold,
                              float nms_threshold) {
  std::vector<Detection> candidates;
  const std::size_t candidate_count = value_count / kValuesPerCandidate;
  candidates.reserve(64);

  for (std::size_t index = 0; index < candidate_count; ++index) {
    const float* row = data + index * kValuesPerCandidate;
    const float confidence = Sigmoid(row[8]);
    if (confidence < confidence_threshold) continue;

    const int color_id = static_cast<int>(std::max_element(row + 9, row + 13) - (row + 9));
    const int number_id = static_cast<int>(std::max_element(row + 13, row + 22) - (row + 13));
    if (number_id == kNotArmorClass) continue;

    Detection detection;
    detection.color_id = color_id;
    detection.number_id = number_id;
    detection.confidence = confidence;
    constexpr int order[4] = {0, 3, 2, 1};
    for (int point = 0; point < 4; ++point) {
      const int source = order[point];
      detection.points[point * 2] = std::clamp(
          (row[source * 2] - letterbox.pad_x) / letterbox.scale,
          0.0F, static_cast<float>(image_width - 1));
      detection.points[point * 2 + 1] = std::clamp(
          (row[source * 2 + 1] - letterbox.pad_y) / letterbox.scale,
          0.0F, static_cast<float>(image_height - 1));
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

bool HasShape(const CVI_TENSOR& tensor, std::initializer_list<int> dimensions) {
  if (tensor.shape.dim_size != dimensions.size()) return false;
  std::size_t index = 0;
  for (const int dimension : dimensions) {
    if (tensor.shape.dim[index++] != dimension) return false;
  }
  return true;
}

void SetPixel(std::uint8_t* image, int width, int height, int x, int y,
              RgbColor color) {
  if (x < 0 || x >= width || y < 0 || y >= height) return;
  std::uint8_t* pixel = image + (y * width + x) * 3;
  pixel[0] = color.red;
  pixel[1] = color.green;
  pixel[2] = color.blue;
}

// 使用整数 Bresenham 算法绘制带宽度的线段。
void DrawLine(std::uint8_t* image, int width, int height, int x0, int y0,
              int x1, int y1, RgbColor color, int thickness) {
  const int dx = std::abs(x1 - x0);
  const int sx = x0 < x1 ? 1 : -1;
  const int dy = -std::abs(y1 - y0);
  const int sy = y0 < y1 ? 1 : -1;
  int error = dx + dy;
  while (true) {
    const int radius = thickness / 2;
    for (int offset_y = -radius; offset_y <= radius; ++offset_y) {
      for (int offset_x = -radius; offset_x <= radius; ++offset_x) {
        SetPixel(image, width, height, x0 + offset_x, y0 + offset_y, color);
      }
    }
    if (x0 == x1 && y0 == y1) break;
    const int doubled_error = 2 * error;
    if (doubled_error >= dy) {
      error += dy;
      x0 += sx;
    }
    if (doubled_error <= dx) {
      error += dx;
      y0 += sy;
    }
  }
}

// 5x7 点阵字形；只包含结果标签实际需要的字符。
std::array<std::uint8_t, 7> Glyph(char character) {
  switch (character) {
    case 'B': return {30, 17, 17, 30, 17, 17, 30};
    case 'L': return {16, 16, 16, 16, 16, 16, 31};
    case 'U': return {17, 17, 17, 17, 17, 17, 14};
    case 'E': return {31, 16, 16, 30, 16, 16, 31};
    case 'R': return {30, 17, 17, 30, 20, 18, 17};
    case '%': return {17, 2, 4, 8, 16, 0, 17};
    case '0': return {14, 17, 19, 21, 25, 17, 14};
    case '1': return {4, 12, 4, 4, 4, 4, 14};
    case '2': return {14, 17, 1, 2, 4, 8, 31};
    case '3': return {30, 1, 1, 14, 1, 1, 30};
    case '4': return {2, 6, 10, 18, 31, 2, 2};
    case '5': return {31, 16, 16, 30, 1, 1, 30};
    case '6': return {14, 16, 16, 30, 17, 17, 14};
    case '7': return {31, 1, 2, 4, 8, 8, 8};
    case '8': return {14, 17, 17, 14, 17, 17, 14};
    case '9': return {14, 17, 17, 15, 1, 1, 14};
    default: return {0, 0, 0, 0, 0, 0, 0};
  }
}

void FillRect(std::uint8_t* image, int width, int height, int left, int top,
              int rectangle_width, int rectangle_height, RgbColor color) {
  for (int y = top; y < top + rectangle_height; ++y) {
    for (int x = left; x < left + rectangle_width; ++x) {
      SetPixel(image, width, height, x, y, color);
    }
  }
}

void DrawText(std::uint8_t* image, int width, int height, int x, int y,
              const std::string& text, RgbColor color, int scale) {
  for (char character : text) {
    const auto glyph = Glyph(character);
    for (int row = 0; row < 7; ++row) {
      for (int column = 0; column < 5; ++column) {
        if ((glyph[row] & (1U << (4 - column))) != 0) {
          FillRect(image, width, height, x + column * scale, y + row * scale,
                   scale, scale, color);
        }
      }
    }
    x += 6 * scale;
  }
}

[[maybe_unused]] void DrawDetections(
    std::uint8_t* image, int width, int height,
    const std::vector<Detection>& detections) {
  for (const Detection& detection : detections) {
    const bool blue = detection.color_id == 0;
    const RgbColor color = blue ? RgbColor{0, 160, 255} : RgbColor{255, 40, 40};
    for (int point = 0; point < 4; ++point) {
      const int next = (point + 1) % 4;
      DrawLine(image, width, height,
               static_cast<int>(std::round(detection.points[point * 2])),
               static_cast<int>(std::round(detection.points[point * 2 + 1])),
               static_cast<int>(std::round(detection.points[next * 2])),
               static_cast<int>(std::round(detection.points[next * 2 + 1])),
               color, 5);
    }

    float left, top, right, bottom;
    GetBounds(detection, left, top, right, bottom);
    const int percentage = std::clamp(
        static_cast<int>(std::round(detection.confidence * 100.0F)), 0, 100);
    const std::string label = (blue ? "BLUE " : "RED ") +
                              std::to_string(detection.number_id) + " " +
                              std::to_string(percentage) + "%";
    constexpr int text_scale = 3;
    const int label_width = static_cast<int>(label.size()) * 6 * text_scale + 8;
    const int label_height = 7 * text_scale + 8;
    const int label_x = std::clamp(static_cast<int>(left), 0,
                                   std::max(0, width - label_width));
    const int label_y = std::clamp(static_cast<int>(top) - label_height, 0,
                                   std::max(0, height - label_height));
    FillRect(image, width, height, label_x, label_y, label_width, label_height,
             RgbColor{0, 0, 0});
    DrawText(image, width, height, label_x + 4, label_y + 4, label, color,
             text_scale);
  }
}

}  // namespace

#ifndef ARMOR_DETECT_NO_MAIN
int main(int argc, char* argv[]) {
  if (argc < 3 || argc > 7) {
    std::cerr << "用法: " << argv[0]
              << " <模型.cvimodel> <图片.jpg> [置信度阈值=0.5] [NMS阈值=0.45]"
                 " [输出.jpg=/root/armor_result.jpg] [连续推理次数=1]\n";
    return 64;
  }
  const float confidence_threshold = argc >= 4 ? std::strtof(argv[3], nullptr) : 0.5F;
  const float nms_threshold = argc >= 5 ? std::strtof(argv[4], nullptr) : 0.45F;
  const char* output_path = argc >= 6 ? argv[5] : "/root/armor_result.jpg";
  const int repeat_count = argc >= 7 ? std::atoi(argv[6]) : 1;
  if (!(confidence_threshold >= 0.0F && confidence_threshold <= 1.0F &&
        nms_threshold >= 0.0F && nms_threshold <= 1.0F) || repeat_count < 1) {
    std::cerr << "阈值必须位于 0 到 1 之间，连续推理次数必须大于 0。\n";
    return 64;
  }

  int image_width = 0;
  int image_height = 0;
  int original_channels = 0;
  stbi_uc* image = stbi_load(argv[2], &image_width, &image_height, &original_channels, 3);
  if (image == nullptr) {
    std::cerr << "JPEG 图片读取失败: " << argv[2] << ", " << stbi_failure_reason() << '\n';
    return 1;
  }

  CVI_MODEL_HANDLE model = nullptr;
  CVI_RC result = CVI_NN_RegisterModel(argv[1], &model);
  if (result != 0) {
    std::cerr << "模型加载失败, CVI_RC=" << result << '\n';
    stbi_image_free(image);
    return 2;
  }

  int exit_code = 0;
  CVI_TENSOR* inputs = nullptr;
  CVI_TENSOR* outputs = nullptr;
  std::int32_t input_count = 0;
  std::int32_t output_count = 0;
  result = CVI_NN_GetInputOutputTensors(model, &inputs, &input_count, &outputs, &output_count);
  if (result != 0) {
    std::cerr << "获取模型张量失败, CVI_RC=" << result << '\n';
    exit_code = 3;
  } else if (input_count != 1 || output_count != 1 || inputs[0].fmt != CVI_FMT_FP32 ||
             outputs[0].fmt != CVI_FMT_FP32 || !HasShape(inputs[0], {1, 3, 320, 320}) ||
             CVI_NN_TensorCount(&outputs[0]) % kValuesPerCandidate != 0) {
    std::cerr << "模型张量格式与装甲板模型预期不一致。\n";
    exit_code = 4;
  } else {
    auto* input_data = static_cast<float*>(CVI_NN_TensorPtr(&inputs[0]));
    if (input_data == nullptr) {
      std::cerr << "模型输入缓冲区为空。\n";
      exit_code = 5;
    } else {
      const LetterboxInfo letterbox =
          PrepareInput(image, image_width, image_height, input_data);
      // 输入保持不变，连续执行 Forward，用于测量不含模型加载和 JPEG 解码的纯推理性能。
      const auto inference_start = std::chrono::steady_clock::now();
      for (int repeat = 0; repeat < repeat_count; ++repeat) {
        result = CVI_NN_Forward(model, inputs, input_count, outputs, output_count);
        if (result != 0) break;
      }
      const auto inference_end = std::chrono::steady_clock::now();
      if (result != 0) {
        std::cerr << "TPU 推理失败, CVI_RC=" << result << '\n';
        exit_code = 6;
      } else {
        const double total_ms = std::chrono::duration<double, std::milli>(
                                    inference_end - inference_start)
                                    .count();
        const double average_ms = total_ms / repeat_count;
        const double inference_fps = average_ms > 0.0 ? 1000.0 / average_ms : 0.0;
        std::cout << std::fixed << std::setprecision(2)
                  << "连续推理: " << repeat_count << " 次, 总耗时=" << total_ms
                  << " ms, 平均=" << average_ms << " ms, TPU FPS="
                  << inference_fps << '\n';
        const auto* output_data = static_cast<const float*>(CVI_NN_TensorPtr(&outputs[0]));
        if (output_data == nullptr) {
          std::cerr << "模型输出缓冲区为空。\n";
          exit_code = 7;
        } else {
          const std::vector<Detection> detections = Decode(
              output_data, CVI_NN_TensorCount(&outputs[0]), letterbox,
              image_width, image_height, confidence_threshold, nms_threshold);
          std::cout << "图片: " << argv[2] << " (" << image_width << 'x' << image_height
                    << "), 检测到装甲板: " << detections.size() << "\n";
          std::cout << std::fixed << std::setprecision(2);
          for (std::size_t index = 0; index < detections.size(); ++index) {
            const Detection& detection = detections[index];
            std::cout << '[' << index << "] 置信度=" << detection.confidence
                      << ", 颜色类别=" << detection.color_id
                      << ", 数字类别=" << detection.number_id << ", 四角点=";
            for (int point = 0; point < 4; ++point) {
              std::cout << (point == 0 ? "(" : " (")
                        << detection.points[point * 2] << ','
                        << detection.points[point * 2 + 1] << ')';
            }
            std::cout << '\n';
          }
          DrawDetections(image, image_width, image_height, detections);
          if (stbi_write_jpg(output_path, image_width, image_height, 3, image, 92) == 0) {
            std::cerr << "结果图片保存失败: " << output_path << '\n';
            exit_code = 9;
          } else {
            std::cout << "结果图片已保存: " << output_path << '\n';
          }
        }
      }
    }
  }

  stbi_image_free(image);
  result = CVI_NN_CleanupModel(model);
  if (result != 0) {
    std::cerr << "模型清理失败, CVI_RC=" << result << '\n';
    if (exit_code == 0) exit_code = 8;
  }
  return exit_code;
}
#endif
