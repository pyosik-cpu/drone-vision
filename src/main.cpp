#include <cstdint>
#include <iostream>
#include <string_view>

#include "cviruntime.h"

namespace {

// 将 Runtime 的数据类型枚举转换为便于阅读的文本。
std::string_view FormatName(CVI_FMT format) {
  switch (format) {
    case CVI_FMT_FP32: return "FP32";
    case CVI_FMT_INT32: return "INT32";
    case CVI_FMT_UINT32: return "UINT32";
    case CVI_FMT_BF16: return "BF16";
    case CVI_FMT_INT16: return "INT16";
    case CVI_FMT_UINT16: return "UINT16";
    case CVI_FMT_INT8: return "INT8";
    case CVI_FMT_UINT8: return "UINT8";
  }
  return "UNKNOWN";
}

// 打印一组张量的名称、形状和数据类型，不访问张量数据。
void PrintTensors(const char* category, const CVI_TENSOR* tensors,
                  std::int32_t tensor_count) {
  std::cout << category << " 数量: " << tensor_count << '\n';
  for (std::int32_t index = 0; index < tensor_count; ++index) {
    const CVI_TENSOR& tensor = tensors[index];
    std::cout << "  [" << index << "] 名称: "
              << (tensor.name != nullptr ? tensor.name : "<null>")
              << ", 形状: [";
    for (std::size_t dimension = 0; dimension < tensor.shape.dim_size;
         ++dimension) {
      if (dimension != 0) std::cout << ", ";
      std::cout << tensor.shape.dim[dimension];
    }
    std::cout << "], 类型: " << FormatName(tensor.fmt)
              << " (" << static_cast<int>(tensor.fmt) << ")\n";
  }
}

}  // namespace

int main(int argc, char* argv[]) {
  const char* model_path = argc > 1
      ? argv[1]
      : "/mnt/cvimodel/scrfd_768_432_int8_1x.cvimodel";

  CVI_MODEL_HANDLE model = nullptr;
  CVI_RC result = CVI_NN_RegisterModel(model_path, &model);
  if (result != 0) {
    std::cerr << "模型加载失败: " << model_path << ", CVI_RC=" << result << '\n';
    return 1;
  }

  CVI_TENSOR* inputs = nullptr;
  CVI_TENSOR* outputs = nullptr;
  std::int32_t input_count = 0;
  std::int32_t output_count = 0;
  result = CVI_NN_GetInputOutputTensors(model, &inputs, &input_count,
                                        &outputs, &output_count);
  if (result != 0) {
    std::cerr << "获取输入输出张量失败, CVI_RC=" << result << '\n';
    const CVI_RC cleanup_result = CVI_NN_CleanupModel(model);
    if (cleanup_result != 0) {
      std::cerr << "模型清理失败, CVI_RC=" << cleanup_result << '\n';
    }
    return 2;
  }

  std::cout << "模型: " << model_path << '\n';
  PrintTensors("输入张量", inputs, input_count);
  PrintTensors("输出张量", outputs, output_count);

  result = CVI_NN_CleanupModel(model);
  if (result != 0) {
    std::cerr << "模型清理失败, CVI_RC=" << result << '\n';
    return 3;
  }

  std::cout << "模型资源已释放。\n";
  return 0;
}

