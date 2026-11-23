#define ARMOR_DETECT_NO_MAIN
#include "armor_detect.cpp"

#include <dirent.h>

#include <fstream>
#include <sstream>

namespace {

using Clock = std::chrono::steady_clock;

double Milliseconds(Clock::time_point begin, Clock::time_point end) {
  return std::chrono::duration<double, std::milli>(end - begin).count();
}

std::vector<std::string> FindJpegs(const std::string& directory) {
  std::vector<std::string> paths;
  DIR* handle = opendir(directory.c_str());
  if (handle == nullptr) return paths;
  while (dirent* entry = readdir(handle)) {
    const std::string name = entry->d_name;
    const bool is_jpeg = name.size() >= 4 &&
        (name.substr(name.size() - 4) == ".jpg" ||
         (name.size() >= 5 && name.substr(name.size() - 5) == ".jpeg"));
    if (is_jpeg) paths.push_back(directory + "/" + name);
  }
  closedir(handle);
  std::sort(paths.begin(), paths.end());
  return paths;
}

std::string BaseName(const std::string& path) {
  const std::size_t slash = path.find_last_of('/');
  return slash == std::string::npos ? path : path.substr(slash + 1);
}

void WriteJson(std::ostream& stream, const std::string& frame,
               const std::vector<Detection>& detections) {
  stream << "{\"frame\":\"" << frame << "\",\"detections\":[";
  for (std::size_t index = 0; index < detections.size(); ++index) {
    if (index != 0) stream << ',';
    const Detection& detection = detections[index];
    stream << "{\"confidence\":" << detection.confidence
           << ",\"color\":" << detection.color_id
           << ",\"number\":" << detection.number_id
           << ",\"points\":[";
    for (int point = 0; point < 4; ++point) {
      if (point != 0) stream << ',';
      stream << '[' << detection.points[point * 2] << ','
             << detection.points[point * 2 + 1] << ']';
    }
    stream << "]}";
  }
  stream << "]}\n";
}

}  // namespace

int main(int argc, char* argv[]) {
  if (argc < 3 || argc > 6) {
    std::cerr << "用法: " << argv[0]
              << " <模型.cvimodel> <JPEG目录> [结果.jsonl=/root/demo_results.jsonl]"
                 " [置信度=0.5] [NMS=0.45]\n";
    return 64;
  }
  const char* model_path = argv[1];
  const std::string image_directory = argv[2];
  const char* json_path = argc >= 4 ? argv[3] : "/root/demo_results.jsonl";
  const float confidence_threshold = argc >= 5 ? std::strtof(argv[4], nullptr) : 0.5F;
  const float nms_threshold = argc >= 6 ? std::strtof(argv[5], nullptr) : 0.45F;
  const std::vector<std::string> images = FindJpegs(image_directory);
  if (images.empty()) {
    std::cerr << "目录中没有 JPEG: " << image_directory << '\n';
    return 1;
  }

  std::ofstream json(json_path);
  if (!json) {
    std::cerr << "无法创建结果文件: " << json_path << '\n';
    return 2;
  }
  json << std::fixed << std::setprecision(3);

  CVI_MODEL_HANDLE model = nullptr;
  CVI_RC result = CVI_NN_RegisterModel(model_path, &model);
  if (result != 0) {
    std::cerr << "模型加载失败, CVI_RC=" << result << '\n';
    return 3;
  }
  CVI_TENSOR* inputs = nullptr;
  CVI_TENSOR* outputs = nullptr;
  std::int32_t input_count = 0;
  std::int32_t output_count = 0;
  result = CVI_NN_GetInputOutputTensors(model, &inputs, &input_count,
                                        &outputs, &output_count);
  if (result != 0 || input_count != 1 || output_count != 1 ||
      inputs[0].fmt != CVI_FMT_FP32 || outputs[0].fmt != CVI_FMT_FP32 ||
      !HasShape(inputs[0], {1, 3, 320, 320})) {
    std::cerr << "模型张量不符合预期, CVI_RC=" << result << '\n';
    CVI_NN_CleanupModel(model);
    return 4;
  }

  double decode_ms = 0.0;
  double preprocess_ms = 0.0;
  double inference_ms = 0.0;
  double postprocess_ms = 0.0;
  std::size_t processed = 0;
  std::size_t detected_frames = 0;
  const auto total_begin = Clock::now();

  for (const std::string& path : images) {
    int width = 0;
    int height = 0;
    int channels = 0;
    const auto decode_begin = Clock::now();
    stbi_uc* image = stbi_load(path.c_str(), &width, &height, &channels, 3);
    const auto decode_end = Clock::now();
    decode_ms += Milliseconds(decode_begin, decode_end);
    if (image == nullptr) {
      std::cerr << "跳过无法读取的图片: " << path << '\n';
      continue;
    }

    auto* input_data = static_cast<float*>(CVI_NN_TensorPtr(&inputs[0]));
    const auto preprocess_begin = Clock::now();
    const LetterboxInfo letterbox = PrepareInput(image, width, height, input_data);
    const auto preprocess_end = Clock::now();
    preprocess_ms += Milliseconds(preprocess_begin, preprocess_end);
    stbi_image_free(image);

    const auto inference_begin = Clock::now();
    result = CVI_NN_Forward(model, inputs, input_count, outputs, output_count);
    const auto inference_end = Clock::now();
    inference_ms += Milliseconds(inference_begin, inference_end);
    if (result != 0) {
      std::cerr << "推理失败: " << path << ", CVI_RC=" << result << '\n';
      break;
    }

    const auto postprocess_begin = Clock::now();
    const auto* output_data = static_cast<const float*>(CVI_NN_TensorPtr(&outputs[0]));
    const std::vector<Detection> detections = Decode(
        output_data, CVI_NN_TensorCount(&outputs[0]), letterbox, width, height,
        confidence_threshold, nms_threshold);
    WriteJson(json, BaseName(path), detections);
    const auto postprocess_end = Clock::now();
    postprocess_ms += Milliseconds(postprocess_begin, postprocess_end);
    ++processed;
    if (!detections.empty()) ++detected_frames;
    std::cout << BaseName(path) << ": " << detections.size() << " 个装甲板\n";
  }

  const auto total_end = Clock::now();
  const double total_ms = Milliseconds(total_begin, total_end);
  const double fps = total_ms > 0.0 ? processed * 1000.0 / total_ms : 0.0;
  std::cout << std::fixed << std::setprecision(2)
            << "\n处理帧数=" << processed << ", 有检测帧=" << detected_frames
            << "\nJPEG解码=" << decode_ms << " ms (平均 " << decode_ms / processed << ")"
            << "\n预处理=" << preprocess_ms << " ms (平均 " << preprocess_ms / processed << ")"
            << "\nTPU推理=" << inference_ms << " ms (平均 " << inference_ms / processed << ")"
            << "\n后处理/JSON=" << postprocess_ms << " ms (平均 " << postprocess_ms / processed << ")"
            << "\n端到端总耗时=" << total_ms << " ms, FPS=" << fps
            << "\n结果=" << json_path << '\n';

  const CVI_RC cleanup_result = CVI_NN_CleanupModel(model);
  return result == 0 && cleanup_result == 0 ? 0 : 5;
}
