#include <iostream>

extern "C" {
#include "cvi_vpss.h"
#include "sophgo_middleware.h"
}

namespace {

constexpr int kVpssGroup = 0;
constexpr int kVpssChannel = 0;
constexpr int kOutputWidth = 1280;
constexpr int kOutputHeight = 720;

}  // namespace

int main() {
  // 必须确保camera-test.sh等媒体程序已经停止，避免争用VI/VPSS资源。
  if (mmf_init() != 0) {
    std::cerr << "媒体系统初始化失败。\n";
    return 1;
  }

  bool channel_created = false;
  int exit_code = 0;
  if (mmf_add_vi_channel(kVpssChannel, kOutputWidth, kOutputHeight,
                         PIXEL_FORMAT_NV21) != 0) {
    std::cerr << "VI/VPSS通道创建失败。\n";
    exit_code = 2;
  } else {
    channel_created = true;
    VIDEO_FRAME_INFO_S frame{};
    const CVI_S32 result = CVI_VPSS_GetChnFrame(
        kVpssGroup, kVpssChannel, &frame, 3000);
    if (result != CVI_SUCCESS) {
      std::cerr << "VPSS取帧失败, CVI_S32=" << result << '\n';
      exit_code = 3;
    } else {
      const VIDEO_FRAME_S& video = frame.stVFrame;
      std::cout << "成功取得一帧：\n"
                << "  宽高: " << video.u32Width << 'x' << video.u32Height << '\n'
                << "  像素格式: " << static_cast<int>(video.enPixelFormat) << '\n'
                << "  stride: [" << video.u32Stride[0] << ", "
                << video.u32Stride[1] << ", " << video.u32Stride[2] << "]\n"
                << "  length: [" << video.u32Length[0] << ", "
                << video.u32Length[1] << ", " << video.u32Length[2] << "]\n"
                << "  物理地址: [0x" << std::hex << video.u64PhyAddr[0]
                << ", 0x" << video.u64PhyAddr[1] << ", 0x"
                << video.u64PhyAddr[2] << std::dec << "]\n";

      // GetChnFrame必须与VPSS的ReleaseChnFrame严格配对。
      const CVI_S32 release_result = CVI_VPSS_ReleaseChnFrame(
          kVpssGroup, kVpssChannel, &frame);
      if (release_result != CVI_SUCCESS) {
        std::cerr << "VPSS释放帧失败, CVI_S32=" << release_result << '\n';
        exit_code = 4;
      }
    }
  }

  if (channel_created && mmf_del_vi_channel(kVpssChannel) != 0) {
    std::cerr << "VI/VPSS通道清理失败。\n";
    if (exit_code == 0) exit_code = 5;
  }
  if (mmf_deinit() != 0) {
    std::cerr << "媒体系统清理失败。\n";
    if (exit_code == 0) exit_code = 6;
  }
  return exit_code;
}
