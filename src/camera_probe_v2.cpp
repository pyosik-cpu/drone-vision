#include <cstring>
#include <iomanip>
#include <iostream>

extern "C" {
#include "middleware_utils.h"
#include "sample_comm.h"
#include "vpss_helper.h"
}

int main() {
  // 配置严格参考同版本官方sample_vi_fd.c，不加载任何AI模型。
  if (CVI_MSG_Init() != CVI_SUCCESS) {
    std::cerr << "CVI_MSG_Init失败。\n";
    return 1;
  }

  SAMPLE_TDL_MW_CONFIG_S config{};
  CVI_S32 result = SAMPLE_TDL_Get_VI_Config(&config.stViConfig);
  if (result != CVI_SUCCESS || config.stViConfig.s32WorkingViNum <= 0) {
    std::cerr << "读取/mnt/data/sensor_cfg.ini失败。\n";
    CVI_MSG_Deinit();
    return 2;
  }

  PIC_SIZE_E picture_size;
  result = SAMPLE_COMM_VI_GetSizeBySensor(
      config.stViConfig.astViInfo[0].stSnsInfo.enSnsType, &picture_size);
  SIZE_S sensor_size{};
  if (result != CVI_SUCCESS ||
      SAMPLE_COMM_SYS_GetPicSize(picture_size, &sensor_size) != CVI_SUCCESS) {
    std::cerr << "获取传感器尺寸失败。\n";
    CVI_MSG_Deinit();
    return 3;
  }

  constexpr CVI_U32 kOutputWidth = 1280;
  constexpr CVI_U32 kOutputHeight = 720;
  config.stVBPoolConfig.u32VBPoolCount = 2;
  auto& sensor_pool = config.stVBPoolConfig.astVBPoolSetup[0];
  sensor_pool.enFormat = VI_PIXEL_FORMAT;
  sensor_pool.u32BlkCount = 5;
  sensor_pool.u32Width = sensor_size.u32Width;
  sensor_pool.u32Height = sensor_size.u32Height;
  sensor_pool.bBind = true;
  sensor_pool.u32VpssGrpBinding = 0;
  sensor_pool.u32VpssChnBinding = VPSS_CHN0;

  auto& output_pool = config.stVBPoolConfig.astVBPoolSetup[1];
  output_pool.enFormat = VI_PIXEL_FORMAT;
  output_pool.u32BlkCount = 5;
  output_pool.u32Width = kOutputWidth;
  output_pool.u32Height = kOutputHeight;
  output_pool.bBind = true;
  output_pool.u32VpssGrpBinding = 0;
  output_pool.u32VpssChnBinding = VPSS_CHN0;

  config.stVPSSPoolConfig.u32VpssGrpCount = 1;
  config.stVPSSPoolConfig.stVpssMode.aenInput[0] = VPSS_INPUT_MEM;
  config.stVPSSPoolConfig.stVpssMode.enMode = VPSS_MODE_DUAL;
  config.stVPSSPoolConfig.stVpssMode.ViPipe[0] = 0;
  config.stVPSSPoolConfig.stVpssMode.aenInput[1] = VPSS_INPUT_ISP;
  config.stVPSSPoolConfig.stVpssMode.ViPipe[1] = 0;

  auto& vpss = config.stVPSSPoolConfig.astVpssConfig[0];
  vpss.bBindVI = true;
  vpss.u32ChnCount = 1;
  vpss.u32ChnBindVI = 0;
  VPSS_GRP_DEFAULT_HELPER2(&vpss.stVpssGrpAttr, sensor_size.u32Width,
                           sensor_size.u32Height, VI_PIXEL_FORMAT, 1);
  VPSS_CHN_DEFAULT_HELPER(&vpss.astVpssChnAttr[0], kOutputWidth,
                          kOutputHeight, VI_PIXEL_FORMAT, true);

  SAMPLE_TDL_Get_Input_Config(&config.stVencConfig.stChnInputCfg);
  config.stVencConfig.u32FrameWidth = kOutputWidth;
  config.stVencConfig.u32FrameHeight = kOutputHeight;
  SAMPLE_TDL_Get_RTSP_Config(&config.stRTSPConfig.stRTSPConfig);

  SAMPLE_TDL_MW_CONTEXT context{};
  result = SAMPLE_TDL_Init_WM(&config, &context);
  if (result != CVI_SUCCESS) {
    std::cerr << "middleware初始化失败, CVI_S32=" << result << '\n';
    CVI_MSG_Deinit();
    return 4;
  }

  int exit_code = 0;
  VIDEO_FRAME_INFO_S frame{};
  result = CVI_VPSS_GetChnFrame(0, VPSS_CHN0, &frame, 3000);
  if (result != CVI_SUCCESS) {
    std::cerr << "VPSS取帧失败, CVI_S32=" << result << '\n';
    exit_code = 5;
  } else {
    const VIDEO_FRAME_S& video = frame.stVFrame;
    std::cout << "成功取得GC2083帧：\n"
              << "  传感器: " << sensor_size.u32Width << 'x'
              << sensor_size.u32Height << '\n'
              << "  VPSS输出: " << video.u32Width << 'x' << video.u32Height << '\n'
              << "  像素格式: " << static_cast<int>(video.enPixelFormat) << '\n'
              << "  stride: [" << video.u32Stride[0] << ", "
              << video.u32Stride[1] << ", " << video.u32Stride[2] << "]\n"
              << "  length: [" << video.u32Length[0] << ", "
              << video.u32Length[1] << ", " << video.u32Length[2] << "]\n"
              << "  物理地址: [0x" << std::hex << video.u64PhyAddr[0]
              << ", 0x" << video.u64PhyAddr[1] << ", 0x"
              << video.u64PhyAddr[2] << std::dec << "]\n";
    result = CVI_VPSS_ReleaseChnFrame(0, VPSS_CHN0, &frame);
    if (result != CVI_SUCCESS) {
      std::cerr << "VPSS释放帧失败, CVI_S32=" << result << '\n';
      exit_code = 6;
    }
  }

  SAMPLE_TDL_Destroy_MW(&context);
  CVI_MSG_Deinit();
  return exit_code;
}
