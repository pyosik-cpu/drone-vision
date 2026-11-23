cmake_minimum_required(VERSION 3.16)

project(sg2002_model_info LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CVI_TPU_SDK_ROOT
    "/home/pyosik/MaixCDK/dl/extracted/cvi_tpu/cvi_tpu_lib_v4.1.0-23_2024.8.7"
    CACHE PATH "CVITEK TPU Runtime SDK 根目录")

set(CVI_RUNTIME_LIBRARY
    "${CVI_TPU_SDK_ROOT}/lib_musl/libcviruntime-static.a")
set(CVI_KERNEL_LIBRARY
    "${CVI_TPU_SDK_ROOT}/lib_musl/libcvikernel-static.a")
set(CVI_MATH_LIBRARY
    "${CVI_TPU_SDK_ROOT}/lib_musl/libcvimath-static.a")

foreach(required_path
        "${CVI_TPU_SDK_ROOT}/include/cviruntime.h"
        "${CVI_RUNTIME_LIBRARY}"
        "${CVI_KERNEL_LIBRARY}"
        "${CVI_MATH_LIBRARY}")
  if(NOT EXISTS "${required_path}")
    message(FATAL_ERROR "缺少 CVITEK SDK 文件: ${required_path}")
  endif()
endforeach()

add_executable(sg2002_model_info src/main.cpp)
target_include_directories(sg2002_model_info PRIVATE "${CVI_TPU_SDK_ROOT}/include")
target_compile_options(sg2002_model_info PRIVATE -Wall -Wextra -Wpedantic)

# 使用当前板端固件复制出的 libsys.so 验证 middleware 头文件/ABI兼容性。
set(CVI_MIDDLEWARE_ROOT
    "/home/pyosik/MaixCDK/components/3rd_party/sophgo-middleware/sophgo-middleware/v2"
    CACHE PATH "CVITEK middleware 源码根目录")
set(SG2002_BOARD_LIB_DIR "${CMAKE_SOURCE_DIR}/third_party/sg2002/lib")
add_executable(sg2002_mmf_version src/mmf_version.cpp)
target_include_directories(sg2002_mmf_version PRIVATE
  "${CVI_MIDDLEWARE_ROOT}/include"
  "${CVI_MIDDLEWARE_ROOT}/uapi"
  "${CVI_MIDDLEWARE_ROOT}/uapi/linux")
target_link_libraries(sg2002_mmf_version PRIVATE
  "${SG2002_BOARD_LIB_DIR}/libsys.so")
target_compile_options(sg2002_mmf_version PRIVATE -Wall -Wextra -Wpedantic)

# 仅供匹配middleware源码版本后的ABI诊断；当前板端libsample与MaixCDK复杂结构体不兼容。
option(BUILD_UNVERIFIED_CAMERA_PROBE
       "构建尚未通过板端ABI验证的GC2083取帧探针" OFF)
if(BUILD_UNVERIFIED_CAMERA_PROBE)
add_executable(sg2002_camera_probe
  src/camera_probe.cpp
  "${CVI_MIDDLEWARE_ROOT}/sample/vio/sophgo_middleware.c")
target_include_directories(sg2002_camera_probe PRIVATE
  "${CVI_MIDDLEWARE_ROOT}/include"
  "${CVI_MIDDLEWARE_ROOT}/include/isp/sg200x"
  "${CVI_MIDDLEWARE_ROOT}/modules/ive/include"
  "${CVI_MIDDLEWARE_ROOT}/uapi"
  "${CVI_MIDDLEWARE_ROOT}/uapi/linux"
  "${CVI_MIDDLEWARE_ROOT}/sample/common"
  "${CVI_MIDDLEWARE_ROOT}/sample/vio")
target_compile_options(sg2002_camera_probe PRIVATE
  -Wall -Wextra -Wpedantic -ffunction-sections -fdata-sections)
target_link_options(sg2002_camera_probe PRIVATE
  "-Wl,--gc-sections" "-Wl,--no-as-needed" "-Wl,--allow-shlib-undefined")
target_link_libraries(sg2002_camera_probe PRIVATE
  "${SG2002_BOARD_LIB_DIR}/libsample.so"
  "${SG2002_BOARD_LIB_DIR}/libsys.so"
  "${SG2002_BOARD_LIB_DIR}/libvi.so"
  "${SG2002_BOARD_LIB_DIR}/libvpss.so"
  "${SG2002_BOARD_LIB_DIR}/libisp.so"
  "${SG2002_BOARD_LIB_DIR}/libae.so"
  "${SG2002_BOARD_LIB_DIR}/libawb.so"
  "${SG2002_BOARD_LIB_DIR}/libaf.so"
  "${SG2002_BOARD_LIB_DIR}/libcvi_bin.so"
  "${SG2002_BOARD_LIB_DIR}/libcvi_bin_isp.so"
  "${SG2002_BOARD_LIB_DIR}/libisp_algo.so"
  "${SG2002_BOARD_LIB_DIR}/libsns_full.so"
  "${SG2002_BOARD_LIB_DIR}/libgdc.so"
  "${SG2002_BOARD_LIB_DIR}/librgn.so"
  "${SG2002_BOARD_LIB_DIR}/libvo.so"
  "${SG2002_BOARD_LIB_DIR}/libvenc.so"
  "${SG2002_BOARD_LIB_DIR}/libvdec.so"
  "${SG2002_BOARD_LIB_DIR}/libmisc.so"
  "${SG2002_BOARD_LIB_DIR}/libini.so"
  pthread dl m)
endif()

# 与板端MMF 6b03c2762匹配的Milk-V SDK v2.0.1摄像头探针。
set(DUO_SDK_V2_ROOT "/home/pyosik/duo-buildroot-sdk-v2"
    CACHE PATH "Milk-V Duo Buildroot SDK v2.0.1根目录")
set(DUO_MPI_ROOT "${DUO_SDK_V2_ROOT}/cvi_mpi")
set(DUO_TDL_ROOT "${DUO_SDK_V2_ROOT}/tdl_sdk")
add_executable(sg2002_camera_probe_v2
  src/camera_probe_v2.cpp
  "${DUO_TDL_ROOT}/sample_video/middleware_utils.c")
add_executable(sg2002_armor_camera
  src/armor_camera.cpp
  src/pose_estimator.cpp
  src/position_filter.cpp
  src/serial_transport.cpp
  "${DUO_TDL_ROOT}/sample_video/middleware_utils.c")
target_include_directories(sg2002_camera_probe_v2 PRIVATE
  "${DUO_MPI_ROOT}/include"
  "${DUO_MPI_ROOT}/include/linux"
  "${DUO_MPI_ROOT}/include/isp/cv181x"
  "${DUO_MPI_ROOT}/sample/common"
  "${DUO_MPI_ROOT}/component/isp/common"
  "${DUO_MPI_ROOT}/3rdparty/inih"
  "${DUO_TDL_ROOT}/sample_video"
  "${DUO_TDL_ROOT}/include"
  "${DUO_TDL_ROOT}/include/core/utils"
  "${DUO_SDK_V2_ROOT}/cvi_rtsp/include/cvi_rtsp")
target_include_directories(sg2002_armor_camera PRIVATE
  "${CVI_TPU_SDK_ROOT}/include"
  "${DUO_MPI_ROOT}/include"
  "${DUO_MPI_ROOT}/include/linux"
  "${DUO_MPI_ROOT}/include/isp/cv181x"
  "${DUO_MPI_ROOT}/sample/common"
  "${DUO_MPI_ROOT}/component/isp/common"
  "${DUO_MPI_ROOT}/3rdparty/inih"
  "${DUO_TDL_ROOT}/sample_video"
  "${DUO_TDL_ROOT}/include"
  "${DUO_TDL_ROOT}/include/core/utils"
  "${DUO_SDK_V2_ROOT}/cvi_rtsp/include/cvi_rtsp")
target_compile_definitions(sg2002_camera_probe_v2 PRIVATE
  _MIDDLEWARE_V2_ ARCH_CV182X)
target_compile_definitions(sg2002_armor_camera PRIVATE
  _MIDDLEWARE_V2_ ARCH_CV182X)
target_compile_options(sg2002_camera_probe_v2 PRIVATE
  -Wall -Wextra -ffunction-sections -fdata-sections)
target_compile_options(sg2002_armor_camera PRIVATE
  -Wall -Wextra -Wpedantic -ffunction-sections -fdata-sections)
target_link_options(sg2002_camera_probe_v2 PRIVATE
  "-Wl,--gc-sections" "-Wl,--no-as-needed" "-Wl,--allow-shlib-undefined")
target_link_options(sg2002_armor_camera PRIVATE
  "-Wl,--gc-sections" "-Wl,--no-as-needed" "-Wl,--allow-shlib-undefined")
target_link_libraries(sg2002_camera_probe_v2 PRIVATE
  "${SG2002_BOARD_LIB_DIR}/libsample.so"
  "${SG2002_BOARD_LIB_DIR}/libsys.so"
  "${SG2002_BOARD_LIB_DIR}/libvi.so"
  "${SG2002_BOARD_LIB_DIR}/libvpss.so"
  "${SG2002_BOARD_LIB_DIR}/libisp.so"
  "${SG2002_BOARD_LIB_DIR}/libae.so"
  "${SG2002_BOARD_LIB_DIR}/libawb.so"
  "${SG2002_BOARD_LIB_DIR}/libaf.so"
  "${SG2002_BOARD_LIB_DIR}/libcvi_bin.so"
  "${SG2002_BOARD_LIB_DIR}/libcvi_bin_isp.so"
  "${SG2002_BOARD_LIB_DIR}/libisp_algo.so"
  "${SG2002_BOARD_LIB_DIR}/libsns_full.so"
  "${SG2002_BOARD_LIB_DIR}/libgdc.so"
  "${SG2002_BOARD_LIB_DIR}/librgn.so"
  "${SG2002_BOARD_LIB_DIR}/libvo.so"
  "${SG2002_BOARD_LIB_DIR}/libvenc.so"
  "${SG2002_BOARD_LIB_DIR}/libvdec.so"
  "${SG2002_BOARD_LIB_DIR}/libmisc.so"
  "${SG2002_BOARD_LIB_DIR}/libini.so"
  "${SG2002_BOARD_LIB_DIR}/libcvi_rtsp.so"
  pthread atomic dl m)

target_link_libraries(sg2002_armor_camera PRIVATE
  "-Wl,--start-group"
  "${CVI_RUNTIME_LIBRARY}"
  "${CVI_KERNEL_LIBRARY}"
  "${CVI_MATH_LIBRARY}"
  "-Wl,--end-group"
  "${SG2002_BOARD_LIB_DIR}/libsample.so"
  "${SG2002_BOARD_LIB_DIR}/libsys.so"
  "${SG2002_BOARD_LIB_DIR}/libvi.so"
  "${SG2002_BOARD_LIB_DIR}/libvpss.so"
  "${SG2002_BOARD_LIB_DIR}/libisp.so"
  "${SG2002_BOARD_LIB_DIR}/libae.so"
  "${SG2002_BOARD_LIB_DIR}/libawb.so"
  "${SG2002_BOARD_LIB_DIR}/libaf.so"
  "${SG2002_BOARD_LIB_DIR}/libcvi_bin.so"
  "${SG2002_BOARD_LIB_DIR}/libcvi_bin_isp.so"
  "${SG2002_BOARD_LIB_DIR}/libisp_algo.so"
  "${SG2002_BOARD_LIB_DIR}/libsns_full.so"
  "${SG2002_BOARD_LIB_DIR}/libgdc.so"
  "${SG2002_BOARD_LIB_DIR}/librgn.so"
  "${SG2002_BOARD_LIB_DIR}/libvo.so"
  "${SG2002_BOARD_LIB_DIR}/libvenc.so"
  "${SG2002_BOARD_LIB_DIR}/libvdec.so"
  "${SG2002_BOARD_LIB_DIR}/libmisc.so"
  "${SG2002_BOARD_LIB_DIR}/libini.so"
  "${SG2002_BOARD_LIB_DIR}/libcvi_rtsp.so"
  pthread atomic dl m)

set(STB_INCLUDE_DIR
    "/home/pyosik/MaixCDK/components/3rd_party/sophgo-middleware/sophgo-middleware/v2/3rdparty/stb"
    CACHE PATH "包含 stb_image.h 的目录")
if(NOT EXISTS "${STB_INCLUDE_DIR}/stb_image.h")
  message(FATAL_ERROR "缺少 JPEG 解码头文件: ${STB_INCLUDE_DIR}/stb_image.h")
endif()

add_executable(sg2002_armor_detect src/armor_detect.cpp)
add_executable(sg2002_armor_batch src/armor_batch.cpp)
target_include_directories(sg2002_armor_detect PRIVATE "${CVI_TPU_SDK_ROOT}/include")
target_include_directories(sg2002_armor_detect SYSTEM PRIVATE "${STB_INCLUDE_DIR}")
target_include_directories(sg2002_armor_batch PRIVATE "${CVI_TPU_SDK_ROOT}/include")
target_include_directories(sg2002_armor_batch SYSTEM PRIVATE "${STB_INCLUDE_DIR}")
target_compile_options(sg2002_armor_detect PRIVATE -Wall -Wextra -Wpedantic)
target_compile_options(sg2002_armor_batch PRIVATE -Wall -Wextra -Wpedantic)

# Runtime 内部会引用 kernel/math；静态库用链接组处理循环依赖。
target_link_libraries(sg2002_model_info PRIVATE
  "-Wl,--start-group"
  "${CVI_RUNTIME_LIBRARY}"
  "${CVI_KERNEL_LIBRARY}"
  "${CVI_MATH_LIBRARY}"
  "-Wl,--end-group"
  pthread
  dl
  m)

target_link_libraries(sg2002_armor_detect PRIVATE
  "-Wl,--start-group"
  "${CVI_RUNTIME_LIBRARY}"
  "${CVI_KERNEL_LIBRARY}"
  "${CVI_MATH_LIBRARY}"
  "-Wl,--end-group"
  pthread
  dl
  m)

target_link_libraries(sg2002_armor_batch PRIVATE
  "-Wl,--start-group"
  "${CVI_RUNTIME_LIBRARY}"
  "${CVI_KERNEL_LIBRARY}"
  "${CVI_MATH_LIBRARY}"
  "-Wl,--end-group"
  pthread
  dl
  m)

