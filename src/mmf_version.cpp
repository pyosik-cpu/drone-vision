#include <cstring>
#include <iostream>

#include "cvi_sys.h"

int main() {
  // 这里只查询middleware版本，不初始化VB、VI、VPSS或摄像头。
  MMF_VERSION_S version{};
  const CVI_S32 result = CVI_SYS_GetVersion(&version);
  if (result != CVI_SUCCESS) {
    std::cerr << "CVI_SYS_GetVersion失败, CVI_S32=" << result << '\n';
    return 1;
  }

  // 厂商结构体使用定长字符数组，限制输出长度以避免异常数据越界。
  const std::size_t length = strnlen(version.version, sizeof(version.version));
  std::cout << "MMF Version: " << std::string(version.version, length) << '\n';
  return 0;
}
