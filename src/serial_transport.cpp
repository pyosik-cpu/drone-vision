#include "serial_transport.h"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace {

speed_t BaudConstant(int baud_rate) {
  switch (baud_rate) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
#ifdef B230400
    case 230400: return B230400;
#endif
#ifdef B460800
    case 460800: return B460800;
#endif
    default: return 0;
  }
}

}  // namespace

SerialTransport::~SerialTransport() { Close(); }

bool SerialTransport::Open(const std::string& device, int baud_rate,
                           std::string& error) {
  Close();
  const speed_t baud = BaudConstant(baud_rate);
  if (baud == 0) {
    error = "不支持的波特率: " + std::to_string(baud_rate);
    return false;
  }

  file_descriptor_ =
      open(device.c_str(), O_RDWR | O_NOCTTY | O_CLOEXEC);
  if (file_descriptor_ < 0) {
    error = "打开" + device + "失败: " + std::strerror(errno);
    return false;
  }

  termios options{};
  if (tcgetattr(file_descriptor_, &options) != 0) {
    error = "读取串口属性失败: " + std::string(std::strerror(errno));
    Close();
    return false;
  }
  cfmakeraw(&options);
  options.c_cflag |= CLOCAL | CREAD;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CRTSCTS;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 0;
  if (cfsetispeed(&options, baud) != 0 || cfsetospeed(&options, baud) != 0 ||
      tcsetattr(file_descriptor_, TCSANOW, &options) != 0) {
    error = "配置串口失败: " + std::string(std::strerror(errno));
    Close();
    return false;
  }
  tcflush(file_descriptor_, TCIOFLUSH);
  return true;
}

bool SerialTransport::Send(const SerialArmorData& data, std::string& error) {
  if (!is_open()) {
    error = "串口未打开";
    return false;
  }

  char line[128]{};
  const int length = std::snprintf(
      line, sizeof(line), "%.4f,%.4f,%.4f,%.2f,%.2f\n", data.x_m,
      data.y_m, data.z_m, data.center_x_px, data.center_y_px);
  if (length <= 0 || static_cast<std::size_t>(length) >= sizeof(line)) {
    error = "格式化串口文本失败";
    return false;
  }

  std::size_t written = 0;
  const std::size_t total = static_cast<std::size_t>(length);
  while (written < total) {
    const ssize_t result =
        write(file_descriptor_, line + written, total - written);
    if (result > 0) {
      written += static_cast<std::size_t>(result);
    } else if (result < 0 && errno == EINTR) {
      continue;
    } else {
      error = "串口写入失败: " + std::string(std::strerror(errno));
      return false;
    }
  }
  return true;
}

void SerialTransport::Close() {
  if (file_descriptor_ >= 0) {
    close(file_descriptor_);
    file_descriptor_ = -1;
  }
}
