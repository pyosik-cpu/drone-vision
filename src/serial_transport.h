#pragma once

#include <cstdint>
#include <string>

struct SerialArmorData {
  float center_x_px = 0.0F;
  float center_y_px = 0.0F;
  float x_m = 0.0F;
  float y_m = 0.0F;
  float z_m = 0.0F;
};

// 向下位机发送一行ASCII数字：x,y,z,center_x,center_y\n。
class SerialTransport {
 public:
  SerialTransport() = default;
  ~SerialTransport();
  SerialTransport(const SerialTransport&) = delete;
  SerialTransport& operator=(const SerialTransport&) = delete;

  bool Open(const std::string& device, int baud_rate, std::string& error);
  bool Send(const SerialArmorData& data, std::string& error);
  void Close();
  bool is_open() const { return file_descriptor_ >= 0; }

 private:
  int file_descriptor_ = -1;
};
