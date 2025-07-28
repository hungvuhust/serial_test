#include "serial/serial.h"
#include <boost/circular_buffer.hpp>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <string>
#include <unistd.h>
#include <vector>

#include "wit_c_sdk/REG.h"
#include "wit_c_sdk/wit_c_sdk.h"

using namespace std;
#define ACC_UPDATE   0x01
#define GYRO_UPDATE  0x02
#define ANGLE_UPDATE 0x04
#define MAG_UPDATE   0x08
#define READ_UPDATE  0x80

#define SWITCH_CTRL 29

std::string   port("/dev/ttyUSB0");
unsigned long baud = 115200;

serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));

struct RingBuffer {
  boost::circular_buffer<uint8_t> buffer_;
  RingBuffer(size_t size) : buffer_(size) {}
  // overload operator<<
  RingBuffer &operator+=(const std::string &data) {
    if (buffer_.size() + data.size() > buffer_.capacity()) {
      std::cout << "Buffer is full" << std::endl;
      return *this;
    }

    buffer_.resize(buffer_.size() + data.size());

    uint8_t *ptr_one = buffer_.array_one().first;
    size_t   len_one = buffer_.array_one().second;
    uint8_t *ptr_two = buffer_.array_two().first;
    size_t   len_two = buffer_.array_two().second;

    if (len_two >= data.size()) {
      std::memcpy(ptr_two + len_two - data.size(), data.data(), data.size());
    } else {
      std::memcpy(ptr_one + len_one + len_two - data.size(), data.data(),
                  data.size() - len_two);
      std::memcpy(ptr_two, data.data() + data.size() - len_two, len_two);
    }
    return *this;
  }

  std::vector<uint8_t> extract_data_delimiter(std::vector<uint8_t> delimiter) {
    size_t start = std::string::npos, end = std::string::npos;

    // Tìm vị trí bắt đầu
    for (size_t i = 0; i < buffer_.size() - 1; ++i) {
      if (buffer_[i] == delimiter[0] && buffer_[i + 1] == delimiter[1]) {
        if (start == std::string::npos) {
          start = i; // Bỏ qua delimiter đầu tiên
        } else {
          end = i; // Kết thúc ngay trước delimiter thứ hai
          break;
        }
      }
    }

    // Nếu không tìm thấy hoặc không hợp lệ, trả về vector rỗng
    if (start == std::string::npos || end == std::string::npos ||
        start >= end) {
      return {};
    }

    std::vector<uint8_t> result =
        std::vector<uint8_t>(buffer_.begin() + start, buffer_.begin() + end);

    buffer_.erase_begin(end);

    return result;
  }
};
static volatile char s_cDataUpdate = 0;

void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum) {
  int i;
  for (i = 0; i < uiRegNum; i++) {
    switch (uiReg) {
    case AX:
    case AY:
    case AZ: s_cDataUpdate |= ACC_UPDATE; break;
    case GX:
    case GY:
    case GZ: s_cDataUpdate |= GYRO_UPDATE; break;
    case HX:
    case HY:
    case HZ: s_cDataUpdate |= MAG_UPDATE; break;
    case Roll:
    case Pitch:
    case Yaw: s_cDataUpdate |= ANGLE_UPDATE; break;
    default: s_cDataUpdate |= READ_UPDATE; break;
    }
    uiReg++;
  }
}

static void SensorUartSend(uint8_t *p_data, uint32_t uiSize) {
  my_serial.write(p_data, uiSize);
  usleep(50000);
}

int main() {
  float fAcc[3], fGyro[3], fAngle[3];
  if (my_serial.isOpen()) {
    cout << "Serial Port is Open" << endl;
  } else {
    cout << "Serial Port is not Open" << endl;
  }
  WitInit(WIT_PROTOCOL_NORMAL, 0x50);
  WitRegisterCallBack(SensorDataUpdata);
  WitSerialWriteRegister(SensorUartSend);

  while (true) {
    WitReadReg(AX, 12);
    // usleep(100000);
    while (my_serial.available()) {
      auto cBuff = my_serial.read(1);
      WitSerialDataIn(cBuff[0]);
    }

    if (s_cDataUpdate) {
      for (int i = 0; i < 3; i++) {
        fAcc[i]   = sReg[AX + i] / 32768.0f * 16.0f;
        fGyro[i]  = sReg[GX + i] / 32768.0f * 2000.0f;
        fAngle[i] = sReg[Roll + i] / 32768.0f * 180.0f;
      }
      if (s_cDataUpdate & ACC_UPDATE) {
        printf("acc:%.3f %.3f %.3f\r\n", fAcc[0], fAcc[1], fAcc[2]);
        s_cDataUpdate &= ~ACC_UPDATE;
      }
      if (s_cDataUpdate & GYRO_UPDATE) {
        printf("gyro:%.3f %.3f %.3f\r\n", fGyro[0], fGyro[1], fGyro[2]);
        s_cDataUpdate &= ~GYRO_UPDATE;
      }
      if (s_cDataUpdate & ANGLE_UPDATE) {
        printf("angle:%.3f %.3f %.3f\r\n", fAngle[0], fAngle[1], fAngle[2]);
        s_cDataUpdate &= ~ANGLE_UPDATE;
      }
      if (s_cDataUpdate & MAG_UPDATE) {
        printf("mag:%d %d %d\r\n", sReg[HX], sReg[HY], sReg[HZ]);
        s_cDataUpdate &= ~MAG_UPDATE;
        printf("====================================\r\n");
      }
    }
  }

  return 0;
}