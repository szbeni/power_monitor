#pragma once

#include <Arduino.h>

inline void modbusCalcCrc(uint8_t* frame, size_t frameSize) {
  uint16_t temp = 0xffff;
  for (size_t i = 0; i < frameSize - 2; i++) {
    temp ^= frame[i];
    for (uint8_t j = 0; j < 8; j++) {
      const uint16_t flag = temp & 0x0001;
      temp >>= 1;
      if (flag) {
        temp ^= 0xA001;
      }
    }
  }
  frame[frameSize - 2] = temp & 0xff;
  frame[frameSize - 1] = temp >> 8;
}

inline bool modbusCheckCrc(uint8_t* frame, size_t frameSize) {
  if (frameSize < 5) {
    return false;
  }
  const uint16_t received = uint16_t(frame[frameSize - 2]) | (uint16_t(frame[frameSize - 1]) << 8);
  modbusCalcCrc(frame, frameSize);
  const uint16_t calculated = uint16_t(frame[frameSize - 2]) | (uint16_t(frame[frameSize - 1]) << 8);
  return received == calculated;
}

inline uint32_t beU32(const uint8_t* p) {
  return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | uint32_t(p[3]);
}
