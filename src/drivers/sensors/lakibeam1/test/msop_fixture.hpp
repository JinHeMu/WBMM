#pragma once
#include <array>
#include <cstdint>
namespace msop_test {
using Packet = std::array<uint8_t, 1206>;
void put16(uint8_t *p, uint16_t v) { p[0] = v; p[1] = v >> 8; }
Packet packet(int first_angle, uint32_t stamp, int valid = 12, int step = 400) {
  Packet out{};
  for (int block = 0; block < 12; ++block) {
    auto *p = out.data() + block * 100;
    put16(p, block < valid ? 0xeeff : 0xffff);
    put16(p + 2, block < valid ? (first_angle + block * step) % 36000 : 0xffff);
    for (int i = 0; i < 16; ++i) {
      put16(p + 4 + i * 6, 1000 + (first_angle + block * step) / 25 + i);
      p[6 + i * 6] = 42;
    }
  }
  for (int i = 0; i < 4; ++i) out[1200 + i] = stamp >> (i * 8);
  return out;
}
}
