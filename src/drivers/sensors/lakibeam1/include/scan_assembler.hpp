#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

namespace lakibeam {
struct Scan {
  int64_t start_ns;
  double duration;
  std::vector<float> ranges, intensities;
};

// MSOP: 12 x 100-byte blocks and a 6-byte tail, little endian (vendor
// manual's byte examples). Azimuth is degrees x 100; timestamp is uint32 us.
class ScanAssembler {
public:
  using Publish = std::function<void(const Scan &)>;
  explicit ScanAssembler(double frequency, Publish publish);
  bool packet(const uint8_t *data, size_t size, int64_t receive_ns);
  size_t rejected_packets() const { return rejected_packets_; }

private:
  void ray(double angle, uint16_t distance, uint8_t intensity, int64_t time_ns);
  void begin(int64_t time_ns);
  double period_, step_ = 0., last_angle_ = 0.;
  Publish publish_;
  bool clock_ready_ = false, have_angle_ = false, started_ = false, period_measured_ = false;
  uint32_t last_device_us_ = 0;
  int64_t packet_ns_ = 0, last_receive_ns_ = 0, start_ns_ = 0;
  int64_t last_published_ns_ = -1, last_published_end_ns_ = -1;
  size_t rejected_packets_ = 0, filled_ = 0;
  std::vector<float> ranges_, intensities_;
};
}  // namespace lakibeam
