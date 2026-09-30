#include "scan_assembler.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace lakibeam {
namespace {
uint16_t u16(const uint8_t *p) { return p[0] | (uint16_t(p[1]) << 8); }
uint32_t u32(const uint8_t *p) { return u16(p) | (uint32_t(u16(p + 2)) << 16); }
}  // namespace

ScanAssembler::ScanAssembler(double frequency, Publish publish)
    : period_(1. / frequency), publish_(std::move(publish)) {
  if (!std::isfinite(frequency) || frequency <= 0. || frequency > 100.)
    throw std::invalid_argument("scanfreq must be in (0,100] Hz");
}

bool ScanAssembler::packet(const uint8_t *data, size_t size, int64_t receive_ns) {
  auto reject = [this] { ++rejected_packets_; return false; };
  if (size != 1206) return reject();
  std::vector<int> blocks;
  for (int i = 0; i < 12; ++i) {
    const auto flag = u16(data + i * 100), angle = u16(data + i * 100 + 2);
    if (flag == 0xffff && angle == 0xffff) continue;  // padded final packet
    if (flag != 0xeeff || angle >= 36000) return reject();
    blocks.push_back(i);
  }
  if (blocks.empty()) return reject();
  // Infer only from physically adjacent valid blocks, including modulo wrap.
  // Never infer resolution from a packet gap or a backwards/duplicate block.
  double step = step_;
  for (size_t i = 1; i < blocks.size(); ++i) {
    if (blocks[i] != blocks[i - 1] + 1) continue;
    const int a = u16(data + blocks[i - 1] * 100 + 2);
    const int b = u16(data + blocks[i] * 100 + 2);
    const int delta = (b - a + 36000) % 36000;
    if (delta != 200 && delta != 400) return reject();
    if (step != 0. && step != delta / 16.) return reject();
    step = delta / 16.;
  }
  if (step == 0.) return reject();
  const uint32_t device_us = u32(data + 1200);
  if (clock_ready_) {
    const uint32_t delta_us = device_us - last_device_us_;
    // Serial arithmetic handles uint32 wrap without accepting late packets.
    if (delta_us == 0 || delta_us >= 0x80000000u) {
      // Device restart/hour rollover is only re-anchored after a receive gap.
      if (receive_ns - last_receive_ns_ <= 1000000000LL) return reject();
      clock_ready_ = false;
    } else if (delta_us > 1000000u || receive_ns < last_receive_ns_ ||
               receive_ns - last_receive_ns_ > 1000000000LL) {
      clock_ready_ = false;
    } else {
      packet_ns_ += int64_t(delta_us) * 1000;
    }
  }
  const double point_ns = period_ * step / 36000. * 1e9;
  if (!clock_ready_) {
    // Anchor relative device time to ROS on receipt of the first complete
    // datagram. Estimate the first ray by subtracting its acquisition span.
    // This is not PTP synchronization; constant transport latency remains.
    packet_ns_ = receive_ns - std::llround((blocks.back() * 16 + 15) * point_ns);
    started_ = have_angle_ = period_measured_ = false;
    clock_ready_ = true;
  }
  step_ = step;
  last_device_us_ = device_us;
  last_receive_ns_ = receive_ns;
  for (int block : blocks) {
    const double base = u16(data + block * 100 + 2);
    for (int i = 0; i < 16; ++i) {
      const auto *point = data + block * 100 + 4 + i * 6;
      const double angle = std::fmod(base + step * i, 36000.);
      ray(angle, u16(point), point[2],
          packet_ns_ + std::llround((block * 16 + i) * point_ns));
    }
  }
  return true;
}

void ScanAssembler::begin(int64_t time_ns) {
  const auto count = static_cast<size_t>(std::llround(36000. / step_));
  ranges_.assign(count, std::numeric_limits<float>::infinity());
  intensities_.assign(count, 0.f);
  start_ns_ = time_ns;
  filled_ = 0;
  started_ = true;
}

void ScanAssembler::ray(double angle, uint16_t distance, uint8_t intensity,
                        int64_t time_ns) {
  bool boundary = !have_angle_ && angle == 0.;
  if (have_angle_) {
    const double delta = std::fmod(angle - last_angle_ + 36000., 36000.);
    if (delta == 0. || delta > 18000.) return;
    // Only a high-to-low wrap is a revolution, not a small backwards jump.
    boundary = last_angle_ >= 27000. && angle < 9000.;
  }
  if (boundary) {
    const int64_t zero_ns = time_ns - std::llround(angle / 36000. * period_ * 1e9);
    if (started_) {
      const double duration = (zero_ns - start_ns_) * 1e-9;
      // configure_sensor=false must also work when the sensor's actual scan
      // frequency differs from the launch hint. Calibrate once at a boundary.
      if (!period_measured_ && duration >= .009 && duration <= .15) {
        period_ = duration;
        period_measured_ = true;
      }
      if (filled_ > 1 && duration > .5 * period_ && duration < 1.5 * period_ &&
          start_ns_ > last_published_ns_ && start_ns_ > last_published_end_ns_) {
        publish_(Scan{start_ns_, duration, ranges_, intensities_});
        last_published_ns_ = start_ns_;
        last_published_end_ns_ = start_ns_ +
            std::llround(duration * 1e9 * (ranges_.size() - 1) / ranges_.size());
      }
    }
    begin(zero_ns);
  }
  if (started_) {
    const auto index = static_cast<size_t>(std::llround(angle / step_)) % ranges_.size();
    if (distance != 0 && distance != 0xffff) {
      ranges_[index] = distance * .001f;
      intensities_[index] = intensity;
    }
    ++filled_;
  }
  have_angle_ = true;
  last_angle_ = angle;
}
}  // namespace lakibeam
