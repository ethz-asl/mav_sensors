//
// Created by brik on 01.07.23.
//

#pragma once

#include <cmath>
#include <ostream>
#include <vector>

#include "mav_sensors_core/sensor.h"

namespace mav_sensors {

class Radar : public SensorType {
 public:
  inline Radar(size_t num_detections) : cfar_detections(num_detections) {}
  inline Radar() : Radar(0) {}
  struct CfarDetection {
    float x{std::nanf("1")};
    float y{std::nanf("1")};
    float z{std::nanf("1")};
    float velocity{std::nanf("1")};
    int16_t snr{-1};
    int16_t noise{-1};
    friend std::ostream& operator<<(std::ostream& os, const CfarDetection& cd) {
      os << "x: " << cd.x << " y: " << cd.y << " z: " << cd.z << " velocity: " << cd.velocity
         << " snr: " << cd.snr << " noise: " << cd.noise;
      return os;
    }
  };
  std::vector<CfarDetection> cfar_detections;
  uint32_t hardware_stamp{0xFFFFFFFF};
  uint64_t unix_stamp_ns{0};
  typedef Radar ReturnType;

  friend std::ostream& operator<<(std::ostream& os, const Radar& r) {
    os << "Radar detections: ";
    for (const auto& detection : r.cfar_detections) {
      os << "- " << detection << " ";
    }
    os << "Hardware stamp: " << r.hardware_stamp << " Unix stamp: " << r.unix_stamp_ns;
    return os;
  }
};

}  // namespace mav_sensors