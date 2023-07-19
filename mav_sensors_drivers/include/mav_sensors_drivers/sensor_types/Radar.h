//
// Created by brik on 01.07.23.
//

#pragma once

#include <cmath>
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
  };
  std::vector<CfarDetection> cfar_detections;
  uint32_t hardware_stamp{0xFFFFFFFF};
  uint64_t unix_stamp_ns{0};
  typedef Radar ReturnType;
};

}  // namespace mav_sensors