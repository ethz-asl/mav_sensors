//
// Created by brik on 01.07.23.
//

#pragma once

#include <cmath>
#include <vector>

#include "mav_sensors/core/sensor.h"

class Radar : public SensorType {
 public:
  inline Radar(size_t num_detections) : cfar_detections(num_detections) {}
  inline Radar() : Radar(0) {}
  struct CfarDetection {
    float x{std::nan("1")};
    float y{std::nan("1")};
    float z{std::nan("1")};
    float velocity{std::nan("1")};
    int16_t snr{-1};
    int16_t noise{-1};
  };
  std::vector<CfarDetection> cfar_detections;
  uint32_t hardware_stamp{0xFFFFFFFF};
  typedef Radar ReturnType;
};