//
// Created by brik on 01.07.23.
//

#pragma once

#include <cmath>
#include <vector>

#include "mav_sensors/core/sensor.h"

class CfarDetections : public SensorType {
 public:
  struct CfarDetection {
    float x{std::nan("1")};
    float y{std::nan("1")};
    float z{std::nan("1")};
    float velocity{std::nan("1")};
  };
  typedef std::vector<CfarDetection> ReturnType;
};