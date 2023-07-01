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
    double x{std::nan("1")};
    double y{std::nan("1")};
    double z{std::nan("1")};
    double velocity{std::nan("1")};
  };
  typedef std::vector<CfarDetection> ReturnType;
};