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
    double x{std::nan};
    double y{std::nan};
    double z{std::nan};
    double velocity{std::nan};
  };
  typedef std::vector<CFarDetection> ReturnType;
};