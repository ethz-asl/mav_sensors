//
// Created by acey on 27.06.23.
//

#pragma once

#include "mav_sensors_core/sensor.h"

class FluidPressure : public SensorType {
 public:
  typedef std::optional<double> ReturnType;
};

