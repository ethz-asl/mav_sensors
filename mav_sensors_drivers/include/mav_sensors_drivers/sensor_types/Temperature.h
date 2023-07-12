//
// Created by acey on 28.06.23.
//

#pragma once

#include "mav_sensors_core/sensor.h"

class Temperature : public SensorType {
 public:
  typedef std::optional<double> ReturnType;
};