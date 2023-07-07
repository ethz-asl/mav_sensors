//
// Created by acey on 07.07.23.
//

#pragma once

#include <optional>

#include <mav_sensors_core/sensor.h>

#include "mav_sensors_core/common/vec.h"

class Gyroscope : public SensorType {
 public:
  typedef std::optional<vec3<double>> ReturnType;
};