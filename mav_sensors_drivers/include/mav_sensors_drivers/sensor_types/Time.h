//
// Created by brik on 17.07.23.
//

#pragma once

#include "mav_sensors_core/sensor.h"

namespace mav_sensors {

class Time : public SensorType {
 public:
  typedef std::optional<uint64_t> ReturnType;
};

}  // namespace mav_sensors