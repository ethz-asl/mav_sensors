//
// Created by brik on 01.07.23.
//

#pragma once

#include <log++.h>

#include "mav_sensors/core/protocols/Serial.h"
#include "mav_sensors/core/sensor.h"
#include "mav_sensors/core/sensor_types/CfarDetections.h"

class Xwr18XxMmwDemo : public Sensor<Serial, CfarDetections> {
 public:
  typedef Sensor<Serial, CfarDetections> super;

  explicit Xwr18XxMmwDemo(SensorConfig sensor_cfg) : cfg_(std::move(sensor_cfg)){};

  bool open() override;

  typename super::TupleReturnType read() override;

  template <typename... T>
  std::tuple<typename T::ReturnType...> read() = delete;

  bool close() override {
    bool success = drv_cfg_.close();
    success &= drv_data_.close();
    return success;
  }

 private:
  Serial drv_cfg_;
  Serial drv_data_;
  SensorConfig cfg_;
};