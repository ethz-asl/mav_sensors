//
// Created by acey on 22.06.23.
//

#pragma once

#include <functional>
#include <optional>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <utility>

#include "mav_sensors/core/protocols/Spi.h"
#include "mav_sensors/core/sensor_config.h"
#include "mav_sensors/core/sensor_types/FluidPressure.h"

template<typename HardwareProtocol>
class BMP390 : public Sensor<HardwareProtocol, FluidPressure> {
 public:
  static_assert(std::is_same_v<HardwareProtocol, Spi>, "This sensor supports Spi only.");
  typedef Sensor<HardwareProtocol, FluidPressure> super;


  explicit BMP390(SensorConfig sensorConfig) : cfg_(std::move(sensorConfig)) {};


  bool open() override {
    return false;
  }


  typename super::TupleReturnType read() override {
    return {};
  }

  template <typename ...T>
  std::tuple<typename T::ReturnType...>read() = delete;


  bool close() override { return false; }

 private:
    HardwareProtocol* drv_;
    SensorConfig cfg_;
};
/*
template<> template<> std::tuple<FluidPressure::ReturnType> BMP390<Spi>::read<FluidPressure>() {
    return {};
}
*/