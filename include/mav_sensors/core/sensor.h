//
// Created by acey on 27.06.23.
//

#pragma once

#include <type_traits>

#include "driver.h"
#include "sensor_config.h"

class SensorType {

};

template <typename D, typename... S>
class Sensor {
  static_assert(std::is_base_of<Driver, D>(), "Driver class does not inherit driver");
  static_assert((std::is_base_of_v<SensorType, S> && ...), "Invalid.");

 public:
  explicit Sensor(Driver* drv) : drv_(drv) {};

  bool open() = 0;

  template <typename T>
  typename std::enable_if<(std::is_same<T, S>::value || ...), T>::type::ReturnType read() {
    return {};
  }

  template <typename...T>
  std::tuple<typename std::enable_if<((std::is_same<T, S>::value || ...) && sizeof...(T) > 1), T>::type::ReturnType...> read() {
    return {};
  }

  bool close() = 0;

 private:
  Driver* drv_ {nullptr};
};
