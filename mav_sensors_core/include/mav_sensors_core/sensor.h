//
// Created by acey on 27.06.23.
//

#pragma once

#include <type_traits>
#include <utility>

#include "driver.h"
#include "sensor_config.h"

namespace mav_sensors {

class SensorType {
  typedef void ReturnType;
};

template <typename D, typename... S>
class Sensor {
  static_assert(std::is_base_of<Driver, D>(), "Driver class does not inherit driver");
  static_assert((std::is_base_of_v<SensorType, S> && ...),
                "One or more SensorTypes do not inherit from SensorType.");

 public:
  using TupleReturnType =
      std::tuple<typename S::ReturnType...>;  // Tuple with values from all sensors
  template <typename... T>
  static constexpr bool IsValidReturnType =
      ((std::is_same<T, S>::value || ...) && sizeof...(T) > 0);

  static_assert(IsValidReturnType<S...>);

  explicit Sensor() = default;
  explicit Sensor(SensorConfig cfg) : cfg_(std::move(cfg)) {}
  virtual bool open() = 0;
  virtual TupleReturnType read() = 0;
  inline void setConfig(SensorConfig cfg) { cfg_ = std::move(cfg); }
  virtual bool close() = 0;

 protected:
  SensorConfig cfg_;
};

}  // namespace mav_sensors