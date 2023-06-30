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

#include <linux/spi/spidev.h>
#include <log++.h>

#include "mav_sensors/core/protocols/Spi.h"
#include "mav_sensors/core/sensor_config.h"
#include "mav_sensors/core/sensor_types/FluidPressure.h"
#include "mav_sensors/core/sensor_types/Temperature.h"

template <typename HardwareProtocol>
class BMP390 : public Sensor<HardwareProtocol, FluidPressure, Temperature> {
 public:
  typedef Sensor<HardwareProtocol, FluidPressure, Temperature> super;

  static_assert(std::is_same_v<HardwareProtocol, Spi>, "This sensor supports Spi only.");

  explicit BMP390(SensorConfig sensorConfig) : cfg_(std::move(sensorConfig)){};

  bool open() override {
    std::optional<std::string> pathOpt = cfg_.get("path");

    if (!pathOpt.has_value()) {
      LOG(E, "Sensor config must have field path");
      return false;
    }

    drv_ = new Spi(pathOpt.value());
    if (!drv_->open()) {
      return false;
    }

    if (!drv_->setMode(SPI_MODE_3)) {
      return false;
    }

    return true;
  }

  typename super::TupleReturnType read() override {
    return std::make_tuple(0, readTemperature());
  }

  template <typename... T>
  std::tuple<typename T::ReturnType...> read() = delete;

  bool close() override { return false; }

 private:
  Temperature::ReturnType readTemperature();
  Spi* drv_{nullptr};
  SensorConfig cfg_;
};

template <typename HardwareProtocol>
Temperature::ReturnType BMP390<HardwareProtocol>::readTemperature() {
  std::vector<byte> a = drv_->xfer({0x80}, 2, 1000000);
  LOG(I, "a size: " << a.size());
  LOG(I, "Chip ID: " << std::hex << +a[0] << " " << +a[1]);
  return 0;
}
/*
template<> template<> std::tuple<FluidPressure::ReturnType> BMP390<Spi>::read<FluidPressure>() {
    return {};
}
*/