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
#include "mav_sensors/impl/barometer/bmp390_spi_ids.h"

using namespace bmp390;

template <typename HardwareProtocol>
class BMP390 : public Sensor<HardwareProtocol, FluidPressure, Temperature> {
 public:
  typedef Sensor<HardwareProtocol, FluidPressure, Temperature> super;

  static_assert(std::is_same_v<HardwareProtocol, Spi>, "This sensor supports Spi only.");

  /**
   *
   * @param sensorConfig
   *
   */
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
    usleep(1e3);

    std::vector<byte> res = drv_->xfer({0x80}, 2, 1000000);

    if (res[1] != 0x60) {
      LOG(E, "Chip ID read failed");
      return false;
    }
    LOG(I, "Chip ID: " << std::hex << +res[0] << " " << +res[1]);
    usleep(1e3);
    return true;
  }

  typename super::TupleReturnType read() override {
    return std::make_tuple(readPressure(), readTemperature());
  }

  template <typename... T>
  std::tuple<typename T::ReturnType...> read() = delete;

  bool close() override { return false; }

 private:
  byte setEightBit(byte byte);
  Temperature::ReturnType readTemperature();
  FluidPressure::ReturnType readPressure();
  Spi* drv_{nullptr};
  SensorConfig cfg_;
};

template <typename HardwareProtocol>
byte BMP390<HardwareProtocol>::setEightBit(byte byte) {
  return byte | 0x80;
}

template <>
Temperature::ReturnType BMP390<Spi>::readTemperature() {
  std::vector<byte> a = drv_->xfer({setEightBit(TEMPERATURE_DATA), 0x00}, 2, 1000000);

  if (a.empty()) {
    LOG(E, "Temperature read failed");
    return {};
  }

  LOG(I, "a size: " << a.size());
  LOG(I, "Temp: " << std::hex << +a[0] << " " << +a[1]);
  return 0;
}

template <>
FluidPressure::ReturnType BMP390<Spi>::readPressure() {
  std::vector<byte> a = drv_->xfer({setEightBit(PRESSURE_DATA), 0x00}, 2, 1000000);
  LOG(I, "Pressure: " << std::hex << +a[0] << " " << +a[1]);
  return 0;
}

template <>
template <>
std::tuple<Temperature::ReturnType> BMP390<Spi>::read<Temperature>() {
  readTemperature();
  return {};
}

template <>
template <>
std::tuple<FluidPressure::ReturnType> BMP390<Spi>::read<FluidPressure>() {
  readPressure();
  return {};
}
