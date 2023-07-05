//
// Created by acey on 22.06.23.
//

#pragma once

#include <bitset>
#include <functional>
#include <optional>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <utility>

#include <bmp3.h>
#include <bmp3_defs.h>
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

  /**
   *
   * @param sensorConfig
   *
   */
  explicit BMP390(SensorConfig sensorConfig) : cfg_(std::move(sensorConfig)){};

  // Read function to be passed to BMP3 device driver.
  static int8_t readReg(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);

  // Write function to be passed to BMP3 device driver.
  static int8_t writeReg(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);

  /*!
   *  @brief Sleep ms function to be passed to BMP3 device driver.
   *
   *  @param[in] period   : Sleep time in micro seconds.
   *  @param[in] intf_ptr : Void pointer that can enable the linking of descriptors for interface
   * related callbacks.
   *
   *  @return void.
   */
  static void usSleep(uint32_t period, [[maybe_unused]] void *intf_ptr) { usleep(period); }

  void printErrorCodeResults(const std::string &api_name, int8_t rslt) const {
    if (rslt != BMP3_OK) {
      LOG(E, api_name.c_str() << "\t");
      if (rslt == BMP3_E_NULL_PTR) {
        LOG(E, "Error [" << int(rslt) << "] : Null pointer");
      } else if (rslt == BMP3_E_COMM_FAIL) {
        LOG(E, "Error [" << int(rslt) << "] : Communication failure");
      } else if (rslt == BMP3_E_INVALID_ODR_OSR_SETTINGS) {
        LOG(E, "Error [" << int(rslt) << "] : Invalid ODR OSR settings");
      } else if (rslt == BMP3_E_CMD_EXEC_FAILED) {
        LOG(E, "Error [" << int(rslt) << "] : Command execution failed");
      } else if (rslt == BMP3_E_CONFIGURATION_ERR) {
        LOG(E, "Error [" << int(rslt) << "] : Configuration error");
      } else if (rslt == BMP3_E_INVALID_LEN) {
        LOG(E, "Error [" << int(rslt) << "] : Invalid length");
      } else if (rslt == BMP3_E_DEV_NOT_FOUND) {
        LOG(E, "Error [" << int(rslt) << "] : Device not found");
      } else if (rslt == BMP3_E_FIFO_WATERMARK_NOT_REACHED) {
        LOG(E, "Error [" << int(rslt) << "] : FIFO watermark not reached");
      } else {
        LOG(E, "Error [" << int(rslt) << "] : Unknown error code");
      }
    }
  }

  bool open() override {
    std::optional<std::string> pathOpt = cfg_.get("path");

    if (!pathOpt.has_value()) {
      LOG(E, "Sensor config must have field path");
      return false;
    }

    drv_.setPath(pathOpt.value());
    if (!drv_.open()) {
      return false;
    }

    if (!drv_.setMode(SPI_MODE_0)) {
      return false;
    }
    usleep(1e3);

    // Init.
    auto rslt = bmp3_init(&dev_);
    printErrorCodeResults("bmp3_init", rslt);
    if (rslt != BMP3_OK) {
      return false;
    }
    LOG(I, "BMP3 chip id is 0x" << std::hex << +dev_.chip_id);

    // Settings.
    settings_.int_settings.drdy_en = BMP3_ENABLE;
    settings_.press_en = BMP3_ENABLE;
    settings_.temp_en = BMP3_ENABLE;

    // Drone settings (TODO: make configurable)
    settings_.odr_filter.press_os = BMP3_OVERSAMPLING_8X;
    settings_.odr_filter.temp_os = BMP3_NO_OVERSAMPLING;
    settings_.odr_filter.iir_filter = BMP3_IIR_FILTER_COEFF_3;
    settings_.odr_filter.odr = BMP3_ODR_50_HZ;

    uint16_t settings_sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_OS |
                            BMP3_SEL_TEMP_OS | BMP3_SEL_ODR | BMP3_SEL_DRDY_EN;
    rslt = bmp3_set_sensor_settings(settings_sel, &settings_, &dev_);
    printErrorCodeResults("bmp3_set_sensor_settings", rslt);
    if (rslt != BMP3_OK) {
      return false;
    }

    // Operation mode
    settings_.op_mode = BMP3_MODE_NORMAL;
    rslt = bmp3_set_op_mode(&settings_, &dev_);
    printErrorCodeResults("bmp3_set_op_mode", rslt);
    if (rslt != BMP3_OK) {
      return false;
    }

    return true;
  }

  typename super::TupleReturnType read() override {
    auto measurement = std::make_tuple(FluidPressure::ReturnType(), Temperature::ReturnType());
    auto rslt = bmp3_get_status(&status_, &dev_);
    printErrorCodeResults("bmp3_get_status", rslt);
    if (rslt != BMP3_OK) return measurement;

    if (status_.intr.drdy == BMP3_ENABLE) {
      rslt = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &data_, &dev_);
      printErrorCodeResults("bmp3_get_sensor_data", rslt);
      if (rslt != BMP3_OK) return measurement;

      std::get<0>(measurement) = data_.pressure;
      std::get<1>(measurement) = data_.temperature;

      /* NOTE : Read status register again to clear data ready interrupt status */
      rslt = bmp3_get_status(&status_, &dev_);
      printErrorCodeResults("bmp3_get_status", rslt);
    } else {
      LOG(W, "Data not ready read all");
    }
    return measurement;
  }

  template <typename... T>
  std::tuple<typename T::ReturnType...> read() = delete;

  bool close() override { return false; }

 private:
  byte setEightBit(byte byte);
  Temperature::ReturnType readTemperature();
  FluidPressure::ReturnType readPressure();
  Spi drv_;
  SensorConfig cfg_;

  /*!
   *  @brief BMP device with communication settings and BMP configuration. The configuration will be
   * overwritten in initialization method.
   */
  bmp3_dev dev_{// Communication.
                .intf_ptr = &drv_,
                .intf = BMP3_SPI_INTF,
                .read = &(BMP390::readReg),
                .write = &(BMP390::writeReg),
                .delay_us = &(BMP390::usSleep)};
  bmp3_settings settings_{0};
  bmp3_data data_ = {0};
  bmp3_status status_ = {{0}};

  inline static const constexpr uint32_t spi_transfer_speed_hz_ = 7500000;
};

template <typename HardwareProtocol>
byte BMP390<HardwareProtocol>::setEightBit(byte byte) {
  return byte | 0x80;
}

template <>
Temperature::ReturnType BMP390<Spi>::readTemperature() {
  auto measurement = Temperature::ReturnType();
  auto rslt = bmp3_get_status(&status_, &dev_);
  printErrorCodeResults("bmp3_get_status", rslt);
  if (rslt != BMP3_OK) return measurement;

  if (status_.intr.drdy == BMP3_ENABLE) {
    rslt = bmp3_get_sensor_data(BMP3_TEMP, &data_, &dev_);
    printErrorCodeResults("bmp3_get_sensor_data", rslt);
    if (rslt != BMP3_OK) return measurement;

    measurement = data_.temperature;

    /* NOTE : Read status register again to clear data ready interrupt status */
    rslt = bmp3_get_status(&status_, &dev_);
    printErrorCodeResults("bmp3_get_status", rslt);
  } else {
    LOG(W, "Data not ready readTemperature");
  }
  return measurement;
}

template <>
FluidPressure::ReturnType BMP390<Spi>::readPressure() {
  auto measurement = Temperature::ReturnType();
  auto rslt = bmp3_get_status(&status_, &dev_);
  printErrorCodeResults("bmp3_get_status", rslt);
  if (rslt != BMP3_OK) return measurement;

  if (status_.intr.drdy == BMP3_ENABLE) {
    rslt = bmp3_get_sensor_data(BMP3_PRESS, &data_, &dev_);
    printErrorCodeResults("bmp3_get_sensor_data", rslt);
    if (rslt != BMP3_OK) return measurement;

    measurement = data_.pressure;

    /* NOTE : Read status register again to clear data ready interrupt status */
    rslt = bmp3_get_status(&status_, &dev_);
    printErrorCodeResults("bmp3_get_status", rslt);
  } else {
    LOG(W, "Data not ready readPressure");
  }
  return measurement;
}

template <>
template <>
std::tuple<Temperature::ReturnType> BMP390<Spi>::read<Temperature>() {
  return {readTemperature()};
}

template <>
template <>
std::tuple<FluidPressure::ReturnType> BMP390<Spi>::read<FluidPressure>() {
  return {readPressure()};
}

template <>
int8_t BMP390<Spi>::readReg(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
  auto res =
      static_cast<Spi *>(intf_ptr)->xfer({reg_addr}, static_cast<int>(len), spi_transfer_speed_hz_);
  std::copy(res.begin(), res.end(), reg_data);
  return res.empty() ? BMP3_E_COMM_FAIL : BMP3_OK;
}

template <>
int8_t BMP390<Spi>::writeReg(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len,
                             void *intf_ptr) {
  std::vector<uint8_t> req = {reg_addr};
  std::copy(&reg_data[0], &reg_data[len], std::back_inserter(req));
  // TODO(rikba): Implement IOCT error.
  auto ret = static_cast<Spi *>(intf_ptr)->xfer(req, 0, spi_transfer_speed_hz_);
  return BMP3_OK;
}