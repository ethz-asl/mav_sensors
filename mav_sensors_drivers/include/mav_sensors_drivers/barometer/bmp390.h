/*
BSD 3-Clause License

Copyright (c) 2024, ETH Zurich, Autonomous Systems Lab, Mariano Biasio, Rik Girod

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

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

#include "mav_sensors_core/protocols/Spi.h"
#include "mav_sensors_core/sensor_config.h"
#include "mav_sensors_drivers/sensor_types/FluidPressure.h"
#include "mav_sensors_drivers/sensor_types/Temperature.h"
#include "mav_sensors_drivers/sensor_types/Time.h"

namespace mav_sensors {

template <typename HardwareProtocol>
class BMP390 : public Sensor<HardwareProtocol, FluidPressure, Temperature, Time> {
 public:
  typedef Sensor<HardwareProtocol, FluidPressure, Temperature, Time> super;

  static_assert(std::is_same_v<HardwareProtocol, Spi>, "This sensor supports Spi only.");

  /**
   *
   * @param SensorConfig cfg
   *
   */
  explicit BMP390(SensorConfig cfg) : super(cfg){};

  explicit BMP390() = default;

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

  bool open() override {
    std::optional<std::string> path = super::cfg_.get("path");

    if (!path.has_value()) {
      LOG(E, "Sensor config must have field path");
      return false;
    }

    drv_.setPath(path.value());
    if (!drv_.open()) {
      return false;
    }

    if (!drv_.setMode(SPI_MODE_0)) {
      return false;
    }
    usleep(1e3);

    // Init.
    if (!checkErrorCodeResults("bmp3_init", bmp3_init(&dev_))) return false;

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
    if (!checkErrorCodeResults("bmp3_set_sensor_settings",
                               bmp3_set_sensor_settings(settings_sel, &settings_, &dev_)))
      return false;

    // Operation mode
    settings_.op_mode = BMP3_MODE_NORMAL;
    if (!checkErrorCodeResults("bmp3_set_op_mode", bmp3_set_op_mode(&settings_, &dev_)))
      return false;
    else
      return true;
  }

  typename super::TupleReturnType read() override {
    bmp3_data data{std::nan("1"), std::nan("1")};
    std::tuple<FluidPressure::ReturnType, Temperature::ReturnType, Time::ReturnType> measurement{};
    auto now = std::chrono::system_clock::now();

    if (!checkErrorCodeResults("bmp3_get_status", bmp3_get_status(&status_, &dev_)))
      return measurement;

    if (status_.intr.drdy == BMP3_ENABLE) {
      if (!checkErrorCodeResults("bmp3_get_sensor_data",
                                 bmp3_get_sensor_data(BMP3_PRESS_TEMP, &data, &dev_)))
        return measurement;

      std::get<0>(measurement) = data.pressure;
      std::get<1>(measurement) = data.temperature;
      std::get<2>(measurement) =
          std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();

      /* NOTE : Read status register again to clear data ready interrupt status */
      checkErrorCodeResults("bmp3_get_status", bmp3_get_status(&status_, &dev_));
    } else {
      LOG(D, "Data not ready read all");
    }
    return measurement;
  }

  template <typename... T>
  std::tuple<typename T::ReturnType...> read() = delete;

  bool close() override { return false; }

 private:
  Temperature::ReturnType readTemperature();
  FluidPressure::ReturnType readPressure();

  bool checkErrorCodeResults(const std::string &api_name, int8_t rslt) const {
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

    return rslt == BMP3_OK;
  }

  Spi drv_;

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
  bmp3_status status_ = {{0}};

  inline static const constexpr uint32_t spi_transfer_speed_hz_ = 7500000;
};

template <>
Temperature::ReturnType BMP390<Spi>::readTemperature();

template <>
FluidPressure::ReturnType BMP390<Spi>::readPressure();
template <>
template <>
std::tuple<Temperature::ReturnType, Time::ReturnType> BMP390<Spi>::read<Temperature, Time>();

template <>
template <>
std::tuple<FluidPressure::ReturnType, Time::ReturnType> BMP390<Spi>::read<FluidPressure, Time>();

template <>
int8_t BMP390<Spi>::readReg(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);

template <>
int8_t BMP390<Spi>::writeReg(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len,
                             void *intf_ptr);

}  // namespace mav_sensors