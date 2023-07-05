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

  // Read function for BMP390 to be passed to BMP device driver.
  static int8_t readReg(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);

  // Write function for BMP390 to be passed to BMP device driver.
  static int8_t writeReg(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);

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

    if (!drv_.setMode(SPI_MODE_3)) {
      return false;
    }
    usleep(1e3);

    std::vector<byte> res = drv_.xfer({setEightBit(CHIP_ID)}, 2, 1000000);

    if (res[1] != CHIP_ID_DEFAULT) {
      LOG(E, "Chip ID read failed");
      return false;
    }
    LOG(I, "Chip ID: 0x" << std::hex << +res[1]);
    usleep(1e3);

    // Read error register
    res = drv_.xfer({setEightBit(ERR_REG)}, 2, 1000000);
    LOG(I, "Error register before reset: 0b" << std::bitset<CHAR_BIT>{+(res[1] & 0b111)});

    // Read status register
    res = drv_.xfer({setEightBit(STATUS)}, 2, 1000000);
    LOG(I, "Status register: 0b" << std::bitset<CHAR_BIT>{+(res[1] & 0b1110000)});

    // Reset the device.
    res = drv_.xfer({CMD, 0xB6}, 0, 1000000);

    // Wait for reset ready
    while (!(drv_.xfer({setEightBit(STATUS)}, 2, 1000000)[1] & 0b10000)) {
      LOG(I, "Resetting device...");
      usleep(1e5);
    }

    // Check device is sleeping

    // Read power control settings.
    res = drv_.xfer({setEightBit(PWR_CTRL)}, 2, 1000000);
    LOG(I, "PWR_CTRL before settings: 0b" << std::bitset<CHAR_BIT>{res[1]});
    usleep(1e5);

    // // Set osrs_p to 8x, osrs_t to 1x
    // res = drv_.xfer({OSR, 0b000011}, 0, 1000000);
    // //  Read OSR settings.
    // res = drv_.xfer({setEightBit(OSR)}, 2, 1000000);
    // LOG(I, "OSR: 0b" << std::bitset<CHAR_BIT>{res[1]});

    // // Set ODR to 50 Hz.
    // res = drv_.xfer({ODR, 0x02}, 0, 1000000);
    // // Read ODR settings.
    // res = drv_.xfer({setEightBit(ODR)}, 2, 1000000);
    // LOG(I, "ODR: 0b" << std::bitset<CHAR_BIT>{res[1]});

    // // Set IIR filter to 2.
    // res = drv_.xfer({CONFIG, 0b0100}, 0, 1000000);

    // Enable pressure and temperature sensor and normal mode.
    res = drv_.xfer({PWR_CTRL, 0b00110011}, 2, 1000000);
    // Read power control settings.
    res = drv_.xfer({setEightBit(PWR_CTRL)}, 2, 1000000);
    LOG(I, "PWR_CTRL: 0b" << std::bitset<CHAR_BIT>{res[1]});
    usleep(1e5);

    res = drv_.xfer({setEightBit(ERR_REG)}, 2, 1000000);
    LOG(I, "Error register after power: 0b" << std::bitset<CHAR_BIT>{+(res[1] & 0b111)});

    return true;
  }

  typename super::TupleReturnType read() override {
    drv_.xfer({PWR_CTRL, 0b00110011}, 0, 1000000);
    usleep(1e5);
    auto res = drv_.xfer({setEightBit(ERR_REG)}, 2, 1000000);
    LOG(I, "Error register after power: 0b" << std::bitset<CHAR_BIT>{+(res[1] & 0b111)});
    return std::make_tuple(readPressure(), readTemperature());
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
  // bmp3_dev dev_{// Communication.
  //               .intf_ptr = drv_,
  //               .intf_ptr_gyro = &gyro_spi_driver_,
  //               .intf = BMI08_SPI_INTF,
  //               .variant = BMI088_VARIANT,
  //               .accel_cfg = bmi08_cfg{.power = BMI08_ACCEL_PM_ACTIVE,
  //                                      .range = BMI088_ACCEL_RANGE_24G,
  //                                      .bw = BMI08_ACCEL_BW_NORMAL,
  //                                      .odr = BMI08_ACCEL_ODR_1600_HZ},
  //               .gyro_cfg = bmi08_cfg{.power = BMI08_GYRO_PM_NORMAL,
  //                                     .range = BMI08_GYRO_RANGE_2000_DPS,
  //                                     .bw = BMI08_GYRO_BW_532_ODR_2000_HZ,
  //                                     .odr = BMI08_GYRO_BW_532_ODR_2000_HZ},
  //               .read_write_len = 32,
  //               .read = &(Bmi088::readReg),
  //               .write = &(Bmi088::writeReg),
  //               .delay_us = &(Bmi088::usSleep)};

  inline static const constexpr uint32_t spi_transfer_speed_hz_ = 10000000;
};

template <typename HardwareProtocol>
byte BMP390<HardwareProtocol>::setEightBit(byte byte) {
  return byte | 0x80;
}

template <>
Temperature::ReturnType BMP390<Spi>::readTemperature() {
  std::vector<byte> a = drv_.xfer({setEightBit(TEMPERATURE_DATA)}, 4, 1000000);

  if (a.empty()) {
    LOG(E, "Temperature read failed");
    return {};
  }

  LOG(I, "a size: " << a.size());
  LOG(I, "Temp: " << std::hex << +a[0] << " " << +a[1] << " " << +a[2] << " " << +a[3]);
  return 0;
}

template <>
FluidPressure::ReturnType BMP390<Spi>::readPressure() {
  std::vector<byte> a = drv_.xfer({setEightBit(PRESSURE_DATA)}, 4, 1000000);
  LOG(I, "Pressure: " << std::hex << +a[0] << " " << +a[1] << " " << +a[2] << " " << +a[3]);
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

// Read function for BMP390 to be passed to BMP device driver.
template <>
int8_t BMP390<Spi>::readReg(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
  auto res =
      static_cast<Spi *>(intf_ptr)->xfer({reg_addr}, static_cast<int>(len), spi_transfer_speed_hz_);
  std::copy(res.begin(), res.end(), reg_data);
  return res.empty() ? BMP3_E_COMM_FAIL : BMP3_OK;
}

// Write function for BMP390 to be passed to BMP device driver.
template <>
int8_t BMP390<Spi>::writeReg(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len,
                             void *intf_ptr) {
  std::vector<uint8_t> req = {reg_addr};
  std::copy(&reg_data[0], &reg_data[len], std::back_inserter(req));
  // TODO(rikba): Implement IOCT error.
  auto ret = static_cast<Spi *>(intf_ptr)->xfer(req, 0, spi_transfer_speed_hz_);
  return BMP3_OK;
}
