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
// Created by acey on 25.08.22.
//

#pragma once

#include <unistd.h>

#include <cstring>
#include <iostream>
#include <string>
#include <utility>

#include <linux/spi/spidev.h>
#include <log++.h>
#include <mav_sensors_core/common/constants.h>
#include <mav_sensors_core/common/vec.h>
#include <mav_sensors_core/protocols/Spi.h>
#include <mav_sensors_core/sensor.h>
#include <mav_sensors_core/sensor_config.h>
#include <sys/ioctl.h>

#include "adis16448_cmds.h"
#include "mav_sensors_drivers/sensor_types/Accelerometer.h"
#include "mav_sensors_drivers/sensor_types/FluidPressure.h"
#include "mav_sensors_drivers/sensor_types/Gyroscope.h"
#include "mav_sensors_drivers/sensor_types/Magnetometer.h"
#include "mav_sensors_drivers/sensor_types/Temperature.h"

namespace mav_sensors {

class Adis16448
    : public Sensor<Spi, Accelerometer, FluidPressure, Gyroscope, Magnetometer, Temperature> {
  typedef Sensor<Spi, Accelerometer, FluidPressure, Gyroscope, Magnetometer, Temperature> super;

 public:
  /**
   * Adis16448 Constructor
   * @param SensorConfig
   */
  explicit Adis16448(SensorConfig cfg);
  explicit Adis16448() = default;

  bool open() override;

  int getRaw(const std::vector<byte> &cmd);

  template <typename... T>
  std::tuple<typename T::ReturnType...> read() = delete;

  /**
   * Burst reads all sensor values.
   * @return tuple with all values.
   */
  typename super::TupleReturnType read() override {
    auto res = driver_.xfer(CMD(GLOB_CMD), burst_len_, spi_burst_speed_hz_);

    if (burst_len_ == DEFAULT_BURST_LEN + 2 && !validateCrc(res)) {
      crc_error_count_++;

      // Since the adis is not synced with the host pc,
      // it is normal to have occasional checksum errors
      if (crc_error_count_ >= 5) {
        LOG_TIMED(
            E, 1,
            "DANGER: Last " << crc_error_count_ << " crc checks failed. Possible connection loss.");
      } else {
        LOG_EVERY(W, 1000, "Reported occasional checksum errors.");
      }
      return {};
    }
    crc_error_count_ = 0;

    vec3<double> gyro_raw{};
    gyro_raw.x = signedWordToInt({res[2], res[3]});
    gyro_raw.y = signedWordToInt({res[4], res[5]});
    gyro_raw.z = signedWordToInt({res[6], res[7]});

    vec3<double> raw_accel{};
    raw_accel.x = (double)signedWordToInt({res[8], res[9]});
    raw_accel.y = (double)signedWordToInt({res[10], res[11]});
    raw_accel.z = (double)signedWordToInt({res[12], res[13]});

    vec3<double> raw_magn{};
    raw_magn.x = signedWordToInt({res[14], res[15]});
    raw_magn.y = signedWordToInt({res[16], res[17]});
    raw_magn.z = signedWordToInt({res[18], res[19]});

    return {convertAcceleration(raw_accel), convertBarometer({res[20], res[21]}),
            convertGyro(gyro_raw), convertMagnetometer(raw_magn),
            convertTemperature({res[22], res[23]})};
  };

  /*!
   *  @brief Reads accelerometer and gyroscope config from registers and prints them out.
   *  @return void.
   */
  void printImuConfig();

  /**
   * Free file descriptor
   * @return true if successful, otherwise false and errno is set.
   */
  bool close() final;

  /**
   * Adis16448 Destructor
   */
  ~Adis16448();

  //! Conversion functions
  static int signedWordToInt(const std::vector<byte> &word);
  static int unsignedWordToInt(const std::vector<byte> &word);
  static bool validateCrc(const std::vector<byte> &burstData);

 private:
  static unsigned short int runCrc(const uint16_t burstData[]);
  static inline const constexpr int DEFAULT_BURST_LEN = 24;

  bool selftest();

  /**
   * Enable crc checksum check on burst read
   * @param b
   * @return true if successful, otherwise false
   */
  bool setBurstCRCEnabled(bool b);

  /**
   * Helper function to read a registry entry.
   */
  [[nodiscard]] std::vector<byte> readReg(uint8_t addr) const;

  /**
   * Helper function to overwrite a registry entry.
   */
  void writeReg(uint8_t addr, const std::vector<byte> &data, const std::string &name) const;

  /**
   * Run a test read sequence for SPI communcation.
   */
  bool testSPI();

  //! Convert spi output to measurement unit required by the ImuInterface

  /**
   * @param gyro
   * @return rad/s
   */
  static vec3<double> convertGyro(vec3<double> gyro);

  /**
   * @param accel
   * @return m/s^2
   */
  static vec3<double> convertAcceleration(vec3<double> accel);

  /**
   * @param magnetometer
   * @return tesla [T]
   */
  static vec3<double> convertMagnetometer(vec3<double> magnetometer);

  /**
   * @param word
   * @return
   */
  static double convertBarometer(const std::vector<byte> &word);

  /**
   * @param word
   * @return
   */
  static double convertTemperature(const std::vector<byte> &word);

  /**
   * Resets the Imu and turns the LED off.
   */
  void softwareReset();

  Spi driver_;
  bool is_config_valid_{true};
  int burst_len_{DEFAULT_BURST_LEN};
  int crc_error_count_{0};

  inline static const constexpr uint32_t spi_transfer_speed_hz_ = 2000000;
  inline static const constexpr uint32_t spi_burst_speed_hz_ = 1000000;
  inline static const constexpr uint32_t spi_response_size_ = 2;
  inline static const constexpr uint32_t ms_ = 100e3;
};

template <>
std::tuple<Accelerometer::ReturnType> Adis16448::read<Accelerometer>() {
  // twos complement format, 1200 LSB/g, 0 g = 0x0000
  vec3<double> acceleration{};

  acceleration.x = signedWordToInt(readReg(XACCL_OUT));
  acceleration.y = signedWordToInt(readReg(YACCL_OUT));
  acceleration.z = signedWordToInt(readReg(ZACCL_OUT));

  return convertAcceleration(acceleration);
}

template <>
std::tuple<FluidPressure::ReturnType> Adis16448::read<FluidPressure>() {
  // 20 μbar per LSB, 0x0000 = 0 mbar
  int res = unsignedWordToInt(readReg(BARO_OUT));
  return res * 0.02;
}

template <>
std::tuple<Gyroscope::ReturnType> Adis16448::read<Gyroscope>() {
  // twos complement format, 25 LSB/°/sec, 0°/sec = 0x0000
  vec3<double> gyro{};

  gyro.x = signedWordToInt(readReg(XGYRO_OUT));
  gyro.y = signedWordToInt(readReg(YGYRO_OUT));
  gyro.z = signedWordToInt(readReg(ZGYRO_OUT));

  return convertGyro(gyro);
}

template <>
std::tuple<Magnetometer::ReturnType> Adis16448::read<Magnetometer>() {
  // twos complement, 7 LSB/mgauss, 0x0000 = 0 mgauss
  vec3<double> magnetometer{};

  magnetometer.x = signedWordToInt(readReg(XMAGN_OUT));
  magnetometer.y = signedWordToInt(readReg(YMAGN_OUT));
  magnetometer.z = signedWordToInt(readReg(ZMAGN_OUT));

  return convertMagnetometer(magnetometer);
}

/**
 * Note that this temperature represents
 * an internal temperature reading, which does not precisely
 * represent external conditions. The intended use of TEMP_OUT
 * is to monitor relative changes in temperature.
 */

template <>
std::tuple<Temperature::ReturnType> Adis16448::read<Temperature>() {
  // Twos complement, 0.07386°C/LSB, 31°C = 0x0000, 12bit
  int a = signedWordToInt(readReg(TEMP_OUT));
  return 31 + (a * 0.07386);
}

}  // namespace mav_sensors