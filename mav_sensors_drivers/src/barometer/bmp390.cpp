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

#include "mav_sensors_drivers/barometer/bmp390.h"

using namespace mav_sensors;

template <>
Temperature::ReturnType BMP390<Spi>::readTemperature() {
  bmp3_data data{std::nan("1"), std::nan("1")};
  Temperature::ReturnType measurement{};
  if (!checkErrorCodeResults("bmp3_get_status", bmp3_get_status(&status_, &dev_)))
    return measurement;

  if (status_.intr.drdy == BMP3_ENABLE) {
    if (!checkErrorCodeResults("bmp3_get_sensor_data",
                               bmp3_get_sensor_data(BMP3_PRESS_TEMP, &data, &dev_)))
      return measurement;

    measurement = data.temperature;

    /* NOTE : Read status register again to clear data ready interrupt status */
    checkErrorCodeResults("bmp3_get_status", bmp3_get_status(&status_, &dev_));
  } else {
    LOG(D, "Data not ready readTemperature");
  }
  return measurement;
}

template <>
FluidPressure::ReturnType BMP390<Spi>::readPressure() {
  bmp3_data data{std::nan("1"), std::nan("1")};
  FluidPressure::ReturnType measurement{};
  if (!checkErrorCodeResults("bmp3_get_status", bmp3_get_status(&status_, &dev_)))
    return measurement;

  if (status_.intr.drdy == BMP3_ENABLE) {
    if (!checkErrorCodeResults("bmp3_get_sensor_data",
                               bmp3_get_sensor_data(BMP3_PRESS, &data, &dev_)))
      return measurement;

    measurement = data.pressure;

    /* NOTE : Read status register again to clear data ready interrupt status */
    checkErrorCodeResults("bmp3_get_status", bmp3_get_status(&status_, &dev_));
  } else {
    LOG(D, "Data not ready readPressure");
  }
  return measurement;
}

template <>
template <>
std::tuple<Temperature::ReturnType, Time::ReturnType> BMP390<Spi>::read<Temperature, Time>() {
  return {readTemperature(), std::chrono::duration_cast<std::chrono::nanoseconds>(
                                 std::chrono::system_clock::now().time_since_epoch())
                                 .count()};
}

template <>
template <>
std::tuple<FluidPressure::ReturnType, Time::ReturnType> BMP390<Spi>::read<FluidPressure, Time>() {
  return {readPressure(), std::chrono::duration_cast<std::chrono::nanoseconds>(
                              std::chrono::system_clock::now().time_since_epoch())
                              .count()};
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