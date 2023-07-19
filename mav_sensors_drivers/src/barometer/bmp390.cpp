//
// Created by acey on 22.06.23.
//

#include "mav_sensors_drivers/barometer/bmp390.h"

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