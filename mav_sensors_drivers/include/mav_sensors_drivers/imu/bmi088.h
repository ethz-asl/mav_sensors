//
// Created by brik on 13.03.23.
//

#pragma once

#include <optional>
#include <string>
#include <utility>

#include <bmi08_defs.h>
#include <bmi08x.h>
#include <linux/spi/spidev.h>
#include <log++.h>
#include <mav_sensors_core/sensor.h>
#include <mav_sensors_core/sensor_config.h>

#include "mav_sensors_core/common/constants.h"
#include "mav_sensors_core/protocols/Spi.h"
#include "mav_sensors_drivers/sensor_types/Accelerometer.h"
#include "mav_sensors_drivers/sensor_types/Gyroscope.h"
#include "mav_sensors_drivers/sensor_types/Time.h"

template <typename HardwareProtocol>
class Bmi088 : public Sensor<HardwareProtocol, Time, Accelerometer, Gyroscope> {
  static_assert(std::is_same<HardwareProtocol, Spi>::value,
                "Bmi088 only supports SPI at the moment.");
  typedef Sensor<HardwareProtocol, Time, Accelerometer, Gyroscope> super;

 public:
  /**
   * Bmi088 Constructor
   * @param path to accelerometer spidev, e.g., "/dev/spidev0.0".
   * @param path to gyro spidev, e.g., "/dev/spidev0.1".
   */
  explicit Bmi088(SensorConfig cfg);

  template <typename... T>
  std::tuple<typename T::ReturnType...> read() = delete;

  bool open() override;

  /**
   * Free file descriptor
   * @return true if successful, otherwise false and errno is set.
   */
  bool close() final;

  /*!
   *  @brief Reads accelerometer and gyroscope config from registers and prints them out.
   *  @return void.
   */
  void printImuConfig();

  /**
   * Burst read to get all values at once.
   * @return struct with all values.
   */

  typename super::TupleReturnType read() override;

 private:
  /*!
  *  @brief Calls the device self test function. A reset should be performed afterwards.

  *  @return Success of selftest.
  */
  bool selftest();

  /*!
  *  @brief Sets up the SPI communication with BMI after a reset.

  *  @return Successful communication test.
  */
  bool setupBmiSpi();

  // Read function for BMI088 to be passed to BMI device driver.
  static int8_t readReg(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);

  // Write function for BMI088 to be passed to BMI device driver.
  static int8_t writeReg(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);

  /*!
   *  @brief Sleep ms function for BMI088 to be passed to BMI device driver.
   *
   *  @param[in] period   : Sleep time in micro seconds.
   *  @param[in] intf_ptr : Void pointer that can enable the linking of descriptors for interface
   * related callbacks.
   *
   *  @return void.
   */
  static void usSleep(uint32_t period, [[maybe_unused]] void *intf_ptr);

  /*!
   *  @brief Prints the execution status of the BMI APIs.
   *
   *  @param[in] api_name : Name of the API whose execution status has to be printed.
   *  @param[in] rslt     : Error code returned by the API whose execution status has to be printed.
   *
   *  @return void.
   */
  void printErrorCodeResults(const std::string &api_name, int8_t rslt) const;

  /*!
  *  @brief Prints the gyro bandwidth setting.

  *  @return void.
  */
  void printGyroBw() const;

  /*!
  *  @brief Prints the gyro output data rate setting.

  *  @return void.
  */
  void printGyroOdr() const;

  /*!
  *  @brief Computes the accelerometer range from settings.

  *  @return Accelerometer range in m/s^2.
  */
  static uint8_t computeAccRange(uint8_t accel_cfg_range);

  /*!
  *  @brief Computes the accelerometer bandwidth from settings.

  *  @return Accelerometer bandwidth in Hz.
  */
  static uint8_t computeAccBw(uint8_t accel_cfg_bw);

  /*!
  *  @brief Computes the accelerometer output data rate (ODR) from settings.

  *  @return Accelerometer output data rate in Hz.
  */
  static double computeAccOdr(uint16_t accel_cfg_odr);

  /*!
  *  @brief Computes the gyroscope range from settings.

  *  @return Gyroscope range in dps.
  */
  static uint16_t computeGyroRange(uint16_t gyro_cfg_range);

  /*!
   * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
   * range 2G, 4G, 8G or 16G.
   *
   * @param val: 16-bit accelerometer register value.
   * @param g_range: Accelerometer range, e.g., BMI088_ACCEL_RANGE_24G.
   *
   * @return Acceleration in m/s
   */
  static double lsbToMps2(int16_t val, int8_t g_range);

  /*!
   * @brief This function converts lsb to degree per second for 16 bit gyro at
   * range 125, 250, 500, 1000 or 2000dps.
   *
   * @param val: 16-bit gyroscope register value.
   * @param g_range: Gyro range, e.g., BMI08_GYRO_RANGE_2000_DPS.
   *
   * @return Angular velocity in radians per second (RPS)
   */
  static double lsbToRps(int16_t val, uint8_t dps_range);

  HardwareProtocol acc_spi_driver_;
  HardwareProtocol gyro_spi_driver_;
  /*!
   *  @brief BMI device with communication settings and IMU configuration. The configuration will be
   * overwritten in initialization method.
   */
  bmi08_dev dev_{// Communication.
                 .intf_ptr_accel = &acc_spi_driver_,
                 .intf_ptr_gyro = &gyro_spi_driver_,
                 .intf = BMI08_SPI_INTF,
                 .variant = BMI088_VARIANT,
                 .accel_cfg = bmi08_cfg{.power = BMI08_ACCEL_PM_ACTIVE,
                                        .range = BMI088_ACCEL_RANGE_24G,
                                        .bw = BMI08_ACCEL_BW_NORMAL,
                                        .odr = BMI08_ACCEL_ODR_1600_HZ},
                 .gyro_cfg = bmi08_cfg{.power = BMI08_GYRO_PM_NORMAL,
                                       .range = BMI08_GYRO_RANGE_2000_DPS,
                                       .bw = BMI08_GYRO_BW_532_ODR_2000_HZ,
                                       .odr = BMI08_GYRO_BW_532_ODR_2000_HZ},
                 .read_write_len = 32,
                 .read = &(Bmi088<HardwareProtocol>::readReg),
                 .write = &(Bmi088<HardwareProtocol>::writeReg),
                 .delay_us = &(Bmi088::usSleep)};

  inline static const constexpr uint32_t spi_transfer_speed_hz_ = 10000000;
  inline static const constexpr uint8_t g_range_min_ = 3;
  inline static const constexpr uint16_t dps_range_max_ = 2000;
  inline static const constexpr double acc_odr_min_ = 12.5;
  inline static const constexpr uint8_t acc_bw_osr_max_ = 4;
  inline static const constexpr uint16_t gyro_odr_max_ = 2000;
  inline static const constexpr uint16_t gyro_bw_max_ = 532;
  inline static const constexpr uint32_t half_scale_ = 1 << (BMI08_16_BIT_RESOLUTION - 1);
  SensorConfig cfg_;
};

template <>
Bmi088<Spi>::Bmi088(SensorConfig cfg) : cfg_(std::move(cfg)) {}

template <>
bool Bmi088<Spi>::setupBmiSpi() {
  // Initialize accelerometer SPI.
  int8_t rslt = bmi08xa_init(&dev_);
  printErrorCodeResults("bmi08xa_init", rslt);
  if (rslt != BMI08_OK || dev_.accel_chip_id != BMI088_ACCEL_CHIP_ID) {
    LOG(E,
        "Failed accelerometer SPI initialization. Chip id: 0x" << std::hex << +dev_.accel_chip_id);
    return false;
  }
  LOG(I, "Accel SPI initialized. Chip id: 0x" << std::hex << +dev_.accel_chip_id);

  // Initialize gyroscope SPI.
  rslt = bmi08g_init(&dev_);
  printErrorCodeResults("bmi08g_init", rslt);
  if (rslt != BMI08_OK || dev_.gyro_chip_id != BMI08_GYRO_CHIP_ID) {
    LOG(E, "Failed gyroscope SPI initialization. Chip id: 0x" << std::hex << +dev_.gyro_chip_id);
    return false;
  }
  LOG(I, "Gyro SPI initialized. Chip id: 0x" << std::hex << +dev_.gyro_chip_id);

  // Soft reset.
  rslt = bmi08a_soft_reset(&dev_);
  printErrorCodeResults("bmi08a_soft_reset", rslt);
  LOG(I, rslt == BMI08_OK, "Accelerometer soft reset.");
  if (rslt != BMI08_OK) {
    return false;
  }

  rslt = bmi08g_soft_reset(&dev_);
  printErrorCodeResults("bmi08g_soft_reset", rslt);
  LOG(I, rslt == BMI08_OK, "Gyroscope soft reset.");
  if (rslt != BMI08_OK) {
    return false;
  }

  // General SPI configuration.
  rslt = bmi08a_set_power_mode(&dev_);
  printErrorCodeResults("bmi08a_set_power_mode", rslt);
  if (rslt != BMI08_OK) {
    return false;
  }

  rslt = bmi08g_set_power_mode(&dev_);
  printErrorCodeResults("bmi08g_set_power_mode", rslt);
  if (rslt != BMI08_OK) {
    return false;
  }

  // TODO(rikba): Not sure if this step is required.
  rslt = bmi08a_load_config_file(&dev_);
  printErrorCodeResults("bmi08a_load_config_file", rslt);
  if (rslt != BMI08_OK) {
    return false;
  }

  rslt = bmi08xa_set_meas_conf(&dev_);
  printErrorCodeResults("bmi08xa_set_meas_conf", rslt);
  if (rslt != BMI08_OK) {
    return false;
  }

  return true;
}

template <>
bool Bmi088<Spi>::selftest() {
  LOG(I, "Performing accelerometer selftest.");
  auto acc_rslt = bmi08xa_perform_selftest(&dev_);
  printErrorCodeResults("bmi08xa_perform_selftest", acc_rslt);
  LOG(I, "Performing gyroscope selftest.");
  auto gyro_rslt = bmi08g_perform_selftest(&dev_);
  printErrorCodeResults("bmi08g_perform_selftest", gyro_rslt);
  return (acc_rslt == BMI08_OK) && (gyro_rslt == BMI08_OK);
}

template <typename HardwareProtocol>
bool Bmi088<HardwareProtocol>::close() {
  return acc_spi_driver_.close() && gyro_spi_driver_.close();
}

template <>
bool Bmi088<Spi>::open() {
  // Create SPI drivers.
  std::optional<std::string> path_acc = cfg_.get("path_acc");
  std::optional<std::string> path_gyro = cfg_.get("path_gyro");

  if (!path_acc.has_value()) {
    LOG(E, "Sensor config must have field path_acl");
    return false;
  }

  if (!path_gyro.has_value()) {
    LOG(E, "Sensor config must have field path_gyro");
    return false;
  }

  acc_spi_driver_.setPath(path_acc.value());
  if (!acc_spi_driver_.open()) {
    LOG(E, "Accelerometer open failed: " << strerror(errno));
    return false;
  }

  gyro_spi_driver_.setPath(path_gyro.value());
  if (!gyro_spi_driver_.open()) {
    LOG(E, "Gyroscope open failed: " << strerror(errno));
    return false;
  }

  if (!acc_spi_driver_.setMode(SPI_MODE_3)) {
    LOG(E, "Accelerometer setmode failed");
    return false;
  }

  if (!gyro_spi_driver_.setMode(SPI_MODE_3)) {
    LOG(E, "Gyroscope setmode failed");
    return false;
  }

  // Reset SPI communication.
  if (!setupBmiSpi()) return false;

  // Self test.
  if (!selftest()) {
    return false;
  }

  // Another reset recommended.
  if (!setupBmiSpi()) return false;

  // Re-configure data acquisition and filtering.
  // TODO(rikba): Expose sync_cfg setting to user.
  bmi08_data_sync_cfg sync_cfg{.mode = BMI08_ACCEL_DATA_SYNC_MODE_400HZ};
  auto rslt = bmi08a_configure_data_synchronization(sync_cfg, &dev_);
  printErrorCodeResults("bmi08a_configure_data_synchronization", rslt);
  LOG(I, "Configured IMU data synchronization.");

  /*set accel interrupt pin configuration*/

  bmi08_int_cfg int_config{
      /*configure host data ready interrupt */
      .accel_int_config_1 =
          bmi08_accel_int_channel_cfg{
              .int_channel = BMI08_INT_CHANNEL_1,
              .int_type = BMI08_ACCEL_SYNC_INPUT,
              .int_pin_cfg = bmi08_int_pin_cfg{.lvl = BMI08_INT_ACTIVE_HIGH,
                                               .output_mode = BMI08_INT_MODE_PUSH_PULL,
                                               .enable_int_pin = BMI08_ENABLE}},
      /*configure Accel syncronization input interrupt pin */
      .accel_int_config_2 =
          bmi08_accel_int_channel_cfg{
              .int_channel = BMI08_INT_CHANNEL_2,
              .int_type = BMI08_ACCEL_INT_SYNC_DATA_RDY,
              .int_pin_cfg = bmi08_int_pin_cfg{.lvl = BMI08_INT_ACTIVE_HIGH,
                                               .output_mode = BMI08_INT_MODE_PUSH_PULL,
                                               .enable_int_pin = BMI08_ENABLE}},
      /*set gyro interrupt pin configuration*/
      .gyro_int_config_1 =
          bmi08_gyro_int_channel_cfg{
              .int_channel = BMI08_INT_CHANNEL_3,
              .int_type = BMI08_GYRO_INT_DATA_RDY,
              .int_pin_cfg = bmi08_int_pin_cfg{.lvl = BMI08_INT_ACTIVE_HIGH,
                                               .output_mode = BMI08_INT_MODE_PUSH_PULL,
                                               .enable_int_pin = BMI08_ENABLE}},
      /* Disable last gyro interrupt pin*/
      .gyro_int_config_2 = bmi08_gyro_int_channel_cfg{
          .int_channel = BMI08_INT_CHANNEL_4,
          .int_type = BMI08_GYRO_INT_DATA_RDY,
          .int_pin_cfg = bmi08_int_pin_cfg{.lvl = BMI08_INT_ACTIVE_HIGH,
                                           .output_mode = BMI08_INT_MODE_PUSH_PULL,
                                           .enable_int_pin = BMI08_DISABLE}}};

  /* Enable synchronization interrupt pin */
  rslt = bmi08a_set_data_sync_int_config(&int_config, &dev_);
  printErrorCodeResults("bmi08a_set_data_sync_int_config", rslt);
  return true;
}

template <>
typename Sensor<Spi, Time, Accelerometer, Gyroscope>::TupleReturnType Bmi088<Spi>::read() {
  std::tuple<Time::ReturnType, Accelerometer::ReturnType, Gyroscope::ReturnType> measurement{};
  bmi08_sensor_data acc{}, gyro{};
  auto now = std::chrono::system_clock::now();

  // TODO(rikba): This is not really burst but rather returning one after the other. Implement
  // actual burst.
  // TODO(rikba): Temperature
  auto rslt = bmi08a_get_synchronized_data(&acc, &gyro, &dev_);
  printErrorCodeResults("bmi08a_get_synchronized_data", rslt);

  if (rslt != BMI08_OK) {
    return measurement;
  }
  vec3<double> acceleration = {lsbToMps2(acc.x, dev_.accel_cfg.range),
                               lsbToMps2(acc.y, dev_.accel_cfg.range),
                               lsbToMps2(acc.z, dev_.accel_cfg.range)};
  vec3<double> gyroscope = {lsbToRps(gyro.x, dev_.gyro_cfg.range),
                            lsbToRps(gyro.y, dev_.gyro_cfg.range),
                            lsbToRps(gyro.z, dev_.gyro_cfg.range)};

  return std::make_tuple(
      std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count(),
      acceleration, gyroscope);
}

template <>
void Bmi088<Spi>::printImuConfig() {
  int8_t rslt = bmi08g_get_meas_conf(&dev_);
  printErrorCodeResults("bmi08g_get_meas_conf", rslt);

  rslt = bmi08a_get_meas_conf(&dev_);
  printErrorCodeResults("bmi08a_get_meas_conf", rslt);

  LOG(I, "accel_cfg.range: " << +computeAccRange(dev_.accel_cfg.range) << " m/s^2");
  LOG(I, "accel_cfg.bw (OSR):" << +computeAccBw(dev_.accel_cfg.bw));
  LOG(I, "accel_cfg.odr: " << computeAccOdr(dev_.accel_cfg.odr) << " Hz");
  LOG(I, "gyro_cfg.range: " << computeGyroRange(dev_.gyro_cfg.range) << " dps");
  printGyroBw();
  printGyroOdr();
}

template <typename T>
int8_t Bmi088<T>::readReg(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
  auto res =
      static_cast<Spi *>(intf_ptr)->xfer({reg_addr}, static_cast<int>(len), spi_transfer_speed_hz_);
  std::copy(res.begin(), res.end(), reg_data);
  return res.empty() ? BMI08_E_COM_FAIL : BMI08_OK;
}

template <typename T>
int8_t Bmi088<T>::writeReg(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len,
                           void *intf_ptr) {
  std::vector<uint8_t> req = {reg_addr};
  std::copy(&reg_data[0], &reg_data[len], std::back_inserter(req));
  // TODO(rikba): Implement IOCTL error.
  auto ret = static_cast<Spi *>(intf_ptr)->xfer(req, 0, spi_transfer_speed_hz_);
  return BMI08_OK;
}

template <typename T>
double Bmi088<T>::lsbToMps2(int16_t val, int8_t g_range) {
  return (mav_sensors_core::g_ * val * computeAccRange(g_range)) / half_scale_;
}

template <typename T>
double Bmi088<T>::lsbToRps(int16_t val, uint8_t dps_range) {
  return ((val) * (M_PI / 180.) * computeGyroRange(dps_range)) / half_scale_;
}

template <typename T>
uint8_t Bmi088<T>::computeAccRange(uint8_t accel_cfg_range) {
  return g_range_min_ * (1 << (accel_cfg_range - BMI088_ACCEL_RANGE_3G));
}

template <typename T>
uint16_t Bmi088<T>::computeGyroRange(uint16_t gyro_cfg_range) {
  return dps_range_max_ / (1 << (gyro_cfg_range - BMI08_GYRO_RANGE_2000_DPS));
}

template <typename T>
uint8_t Bmi088<T>::computeAccBw(uint8_t accel_cfg_bw) {
  return acc_bw_osr_max_ / (1 << (accel_cfg_bw - BMI08_ACCEL_BW_OSR4));
}

template <typename T>
double Bmi088<T>::computeAccOdr(uint16_t accel_cfg_odr) {
  return acc_odr_min_ * (1 << (accel_cfg_odr - BMI08_ACCEL_ODR_12_5_HZ));
}

template <typename T>
void Bmi088<T>::printErrorCodeResults(const std::string &api_name, int8_t rslt) const {
  if (rslt != BMI08_OK) {
    LOG(E, api_name.c_str() << "\t");
    if (rslt == BMI08_E_NULL_PTR) {
      LOG(E, "Error [" << int(rslt) << "] : Null pointer");
    } else if (rslt == BMI08_E_COM_FAIL) {
      LOG(E, "Error [" << int(rslt) << "] : Communication failure");
    } else if (rslt == BMI08_E_DEV_NOT_FOUND) {
      LOG(E, "Error [" << int(rslt) << "] : Device not found");
    } else if (rslt == BMI08_E_OUT_OF_RANGE) {
      LOG(E, "Error [" << int(rslt) << "] : Out of Range");
    } else if (rslt == BMI08_E_INVALID_INPUT) {
      LOG(E, "Error [" << int(rslt) << "] : Invalid input");
    } else if (rslt == BMI08_E_CONFIG_STREAM_ERROR) {
      LOG(E, "Error [" << int(rslt) << "] : Config stream error");
    } else if (rslt == BMI08_E_RD_WR_LENGTH_INVALID) {
      LOG(E, "Error [" << int(rslt) << "] : Invalid Read write length");
    } else if (rslt == BMI08_E_INVALID_CONFIG) {
      LOG(E, "Error [" << int(rslt) << "] : Invalid config");
    } else if (rslt == BMI08_E_FEATURE_NOT_SUPPORTED) {
      LOG(E, "Error [" << int(rslt) << "] : Feature not supported");
    } else if (rslt == BMI08_W_FIFO_EMPTY) {
      LOG(W, "Warning [" << int(rslt) << "] : FIFO empty");
    } else {
      LOG(E, "Error [" << int(rslt) << "] : Unknown error code");
    }
  }
}

template <typename T>
void Bmi088<T>::usSleep(uint32_t period, void *intf_ptr) {
  usleep(period);
}

template <typename T>
void Bmi088<T>::printGyroBw() const {
  uint16_t bw = 0xFFFF;
  if (dev_.gyro_cfg.bw == BMI08_GYRO_BW_532_ODR_2000_HZ) {
    bw = 532;
  } else if (dev_.gyro_cfg.bw == BMI08_GYRO_BW_230_ODR_2000_HZ) {
    bw = 230;
  } else if (dev_.gyro_cfg.bw == BMI08_GYRO_BW_116_ODR_1000_HZ) {
    bw = 116;
  } else if (dev_.gyro_cfg.bw == BMI08_GYRO_BW_47_ODR_400_HZ) {
    bw = 47;
  } else if (dev_.gyro_cfg.bw == BMI08_GYRO_BW_23_ODR_200_HZ) {
    bw = 23;
  } else if (dev_.gyro_cfg.bw == BMI08_GYRO_BW_12_ODR_100_HZ) {
    bw = 12;
  } else if (dev_.gyro_cfg.bw == BMI08_GYRO_BW_64_ODR_200_HZ) {
    bw = 64;
  } else if (dev_.gyro_cfg.bw == BMI08_GYRO_BW_32_ODR_100_HZ) {
    bw = 32;
  }

  if (bw != 0xFFFF) {
    LOG(I, "gyro_cfg.bw: " << bw << " Hz");
  } else {
    LOG(E, "Error printing gyro bandwidth.");
  }
}

template <typename T>
void Bmi088<T>::printGyroOdr() const {
  uint16_t odr = 0xFFFF;
  if (dev_.gyro_cfg.bw == BMI08_GYRO_BW_532_ODR_2000_HZ) {
    odr = 2000;
  } else if (dev_.gyro_cfg.odr == BMI08_GYRO_BW_230_ODR_2000_HZ) {
    odr = 2000;
  } else if (dev_.gyro_cfg.odr == BMI08_GYRO_BW_116_ODR_1000_HZ) {
    odr = 1000;
  } else if (dev_.gyro_cfg.odr == BMI08_GYRO_BW_47_ODR_400_HZ) {
    odr = 400;
  } else if (dev_.gyro_cfg.odr == BMI08_GYRO_BW_23_ODR_200_HZ) {
    odr = 200;
  } else if (dev_.gyro_cfg.odr == BMI08_GYRO_BW_12_ODR_100_HZ) {
    odr = 100;
  } else if (dev_.gyro_cfg.odr == BMI08_GYRO_BW_64_ODR_200_HZ) {
    odr = 200;
  } else if (dev_.gyro_cfg.odr == BMI08_GYRO_BW_32_ODR_100_HZ) {
    odr = 100;
  }

  if (odr != 0xFFFF) {
    LOG(I, "gyro_cfg.odr: " << odr << " Hz");
  } else {
    LOG(E, "Error printing gyro ODR.");
  }
}

template <>
template <>
std::tuple<Gyroscope::ReturnType> Bmi088<Spi>::read<Gyroscope>() {
  bmi08_sensor_data gyro{};
  auto rslt = bmi08g_get_data(&gyro, &dev_);
  printErrorCodeResults("bmi08g_get_data", rslt);

  if (rslt != BMI08_OK) {
    return std::make_tuple(std::nullopt);
  }

  return std::make_tuple(
      vec3<double>({lsbToRps(gyro.x, dev_.gyro_cfg.range), lsbToRps(gyro.y, dev_.gyro_cfg.range),
                    lsbToRps(gyro.z, dev_.gyro_cfg.range)}));
}

template <>
template <>
std::tuple<Accelerometer::ReturnType> Bmi088<Spi>::read<Accelerometer>() {
  bmi08_sensor_data acc{};
  auto rslt = bmi08a_get_data(&acc, &dev_);
  printErrorCodeResults("bmi08a_get_data", rslt);

  if (rslt != BMI08_OK) {
    return std::make_tuple(std::nullopt);
  }
  return std::make_tuple(vec3<double>{lsbToMps2(acc.x, dev_.accel_cfg.range),
                                      lsbToMps2(acc.y, dev_.accel_cfg.range),
                                      lsbToMps2(acc.z, dev_.accel_cfg.range)});
}
