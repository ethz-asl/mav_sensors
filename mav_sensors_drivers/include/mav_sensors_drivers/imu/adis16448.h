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

template <typename HardwareProtocol>
class Adis16448 : public Sensor<HardwareProtocol, Accelerometer, FluidPressure, Gyroscope,
                                Magnetometer, Temperature> {
  // TODO add support for I2C
  static_assert(std::is_same<HardwareProtocol, Spi>::value, "Sensor only supports SPI");
  typedef Sensor<HardwareProtocol, Accelerometer, FluidPressure, Gyroscope, Magnetometer,
                 Temperature>
      super;

 public:
  /**
   * Adis16448 Constructor
   * @param path to spidev, e.g., "/dev/spidev0.1".
   */
  explicit Adis16448(SensorConfig cfg_);

  /**
   * Adis16448 Destructor
   */
  ~Adis16448();

  bool open() override;

  int getRaw(std::vector<byte> cmd);

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

  static int signedWordToInt(const std::vector<byte> &word);
  static int unsignedWordToInt(const std::vector<byte> &word);
  static bool validateCrc(const std::vector<byte> &burstData);

 private:
  static unsigned short int runCRC(const uint16_t burstData[]);
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

  HardwareProtocol driver_;
  int burst_len_{DEFAULT_BURST_LEN};
  int crc_error_count_{0};

  SensorConfig cfg_;

  inline static const constexpr uint32_t spi_transfer_speed_hz_ = 2000000;
  inline static const constexpr uint32_t spi_burst_speed_hz_ = 1000000;
  inline static const constexpr uint32_t spi_response_size_ = 2;
  inline static const constexpr uint32_t ms_ = 100e3;
};

template <typename HardwareProtocol>
Adis16448<HardwareProtocol>::Adis16448(SensorConfig cfg_) : cfg_(std::move(cfg_)) {}

template <typename HardwareProtocol>
int Adis16448<HardwareProtocol>::signedWordToInt(const std::vector<byte> &word) {
  return (((int)*(signed char *)(word.data())) * 1 << CHAR_BIT) | word[1];
}

template <typename HardwareProtocol>
int Adis16448<HardwareProtocol>::unsignedWordToInt(const std::vector<byte> &word) {
  return ((word[0] << CHAR_BIT) + word[1]);
}

// Spi specialization
template <>
std::vector<byte> Adis16448<Spi>::readReg(uint8_t addr) const {
  return driver_.xfer(CMD(addr), spi_response_size_, spi_transfer_speed_hz_);
}

template <>
void Adis16448<Spi>::writeReg(uint8_t addr, const std::vector<byte> &data,
                              const std::string &name) const {
  LOG(I, std::hex << "Adis16448 " << name.c_str() << ": 0x" << +data[0] << ", 0x" << +data[1]);
  // Set MSB
  addr = (addr & 0x7F) | 0x80;
  // Send low word.
  auto ret = driver_.xfer({addr, data[1]}, 0, spi_transfer_speed_hz_);
  // Increment address.
  addr = (addr | 0x1);
  // Send high word.
  ret = driver_.xfer({addr, data[0]}, 0, spi_transfer_speed_hz_);
}

template <typename HardwareProtocol>
bool Adis16448<HardwareProtocol>::open() {
  if (!driver_.open()) {
    LOG(E, "open failed: " << strerror(errno));
    return false;
  }

  if (!driver_.setMode(SPI_MODE_3)) {
    LOG(E, "Setmode failed");
    return false;
  }

  if (!testSPI()) {
    LOG(E, "SPI test read failed.");
    return false;
  }

  // Selftest
  if (!selftest()) {
    return false;
  }

  // Software reset.
  softwareReset();

  // Calibration factory reset.
  LOG(I, "Adis16448 factory calibration.");
  writeReg(GLOB_CMD, {0x0, 1 << 1}, "GLOB_CMD");
  usleep(ms_);

  // TODO(rikba): Gyro auto-calibration.

  // General configuration.
  LOG(I, "Adis16448 configuration.");
  std::vector<byte> msc_ctrl = {0x00, 0x06};
  writeReg(MSC_CTRL, msc_ctrl, "MSC_CTRL");

  std::vector<byte> smpl_prd = {0x00, 0x01};
  // Factor 2 decimation to reduce update cycles (bad CRC).
  // TODO(rikba): Remove when DR handling is done.
  smpl_prd[0] |= (0b00001 << 0);
  writeReg(SMPL_PRD, smpl_prd, "SMPL_PRD");

  std::vector<byte> sens_avg = {0x04, 0x04};
  // sens_avg[1] &= ~(0b111 << 0); // Clear digital filter.
  writeReg(SENS_AVG, sens_avg, "SENS_AVG");

  writeReg(ALM_CTRL, {0x00, 0x00}, "ALM_CTRL");

  std::vector<byte> gpio_ctrl = {0x00, 0x00};
  gpio_ctrl[0] &= ~(1 << 1);  // Clear DIO2 to light LED.
  gpio_ctrl[1] |= (1 << 1);   // Set DIO2 output.
  writeReg(GPIO_CTRL, gpio_ctrl, "GPIO_CTRL");

  auto burstCrc = cfg_.get("burst_crc");
  if (burstCrc.has_value()) {
    if (burstCrc.value() == "true") {
      setBurstCRCEnabled(true);
    }
    if (burstCrc.value() == "false") {
      setBurstCRCEnabled(false);
    }
  }
  return true;
}

template <>
Adis16448<Spi>::~Adis16448() {
  // TODO(rikba): This does not work...
  std::vector<byte> gpio_ctrl = readReg(GPIO_CTRL);
  gpio_ctrl[1] &= ~(1 << 1);  // Clear DIO2 output to disable LED.
  writeReg(GPIO_CTRL, gpio_ctrl, "GPIO_CTRL");
}

template <typename HardwareProtocol>
void Adis16448<HardwareProtocol>::printImuConfig() {
  auto smpl_prd = readReg(SMPL_PRD);
  auto D = smpl_prd[1] & 0b111111;
  LOG(I, "smpl_prd decimation rate variable D: " << +(D));
  LOG(I, "Output data rate (ODR): " << 819.2 / (1 << D) << " Hz");

  auto sens_avg = readReg(SENS_AVG);
  auto B = sens_avg[0] & 0b111;
  LOG(I, "sens_avg filter size variable B: " << +(B));
  LOG(I, "Bartlett windows size: " << (1 << B));

  auto gyro_range = sens_avg[1] & 0b111;
  LOG(I, "sens_avg gyro range: " << 250 * (1 << (gyro_range - 1)) << " dps");
}

template <typename HardwareProtocol>
unsigned short int Adis16448<HardwareProtocol>::runCRC(const uint16_t *burstData) {
  unsigned char i;         // Tracks each burstData word
  unsigned char ii;        // Counter for each bit of the current burstData word
  unsigned int data;       // Holds the lower/Upper byte for CRC computation
  unsigned int crc;        // Holds the CRC value
  unsigned int lowerByte;  // Lower Byte of burstData word
  unsigned int upperByte;  // Upper Byte of burstData word
  unsigned int POLY;       // Divisor used during CRC computation
  POLY = 0x1021;           // Define divisor
  crc = 0xFFFF;            // Set CRC to \f1\u8208?\f0 1 prior to beginning CRC computation
  // Compute CRC on burst data starting from XGYRO_OUT and ending with TEMP_OUT.
  // Start with the lower byte and then the upper byte of each word.
  // i.e. Compute XGYRO_OUT_LSB CRC first and then compute XGYRO_OUT_MSB CRC.
  for (i = 1; i < 12; i++) {
    upperByte = (burstData[i] >> 8) & 0xFF;
    lowerByte = (burstData[i] & 0xFF);
    data = lowerByte;  // Compute lower byte CRC first
    for (ii = 0; ii < 8; ii++, data >>= 1) {
      if ((crc & 0x0001) ^ (data & 0x0001))
        crc = (crc >> 1) ^ POLY;
      else
        crc >>= 1;
    }
    data = upperByte;  // Compute upper byte of CRC
    for (ii = 0; ii < 8; ii++, data >>= 1) {
      if ((crc & 0x0001) ^ (data & 0x0001))
        crc = (crc >> 1) ^ POLY;
      else
        crc >>= 1;
    }
  }
  crc = ~crc;  // Compute complement of CRC
  data = crc;
  crc = (crc << 8) | (data >> 8 & 0xFF);  // Perform byte swap prior to returning CRC
  return crc;
}

template <typename HardwareProtocol>
bool Adis16448<HardwareProtocol>::validateCrc(const std::vector<byte> &burstData) {
  if (burstData.size() != DEFAULT_BURST_LEN + 2) {
    return false;
  }

  int expected_crc = unsignedWordToInt({burstData[24], burstData[25]});
  uint16_t sampleAsWord[12];
  memset(sampleAsWord, 0, sizeof(sampleAsWord));

  int count = 0;

  for (int i = 0; i < 24; i += 2) {
    uint16_t a = (uint16_t)Adis16448::unsignedWordToInt({burstData[i], burstData[i + 1]});
    sampleAsWord[count] = a;
    count++;
  }

  unsigned short int actual_crc = runCRC(sampleAsWord);

  return actual_crc == expected_crc;
}

template <typename HardwareProtocol>
bool Adis16448<HardwareProtocol>::selftest() {
  // Start self test.
  LOG(I, "Adis16448 self-test.");
  auto msc_ctrl = readReg(MSC_CTRL);
  msc_ctrl[0] = (1 << 2) | msc_ctrl[0];  // Set bit 10 (3rd high bit).
  writeReg(MSC_CTRL, msc_ctrl, "MSC_CTRL");

  while (msc_ctrl[0] & (1 << 2)) {
    LOG(D, "Testing.");
    usleep(ms_);  // Self test requires 45ms. Wait 100ms.
    msc_ctrl = readReg(MSC_CTRL);
  }

  std::vector<byte> res = readReg(DIAG_STAT);

  if (res.empty()) {
    return false;
  }

  if (res[1] & (1 << 5)) {
    LOG(E, "ADIS16448 self-test failed.");
    LOG(E, res[1] & (1 << 0), "Magnetometer functional test failure.");
    LOG(E, res[1] & (1 << 1), "Barometer functional test failure.");
    LOG(E, res[0] & (1 << 2), "X-axis gyroscope self-test failure.");
    LOG(E, res[0] & (1 << 3), "Y-axis gyroscope self-test failure.");
    LOG(E, res[0] & (1 << 4), "Z-axis gyroscope self-test failure.");
    LOG(E, res[0] & (1 << 5), "X-axis accelerometer self-test failure.");
    LOG(E, res[0] & (1 << 6), "Y-axis accelerometer self-test failure.");
    LOG(E, res[0] & (1 << 7), "Z-axis accelerometer self-test failure.");

    return false;
  }

  LOG(I, "Adis16448 self-check passed");
  return true;
}

template <typename HardwareProtocol>
bool Adis16448<HardwareProtocol>::setBurstCRCEnabled(bool b) {
  if (b) {
    auto msc_ctrl = readReg(MSC_CTRL);
    msc_ctrl[1] = (1 << 4) | msc_ctrl[1];  // Set lower bit 4.
    writeReg(MSC_CTRL, msc_ctrl, "MSC_CTRL");
    usleep(ms_);  // wait 1ms
    auto res = readReg(MSC_CTRL);

    if (res[1] & (1 << 4)) {
      // increase rx buffer length by 2 bytes for 16bit crc value
      burst_len_ = DEFAULT_BURST_LEN + 2;
      LOG(I, "Enabled CRC on burst");
      return true;
    }

    LOG(E, "Error on burst mode enable");
    return false;
  } else {
    auto msc_ctrl = readReg(MSC_CTRL);
    msc_ctrl[1] = (~(1 << 4)) & msc_ctrl[1];  // Clear lower bit 4.
    writeReg(MSC_CTRL, msc_ctrl, "MSC_CTRL");
    usleep(ms_);  // wait 1ms
    auto res = readReg(MSC_CTRL);

    if (!(res[1] & (1 << 4))) {
      burst_len_ = DEFAULT_BURST_LEN;

      LOG(I, "Disabled CRC on burst");
      return true;
    }

    LOG(E, "Error on burst mode disable: " << (int)res[0] << ", " << (int)res[1]);
    return false;
  }
}

template <typename HardwareProtocol>
bool Adis16448<HardwareProtocol>::testSPI() {
  auto res = readReg(PROD_ID);

  if (res.empty()) {
    return false;
  }

  LOG(I, std::hex << "Adis16448 PROD_ID: 0x" << +res[0] << +res[1]);

  return res[0] == 0x40 && res[1] == 0x40;
}

template <typename HardwareProtocol>
void Adis16448<HardwareProtocol>::softwareReset() {
  LOG(I, "Adis16448 software reset.");
  writeReg(GLOB_CMD, {0x0, 1 << 7}, "GLOB_CMD");
  usleep(ms_);
}

template <typename HardwareProtocol>
bool Adis16448<HardwareProtocol>::close() {
  softwareReset();
  return driver_.close();
}

template <typename HardwareProtocol>
vec3<double> Adis16448<HardwareProtocol>::convertGyro(vec3<double> gyro) {
  gyro /= 25.;                  // convert to degrees
  return gyro * (M_PI / 180.);  // Convert to rad/s and return
}

template <typename HardwareProtocol>
double Adis16448<HardwareProtocol>::convertTemperature(const std::vector<byte> &word) {
  return 31 + (signedWordToInt(word) * 0.07386);
}

template <typename HardwareProtocol>
double Adis16448<HardwareProtocol>::convertBarometer(const std::vector<byte> &word) {
  return unsignedWordToInt(word) * 0.02;
}

template <typename HardwareProtocol>
vec3<double> Adis16448<HardwareProtocol>::convertMagnetometer(vec3<double> magnetometer) {
  magnetometer /= 7.;         // Convert to mG;
  magnetometer /= 10000000.;  // Convert to tesla
  return magnetometer;
}

template <typename HardwareProtocol>
vec3<double> Adis16448<HardwareProtocol>::convertAcceleration(vec3<double> accel) {
  accel /= 1200.;  // Convert to g
  return accel * mav_sensors_core::g_;
}

template <typename HardwareProtocol>
int Adis16448<HardwareProtocol>::getRaw(std::vector<byte> cmd) {
  std::vector<byte> res = driver_.xfer(cmd, spi_response_size_, spi_transfer_speed_hz_);
  return unsignedWordToInt(res);
}

template <>
template <>
std::tuple<Accelerometer::ReturnType> Adis16448<Spi>::read<Accelerometer>() {
  // twos complement format, 1200 LSB/g, 0 g = 0x0000
  vec3<double> acceleration{};

  acceleration.x = signedWordToInt(readReg(XACCL_OUT));
  acceleration.y = signedWordToInt(readReg(YACCL_OUT));
  acceleration.z = signedWordToInt(readReg(ZACCL_OUT));

  return convertAcceleration(acceleration);
}

template <>
template <>
std::tuple<FluidPressure::ReturnType> Adis16448<Spi>::read<FluidPressure>() {
  // 20 μbar per LSB, 0x0000 = 0 mbar
  int res = unsignedWordToInt(readReg(BARO_OUT));
  return res * 0.02;
}

template <>
template <>
std::tuple<Gyroscope::ReturnType> Adis16448<Spi>::read<Gyroscope>() {
  // twos complement format, 25 LSB/°/sec, 0°/sec = 0x0000
  vec3<double> gyro{};

  gyro.x = signedWordToInt(readReg(XGYRO_OUT));
  gyro.y = signedWordToInt(readReg(YGYRO_OUT));
  gyro.z = signedWordToInt(readReg(ZGYRO_OUT));

  return convertGyro(gyro);
}

template <>
template <>
std::tuple<Magnetometer::ReturnType> Adis16448<Spi>::read<Magnetometer>() {
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
template <>
std::tuple<Temperature::ReturnType> Adis16448<Spi>::read<Temperature>() {
  // Twos complement, 0.07386°C/LSB, 31°C = 0x0000, 12bit
  int a = signedWordToInt(readReg(TEMP_OUT));
  return 31 + (a * 0.07386);
}
