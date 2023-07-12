//
// Created by brik on 01.07.23.
//

#pragma once

#include <log++.h>
#include <mav_sensors_core/protocols/Gpio.h>
#include <mav_sensors_core/protocols/Serial.h>
#include <mav_sensors_core/sensor.h>
#include <mav_sensors_core/sensor_config.h>

#include "mav_sensors_drivers/sensor_types/Radar.h"

class Xwr18XxMmwDemo : public Sensor<Serial, Radar> {
 public:
  typedef Sensor<Serial, Radar> super;

  explicit Xwr18XxMmwDemo(SensorConfig sensor_cfg) : cfg_(std::move(sensor_cfg)){};

  bool open() override;

  typename super::TupleReturnType read() override;

  template <typename... T>
  std::tuple<typename T::ReturnType...> read() = delete;

  bool close() override {
    bool success = true;
    if (trigger_enabled_) {  // Set GPIO high to keep radar running while turning it off.
      if (!gpio_->setGpioState(GpioState::HIGH)) {
        LOG(E, "Failed to set gpio to high " << ::strerror(errno));
      }
    }
    // Stop sensor and reset hardware trigger.
    std::vector<std::string> reset_trigger{{"sensorStop"}, {"frameCfg 0 2 128 0 100 1 0"}};
    for (auto& cmd : reset_trigger) {
      cmd += "\x0D";
      if (drv_cfg_.write(cmd.c_str(), cmd.length()) != cmd.length()) {
        LOG(E, "Reset trigger failed. Radar will fail to start without hardware trigger enabled");
        success = false;
      } else {
        std::vector<byte> buf(512);
        ssize_t res = drv_cfg_.read(buf.data(), buf.size(), kPrompt.size(), 50);
        if (res <= 0) {
          LOG(E, "Error on read" << ::strerror(errno));
          success = false;
        }
        LOG(D, "Read: " << std::string(buf.begin(), buf.end()););
      }
    }
    LOG(I, "Hardware trigger reset.");

    success &= drv_cfg_.close();
    success &= drv_data_.close();

    if (gpio_.has_value()) {
      if (gpio_.value().close()) {
        LOG(I, "Closed gpio " << gpio_.value().getPath());
      } else {
        LOG(E, "Error on close " << ::strerror(errno));
        success = false;
      }
    }

    return success;
  }

 private:
  /**
   * Read xwr18xx radar config from file
   * @return
   */
  [[nodiscard]] bool loadConfig(const std::string& path) const;

  /**
   * @brief Parse a value from a byte vector in big endian format.
   */
  template <typename T>
  [[nodiscard]] T parse(const std::vector<byte>& data, size_t* offset) const {
    T value = 0;
    for (size_t i = sizeof(T); i-- > 0;) {
      value |= data[*offset + i] << (8 * i);
    }
    *offset += sizeof(T);
    return value;
  }

  bool trigger_enabled_{false};
  int trigger_delay_{500};
  std::optional<Gpio> gpio_;

  Serial drv_cfg_;
  Serial drv_data_;
  SensorConfig cfg_;

  inline static const constexpr std::string_view kPrompt{"mmwDemo:/>"};
  inline static const constexpr uint8_t kTimeout = 100;
  inline static const constexpr size_t kHeaderSize = 8 * sizeof(uint32_t);
  inline static const constexpr uint32_t kHeaderVersion = 0x03060000;  // Currently tested version.
  inline static const constexpr uint32_t kHeaderPlatform = 0xa1843;    // Currently tested platform.
  inline static const std::vector<uint8_t> kMagicKey = {0x02, 0x01, 0x04, 0x03,
                                                        0x06, 0x05, 0x08, 0x07};

  enum MmwDemoOutputMessageType {
    MMWDEMO_OUTPUT_MSG_DETECTED_POINTS = 1,
    MMWDEMO_OUTPUT_MSG_RANGE_PROFILE,
    MMWDEMO_OUTPUT_MSG_NOISE_PROFILE,
    MMWDEMO_OUTPUT_MSG_AZIMUT_STATIC_HEAT_MAP,
    MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP,
    MMWDEMO_OUTPUT_MSG_STATS,
    MMWDEMO_OUTPUT_MSG_DETECTED_POINTS_SIDE_INFO,
    MMWDEMO_OUTPUT_MSG_AZIMUT_ELEVATION_STATIC_HEAT_MAP,
    MMWDEMO_OUTPUT_MSG_TEMPERATURE_STATS,
    MMWDEMO_OUTPUT_MSG_MAX
  };
};