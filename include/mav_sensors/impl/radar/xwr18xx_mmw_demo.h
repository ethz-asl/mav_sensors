//
// Created by brik on 01.07.23.
//

#pragma once

#include <log++.h>

#include "mav_sensors/core/protocols/Serial.h"
#include "mav_sensors/core/sensor.h"
#include "mav_sensors/core/sensor_types/CfarDetections.h"

class Xwr18XxMmwDemo : public Sensor<Serial, CfarDetections> {
 public:
  typedef Sensor<Serial, CfarDetections> super;

  explicit Xwr18XxMmwDemo(SensorConfig sensor_cfg) : cfg_(std::move(sensor_cfg)){};

  bool open() override;

  typename super::TupleReturnType read() override;

  template <typename... T>
  std::tuple<typename T::ReturnType...> read() = delete;

  bool close() override {
    bool success = drv_cfg_.close();
    success &= drv_data_.close();
    return success;
  }

 private:
  template <typename T>
  T parse(const std::vector<byte>& data, size_t* offset) {
    T value = 0;
    for (size_t i = sizeof(T); i-- > 0;) {
      value |= data[*offset + i] << (8 * i);
    }
    *offset += sizeof(T);
    return value;
  }

  Serial drv_cfg_;
  Serial drv_data_;
  SensorConfig cfg_;

  inline static const constexpr uint8_t kTimeout = 100;
  inline static const constexpr size_t kHeaderSize = 8 * sizeof(uint32_t);

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