// Copyright (c) 2024 ETH Zurich, Autonomous Systems Lab, Mariano Biasio, Rik Girod

//
// Created by acey on 27.06.23.
//

#include "mav_sensors_core/sensor_config.h"

using namespace mav_sensors;

SensorConfig::SensorConfig() = default;

void SensorConfig::set(const std::string& key, const std::string& value) {
  cfg_.insert({key, value});
}

std::optional<std::string> SensorConfig::get(const std::string& key) const {
  auto it = cfg_.find(key);

  if (it != cfg_.end()) {
    return it->second;
  }

  return std::nullopt;
}
