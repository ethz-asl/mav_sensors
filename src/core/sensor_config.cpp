//
// Created by acey on 27.06.23.
//

#include "mav_sensors/core/sensor_config.h"

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
