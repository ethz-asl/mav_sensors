//
// Created by acey on 27.06.23.
//

#pragma once

#include <optional>
#include <string>
#include <unordered_map>


/**
 * Sensor specific properties for initialization
 */
class SensorConfig {
 public:
  SensorConfig();

  void set(const std::string& key, const std::string& value);

  [[nodiscard]] std::optional<std::string> get(const std::string& key) const;

 private:
  std::unordered_map<std::string, std::string> cfg_{};
};
