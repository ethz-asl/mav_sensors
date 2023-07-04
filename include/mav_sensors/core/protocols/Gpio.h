//
// Created by acey on 04.07.23.
//

#pragma once

#include "mav_sensors/core/driver.h"

enum class GpioDirection { IN, OUT };
enum class GpioState { HIGH, LOW };

/**
 * Interface to interact with gpios via sysfs
 */
class Gpio : public Driver {
 public:
  Gpio(int gpio_nr, std::string gpio_name, GpioDirection = GpioDirection::OUT);

  bool open() override;
  bool close() override;
  bool setDirection(GpioDirection gpio_direction);
  bool setGpioState(GpioState gpio_mode);

 private:
  std::string gpio_path_{};
  int gpio_nr_{};
  std::string gpio_name_{};
  GpioDirection direction_{};

  inline static const constexpr char* SYSFS_PATH = "/sys/class";
  inline static const constexpr char* SYSFS_EXPORT = "/export";
  inline static const constexpr char* SYSFS_UNEXPORT = "/unexport";
  inline static const constexpr char* GPIO = "/gpio";
};