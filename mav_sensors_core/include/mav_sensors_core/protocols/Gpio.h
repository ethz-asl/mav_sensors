// Copyright (c) 2024 ETH Zurich, Autonomous Systems Lab, Mariano Biasio, Rik Girod

//
// Created by acey on 04.07.23.
//

#pragma once

#include "mav_sensors_core/driver.h"

namespace mav_sensors {

enum class GpioDirection { IN, OUT };
enum class GpioState { HIGH, LOW };

/**
 * Interface to interact with gpios via sysfs
 */
class Gpio : public Driver {
 public:
  /**
   * @param gpio_nr gpio number (e.g. gpio-443)
   * @param name sysfs filename of exported pin
   * @param direction gpio direction
   */
  Gpio(int gpio_nr, const std::string& name, GpioDirection direction = GpioDirection::OUT);

  /**
   * @brief Exports the gpio
   * @return true if successful, otherwise false and errno is set
   */
  bool open() override;

  /**
   * @brief Unexports the gpio
   * @return true if successful, otherwise false and errno is set
   */
  bool close() override;

  [[nodiscard]] bool setDirection(GpioDirection gpio_direction);
  [[nodiscard]] bool setGpioState(GpioState gpio_state) const;

  /**
   * @brief Checks if the gpio pin is exported by sysfs. A gpio needs to be exported before it can
   * be configured and used.
   * @return true if exported, otherwise false
   */
  [[nodiscard]] bool isExported() const;

 private:
  int gpio_nr_{};
  GpioDirection direction_{};

  inline static const constexpr char* SYSFS_PATH = "/sys/class";
  inline static const constexpr char* SYSFS_EXPORT = "/export";
  inline static const constexpr char* SYSFS_UNEXPORT = "/unexport";
  inline static const constexpr char* GPIO = "/gpio";
};

}  // namespace mav_sensors