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
  explicit Gpio(int gpio_nr, GpioDirection = GpioDirection::OUT);

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

  [[nodiscard]] bool setDirection(GpioDirection gpio_direction) const;
  [[nodiscard]] bool setGpioState(GpioState gpio_state) const;

  /**
   * @brief Checks if the gpio pin is exported by sysfs. A gpio needs to be exported before it can
   * be configured and used.
   * @return true if exported, otherwise false
   */
  [[nodiscard]] bool isExported() const;

 private:
  std::string gpio_path_{};
  int gpio_nr_{};
  GpioDirection direction_{};

  inline static const constexpr char* SYSFS_PATH = "/sys/class";
  inline static const constexpr char* SYSFS_EXPORT = "/export";
  inline static const constexpr char* SYSFS_UNEXPORT = "/unexport";
  inline static const constexpr char* GPIO = "/gpio";
};