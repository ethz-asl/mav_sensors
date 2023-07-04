//
// Created by acey on 04.07.23.
//

#include "mav_sensors/core/protocols/Gpio.h"

#include <utility>

#include "mav_sensors/core/protocols/common/PosixFilesystem.h"

Gpio::Gpio(int gpio_nr, std::string gpio_name, GpioDirection direction)
    : direction_(direction), gpio_name_(std::move(gpio_name)), gpio_nr_(gpio_nr) {}

bool Gpio::open() {
  return PosixFilesystem::write(SYSFS_PATH + std::string(GPIO) + std::string(SYSFS_EXPORT),
                                std::to_string(gpio_nr_));
}

bool Gpio::close() {
  return PosixFilesystem::write(SYSFS_PATH + std::string(GPIO) + std::string(SYSFS_UNEXPORT),
                                std::to_string(gpio_nr_));
}

bool Gpio::setDirection(GpioDirection gpio_direction) { std::string direction;
  switch (gpio_direction) {
    case GpioDirection::IN:
      direction = "in";
      break;
    case GpioDirection::OUT:
      direction = "out";
      break;
    default:
      return false;
  }
  return PosixFilesystem::write(gpio_path_ + "/direction", direction); }


bool Gpio::setGpioState(GpioState gpio_mode) {
  if (direction_ == GpioDirection::IN) {
    return false;
  }
  std::string mode;
  switch (gpio_mode) {
    case GpioState::HIGH:
      mode = "1";
      break;
    case GpioState::LOW:
      mode = "0";
      break;
    default:
      return false;
  }
  return PosixFilesystem::write(gpio_path_ + "/value", mode);
}
