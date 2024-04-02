// Copyright (c) 2024 ETH Zurich, Autonomous Systems Lab, Mariano Biasio, Rik Girod

//
// Created by acey on 04.07.23.
//

#include "mav_sensors_core/protocols/Gpio.h"

#include "mav_sensors_core/protocols/common/PosixFilesystem.h"

using namespace mav_sensors;

Gpio::Gpio(int gpio_nr, const std::string& name, GpioDirection direction)
    : direction_(direction),
      gpio_nr_(gpio_nr),
      Driver(SYSFS_PATH + std::string(GPIO) + "/" + name) {}

bool Gpio::open() {
  return PosixFilesystem::write(SYSFS_PATH + std::string(GPIO) + std::string(SYSFS_EXPORT),
                                std::to_string(gpio_nr_));
}

bool Gpio::close() {
  return PosixFilesystem::write(SYSFS_PATH + std::string(GPIO) + std::string(SYSFS_UNEXPORT),
                                std::to_string(gpio_nr_));
}

bool Gpio::setDirection(GpioDirection gpio_direction) {
  std::string direction;
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

  if (PosixFilesystem::write(path_ + "/direction", direction)) {
    direction_ = gpio_direction;
    return true;
  }

  return false;
}

bool Gpio::setGpioState(GpioState gpio_state) const {
  if (direction_ == GpioDirection::IN) {
    return false;
  }
  std::string mode;
  switch (gpio_state) {
    case GpioState::HIGH:
      mode = "1";
      break;
    case GpioState::LOW:
      mode = "0";
      break;
    default:
      return false;
  }
  return PosixFilesystem::write(path_ + "/value", mode);
}

bool Gpio::isExported() const { return PosixFilesystem::directoryExists((path_ + "/")); }
