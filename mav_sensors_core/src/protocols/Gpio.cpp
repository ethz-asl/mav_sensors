/*
BSD 3-Clause License

Copyright (c) 2024, ETH Zurich, Autonomous Systems Lab, Mariano Biasio, Rik Girod

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

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
