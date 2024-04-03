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