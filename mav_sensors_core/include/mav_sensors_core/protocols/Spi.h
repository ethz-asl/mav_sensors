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
// Created by acey on 27.06.23.
//

#pragma once

#include <vector>

#include "mav_sensors_core/driver.h"

namespace mav_sensors {

typedef unsigned char byte;

class Spi : public Driver {
 public:
  explicit Spi();
  explicit Spi(std::string path);
  bool open() override;
  [[nodiscard]] bool setMode(uint8_t mode) const;
  void setPath(std::string path);

  /**
   * Spi half-duplex transaction
   *
   * @param cmd vector with multiple bytes
   * @param response_len response length
   * @param speed_hz SPI transfer clock speed
   * @return vector with response or empty vector on failure
   */
  [[nodiscard]] std::vector<byte> xfer(const std::vector<byte> &cmd, int response_len,
                                       uint32_t speed_hz) const;

  /**
   * Spi duplex transaction
   *
   * @param cmds multiple commands with multiple bytes
   * @param speed_hz SPI clock speed
   * @return output of each command in command order
   */
  [[nodiscard]] std::vector<std::vector<byte>> xfer2(const std::vector<std::vector<byte>> &cmds,
                                                     uint32_t speed_hz) const;

  bool close() override;

  ~Spi();

  //! Getters
  [[nodiscard]] bool isOpen() const;
  [[nodiscard]] int getFd() const;
  [[nodiscard]] const std::string &getPath() const;

 private:
  bool is_open_{false};
  int fd_{};
  std::string path_;
};

}  // namespace mav_sensors