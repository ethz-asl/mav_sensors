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
// Created by brik on 01.07.23.
//

#pragma once

#include <vector>

#include <asm/termbits.h>

#include "mav_sensors_core/driver.h"

namespace mav_sensors {

typedef unsigned char byte;

class Serial : public Driver {
 public:
  explicit Serial();
  bool open() override;
  bool close() override;

  /**
   * Read data from serial port. Blocks until any data is available.
   * @param data buffer
   * @param len_data number of bytes
   * @return Number of bytes read or a negative value on failure
   */
  [[nodiscard]] ssize_t read(void* data, size_t len_data) const;

  /**
   * @brief Read data from serial port. Wait for v_min bytes to be available or until v_time.
   * @param data data buffer
   * @param len_data data buffer length
   * @param v_min number of bytes
   * @param v_time in deciseconds (1 deci second = 0.1 second)
   * @return number of bytes read or a negative value on failure
   */
  [[nodiscard]] ssize_t read(void* data, size_t len_data, uint8_t v_min, uint8_t v_time) const;

  /**
   *
   * @param data data buffer
   * @param len_data data buffer length
   * @return number of bytes written or a negative value on failure
   */
  [[nodiscard]] ssize_t write(const void* data, size_t len_data) const;

  /**
   * @brief Flush read buffer.
   */
  [[nodiscard]] bool flushReadBuffer() const;

  /**
   * @brief Flush write buffer.
   */
  [[nodiscard]] bool flushWriteBuffer() const;

  /**
   * @brief Flush read and write buffer.
   */
  [[nodiscard]] bool flushBuffers() const;

  void setPath(std::string path);

  //! Setters cflag
  [[nodiscard]] bool setControlParityBit(bool bit = false) const;
  [[nodiscard]] bool setControlStopBit(bool bit = false) const;
  [[nodiscard]] bool setControlNumberOfBitsPerByte(int bits = 8) const;
  [[nodiscard]] bool setControlBaudRate(speed_t baud) const;
  [[nodiscard]] bool setControlFlowControl(bool bit = false) const;
  [[nodiscard]] bool setControlLocal(bool bit = true) const;
  [[nodiscard]] bool setControlRead(bool bit = true) const;

  //! Setters lflag
  [[nodiscard]] bool setLocalCanonicalMode(bool bit = false) const;
  [[nodiscard]] bool setLocalEcho(bool bit = false) const;
  [[nodiscard]] bool setLocalEchoErase(bool bit = false) const;
  [[nodiscard]] bool setLocalEchoNewLine(bool bit = false) const;
  [[nodiscard]] bool setLocalSignal(bool bit = false) const;

  //! Setters iflag
  [[nodiscard]] bool setSoftwareFlowControl(bool bit = false) const;
  [[nodiscard]] bool setSpecialCharacterProcessing(bool bit = false) const;

  //! Setters oflag
  [[nodiscard]] bool setOutputProcessing(bool bit = false) const;

  ~Serial();

  //! Getters
  [[nodiscard]] bool isOpen() const;
  [[nodiscard]] int getFd() const;
  [[nodiscard]] const std::string& getPath() const override;
  [[nodiscard]] int available() const;

 private:
  bool is_open_{false};
  int fd_{};
};

}  // namespace mav_sensors