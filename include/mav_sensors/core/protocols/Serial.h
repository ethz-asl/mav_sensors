//
// Created by brik on 01.07.23.
//

#pragma once

#include <vector>

#include <asm/termbits.h>

#include "mav_sensors/core/driver.h"

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
   * @brief Read data from serial port. Wait for v_min bytes to be available or until timeout.
   * @param data data buffer
   * @param len_data data buffer length
   * @param v_min number of bytes
   * @param timeout in deciseconds (1 deci second = 0.1 second)
   * @return number of bytes read or a negative value on failure
   */
  [[nodiscard]] ssize_t read(void* data, size_t len_data, uint8_t v_min, uint8_t timeout) const;

  /**
   *
   * @param data data buffer
   * @param len_data data buffer length
   * @return number of bytes written or a negative value on failure
   */
  [[nodiscard]] ssize_t write(const void* data, size_t len_data) const;

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
  [[nodiscard]] const std::string& getPath() const;
  [[nodiscard]] int available() const;

 private:
  bool is_open_{false};
  int fd_{};
  std::string path_{"/dev/ttyUSB0"};
};