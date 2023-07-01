//
// Created by brik on 01.07.23.
//

#pragma once

#include <vector>

#include "mav_sensors/core/driver.h"

typedef unsigned char byte;

class Serial : public Driver {
 public:
  explicit Serial(std::string path);
  bool open() override;
  bool close() override;

  [[nodiscard]] bool setParityBit(bool bit = false) const;
  [[nodiscard]] bool setStopBit(bool bit = false) const;
  [[nodiscart]] bool setNumberOfBitsPerByte(int bits = 8) const;
  [[nodiscard]] bool setBaudRate(uint32_t baud) const;
  [[nodiscard]] bool setFlowControlBit(bool bit = false) const;
  [[nodiscart]] bool setLocalBit(bool bit = true) const;
  [[nodiscart]] bool setReadBit(bool bit = true) const;

  ~Serial();

  //! Getters
  [[nodiscard]] bool isOpen() const;
  [[nodiscard]] int getFd() const;
  [[nodiscard]] const std::string &getPath() const;

 private:
  bool is_open_{false};
  int fd_{};
  std::string path_;
};

// Defaults values are: readTimeout = UART_WAIT_FOREVER; writeTimeout = UART_WAIT_FOREVER;
// readReturnMode = UART_RETURN_NEWLINE; readDataMode = UART_DATA_TEXT; writeDataMode =
// UART_DATA_TEXT; readEcho = UART_ECHO_ON;