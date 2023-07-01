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

  //! Setters cflag
  [[nodiscard]] bool setControlParityBit(bool bit = false) const;
  [[nodiscard]] bool setControlStopBit(bool bit = false) const;
  [[nodiscard]] bool setControlNumberOfBitsPerByte(int bits = 8) const;
  [[nodiscard]] bool setControlBaudRate(uint32_t baud) const;
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
  [[nodiscard]] const std::string &getPath() const;

 private:
  bool is_open_{false};
  int fd_{};
  std::string path_;
};

// Defaults values are: readTimeout = UART_WAIT_FOREVER; writeTimeout = UART_WAIT_FOREVER;
// readReturnMode = UART_RETURN_NEWLINE; readDataMode = UART_DATA_TEXT; writeDataMode =
// UART_DATA_TEXT; readEcho = UART_ECHO_ON;