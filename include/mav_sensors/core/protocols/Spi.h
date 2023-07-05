//
// Created by acey on 27.06.23.
//

#pragma once

#include <vector>
#include "mav_sensors/core/driver.h"

typedef unsigned char byte;

class Spi : public Driver {
 public:
  explicit Spi();
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
