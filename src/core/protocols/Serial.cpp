#include "mav_sensors/core/protocols/Serial.h"

#include <fcntl.h>

#include <log++.h>
#include <sys/ioctl.h>
#include <asm/termbits.h> 

// A linux user space serial driver class.

// Created by brik on 01.07.23.

Serial::Serial(std::string path) : path_(std::move(path)) {}

bool Serial::open() {
  fd_ = ::open(path_.data(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    LOG(E, "Error on open: " << strerror(errno));
    return false;
  }
  is_open_ = true;
  return true;
}

bool Serial::close() {
  if (is_open_) {
    if (::close(fd_) < 0) {
      LOG(E, "Error on close: " << strerror(errno));
      return false;
    }
    is_open_ = false;
  }
  return true;
}

bool Serial::setParityBit(bool bit) const {
  struct termios2 tty;
  if (ioctl(fd_, TCGETS2, &tty) < 0) {
    LOG(E, "Error on TCGETS2: " << strerror(errno));
    return false;
  }

  if (bit) {
    tty.c_cflag |= PARENB;
  } else {
    tty.c_cflag &= ~PARODD;
  }

  if (ioctl(fd_, TCSETS2, &tty) < 0) {
    LOG(E, "Error on TCSETS2: " << strerror(errno));
    return false;
  }
  return true;
}

// Defaults values are: readTimeout = UART_WAIT_FOREVER; writeTimeout = UART_WAIT_FOREVER;
// readReturnMode = UART_RETURN_NEWLINE; readDataMode = UART_DATA_TEXT; writeDataMode =
// UART_DATA_TEXT; readEcho = UART_ECHO_ON; baudRate = 115200; dataLength = UART_LEN_8; stopBits =
// UART_STOP_ONE; parityType = UART_PAR_NONE;