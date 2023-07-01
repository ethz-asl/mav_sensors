#include "mav_sensors/core/protocols/Serial.h"

#include <fcntl.h>

#include <asm/termbits.h>
#include <log++.h>
#include <sys/ioctl.h>

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
    LOG(E, "Error on TCGETS2 for parity bit: " << strerror(errno));
    return false;
  }

  if (bit) {
    tty.c_cflag |= PARENB;
  } else {
    tty.c_cflag &= ~PARODD;
  }

  if (ioctl(fd_, TCSETS2, &tty) < 0) {
    LOG(E, "Error on TCSETS2 for parity bit: " << strerror(errno));
    return false;
  }
  return true;
}

bool Serial::setStopBit(bool bit) const {
  struct termios2 tty;
  if (ioctl(fd_, TCGETS2, &tty) < 0) {
    LOG(E, "Error on TCGETS2 for stop bit: " << strerror(errno));
    return false;
  }

  if (bit) {
    tty.c_cflag |= CSTOPB;
  } else {
    tty.c_cflag &= ~CSTOPB;
  }

  if (ioctl(fd_, TCSETS2, &tty) < 0) {
    LOG(E, "Error on TCSETS2 for stop bit: " << strerror(errno));
    return false;
  }
  return true;
}

bool Serial::setNumberOfBitsPerByte(int bits) const {
  struct termios2 tty;
  if (ioctl(fd_, TCGETS2, &tty) < 0) {
    LOG(E, "Error on TCGETS2 for number of bits per byte: " << strerror(errno));
    return false;
  }

  tty.c_cflag &= ~CSIZE;
  switch (bits) {
    case 5:
      tty.c_cflag |= CS5;
      break;
    case 6:
      tty.c_cflag |= CS6;
      break;
    case 7:
      tty.c_cflag |= CS7;
      break;
    case 8:
      tty.c_cflag |= CS8;
      break;
    default:
      LOG(E, "Invalid number of bits per byte: " << bits);
      return false;
  }

  if (ioctl(fd_, TCSETS2, &tty) < 0) {
    LOG(E, "Error on TCSETS2 for number of bits per byte: " << strerror(errno));
    return false;
  }
  return true;
}

bool Serial::setBaudRate(uint32_t baud) const {
  struct termios2 tty;
  if (ioctl(fd_, TCGETS2, &tty) < 0) {
    LOG(E, "Error on TCGETS2 for baud rate: " << strerror(errno));
    return false;
  }

  tty.c_cflag &= ~CBAUD;
  tty.c_cflag |= BOTHER;
  tty.c_ispeed = baud;
  tty.c_ospeed = baud;

  if (ioctl(fd_, TCSETS2, &tty) < 0) {
    LOG(E, "Error on TCSETS2 for baud rate: " << strerror(errno));
    return false;
  }
  return true;
}

bool Serial::setFlowControlBit(bool bit) const {
  struct termios2 tty;
  if (ioctl(fd_, TCGETS2, &tty) < 0) {
    LOG(E, "Error on TCGETS2 for flow control bit: " << strerror(errno));
    return false;
  }

  if (bit) {
    tty.c_cflag |= CRTSCTS;
  } else {
    tty.c_cflag &= ~CRTSCTS;
  }

  if (ioctl(fd_, TCSETS2, &tty) < 0) {
    LOG(E, "Error on TCSETS2 for flow control bit: " << strerror(errno));
    return false;
  }
  return true;
}

bool Serial::setLocalBit(bool bit) const {
  struct termios2 tty;
  if (ioctl(fd_, TCGETS2, &tty) < 0) {
    LOG(E, "Error on TCGETS2 for local bit: " << strerror(errno));
    return false;
  }

  if (bit) {
    tty.c_cflag |= CLOCAL;
  } else {
    tty.c_cflag &= ~CLOCAL;
  }

  if (ioctl(fd_, TCSETS2, &tty) < 0) {
    LOG(E, "Error on TCSETS2 for local bit: " << strerror(errno));
    return false;
  }
  return true;
}

bool Serial::setReadBit(bool bit) const {
  struct termios2 tty;
  if (ioctl(fd_, TCGETS2, &tty) < 0) {
    LOG(E, "Error on TCGETS2 for read bit: " << strerror(errno));
    return false;
  }

  if (bit) {
    tty.c_cflag |= CREAD;
  } else {
    tty.c_cflag &= ~CREAD;
  }

  if (ioctl(fd_, TCSETS2, &tty) < 0) {
    LOG(E, "Error on TCSETS2 for read bit: " << strerror(errno));
    return false;
  }
  return true;
}

// Defaults values are: readTimeout = UART_WAIT_FOREVER; writeTimeout = UART_WAIT_FOREVER;
// readReturnMode = UART_RETURN_NEWLINE; readDataMode = UART_DATA_TEXT; writeDataMode =
// UART_DATA_TEXT; readEcho = UART_ECHO_ON; baudRate = 115200; dataLength = UART_LEN_8; stopBits =
// UART_STOP_ONE; parityType = UART_PAR_NONE;