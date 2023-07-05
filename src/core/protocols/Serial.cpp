#include "mav_sensors/core/protocols/Serial.h"

#include <fcntl.h>

#include <log++.h>
#include <sys/ioctl.h>

// A linux user space serial driver class.

// Created by brik on 01.07.23.

Serial::Serial() = default;
Serial::~Serial() { ::close(fd_); }

void Serial::setPath(std::string path) {
  path_ = std::move(path);
}

const std::string& Serial::getPath() const { return path_; }

bool Serial::open() {
  fd_ = ::open(path_.c_str(), O_RDWR);
  if (fd_ < 0) {
    LOG(E, "Error on Serial open: " << strerror(errno));
    return false;
  }
  is_open_ = true;

  // Set default cflags.
  if (!setControlParityBit()) return false;
  if (!setControlStopBit()) return false;
  if (!setControlNumberOfBitsPerByte()) return false;
  if (!setControlFlowControl()) return false;
  if (!setControlLocal()) return false;
  if (!setControlRead()) return false;

  // Set default lflags.
  if (!setLocalCanonicalMode()) return false;
  if (!setLocalEcho()) return false;
  if (!setLocalEchoErase()) return false;
  if (!setLocalEchoNewLine()) return false;
  if (!setLocalSignal()) return false;

  // Set default iflags.
  if (!setSoftwareFlowControl()) return false;
  if (!setSpecialCharacterProcessing()) return false;

  // Set default oflags.
  if (!setOutputProcessing()) return false;

  return true;
}

ssize_t Serial::read(void* data, size_t len_data) const {
  ssize_t n = ::read(fd_, data, len_data);
  LOG(E, n < 0, "Error on read: " << strerror(errno));
  return n;
}

ssize_t Serial::read(void* data, size_t len_data, uint8_t v_min, uint8_t v_time) const {
  struct termios2 tty {};
  if (::ioctl(fd_, TCGETS2, &tty) < 0) {
    LOG(E, "Error on TCGETS2 for read: " << strerror(errno));
    return -1;
  }

  uint8_t old_cc_vmin = tty.c_cc[VMIN];
  uint8_t old_cc_vtime = tty.c_cc[VTIME];

  tty.c_cc[VTIME] = v_time;
  tty.c_cc[VMIN] = v_min;

  if (::ioctl(fd_, TCSETS2, &tty) < 0) {
    LOG(E, "Error on TCSETS2 for read: " << strerror(errno));
    return -1;
  }

  ssize_t n = read(data, len_data);

  tty.c_cc[VTIME] = old_cc_vtime;
  tty.c_cc[VMIN] = old_cc_vmin;

  if (::ioctl(fd_, TCSETS2, &tty) < 0) {
    LOG(E, "Error on TCSETS2 for read: " << strerror(errno));
    return -1;
  }

  return n;
}

ssize_t Serial::write(const void* data, size_t len_data) const {
  ssize_t n = ::write(fd_, data, len_data);
  LOG(E, n < 0, "Error on write: " << strerror(errno));
  return n;
}

bool Serial::setControlParityBit(bool bit) const {
  struct termios2 tty {};
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

bool Serial::setControlStopBit(bool bit) const {
  struct termios2 tty {};
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

bool Serial::setControlNumberOfBitsPerByte(int bits) const {
  struct termios2 tty {};
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

bool Serial::setControlBaudRate(speed_t baud) const {
  struct termios2 tty {};
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

bool Serial::setControlFlowControl(bool bit) const {
  struct termios2 tty {};
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

bool Serial::setControlLocal(bool bit) const {
  struct termios2 tty {};
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

bool Serial::setControlRead(bool bit) const {
  struct termios2 tty {};
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

bool Serial::setLocalCanonicalMode(bool bit) const {
  struct termios2 tty {};
  if (ioctl(fd_, TCGETS2, &tty) < 0) {
    LOG(E, "Error on TCGETS2 for local canonical mode bit: " << strerror(errno));
    return false;
  }

  if (bit) {
    tty.c_lflag |= ICANON;
  } else {
    tty.c_lflag &= ~ICANON;
  }

  if (ioctl(fd_, TCSETS2, &tty) < 0) {
    LOG(E, "Error on TCSETS2 for local canonical mode bit: " << strerror(errno));
    return false;
  }
  return true;
}

bool Serial::setLocalEcho(bool bit) const {
  struct termios2 tty {};
  if (ioctl(fd_, TCGETS2, &tty) < 0) {
    LOG(E, "Error on TCGETS2 for local echo bit: " << strerror(errno));
    return false;
  }

  if (bit) {
    tty.c_lflag |= ECHO;
  } else {
    tty.c_lflag &= ~ECHO;
  }

  if (ioctl(fd_, TCSETS2, &tty) < 0) {
    LOG(E, "Error on TCSETS2 for local echo bit: " << strerror(errno));
    return false;
  }
  return true;
}

bool Serial::setLocalEchoErase(bool bit) const {
  struct termios2 tty {};
  if (ioctl(fd_, TCGETS2, &tty) < 0) {
    LOG(E, "Error on TCGETS2 for local echo erase bit: " << strerror(errno));
    return false;
  }

  if (bit) {
    tty.c_lflag |= ECHOE;
  } else {
    tty.c_lflag &= ~ECHOE;
  }

  if (ioctl(fd_, TCSETS2, &tty) < 0) {
    LOG(E, "Error on TCSETS2 for local echo erase bit: " << strerror(errno));
    return false;
  }
  return true;
}

bool Serial::setLocalEchoNewLine(bool bit) const {
  struct termios2 tty {};
  if (ioctl(fd_, TCGETS2, &tty) < 0) {
    LOG(E, "Error on TCGETS2 for local echo new line bit: " << strerror(errno));
    return false;
  }

  if (bit) {
    tty.c_lflag |= ECHONL;
  } else {
    tty.c_lflag &= ~ECHONL;
  }

  if (ioctl(fd_, TCSETS2, &tty) < 0) {
    LOG(E, "Error on TCSETS2 for local echo new line bit: " << strerror(errno));
    return false;
  }
  return true;
}

bool Serial::setLocalSignal(bool bit) const {
  struct termios2 tty {};
  if (ioctl(fd_, TCGETS2, &tty) < 0) {
    LOG(E, "Error on TCGETS2 for local signal bit: " << strerror(errno));
    return false;
  }

  if (bit) {
    tty.c_lflag |= ISIG;
  } else {
    tty.c_lflag &= ~ISIG;
  }

  if (ioctl(fd_, TCSETS2, &tty) < 0) {
    LOG(E, "Error on TCSETS2 for local signal bit: " << strerror(errno));
    return false;
  }
  return true;
}

bool Serial::setSoftwareFlowControl(bool bit) const {
  struct termios2 tty {};
  if (ioctl(fd_, TCGETS2, &tty) < 0) {
    LOG(E, "Error on TCGETS2 for software flow control: " << strerror(errno));
    return false;
  }

  if (bit) {
    tty.c_iflag |= IXON | IXOFF | IXANY;
  } else {
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  }

  if (ioctl(fd_, TCSETS2, &tty) < 0) {
    LOG(E, "Error on TCSETS2 for software flow control: " << strerror(errno));
    return false;
  }
  return true;
}

bool Serial::setSpecialCharacterProcessing(bool bit) const {
  struct termios2 tty {};
  if (ioctl(fd_, TCGETS2, &tty) < 0) {
    LOG(E, "Error on TCGETS2 for special character processing: " << strerror(errno));
    return false;
  }

  if (bit) {
    tty.c_iflag |= IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL;
  } else {
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
  }

  if (ioctl(fd_, TCSETS2, &tty) < 0) {
    LOG(E, "Error on TCSETS2 for special character processing: " << strerror(errno));
    return false;
  }
  return true;
}

bool Serial::setOutputProcessing(bool bit) const {
  struct termios2 tty {};
  if (ioctl(fd_, TCGETS2, &tty) < 0) {
    LOG(E, "Error on TCGETS2 output processing: " << strerror(errno));
    return false;
  }

  if (bit) {
    tty.c_oflag |= OPOST | ONLCR;
  } else {
    tty.c_oflag &= ~(OPOST | ONLCR);
  }

  if (ioctl(fd_, TCSETS2, &tty) < 0) {
    LOG(E, "Error on TCSETS2 output processing: " << strerror(errno));
    return false;
  }
  return true;
}

bool Serial::close() {
  if (::close(fd_) != 0) {
    LOG(E, "Error closing fd: " << strerror(errno));
    return false;
  }
  is_open_ = false;
  return true;
}

int Serial::available() const {
  int bytes_available = 0;
  if (::ioctl(fd_, FIONREAD, &bytes_available) < 0) {
    LOG(E, "Error on FIONREAD: " << strerror(errno));
    return -1;
  }
  return bytes_available;
}

// Defaults values are: readTimeout = UART_WAIT_FOREVER; writeTimeout = UART_WAIT_FOREVER;
// readReturnMode = UART_RETURN_NEWLINE; readDataMode = UART_DATA_TEXT; writeDataMode =
// UART_DATA_TEXT; readEcho = UART_ECHO_ON; baudRate = 115200; dataLength = UART_LEN_8; stopBits =
// UART_STOP_ONE; parityType = UART_PAR_NONE;

int Serial::getFd() const { return fd_; }

