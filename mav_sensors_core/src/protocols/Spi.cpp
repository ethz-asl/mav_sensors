//
// Created by acey on 22.08.22.
//

#include "mav_sensors_core/protocols/Spi.h"

#include <fcntl.h>
#include <unistd.h>

#include <cstring>
#include <utility>

#include <linux/spi/spidev.h>
#include <log++.h>
#include <sys/ioctl.h>

using namespace mav_sensors;

Spi::Spi() = default;

Spi::Spi(std::string path) : path_(std::move(path)) {}

void Spi::setPath(std::string path) { path_ = std::move(path); }

bool Spi::open() {
  fd_ = ::open(path_.data(), O_RDWR);
  if (fd_ < 0) {
    LOG(E, "Error on Spi open: " << strerror(errno));
    return false;
  }
  is_open_ = true;
  return true;
}

bool Spi::setMode(uint8_t mode) const {
  int res = ioctl(fd_, SPI_IOC_WR_MODE, &mode);

  if (res < 0) {
    LOG(E, "ioctl SPI_IOC_WR_MODE failed: " << strerror(errno));
    return false;
  }
  return true;
}

std::vector<std::vector<byte>> Spi::xfer2(const std::vector<std::vector<byte>> &cmds,
                                          const uint32_t speed_hz) const {
  std::vector<std::vector<byte>> res{};
  for (int i = 0; i < cmds.size() + 1; i++) {
    std::vector<byte> cmd{};
    if (i < cmds.size()) {
      const std::vector<byte> &a = cmds.at(i);
      cmd = a;
    } else {
      cmd = {0x00, 0x00};
    }

    if (cmd.size() > sizeof(__u64)) {
      LOG(E, "cmd buffer to big " << cmd.size() << " > " << sizeof(__u64));
      return {};
    }

    struct spi_ioc_transfer xfer[1];
    unsigned char buf[32]{};

    xfer->speed_hz = speed_hz;

    int len = 2;
    memset(xfer, 0, sizeof xfer);
    memset(buf, 0, sizeof buf);

    // Send a read command
    buf[0] = cmd[0];
    buf[1] = cmd[1];

    xfer[0].tx_buf = (unsigned long)buf;
    xfer[0].len = len;

    unsigned char buf2[len];
    memset(buf2, 0, sizeof buf2);
    xfer[0].rx_buf = (unsigned long)buf2;
    xfer[0].len = len;

    int status = ioctl(fd_, SPI_IOC_MESSAGE(1), xfer);
    if (status < 0) {
      LOG(E, "ioctl SPI_IOC_MESSAGE failed: " << strerror(errno));
      return {};
    }

    if (i == 0) {
      continue;
    }

    std::vector<unsigned char> ret{};

    for (int j = 0; j < len; j++) {
      ret.push_back(buf2[j]);
    }
    res.push_back(ret);
  }
  return res;
}

std::vector<byte> Spi::xfer(const std::vector<byte> &cmd, const int response_len,
                            const uint32_t speed_hz) const {
  // Create one transfer to send command and one to receive response.
  // TODO(rikba): Handle only transmit case.
  struct spi_ioc_transfer xfer[2];
  memset(xfer, 0, sizeof xfer);

  // Configure transmit
  xfer[0].tx_buf = (unsigned long)cmd.data();
  xfer[0].len = cmd.size();
  xfer[0].speed_hz = speed_hz;
  xfer[0].delay_usecs = 0;
  xfer[0].bits_per_word = CHAR_BIT;
  xfer[0].cs_change = 0;
  xfer[0].tx_nbits = 0;
  xfer[0].rx_nbits = 0;
  xfer[0].pad = 0;

  // Configure receive
  std::vector<byte> res(response_len, 0);

  xfer[1].rx_buf = (unsigned long)res.data();
  xfer[1].len = response_len;
  xfer[1].speed_hz = speed_hz;
  xfer[1].delay_usecs = 0;
  xfer[1].bits_per_word = CHAR_BIT;
  xfer[1].cs_change = 1;  // Deselect at end of transaction.
  xfer[1].tx_nbits = 0;
  xfer[1].rx_nbits = 0;
  xfer[1].pad = 0;

  int status = ioctl(fd_, SPI_IOC_MESSAGE(2), xfer);
  if (status < 0) {
    LOG(E, "ioctl SPI_IOC_MESSAGE failed: " << strerror(errno));
    return {};
  }

  // TODO(rikba): Handle case where empty vector is returned but no error.
  return res;
}

bool Spi::close() {
  if (::close(fd_) != 0) {
    LOG(I, "Error closing fd: " << strerror(errno));
    return false;
  }
  is_open_ = false;
  return true;
}

Spi::~Spi() { ::close(fd_); }

bool Spi::isOpen() const { return is_open_; }

int Spi::getFd() const { return fd_; }

const std::string &Spi::getPath() const { return path_; }
