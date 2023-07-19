//
// Created by acey on 24.02.22.
//

#include "mav_sensors_core/protocols/common/PosixFilesystem.h"

#include "sys/stat.h"
#include <fcntl.h>
#include <unistd.h>

using namespace mav_sensors;

bool PosixFilesystem::write(const std::string &path, const std::string &message) noexcept {
  int fd = open((path).c_str(), O_WRONLY);
  if (fd == -1) {
    return false;
  }

  ssize_t nbytes = ::write(fd, message.c_str(), message.size());

  close(fd);
  return nbytes == static_cast<ssize_t>(message.size());
}

bool PosixFilesystem::read(const std::string &path, void *buffer, size_t buffer_size) noexcept {
  int fd = open(path.c_str(), O_RDONLY);
  if (fd == -1) {
    return false;
  }
  ssize_t nbytes = ::read(fd, buffer, buffer_size);
  close(fd);
  return nbytes == static_cast<ssize_t>(buffer_size);
}

bool PosixFilesystem::directoryExists(const char *path) noexcept {
  if (path == nullptr) {
    return false;
  }

  struct stat fileInfo {};
  if (lstat(path, &fileInfo) != 0) {
    return false;
  }

  return S_ISDIR(fileInfo.st_mode);
}

bool PosixFilesystem::directoryExists(const std::string &path) noexcept {
  return directoryExists(path.c_str());
}
