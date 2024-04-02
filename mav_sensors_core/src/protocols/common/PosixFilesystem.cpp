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
