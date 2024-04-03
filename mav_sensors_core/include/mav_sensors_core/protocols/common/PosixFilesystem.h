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

#pragma once

#include <string>

namespace mav_sensors {
/**
 * Wrapper for posix functions with some additional checks.
 * These functions are only recommended when interacting with sysfs and other system files.
 */
class PosixFilesystem {
 public:
  /**
   * Wrapper for posix write(2).
   * @param path
   * @param message
   * @return true if successful, otherwise false and errno is set.
   */
  [[nodiscard]] static bool write(const std::string &path, const std::string &message) noexcept;

  /**
   * Wrapper for posix read(2)
   * @param path Absolute path
   * @param buffer buffer to read data to
   * @param buffer_size sizeof(buffer)
   * @return true if successful, otherwise false and errno is set.
   */
  [[nodiscard]] static bool read(const std::string &path, void *buffer,
                                 size_t buffer_size) noexcept;

  /**
   * Checks if a directory exists at the given location
   * @param path Absolute path
   * @return true if directory exists, otherwise false
   */
  [[nodiscard]] static bool directoryExists(const std::string &path) noexcept;
  [[nodiscard]] static bool directoryExists(const char *path) noexcept;
};
}  // namespace mav_sensors