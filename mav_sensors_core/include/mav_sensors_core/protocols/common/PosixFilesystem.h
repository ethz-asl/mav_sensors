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