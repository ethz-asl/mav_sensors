//
// Created by acey on 27.06.23.
//

#pragma once

#include <utility>

#include "sensor_config.h"

namespace mav_sensors {

class Driver {
 public:
  Driver() = default;

  /**
   * @param path file path
   */
  explicit Driver(std::string path);

  virtual bool open() = 0;

  virtual bool close() = 0;

  [[nodiscard]] virtual const std::string& getPath() const;

 protected:
  std::string path_;
};

}  // namespace mav_sensors