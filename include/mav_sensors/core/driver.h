//
// Created by acey on 27.06.23.
//

#pragma once

#include <utility>

#include "sensor_config.h"

class Driver {
 public:
  virtual bool open() = 0;

  virtual bool close() = 0;

  virtual const std::string& getPath() const;

 protected:
  std::string path_;
  SensorConfig cfg_;
};
