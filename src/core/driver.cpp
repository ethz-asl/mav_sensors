//
// Created by acey on 27.06.23.
//

#include "mav_sensors/core/driver.h"

#include <utility>

Driver::Driver(std::string path) : path_(std::move(path)) {}

const std::string& Driver::getPath() const { return path_; }
