// Copyright (c) 2024 ETH Zurich, Autonomous Systems Lab, Mariano Biasio, Rik Girod

//
// Created by acey on 27.06.23.
//

#include "mav_sensors_core/driver.h"

#include <utility>

using namespace mav_sensors;

Driver::Driver(std::string path) : path_(std::move(path)) {}

const std::string& Driver::getPath() const { return path_; }
