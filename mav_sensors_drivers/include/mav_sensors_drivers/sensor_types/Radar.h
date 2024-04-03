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
// Created by brik on 01.07.23.
//

#pragma once

#include <cmath>
#include <ostream>
#include <vector>

#include "mav_sensors_core/sensor.h"

namespace mav_sensors {

class Radar : public SensorType {
 public:
  inline Radar(size_t num_detections) : cfar_detections(num_detections) {}
  inline Radar() : Radar(0) {}
  struct CfarDetection {
    float x{std::nanf("1")};
    float y{std::nanf("1")};
    float z{std::nanf("1")};
    float velocity{std::nanf("1")};
    int16_t snr{-1};
    int16_t noise{-1};
    friend std::ostream& operator<<(std::ostream& os, const CfarDetection& cd) {
      os << "x: " << cd.x << " y: " << cd.y << " z: " << cd.z << " velocity: " << cd.velocity
         << " snr: " << cd.snr << " noise: " << cd.noise;
      return os;
    }
  };
  std::vector<CfarDetection> cfar_detections;
  uint32_t hardware_stamp{0xFFFFFFFF};
  uint64_t unix_stamp_ns{0};
  typedef Radar ReturnType;

  friend std::ostream& operator<<(std::ostream& os, const Radar& r) {
    os << "Radar detections: ";
    for (const auto& detection : r.cfar_detections) {
      os << "- " << detection << " ";
    }
    os << "Hardware stamp: " << r.hardware_stamp << " Unix stamp: " << r.unix_stamp_ns;
    return os;
  }
};

}  // namespace mav_sensors