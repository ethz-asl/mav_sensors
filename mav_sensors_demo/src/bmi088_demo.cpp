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
// Created by brik on 17.07.23.
//

#include <log++.h>

#include "mav_sensors_core/protocols/Spi.h"
#include "mav_sensors_core/sensor_config.h"
#include "mav_sensors_drivers/imu/bmi088.h"
#include "mav_sensors_drivers/sensor_types/Accelerometer.h"
#include "mav_sensors_drivers/sensor_types/Gyroscope.h"
#include "mav_sensors_drivers/sensor_types/Time.h"

using namespace mav_sensors;

int main(int argc, char** argv) {
  LOG_INIT(*argv);
  SensorConfig cfg;
  cfg.set("path_acc", "/dev/spidev0.0");
  cfg.set("path_gyro", "/dev/spidev0.1");

  Bmi088<Spi> bmi088(cfg);
  if (!bmi088.open()) {
    LOG(F, "Open failed");
    return 1;
  }

  for (int i = 0; i < 100; i++) {
    auto measurements = bmi088.read();
    LOG(I, std::get<0>(measurements).has_value(),
        "Acceleration: " << std::get<0>(measurements).value().x << " m/s^2, "
                         << std::get<0>(measurements).value().y << " m/s^2, "
                         << std::get<0>(measurements).value().z << " m/s^2");
    LOG(I, std::get<1>(measurements).has_value(),
        "Angular Velocity: " << std::get<1>(measurements).value().x << " rad/s, "
                             << std::get<1>(measurements).value().y << " rad/s, "
                             << std::get<1>(measurements).value().z << " rad/s");
    LOG(I, std::get<2>(measurements).has_value(),
        "Time: " << std::get<2>(measurements).value() << " s");
  }
  bmi088.close();
}
