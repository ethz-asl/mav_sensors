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

#include <log++.h>

#include "mav_sensors_core/sensor_config.h"
#include "mav_sensors_drivers/radar/xwr18xx_mmw_demo.h"

using namespace mav_sensors;

int main(int argc, char** argv) {
  LOG_INIT(*argv);
  SensorConfig cfg;
  std::string current_file_path = __FILE__;
  size_t src_idx = current_file_path.rfind(std::string("/src/"));
  if (src_idx != std::string::npos) {
    auto pkg_directory = current_file_path.substr(0, src_idx);
    cfg.set("path_cfg_file",
            pkg_directory + "/cfg/radar/xwr18xx_AOP_profile_best_velocity_resolution.cfg");
  }
  cfg.set("path_cfg", "/dev/ttyUSB0");
  cfg.set("path_data", "/dev/ttyUSB1");
  cfg.set("trigger", "true");
  cfg.set("trigger_delay", "500");  // in ns
  cfg.set("trigger_gpio", "443");
  cfg.set("trigger_gpio_name", "PR.00");

  Xwr18XxMmwDemo radar(cfg);
  if (!radar.open()) {
    LOG(F, "Open failed.");
    return 1;
  }

  int num_reads = 10;
  while (num_reads--) {
    auto measurement = radar.read();
    LOG(I, "Unix stamp: " << std::get<Radar>(measurement).unix_stamp_ns);
    LOG(I, "Hardware stamp: " << std::get<Radar>(measurement).hardware_stamp);
    LOG(I, "Number of detections: " << std::get<Radar>(measurement).cfar_detections.size());
    for (const auto& detection : std::get<Radar>(measurement).cfar_detections) {
      LOG(I, "Detections: " << detection.x << " " << detection.y << " " << detection.z << " "
                            << detection.velocity << " " << detection.snr << " "
                            << detection.noise);
    }
    sleep(1);
  }
  radar.close();
}
