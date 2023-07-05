//
// Created by brik on 01.07.23.
//

#include <log++.h>

#include "mav_sensors/core/sensor_config.h"
#include "mav_sensors/impl/radar/xwr18xx_mmw_demo.h"

int main(int argc, char** argv) {
  LOG_INIT(*argv);
  SensorConfig sensorConfig;
  std::string current_file_path = __FILE__;
  size_t src_idx = current_file_path.rfind(std::string("/src/"));
  if (src_idx != std::string::npos) {
    auto pkg_directory = current_file_path.substr(0, src_idx);
    sensorConfig.set("path_cfg_file",
                     pkg_directory + "/cfg/radar/xwr18xx_AOP_profile_best_velocity_resolution.cfg");
  }
  sensorConfig.set("path_cfg", "/dev/ttyUSB0");
  sensorConfig.set("path_data", "/dev/ttyUSB1");
  sensorConfig.set("trigger", "true");
  sensorConfig.set("trigger_delay", "500"); //in ns
  sensorConfig.set("trigger_gpio", "443");
  sensorConfig.set("trigger_gpio_name", "PR.00");

  Xwr18XxMmwDemo radar(sensorConfig);
  if (!radar.open()) {
    LOG(F, "Open failed.");
    return 1;
  }

  int num_reads = 100;
  while (num_reads--) {
    auto measurement = radar.read();
    LOG(I, "Hardware stamp: " << std::get<0>(measurement).hardware_stamp);
    LOG(I, "Number of detections: " << std::get<0>(measurement).cfar_detections.size());
    for (const auto& detection : std::get<0>(measurement).cfar_detections) {
      LOG(I, "Detections: " << detection.x << " " << detection.y << " " << detection.z << " "
                            << detection.velocity << " " << detection.snr << " "
                            << detection.noise);
    }
    sleep(1);
  }
  radar.close();
}
