//
// Created by brik on 01.07.23.
//

#include <log++.h>

#include "mav_sensors/core/sensor_config.h"
#include "mav_sensors/impl/radar/xwr18xx_mmw_demo.h"

int main(int argc, char** argv) {
  LOG_INIT(*argv);
  SensorConfig sensorConfig;
  //sensorConfig.set("path_cfg_file", "xwr18xx_AOP_profile_2023_07_03T09_17_14_772.cfg");
  sensorConfig.set("path_cfg", "/dev/ttyUSB0");
  sensorConfig.set("path_data", "/dev/ttyUSB1");

  Xwr18XxMmwDemo radar(sensorConfig);
  radar.open();

  int num_reads = 100;
  while (num_reads--) {
    auto measurement = radar.read();
    LOG(I, "Hardware stamp: " << std::get<0>(measurement).hardware_stamp);
    LOG(I, "Number of detections: " << std::get<0>(measurement).cfar_detections.size());
    for (const auto& detection : std::get<0>(measurement).cfar_detections) {
      LOG(I, "Detections: " << detection.x << " " << detection.y << " " << detection.z << " " << detection.velocity << " " << detection.snr << " " << detection.noise);
    }
  }
  radar.close();
}
