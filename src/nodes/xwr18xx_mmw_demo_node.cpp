//
// Created by brik on 01.07.23.
//

#include <log++.h>

#include "mav_sensors/core/sensor_config.h"
#include "mav_sensors/impl/radar/xwr18xx_mmw_demo.h"

int main(int argc, char** argv) {
  LOG_INIT(*argv);
  SensorConfig sensorConfig;
  sensorConfig.set("path_cfg", "/dev/ttyUSB0");
  sensorConfig.set("path_data", "/dev/ttyUSB1");

  Xwr18XxMmwDemo radar(sensorConfig);
  radar.open();

  int num_reads = 100;
  while (num_reads--) {
    auto detections = radar.read();
    LOG(I, "Detections: %d", std::get<0>(detections).size());
    for (const auto& detection : std::get<0>(detections)) {
      LOG(I, "Detections: " << detection.x << " " << detection.y << " " << detection.z << " " << detection.velocity);
    }
  }
  radar.close();
}
