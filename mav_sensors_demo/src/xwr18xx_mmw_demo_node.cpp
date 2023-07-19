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
