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

  for (int i = 0; i < 5; i++) {
    auto measurements = bmi088.read();

    LOG(I, std::get<0>(measurements).has_value(),
        "Time: " << std::get<0>(measurements).value() << " s");
    LOG(I, std::get<1>(measurements).has_value(),
        "Acceleration: " << std::get<1>(measurements).value().x << " m/s^2, "
                         << std::get<1>(measurements).value().y << " m/s^2, "
                         << std::get<1>(measurements).value().z << " m/s^2");
    LOG(I, std::get<2>(measurements).has_value(),
        "Angular Velocity: " << std::get<2>(measurements).value().x << " rad/s, "
                             << std::get<2>(measurements).value().y << " rad/s, "
                             << std::get<2>(measurements).value().z << " rad/s");
    sleep(1.0);
  }
  bmi088.close();
}
