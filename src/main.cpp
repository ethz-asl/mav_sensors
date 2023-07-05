//
// Created by acey on 21.06.23.
//

#include <log++.h>

#include "mav_sensors/core/protocols/Spi.h"
#include "mav_sensors/core/sensor_config.h"
#include "mav_sensors/core/sensor_types/FluidPressure.h"
#include "mav_sensors/impl/barometer/bmp390.h"

int main(int argc, char** argv) {
  LOG_INIT(*argv);
  SensorConfig sensorConfig;
  sensorConfig.set("path", "/dev/spidev2.0");


  BMP390<Spi> bmp390(sensorConfig);
  if (!bmp390.open()) {
    LOG(F, "Open failed");
    return 1;
  }

  for (int i = 0; i < 5; i++) {
    auto a = bmp390.read();
    bmp390.read<FluidPressure>();
    bmp390.read<Temperature>();
    double val = std::get<0>(a).value();
    LOG(I, "Value: " << val);
    sleep(1);
  }
  bmp390.close();
}
