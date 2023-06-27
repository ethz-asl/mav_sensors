//
// Created by acey on 21.06.23.
//

#include <log++.h>

#include "mav_sensors/core/protocols/Spi.h"
#include "mav_sensors/core/sensor_config.h"
#include "mav_sensors/core/sensor_types/FluidPressure.h"
#include "mav_sensors/impl/barometer/bmp390.h"

int main() {
  SensorConfig sensorConfig;
  sensorConfig.set("path", "/dev/spidev0.1");

  std::vector<std::function<void()>> objects;



  BMP390<Spi> bmp390(sensorConfig);


  bmp390.open();


  std::tuple<FluidPressure::ReturnType> a = bmp390.read();
  double val = std::get<0>(a);
  LOG(I, "Value: " << val);
  bmp390.close();
}
