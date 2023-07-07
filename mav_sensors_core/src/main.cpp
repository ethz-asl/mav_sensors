//
// Created by acey on 21.06.23.
//

#include <log++.h>

#include "../../mav_sensors_drivers/include/mav_sensors_drivers/sensor_types/FluidPressure.h"
#include "mav_sensors_core/protocols/Spi.h"
#include "mav_sensors_core/barometer/bmp390.h"
#include "mav_sensors_core/sensor_config.h"

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
    auto measurements = bmp390.read();
    LOG(I, std::get<0>(measurements).has_value(),
        "Pressure: " << std::get<0>(measurements).value() << " Pa");
    LOG(I, std::get<1>(measurements).has_value(),
        "Temperature: " << std::get<1>(measurements).value() << " C");
    auto m_t = bmp390.read<Temperature>();
    LOG(I, std::get<0>(m_t).has_value(),
        "Temperature (single measurement): " << std::get<0>(m_t).value() << " C");
    auto m_p = bmp390.read<FluidPressure>();
    LOG(I, std::get<0>(m_p).has_value(),
        "Pressure (single measurement): " << std::get<0>(m_p).value() << " Pa");
    sleep(1);
  }
  bmp390.close();
}
