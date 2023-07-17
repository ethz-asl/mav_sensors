//
// Created by acey on 21.06.23.
//

#include <log++.h>

#include "mav_sensors_core/protocols/Spi.h"
#include "mav_sensors_core/sensor_config.h"
#include "mav_sensors_drivers/barometer/bmp390.h"
#include "mav_sensors_drivers/sensor_types/FluidPressure.h"

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
        "Time: " << std::get<0>(measurements).value() << " s");
    LOG(I, std::get<1>(measurements).has_value(),
        "Pressure: " << std::get<1>(measurements).value() << " Pa");
    LOG(I, std::get<2>(measurements).has_value(),
        "Temperature: " << std::get<2>(measurements).value() << " C");
    auto m_t = bmp390.read<Time, Temperature>();
    LOG(I, std::get<0>(m_t).has_value() && std::get<1>(m_t).has_value(),
        "Temperature (single measurement): " << std::get<1>(m_t).value()
                                             << " C at t=" << std::get<0>(m_t).value() << " s");
    auto m_p = bmp390.read<Time, FluidPressure>();
    LOG(I, std::get<0>(m_p).has_value() && std::get<1>(m_p).has_value(),
        "Pressure (single measurement): " << std::get<1>(m_p).value()
                                          << " Pa at t=" << std::get<0>(m_p).value() << " s");
    sleep(1);
  }
  bmp390.close();
}
