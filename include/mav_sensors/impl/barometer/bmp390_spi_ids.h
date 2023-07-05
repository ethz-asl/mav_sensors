//
// Created by acey on 30.06.23.
//

#ifndef MAV_SENSORS_BMP390_SPI_IDS_H
#define MAV_SENSORS_BMP390_SPI_IDS_H

#include <cstdint>
namespace bmp390 {
  inline static constexpr const unsigned char CHIP_ID{0x00};
  inline static constexpr const unsigned char REV_ID{0x01};
  inline static constexpr const unsigned char ERR_REG{0x02};
  inline static constexpr const unsigned char STATUS{0x03};
  inline static constexpr const unsigned char PRESSURE_DATA{0x04};
  inline static constexpr const unsigned char TEMPERATURE_DATA{0x07};
  inline static constexpr const unsigned char SENSOR_TIME{0x0C};
  inline static constexpr const unsigned char EVENT{0x10};
  inline static constexpr const unsigned char INT_STATUS{0x11};
  inline static constexpr const unsigned char FIFO_LENGTH{0x12};
  inline static constexpr const unsigned char FIFO_DATA{0x14};
  inline static constexpr const unsigned char CHIP_ID_DEFAULT{0x60};
}

#endif  // MAV_SENSORS_BMP390_SPI_IDS_H
