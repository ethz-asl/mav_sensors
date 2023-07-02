//
// Created by brik on 01.07.2023
//

#include "mav_sensors/impl/radar/xwr18xx_mmw_demo.h"

typename Xwr18XxMmwDemo::super::TupleReturnType Xwr18XxMmwDemo::read() {
  // Read data from serial buffer and detect magic key 0x02, 0x01, 0x04, 0x03, 0x06, 0x05, 0x08,
  // 0x06.
  std::vector<byte> data(1);
  auto n = drv_data_.read(&data);
  bool new_data = false;
  while (n > 0 && data[0] != 0x02) {
    n = drv_data_.read(&data);
  }
  if (data[0] == 0x02) {
    n = drv_data_.read(&data);
    if (data[0] == 0x01) {
      n = drv_data_.read(&data);
      if (data[0] == 0x04) {
        n = drv_data_.read(&data);
        if (data[0] == 0x03) {
          n = drv_data_.read(&data);
          if (data[0] == 0x06) {
            n = drv_data_.read(&data);
            if (data[0] == 0x05) {
              n = drv_data_.read(&data);
              if (data[0] == 0x08) {
                n = drv_data_.read(&data);
                if (data[0] == 0x07) {
                  new_data = true;
                }
              }
            }
          }
        }
      }
    }
  }

  if (!new_data) return std::make_tuple(CfarDetections::ReturnType());

  // Read header.
  std::vector<byte> header(kHeaderSize);
  n = drv_data_.blockingRead(&header, header.size(), kTimeout);
  if (n != header.size()) {
    LOG(E, "Failed to read header");
    return std::make_tuple(CfarDetections::ReturnType());
  }

  // Parse header.
  size_t offset = 0;
  auto version = parse<uint32_t>(header, &offset);
  LOG(W, version != 0x03060000,
      "XWR18XX firmware not version 0x" << std::hex << 0x03060000 << " but 0x" << +version);
  uint32_t totalPacketLen = parse<uint32_t>(header, &offset);
  LOG(I, "Total packet length: " << totalPacketLen);
  uint32_t platform = parse<uint32_t>(header, &offset);
  LOG(I, "Platform: " << std::hex << +platform);
  uint32_t frameNumber = parse<uint32_t>(header, &offset);
  LOG(I, "Frame number: " << frameNumber);
  uint32_t timeCpuCycles = parse<uint32_t>(header, &offset);
  LOG(I, "Time CPU cycles: " << timeCpuCycles);
  uint32_t numDetectedObj = parse<uint32_t>(header, &offset);
  LOG(I, "Number of detected objects: " << numDetectedObj);
  uint32_t numTLVs = parse<uint32_t>(header, &offset);
  LOG(I, "Number of TLVs: " << numTLVs);
  uint32_t subFrameNumber = parse<uint32_t>(header, &offset);
  LOG(I, "Subframe number: " << subFrameNumber);

  // Read TLV.
  std::vector<byte> tlv(totalPacketLen - header.size() - 4 * sizeof(uint16_t));
  size_t bytes_missing = tlv.size();

  while (bytes_missing) {
    LOG(I, "Reading TLV, " << bytes_missing << " bytes missing");
    LOG(I, "Available bytes: " << drv_data_.available() << " bytes");
    uint8_t n_chunk = bytes_missing > 0xFF ? 0xFF : uint8_t(bytes_missing);
    std::vector<byte> chunk(n_chunk);
    n = drv_data_.blockingRead(&chunk, n_chunk, 0);
    if (n <= 0) {
      LOG(E, "Failed to read TLV: " << n << " out of " << +n_chunk << " bytes read");
      return std::make_tuple(CfarDetections::ReturnType());
    }
    LOG(I, "Read " << n << " bytes of TLV");
    std::copy(chunk.begin(), chunk.begin() + n, tlv.begin() + tlv.size() - bytes_missing);
    bytes_missing -= n;
  }

  // Parse data.
  return std::make_tuple(CfarDetections::ReturnType());
}

bool Xwr18XxMmwDemo::open() {
  std::optional<std::string> path_cfg = cfg_.get("path_cfg");
  if (!path_cfg.has_value()) {
    LOG(E, "Sensor config must have field path_cfg");
    return false;
  }

  std::optional<std::string> path_data = cfg_.get("path_data");
  if (!path_data.has_value()) {
    LOG(E, "Sensor config must have field path_data");
    return false;
  }

  drv_cfg_.setPath(path_cfg.value());
  if (!drv_cfg_.open()) return false;

  drv_data_.setPath(path_data.value());
  if (!drv_data_.open()) return false;

  // Default mmw uart config is: readTimeout = UART_WAIT_FOREVER; writeTimeout =
  // UART_WAIT_FOREVER; readReturnMode = UART_RETURN_NEWLINE; readDataMode = UART_DATA_TEXT;
  // writeDataMode = UART_DATA_TEXT; readEcho = UART_ECHO_ON; baudRate = 115200; dataLength =
  // UART_LEN_8; stopBits = UART_STOP_ONE; parityType = UART_PAR_NONE;

  if (!drv_cfg_.setControlBaudRate(115200)) return false;
  if (!drv_data_.setControlBaudRate(921600)) return false;

  return true;
}