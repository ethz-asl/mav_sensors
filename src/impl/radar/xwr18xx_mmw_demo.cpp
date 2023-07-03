//
// Created by brik on 01.07.2023
//

#include "mav_sensors/impl/radar/xwr18xx_mmw_demo.h"

#include "mav_sensors/core/sensor_types/Radar.h"

template <>
Xwr18XxMmwDemo::MmwDemoOutputMessageType Xwr18XxMmwDemo::parse(const std::vector<byte>& data,
                                                               size_t* offset) {
  int value = 0;
  for (size_t i = sizeof(MMWDEMO_OUTPUT_MSG_DETECTED_POINTS); i-- > 0;) {
    value |= data[*offset + i] << (8 * i);
  }
  *offset += sizeof(Xwr18XxMmwDemo::MmwDemoOutputMessageType);
  return static_cast<Xwr18XxMmwDemo::MmwDemoOutputMessageType>(value);
}

template <>
float Xwr18XxMmwDemo::parse(const std::vector<byte>& data, size_t* offset) {
  auto value = parse<uint32_t>(data, offset);
  float result;
  std::memcpy(&result, &value, sizeof(float));
  return result;
}

typename Xwr18XxMmwDemo::super::TupleReturnType Xwr18XxMmwDemo::read() {
  // Read data from serial buffer and detect magic key
  std::vector<byte> magic_bit(1);
  size_t i = 0;
  while (i < kMagicKey.size()) {
    auto n = drv_data_.read(&magic_bit);
    if (n > 0 && magic_bit[0] == kMagicKey[i]) {
      i++;  // Magic bit found. Increment counter.
    } else if (!drv_data_.available()) {
      LOG(W, "Magic key not found.");
      return std::make_tuple(Radar::ReturnType());
    } else {
      i = 0;  // Magic bit not found. Reset counter.
    }
  }

  // Read header.
  std::vector<byte> header(kHeaderSize);
  auto n = drv_data_.read(&header, header.size(), kTimeout);
  if (n != header.size()) {
    LOG(E, "Failed to read header");
    return std::make_tuple(Radar::ReturnType());
  }

  // Parse header.
  size_t offset = 0;
  auto version = parse<uint32_t>(header, &offset);
  LOG(W, version != kHeaderVersion,
      "XWR18XX firmware not version 0x" << std::hex << kHeaderVersion << " but 0x" << +version);
  uint32_t total_packet_len = parse<uint32_t>(header, &offset);
  uint32_t platform = parse<uint32_t>(header, &offset);
  LOG(W, platform != kHeaderPlatform,
      "XWR18XX platform not 0x" << std::hex << +kHeaderPlatform << " but 0x" << +platform);
  uint32_t frame_number = parse<uint32_t>(header, &offset);
  uint32_t time_cpu_cycles = parse<uint32_t>(header, &offset);
  uint32_t num_detected_obj = parse<uint32_t>(header, &offset);
  uint32_t num_tlvs = parse<uint32_t>(header, &offset);
  uint32_t sub_frame_number = parse<uint32_t>(header, &offset);
  Radar::ReturnType measurement(num_detected_obj);
  measurement.hardware_stamp = time_cpu_cycles;

  // Read TLV.
  std::vector<byte> tlv(total_packet_len - header.size() - 4 * sizeof(uint16_t));
  size_t bytes_missing = tlv.size();

  while (bytes_missing) {
    uint8_t n_chunk = bytes_missing > 0xFF ? 0xFF : uint8_t(bytes_missing);
    std::vector<byte> chunk(n_chunk);
    n = drv_data_.read(&chunk, n_chunk, kTimeout);
    if (n <= 0) {
      LOG(E, "Failed to read TLV: " << n << " out of " << +n_chunk << " bytes read");
      return std::make_tuple(measurement);
    }
    std::copy(chunk.begin(), chunk.begin() + n, tlv.begin() + tlv.size() - bytes_missing);
    bytes_missing -= n;
  }

  // Parse data.
  offset = 0;
  while (offset < tlv.size()) {
    auto tlv_type = parse<MmwDemoOutputMessageType>(tlv, &offset);
    auto tlv_length = parse<uint32_t>(tlv, &offset);
    if (tlv_type == MMWDEMO_OUTPUT_MSG_DETECTED_POINTS) {
      // Point cloud.
      for (size_t i = 0; i < num_detected_obj; ++i) {
        measurement.cfar_detections[i].x = parse<float>(tlv, &offset);
        measurement.cfar_detections[i].y = parse<float>(tlv, &offset);
        measurement.cfar_detections[i].z = parse<float>(tlv, &offset);
        measurement.cfar_detections[i].velocity = parse<float>(tlv, &offset);
      }
    } else if (tlv_type == MMWDEMO_OUTPUT_MSG_DETECTED_POINTS_SIDE_INFO) {
      for (size_t i = 0; i < num_detected_obj; ++i) {
        measurement.cfar_detections[i].snr = parse<int16_t>(tlv, &offset);
        measurement.cfar_detections[i].noise = parse<int16_t>(tlv, &offset);
      }
    } else {
      // Skip.
      offset += tlv_length;
    }
  }

  return std::make_tuple(measurement);
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