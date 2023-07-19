//
// Created by brik on 01.07.2023
//

#include "mav_sensors_drivers/radar/xwr18xx_mmw_demo.h"

#include <cstring>
#include <fstream>

#include "mav_sensors_drivers/sensor_types/Radar.h"

using namespace mav_sensors;

template <>
Xwr18XxMmwDemo::MmwDemoOutputMessageType Xwr18XxMmwDemo::parse(const std::vector<byte>& data,
                                                               size_t* offset) const {
  int value = 0;
  for (size_t i = sizeof(MMWDEMO_OUTPUT_MSG_DETECTED_POINTS); i-- > 0;) {
    value |= data[*offset + i] << (8 * i);
  }
  *offset += sizeof(Xwr18XxMmwDemo::MmwDemoOutputMessageType);
  return static_cast<Xwr18XxMmwDemo::MmwDemoOutputMessageType>(value);
}

template <>
float Xwr18XxMmwDemo::parse(const std::vector<byte>& data, size_t* offset) const {
  auto value = parse<uint32_t>(data, offset);
  float result;
  std::memcpy(&result, &value, sizeof(float));
  return result;
}

typename Xwr18XxMmwDemo::super::TupleReturnType Xwr18XxMmwDemo::read() {
  std::tuple<Radar::ReturnType> measurement;

  auto now = std::chrono::system_clock::now();
  if (trigger_enabled_) {
    // Flush read buffer, maybe move this to serial driver
    if (!drv_data_.flushReadBuffer()) return measurement;

    struct timespec sleepTime {
      0, trigger_delay_
    };

    // Overwrite time stamp just before triggering radar
    now = std::chrono::system_clock::now();
    if (!gpio_->setGpioState(GpioState::HIGH)) {
      LOG(E, "Failed to set gpio to high " << ::strerror(errno));
    }
    nanosleep(&sleepTime, nullptr);
    if (!gpio_->setGpioState(GpioState::LOW)) {
      LOG(E, "Failed to set gpio to low " << ::strerror(errno));
    }
  }

  // Read data from serial buffer and detect magic key
  std::vector<byte> magic_bit(1);
  size_t i = 0;
  while (i < kMagicKey.size()) {
    auto n = drv_data_.read(magic_bit.data(), magic_bit.size());
    if (n > 0 && magic_bit[0] == kMagicKey[i]) {
      i++;  // Magic bit found. Increment counter.
    } else if (!drv_data_.available()) {
      LOG(W, "Magic key not found.");
      return measurement;
    } else {
      i = 0;  // Magic bit not found. Reset counter.
    }
  }
  if (!trigger_enabled_) {
    // Overwrite time stamp just after reading magic key.
    now = std::chrono::system_clock::now();
  }

  // Read header.
  std::vector<byte> header(kHeaderSize);
  auto n = drv_data_.read(header.data(), header.size(), header.size(), kTimeout);
  if (n != header.size()) {
    LOG(E, "Failed to read header");
    return measurement;
  }

  // Parse header.
  size_t offset = 0;
  auto version = parse<uint32_t>(header, &offset);
  LOG(W, version != kHeaderVersion,
      "XWR18XX firmware not version 0x" << std::hex << kHeaderVersion << " but 0x" << +version);
  auto total_packet_len = parse<uint32_t>(header, &offset);
  auto platform = parse<uint32_t>(header, &offset);
  LOG(W, platform != kHeaderPlatform,
      "XWR18XX platform not 0x" << std::hex << +kHeaderPlatform << " but 0x" << +platform);
  auto frame_number = parse<uint32_t>(header, &offset);
  auto time_cpu_cycles = parse<uint32_t>(header, &offset);
  auto num_detected_obj = parse<uint32_t>(header, &offset);
  auto num_tlvs = parse<uint32_t>(header, &offset);
  auto sub_frame_number = parse<uint32_t>(header, &offset);
  std::get<Radar>(measurement).cfar_detections.resize(num_detected_obj);
  std::get<Radar>(measurement).unix_stamp_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
  std::get<Radar>(measurement).hardware_stamp = time_cpu_cycles;

  // Read TLV.
  std::vector<byte> tlv(total_packet_len - header.size() - 4 * sizeof(uint16_t));
  size_t bytes_missing = tlv.size();

  while (bytes_missing) {
    uint8_t n_chunk = bytes_missing > 0xFF ? 0xFF : uint8_t(bytes_missing);
    std::vector<byte> chunk(n_chunk);
    n = drv_data_.read(chunk.data(), chunk.size(), n_chunk, kTimeout);
    if (n <= 0) {
      LOG(E, "Failed to read TLV: " << n << " out of " << +n_chunk << " bytes read");
      return measurement;
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
        std::get<Radar>(measurement).cfar_detections[i].x = parse<float>(tlv, &offset);
        std::get<Radar>(measurement).cfar_detections[i].y = parse<float>(tlv, &offset);
        std::get<Radar>(measurement).cfar_detections[i].z = parse<float>(tlv, &offset);
        std::get<Radar>(measurement).cfar_detections[i].velocity = parse<float>(tlv, &offset);
      }
    } else if (tlv_type == MMWDEMO_OUTPUT_MSG_DETECTED_POINTS_SIDE_INFO) {
      for (size_t i = 0; i < num_detected_obj; ++i) {
        std::get<Radar>(measurement).cfar_detections[i].snr = parse<int16_t>(tlv, &offset);
        std::get<Radar>(measurement).cfar_detections[i].noise = parse<int16_t>(tlv, &offset);
      }
    } else {
      // Skip.
      offset += tlv_length;
    }
  }

  return measurement;
}

bool Xwr18XxMmwDemo::open() {
  ConfigOptional path_cfg = cfg_.get("path_cfg");
  if (!path_cfg.has_value()) {
    LOG(E, "Sensor config must have field path_cfg");
    return false;
  }

  ConfigOptional path_data = cfg_.get("path_data");
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

  ConfigOptional cfg_file_path = cfg_.get("path_cfg_file");
  if (!cfg_file_path.has_value()) {
    LOG(I, "Sensor config doesn't have field path_cfg_file. Skipping config load.");
  } else {
    if (!loadConfig(cfg_file_path.value())) {
      LOG(W, "Skipped config load");
    }
  }

  ConfigOptional trigger = cfg_.get("trigger");

  if (!trigger.has_value()) {
    LOG(E, "Sensor config must have field trigger");
    return false;
  }

  if (trigger.value() == "true") {
    ConfigOptional gpio = cfg_.get("trigger_gpio");
    ConfigOptional delay = cfg_.get("trigger_delay");
    ConfigOptional gpio_name = cfg_.get("trigger_gpio_name");
    if (!gpio.has_value()) {
      LOG(E, "Sensor config must have field trigger_gpio");
      return false;
    }

    if (!delay.has_value()) {
      LOG(W, "Trigger delay doesn't have field trigger_delay. Setting 500ns as default.");
    }

    if (!gpio_name.has_value()) {
      LOG(E, "Sensor config must have field trigger_gpio_name");
      return false;
    }

    try {
      trigger_delay_ = std::stoi(delay.value());
      LOG(I, "Trigger delay set to: " << trigger_delay_);
    } catch (const std::invalid_argument& e) {
      LOG(E, "Field trigger_delay is not of integral type");
      return false;
    } catch (const std::out_of_range& e) {
      LOG(E, "Value of field trigger_delay is too large");
      return false;
    }

    try {
      gpio_ = Gpio(std::stoi(gpio.value()), gpio_name.value());
    } catch (const std::invalid_argument& e) {
      LOG(E, "Field trigger_gpio is not of integral type");
      return false;
    } catch (const std::out_of_range& e) {
      LOG(E, "Value of field trigger_gpio is too large");
      return false;
    }

    if (!gpio_->isExported()) {
      if (!gpio_->open()) {
        LOG(E, "Error on gpio open: " << ::strerror(errno));
        return false;
      }
      LOG(I, "Opened Gpio: " << gpio_->getPath());
    } else {
      LOG(W, "Gpio " << gpio_->getPath() << " already exported.");
    }

    usleep(100000);  // Wait for open.
    if (!gpio_->setDirection(GpioDirection::OUT)) {
      LOG(E, "Error setting gpio direction: " << ::strerror(errno));
      return false;
    }
    LOG(I, "Set gpio direction to out");

    // Set GPIO low at startup. Radar will not measure.
    if (!gpio_->setGpioState(GpioState::LOW)) {
      LOG(E, "Failed to set gpio to low " << ::strerror(errno));
    }

    trigger_enabled_ = true;
    LOG(I, "Trigger: enabled");
  } else if (trigger.value() == "false") {
    LOG(I, "Trigger: disabled");
  } else {
    LOG(E, "Invalid value in field trigger. Valid values are: true/false");
  }

  return true;
}

bool Xwr18XxMmwDemo::loadConfig(const std::string& file_path) const {
  std::vector<std::string> lines{};

  std::ifstream config_file(file_path);
  if (config_file.is_open()) {
    std::string line;

    while (std::getline(config_file, line)) {
      if (!line.empty() && line[0] != '%') {  // Ignore comments in config
        line += "\x0D";
        lines.push_back(line);
      }
    }
    config_file.close();
  } else {
    LOG(E, "Failed to open config file " << file_path << ": " << ::strerror(errno));
    return false;
  }

  for (const auto& line : lines) {
    if (drv_cfg_.write(line.data(), line.length()) != line.length()) {
      LOG(E, "Error on config write" << ::strerror(errno));
    }
    std::vector<byte> buf(512);
    ssize_t res = drv_cfg_.read(buf.data(), buf.size(), kPrompt.size(), 50);

    if (res <= 0) {
      LOG(E, "Error on read" << ::strerror(errno));
    }

    LOG(D, "Read: " << std::string(buf.begin(), buf.end()););
  }
  return true;
}
