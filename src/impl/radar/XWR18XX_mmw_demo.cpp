//
// Created by acey on 01.07.2023
//

#include "mav_sensors/impl/radar/XWR18XX_mmw_demo.h"

typename XWR18XXMmwDemo::super::TupleReturnType XWR18XXMmwDemo::read() {
  return std::make_tuple(CfarDetections::ReturnType());
}

bool XWR18XXMmwDemo::open() {
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