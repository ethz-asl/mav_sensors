/*
BSD 3-Clause License

Copyright (c) 2024, ETH Zurich, Autonomous Systems Lab, Mariano Biasio, Rik Girod

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

//
// Created by acey on 23.08.22.
//

#ifndef MAV_IMU_INCLUDE_ADIS16448_CMDS_H_
#define MAV_IMU_INCLUDE_ADIS16448_CMDS_H_

#define CMD(x) \
  { (x), 0x00 }

#define FLASH_CNT 0x00
#define XGYRO_OUT 0x04
#define YGYRO_OUT 0x06
#define ZGYRO_OUT 0x08
#define XACCL_OUT 0x0A
#define YACCL_OUT 0x0C
#define ZACCL_OUT 0x0E
#define XMAGN_OUT 0x10
#define YMAGN_OUT 0x12
#define ZMAGN_OUT 0x14
#define BARO_OUT 0x16
#define TEMP_OUT 0x18
#define XGYRO_OFF 0x1A
#define YGYRO_OFF 0x1C
#define ZGYRO_OFF 0x1E
#define XACCL_OFF 0x20
#define YACCL_OFF 0x22
#define ZACCL_OFF 0x24
#define XMAGN_HIC 0x26
#define YMAGN_HIC 0x28
#define ZMAGN_HIC 0x2A
#define XMAGN_SIC 0x2C
#define YMAGN_SIC 0x2E
#define ZMAGN_SIC 0x30
#define GPIO_CTRL 0x32
#define MSC_CTRL 0x34
#define SMPL_PRD 0x36
#define SENS_AVG 0x38
#define SEQ_CNT 0x3A
#define DIAG_STAT 0x3C
#define GLOB_CMD 0x3E
#define ALM_MAG1 0x40
#define ALM_MAG2 0x42
#define ALM_SMPL1 0x44
#define ALM_SMPL2 0x46
#define ALM_CTRL 0x48
#define LOT_ID1 0x52
#define LOT_ID2 0x54
#define PROD_ID 0x56
#define SERIAL_NUM 0x58

#endif //MAV_IMU_INCLUDE_ADIS16448_CMDS_H_
