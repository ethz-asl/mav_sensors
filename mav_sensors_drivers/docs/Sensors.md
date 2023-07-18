# Sensors

## Adis16448

Adis16448 is an IMU sensor from Analog Devices. It has a triaxial gyroscope, triaxial accelerometer, a triaxial
magnetometer and a temperature sensor).

### Hardware protocols

| Protocol | Implemented |
|----------|-------------|
| Spi      | Yes         |


### Sensor configuration

| Field     | Value                | Example        | Required |
|-----------|----------------------|----------------|----------|
| path      | Spi device path      | /dev/spidev0.0 | Yes      |
| burst_crc | Burst CRC check bool | true           | No       |

## BMI088

BMI088 is a 6-axis IMU sensor from Bosch Sensortec. It has a triaxial gyroscope and a triaxial accelerometer.

### Hardware protocols

| Protocol | Implemented |
|----------|-------------|
| I2C      | No          |
| Spi      | Yes         |

### Sensor configuration

| Field     | Value                         | Example        | Required |
|-----------|-------------------------------|----------------|----------|
| path_gyro | Spi gyro device path          | /dev/spidev0.1 | Yes      |
| path_acc  | Spi accelerometer device path | /dev/spidev0.0 | Yes      |


## BMP390

BMP390 is a pressure sensor from Bosch Sensortec. It has a pressure and a temperature sensor.

### Hardware protocols
| Protocol | Implemented |
|----------|-------------|
| I2C      | No          |
| Spi      | Yes         |

### Sensor configuration
| Field     | Value                         | Example        | Required |
|-----------|-------------------------------|----------------|----------|
| path      | Spi device path               | /dev/spidev0.0 | Yes      |


## xwr18xx
xwr18xx is a radar sensor from Texas Instruments. It has a 77GHz radar and a temperature sensor.

### Hardware protocols

| Protocol              | Implemented |
|-----------------------|-------------|
| Serial                | Yes         |
| GPIO (for triggering) | Yes         |

### Sensor configuration

| Field             | Value                     | Example      | Required |
|-------------------|---------------------------|--------------|----------|
| path_cfg          | Serial config device path | /dev/ttyUSB0 | Yes      |
| path_data         | Serial data device path   | /dev/ttyUSB1 | Yes      |
| trigger           | Trigger enable bool       | true         | Yes      |
| trigger_gpio      | GPIO pin number           | 443          | No (*)   |
| trigger_gpio_name | GPIO sysfs name           | PR.00        | No (*)   |
| trigger_delay     | Trigger delay in ns       | 100          | No (*)   |

(*) Yes if trigger bool is true
