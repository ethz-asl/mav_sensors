# mav_sensors
Linux user space sensor drivers.

Supporting BMI088 and ADIS16448 IMU, BMP390 barometer, and TI awr1843aop radar.

**Paper**: https://arxiv.org/pdf/2408.05764
```
@inproceedings{girod2024brio,
author = {Rik Girod and Marco Hauswirth and Patrick Pfreundschuh and Mariano Biasio and Roland Siegwart},
title = {A robust baro-radar-inertial odometry m-estimator for multicopter navigation in cities and forests},
booktitle={IEEE Int. Conf. Multisensor Fusion Integration Intell. Syst.},
year={2024}
}
```

# Related packages

| Package         | Description                       | Link                                                           |
| --------------- | --------------------------------- | -------------------------------------------------------------- |
| rio             | radar-inertial odometry estimator | [rio](https://github.com/ethz-asl/rio)                         |
| mav_sensors_ros | ROS sensor interface              | [mav_sensors_ros](https://github.com/ethz-asl/mav_sensors_ros) |
