//
// Created by haumarco on 13.02.24.
//

#include <log++.h>

#include "mav_sensors_core/sensor_config.h"
#include "mav_sensors_drivers/radar/xwr18xx_mmw_demo.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

using namespace mav_sensors;

class Xwr18XxMmwDemoNode {
 public:
  Xwr18XxMmwDemoNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
      : nh_(nh), nh_private_(nh_private) {}


  std::unique_ptr<Xwr18XxMmwDemo> radar_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Publisher cfar_pub_ = nh_private_.advertise<sensor_msgs::PointCloud2>("cfar_detections", 1);

  void run() {
    SensorConfig cfg;
    std::string current_file_path = __FILE__;
    std::string radar_config;
    if (!nh_private_.getParam("radar_config", radar_config)) {
      LOG(F, "Failed to get radar_config param.");
      return;
    }

    size_t src_idx = current_file_path.rfind(std::string("/src/"));
    if (src_idx != std::string::npos) {
      auto pkg_directory = current_file_path.substr(0, src_idx);
      cfg.set("path_cfg_file", pkg_directory + "/cfg/radar/" + radar_config);
    }
    cfg.set("path_cfg", "/dev/radar0");
    cfg.set("path_data", "/dev/radar1");
    cfg.set("trigger", "true");
    cfg.set("trigger_delay", "500");  // in ns
    cfg.set("trigger_gpio", "389");
    cfg.set("trigger_gpio_name", "PG.06");

    radar_ = std::make_unique<Xwr18XxMmwDemo>(cfg);
    if (!radar_->open()) {
      LOG(F, "Open failed.");
      return;
    }
    double poll_rate = 10.0;

    ros::Timer timer_ = nh_private_.createTimer(ros::Duration(1.0 / poll_rate), [this](const ros::TimerEvent& event) {
        timerCallback(event);
    });
    ros::spin();
    radar_->close();
  }

 private:
  void timerCallback(const ros::TimerEvent& event) {
    // auto time1 = ros::Time::now().toSec();
    auto measurement = radar_->read();
    // LOG(I, "timed " << ros::Time::now().toSec()- time1);
    // LOG(I, "Unix stamp: " << std::get<Radar>(measurement).unix_stamp_ns);
    // LOG(I, "Hardware stamp: " << std::get<Radar>(measurement).hardware_stamp);
    LOG(I, "Number of detections: " << std::get<Radar>(measurement).cfar_detections.size());
    publish_pointcloud(measurement);
    return;
  }

  void publish_pointcloud(std::tuple<mav_sensors::Radar::ReturnType> measurement) {
    LOG_FIRST(I, 1, "Publishing first radar cfar detections.");
    sensor_msgs::PointCloud2 msg;
    uint64_t unix_t = std::get<Radar>(measurement).unix_stamp_ns;
    uint64_t sec = unix_t * 1e-9;
    uint64_t nsec = unix_t - sec * 1e9;
    msg.header.stamp = ros::Time(sec, nsec);
    msg.header.frame_id = "awr1843aop";
    msg.height = 1;
    msg.width = std::get<Radar>(measurement).cfar_detections.size();

    msg.fields.resize(6);
    msg.fields[0].name = "x";
    msg.fields[0].offset = 0;
    msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    msg.fields[0].count = 1;

    msg.fields[1].name = "y";
    msg.fields[1].offset = 4;
    msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    msg.fields[1].count = 1;

    msg.fields[2].name = "z";
    msg.fields[2].offset = 8;
    msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    msg.fields[2].count = 1;

    msg.fields[3].name = "doppler";
    msg.fields[3].offset = 12;
    msg.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
    msg.fields[3].count = 1;

    msg.fields[4].name = "snr";
    msg.fields[4].offset = 16;
    msg.fields[4].datatype = sensor_msgs::PointField::INT16;
    msg.fields[4].count = 1;

    msg.fields[5].name = "noise";
    msg.fields[5].offset = 18;
    msg.fields[5].datatype = sensor_msgs::PointField::INT16;
    msg.fields[5].count = 1;

    int n = 1;
    msg.is_bigendian = *(char*)&n != 1;
    msg.point_step = 20;
    msg.row_step = msg.point_step * msg.width;
    msg.is_dense = true;

    msg.data.resize(msg.row_step * msg.height);
    for (size_t i = 0;
        i < std::get<Radar>(measurement).cfar_detections.size(); i++) {
      char x[sizeof(float)];
      memcpy(x, &std::get<Radar>(measurement).cfar_detections[i].x,
            sizeof(float));
      msg.data[i * msg.point_step + msg.fields[0].offset + 0] = x[0];
      msg.data[i * msg.point_step + msg.fields[0].offset + 1] = x[1];
      msg.data[i * msg.point_step + msg.fields[0].offset + 2] = x[2];
      msg.data[i * msg.point_step + msg.fields[0].offset + 3] = x[3];

      char y[sizeof(float)];
      memcpy(y, &std::get<Radar>(measurement).cfar_detections[i].y,
            sizeof(float));
      msg.data[i * msg.point_step + msg.fields[1].offset + 0] = y[0];
      msg.data[i * msg.point_step + msg.fields[1].offset + 1] = y[1];
      msg.data[i * msg.point_step + msg.fields[1].offset + 2] = y[2];
      msg.data[i * msg.point_step + msg.fields[1].offset + 3] = y[3];

      char z[sizeof(float)];
      memcpy(z, &std::get<Radar>(measurement).cfar_detections[i].z,
            sizeof(float));
      msg.data[i * msg.point_step + msg.fields[2].offset + 0] = z[0];
      msg.data[i * msg.point_step + msg.fields[2].offset + 1] = z[1];
      msg.data[i * msg.point_step + msg.fields[2].offset + 2] = z[2];
      msg.data[i * msg.point_step + msg.fields[2].offset + 3] = z[3];

      char doppler[sizeof(float)];
      memcpy(doppler,
            &std::get<Radar>(measurement).cfar_detections[i].velocity,
            sizeof(float));
      msg.data[i * msg.point_step + msg.fields[3].offset + 0] = doppler[0];
      msg.data[i * msg.point_step + msg.fields[3].offset + 1] = doppler[1];
      msg.data[i * msg.point_step + msg.fields[3].offset + 2] = doppler[2];
      msg.data[i * msg.point_step + msg.fields[3].offset + 3] = doppler[3];

      char snr[sizeof(int16_t)];
      memcpy(snr, &std::get<Radar>(measurement).cfar_detections[i].snr,
            sizeof(int16_t));
      msg.data[i * msg.point_step + msg.fields[4].offset + 0] = snr[0];
      msg.data[i * msg.point_step + msg.fields[4].offset + 1] = snr[1];

      char noise[sizeof(int16_t)];
      memcpy(noise, &std::get<Radar>(measurement).cfar_detections[i].noise,
            sizeof(int16_t));
      msg.data[i * msg.point_step + msg.fields[5].offset + 0] = noise[0];
      msg.data[i * msg.point_step + msg.fields[5].offset + 1] = noise[1];
    }
    cfar_pub_.publish(msg);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "xwr18xx_mmw_data_pub_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  Xwr18XxMmwDemoNode node(nh, nh_private);
  node.run();
  return 0;
}