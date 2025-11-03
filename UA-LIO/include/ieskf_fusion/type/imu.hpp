#ifndef IESKF_FUSION_IMU_HPP
#define IESKF_FUSION_IMU_HPP

#include <Eigen/Dense>

#include "sensor_msgs/Imu.h"

namespace ieskf_fusion {
class IMU {
 public:
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyroscope = Eigen::Vector3d::Zero();
  double time_stamp = 0.0;

  IMU(const sensor_msgs::Imu::ConstPtr& msg) {
    acceleration =
        Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y,
                        msg->linear_acceleration.z);
    gyroscope =
        Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y,
                        msg->angular_velocity.z);
    time_stamp = msg->header.stamp.toSec();
  }

  IMU() = default;

  ~IMU() = default;

  IMU operator+(const IMU& imu) {
    IMU res;
    res.acceleration = this->acceleration + imu.acceleration;
    res.gyroscope = this->gyroscope + imu.gyroscope;
    return res;
  }

  IMU operator*(double k) {
    IMU res;
    res.acceleration = this->acceleration * k;
    res.gyroscope = this->gyroscope * k;
    return res;
  }

  IMU operator/(double k) {
    IMU res;
    res.acceleration = this->acceleration / k;
    res.gyroscope = this->gyroscope / k;
    return res;
  }

  void clear() {
    acceleration = Eigen::Vector3d::Zero();
    gyroscope = Eigen::Vector3d::Zero();
    time_stamp = 0.0;
  }
};
}  // namespace ieskf_fusion

#endif