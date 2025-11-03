#ifndef IESKF_FUSION_IMU_SUBSCRIBER_HPP
#define IESKF_FUSION_IMU_SUBSCRIBER_HPP

#include <ros/ros.h>

#include <deque>

#include "ieskf_fusion/type/imu.hpp"
#include "sensor_msgs/Imu.h"

namespace ieskf_fusion {
class IMUSubscriber {
 public:
  IMUSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
  IMUSubscriber() = default;

  bool AddIMUData(std::deque<IMU>& imus_data_buff);

 private:
  void msg_callback(const sensor_msgs::Imu::ConstPtr& imu_msg_ptr);

  ros::NodeHandle node_;
  ros::Subscriber sub_;
  std::deque<IMU> new_imu_data_buff_;

  std::mutex buff_mutex_;
};
}  // namespace ieskf_fusion

#endif