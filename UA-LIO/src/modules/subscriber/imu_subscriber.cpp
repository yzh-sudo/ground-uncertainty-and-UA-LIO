#include "ieskf_fusion/modules/subscriber/imu_subscriber.hpp"

namespace ieskf_fusion {
IMUSubscriber::IMUSubscriber(ros::NodeHandle& nh, std::string topic_name,
                             size_t buff_size)
    : node_(nh) {
  sub_ = node_.subscribe(topic_name, buff_size, &IMUSubscriber::msg_callback,
                         this, ros::TransportHints().tcpNoDelay());
}

void IMUSubscriber::msg_callback(
    const sensor_msgs::Imu::ConstPtr& imu_msg_ptr) {
  std::lock_guard<std::mutex> lock(buff_mutex_);
  IMU imu_data(imu_msg_ptr);

  new_imu_data_buff_.push_back(imu_data);
}

bool IMUSubscriber::AddIMUData(std::deque<IMU>& imus_data_buff) {
  std::lock_guard<std::mutex> lock(buff_mutex_);
  if (!new_imu_data_buff_.empty()) {
    imus_data_buff.insert(imus_data_buff.end(), new_imu_data_buff_.begin(),
                          new_imu_data_buff_.end());
    new_imu_data_buff_.clear();
    return true;
  }
  return false;
}
}  // namespace ieskf_fusion