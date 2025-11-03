#ifndef IESKF_FUSION_LIDAR_SUBSCRIBER_HPP
#define IESKF_FUSION_LIDAR_SUBSCRIBER_HPP

#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>

#include <deque>

#include "ieskf_fusion/type/pointcloud.hpp"
#include "livox_ros_driver/CustomMsg.h"
#include "sensor_msgs/PointCloud2.h"

namespace ieskf_fusion {
class LidarSubscriber {
 public:
  LidarSubscriber(ros::NodeHandle& nh, std::string& topic_name,
                  size_t buff_size, std::string& lidar_type);
  LidarSubscriber() = default;

  bool AddLidarData(std::deque<PointCloud>& lidar_data_buff);

 private:
  void pointcloud_callback(
      const sensor_msgs::PointCloud2::ConstPtr& lidar_msg_ptr);
  void ouster_callback(const sensor_msgs::PointCloud2::ConstPtr& lidar_msg_ptr);
  void mid360_callback(
      const livox_ros_driver::CustomMsg::ConstPtr& lidar_msg_ptr);

  ros::NodeHandle node_;
  ros::Subscriber sub_;
  std::deque<PointCloud> new_lidar_data_buff_;

  std::mutex buff_mutex_;
  double blind = 0.5;
};
}  // namespace ieskf_fusion

#endif