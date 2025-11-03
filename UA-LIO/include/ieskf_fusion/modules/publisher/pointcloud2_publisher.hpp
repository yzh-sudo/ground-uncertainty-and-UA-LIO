#ifndef IESKF_FUSION_POINTCLOUD2_PUBLISHER_HPP
#define IESKF_FUSION_POINTCLOUD2_PUBLISHER_HPP

#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>

#include <Eigen/Dense>
#include <string>

#include "ieskf_fusion/type/pointcloud.hpp"
#include "sensor_msgs/PointCloud2.h"

namespace ieskf_fusion {
class PointCloudPublisher {
 public:
  PointCloudPublisher(ros::NodeHandle& nh, std::string topic_name,
                      std::string frame_id, int buff_size);
  PointCloudPublisher() = default;

  void Publish(const pcl::PointCloud<PointType>::ConstPtr& cloud_ptr,
               double time);
  void Publish(const pcl::PointCloud<PointType>::ConstPtr& cloud_ptr);

  void PublishData(const pcl::PointCloud<PointType>::ConstPtr& cloud_ptr,
                   ros::Time time);

 private:
  ros::NodeHandle node_;
  ros::Publisher publisher_;
  sensor_msgs::PointCloud2 pointcloud_;
  std::string frame_id_;
};
}  // namespace ieskf_fusion
#endif