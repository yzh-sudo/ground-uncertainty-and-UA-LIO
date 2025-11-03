#include "ieskf_fusion/modules/publisher/pointcloud2_publisher.hpp"

#include <utility>

namespace ieskf_fusion {
PointCloudPublisher::PointCloudPublisher(ros::NodeHandle& nh,
                                         std::string topic_name,
                                         std::string frame_id, int buff_size)
    : node_(nh) {
  publisher_ = node_.advertise<sensor_msgs::PointCloud2>(topic_name, buff_size);
  frame_id_ = std::move(frame_id);
}

void PointCloudPublisher::Publish(
    const pcl::PointCloud<PointType>::ConstPtr& cloud_ptr) {
  PublishData(cloud_ptr, ros::Time::now());
}

void PointCloudPublisher::Publish(
    const pcl::PointCloud<PointType>::ConstPtr& cloud_ptr, double time) {
  ros::Time ros_time(time);
  PublishData(cloud_ptr, ros_time);
}

void PointCloudPublisher::PublishData(
    const pcl::PointCloud<PointType>::ConstPtr& cloud_ptr, ros::Time time) {
  pcl::toROSMsg(*cloud_ptr, pointcloud_);
  pointcloud_.header.stamp = time;
  pointcloud_.header.frame_id = frame_id_;
  publisher_.publish(pointcloud_);
}
}  // namespace ieskf_fusion