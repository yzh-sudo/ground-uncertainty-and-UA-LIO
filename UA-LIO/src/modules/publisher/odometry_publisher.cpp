#include "ieskf_fusion/modules/publisher/odometry_publisher.hpp"

namespace ieskf_fusion {
OdometryPublisher::OdometryPublisher(ros::NodeHandle& nh,
                                     std::string topic_name,
                                     std::string base_frame_id,
                                     std::string child_frame_id, int buff_size)
    : node_(nh) {
  publisher_ = node_.advertise<nav_msgs::Odometry>(topic_name, buff_size);
  odometry_.header.frame_id = std::move(base_frame_id);
  odometry_.child_frame_id = std::move(child_frame_id);
}

void OdometryPublisher::Publish(const Eigen::Matrix4f& transform_matrix) {
  PublishData(transform_matrix, ros::Time::now());
}

void OdometryPublisher::Publish(const Eigen::Matrix4f& transform_matrix,
                                double time) {
  ros::Time ros_time(time);
  PublishData(transform_matrix, ros_time);
}

void OdometryPublisher::PublishData(const Eigen::Matrix4f& transform_matrix,
                                    ros::Time time) {
  odometry_.header.stamp = time;

  odometry_.pose.pose.position.x = transform_matrix(0, 3);
  odometry_.pose.pose.position.y = transform_matrix(1, 3);
  odometry_.pose.pose.position.z = transform_matrix(2, 3);

  Eigen::Quaternionf q;
  q = transform_matrix.block<3, 3>(0, 0);
  odometry_.pose.pose.orientation.x = q.x();
  odometry_.pose.pose.orientation.y = q.y();
  odometry_.pose.pose.orientation.z = q.z();
  odometry_.pose.pose.orientation.w = q.w();

  publisher_.publish(odometry_);
}
}  // namespace ieskf_fusion