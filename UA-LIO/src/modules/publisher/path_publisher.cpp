#include "ieskf_fusion/modules/publisher/path_publisher.hpp"

namespace ieskf_fusion {
PathPublisher::PathPublisher(ros::NodeHandle& nh, std::string topic_name,
                             std::string frame_id, int buff_size)
    : node_(nh) {
  publisher_ = node_.advertise<nav_msgs::Path>(topic_name, buff_size);
  path_.header.frame_id = std::move(frame_id);
}

void PathPublisher::Publish(const Eigen::Matrix4f& transform_matrix) {
  PublishData(transform_matrix, ros::Time::now());
}

void PathPublisher::Publish(const Eigen::Matrix4f& transform_matrix,
                            double time) {
  ros::Time ros_time(time);
  PublishData(transform_matrix, ros_time);
}

void PathPublisher::PublishData(const Eigen::Matrix4f& transform_matrix,
                                ros::Time time) {
  geometry_msgs::PoseStamped pose_stamped;

  pose_stamped.header.stamp = time;
  pose_stamped.header.frame_id = path_.header.frame_id;

  pose_stamped.pose.position.x = transform_matrix(0, 3);
  pose_stamped.pose.position.y = transform_matrix(1, 3);
  pose_stamped.pose.position.z = transform_matrix(2, 3);

  Eigen::Quaternionf q;
  q = transform_matrix.block<3, 3>(0, 0);
  pose_stamped.pose.orientation.x = q.x();
  pose_stamped.pose.orientation.y = q.y();
  pose_stamped.pose.orientation.z = q.z();
  pose_stamped.pose.orientation.w = q.w();

  path_.poses.push_back(pose_stamped);
  path_.header.stamp = time;

  publisher_.publish(path_);
}
}  // namespace ieskf_fusion