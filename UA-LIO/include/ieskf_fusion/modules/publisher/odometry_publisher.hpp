#ifndef IESKF_FUSION_ODOMETRY_PUBLISHER_HPP
#define IESKF_FUSION_ODOMETRY_PUBLISHER_HPP

#include <ros/ros.h>

#include <Eigen/Dense>
#include <string>

#include "nav_msgs/Odometry.h"

namespace ieskf_fusion {
class OdometryPublisher {
 public:
  OdometryPublisher(ros::NodeHandle& nh, std::string topic_name,
                    std::string base_frame_id, std::string child_frame_id,
                    int buff_size);
  OdometryPublisher() = default;

  void Publish(const Eigen::Matrix4f& transform_matrix, double time);
  void Publish(const Eigen::Matrix4f& transform_matrix);

  void PublishData(const Eigen::Matrix4f& transform_matrix, ros::Time time);

 private:
  ros::NodeHandle node_;
  ros::Publisher publisher_;
  nav_msgs::Odometry odometry_;
};
}  // namespace ieskf_fusion
#endif