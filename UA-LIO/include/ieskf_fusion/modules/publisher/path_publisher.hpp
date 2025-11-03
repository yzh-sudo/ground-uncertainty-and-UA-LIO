#ifndef IESKF_FUSION_PATH_PUBLISHER_HPP
#define IESKF_FUSION_PATH_PUBLISHER_HPP

#include <ros/ros.h>

#include <Eigen/Dense>
#include <string>

#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"

namespace ieskf_fusion {
class PathPublisher {
 public:
  PathPublisher(ros::NodeHandle& nh, std::string topic_name,
                std::string frame_id, int buff_size);
  PathPublisher() = default;

  void Publish(const Eigen::Matrix4f& transform_matrix, double time);
  void Publish(const Eigen::Matrix4f& transform_matrix);

  void PublishData(const Eigen::Matrix4f& transform_matrix, ros::Time time);

 private:
  ros::NodeHandle node_;
  ros::Publisher publisher_;
  nav_msgs::Path path_;
};
}  // namespace ieskf_fusion
#endif