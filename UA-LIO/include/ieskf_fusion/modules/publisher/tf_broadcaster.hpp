#ifndef IESKF_FUSION_TF_BROADCASTER_HPP
#define IESKF_FUSION_TF_BROADCASTER_HPP

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Dense>
#include <string>

namespace ieskf_fusion {
class TFBroadCaster {
 public:
  TFBroadCaster(std::string frame_id, std::string child_frame_id);
  TFBroadCaster() = default;
  void SendTransform(Eigen::Matrix4f pose, double time);

 protected:
  tf::StampedTransform transform_;
  tf::TransformBroadcaster broadcaster_;
};
}  // namespace ieskf_fusion
#endif