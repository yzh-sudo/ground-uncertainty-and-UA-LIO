#ifndef IESKF_FUSION_PROPAGATE_HPP
#define IESKF_FUSION_PROPAGATE_HPP

#include "ieskf_fusion/ieskf/ieskf.hpp"
#include "ieskf_fusion/modules/publisher/odometry_publisher.hpp"
#include "ieskf_fusion/type/measure_group.hpp"

namespace ieskf_fusion {
class Propagate {
 public:
  struct IMUPose6d {
    double time;
    Eigen::Vector3d acc;
    Eigen::Vector3d ang;
    Eigen::Vector3d vel;
    Eigen::Vector3d pos;
    Eigen::Quaterniond rot;
    IMUPose6d(double time_ = 0, Eigen::Vector3d acc_ = Eigen::Vector3d::Zero(),
              Eigen::Vector3d ang_ = Eigen::Vector3d::Zero(),
              Eigen::Vector3d vel_ = Eigen::Vector3d::Zero(),
              Eigen::Vector3d pos_ = Eigen::Vector3d::Zero(),
              Eigen::Quaterniond rot_ = Eigen::Quaterniond::Identity()) {
      time = time_;
      acc = acc_;
      ang = ang_;
      vel = vel_;
      pos = pos_;
      rot = rot_;
    }
  };
  Propagate();
  ~Propagate();

  void Propagation(MeasureGroup& measures, IESKF::Ptr ieskf_ptr,
                   std::shared_ptr<OdometryPublisher> predict_odom_pub_pt);
  void ForwardPropagation(
      MeasureGroup& measures, IESKF::Ptr ieskf_ptr,
      std::shared_ptr<OdometryPublisher> predict_odom_pub_ptr);
  void BackPropagation(MeasureGroup& measures, IESKF::Ptr ieskf_ptr);
  double getPredictDis();

  Eigen::Vector3d acc_last = Eigen::Vector3d::Zero();
  Eigen::Vector3d ang_last = Eigen::Vector3d::Zero();
  double last_lidar_end_time_;

  double imu_scale;
  double pre_dis = 0;
  IMU last_imu;
  std::vector<IMUPose6d> IMUpose;
  std::vector<IMUPose6d> IMU_bppose;

 private:
};
}  // namespace ieskf_fusion

#endif