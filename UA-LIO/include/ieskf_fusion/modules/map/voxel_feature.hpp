#ifndef IESKF_FUSION_VOXEL_FEATURE_HPP
#define IESKF_FUSION_VOXEL_FEATURE_HPP

#include <Eigen/Dense>

namespace ieskf_fusion {

struct PointWithCov {
  Eigen::Vector3d point;
  Eigen::Vector3d world_point;
  Eigen::Matrix3d cov;  //世界系下点的不确定性
};

struct Plane {
  Eigen::Vector3d normal;
  Eigen::Vector3d local_normal;
  Eigen::Vector3d center;
  Eigen::Matrix3d covariance;
  Eigen::Matrix<double, 6, 6> plane_cov;

  bool is_plane = false;
  bool is_inited = false;
  bool is_merged = false;

  int points_num;

  // judge parameters
  float d = 0;
  float radius = 0;

  // visualization parameters
  Eigen::Vector3d x_normal;
  Eigen::Vector3d y_normal;
  float min_eigen_value;
  float mid_eigen_value;
  float max_eigen_value;
  int plane_id;
};

}  // namespace ieskf_fusion

#endif