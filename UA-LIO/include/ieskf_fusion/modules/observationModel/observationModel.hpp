#ifndef IESKF_FUSION_OBSERVATION_MODEL_HPP
#define IESKF_FUSION_OBSERVATION_MODEL_HPP

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Dense>

#include "ieskf_fusion/math/SO3.hpp"
#include "ieskf_fusion/type/point.hpp"
#include "ieskf_fusion/type/state.hpp"

namespace ieskf_fusion {
class ObservationModel {
 public:
  template <typename PointType, typename T>
  static PointType transformPoint(PointType point,
                                  const Eigen::Quaternion<T>& q,
                                  const Eigen::Matrix<T, 3, 1>& t) {
    Eigen::Matrix<T, 3, 1> ep = {point.x, point.y, point.z};
    ep = q * ep + t;
    point.x = ep.x();
    point.y = ep.y();
    point.z = ep.z();
    return point;
  }

  virtual ~ObservationModel() = default;

  virtual void setCurrentPointCloud(pcl::PointCloud<PointType>::Ptr cloud) = 0;
  virtual void setMap(pcl::PointCloud<PointType>::ConstPtr map_cloud,
                      pcl::KdTreeFLANN<PointType>::ConstPtr map_kdtree) = 0;

  virtual bool compute(const State& state, Eigen::MatrixXd& Z,
                       Eigen::MatrixXd& H, bool converge) = 0;

 protected:
  pcl::KdTreeFLANN<PointType>::ConstPtr map_kdtree_ptr;
  pcl::PointCloud<PointType>::Ptr current_cloud_ptr;
  pcl::PointCloud<PointType>::ConstPtr local_map_ptr;
};
}  // namespace ieskf_fusion

#endif