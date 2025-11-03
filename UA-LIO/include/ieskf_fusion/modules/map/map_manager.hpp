#ifndef IESKF_FUSION_MAP_MANAGER_HPP
#define IESKF_FUSION_MAP_MANAGER_HPP
#define PCL_NO_PRECOMPILE

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "ieskf_fusion/type/pointcloud.hpp"
#include "ieskf_fusion/type/state.hpp"

namespace ieskf_fusion {
class MapManager {
 public:
  MapManager(double resolution);
  ~MapManager();
  void reset();
  void addScan(pcl::PointCloud<PointType>::Ptr curr_scan, const State& state);
  pcl::PointCloud<PointType>::ConstPtr getLocalMap();
  pcl::PointCloud<PointType>::ConstPtr getMap();
  pcl::KdTreeFLANN<PointType>::ConstPtr readKDtree();

 private:
  pcl::PointCloud<PointType>::Ptr local_map_ptr;
  pcl::PointCloud<PointType>::Ptr global_map_ptr;

  pcl::KdTreeFLANN<PointType>::Ptr kdtree_ptr;

  pcl::VoxelGrid<PointType> voxel_filter;
  double local_map_resolution;

  bool isInBoundingBox(const PointType& point, const Eigen::Vector3d& pos_t,
                       double half_side_length);
};
}  // namespace ieskf_fusion

#endif