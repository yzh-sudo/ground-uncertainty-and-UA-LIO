#ifndef IESKF_FUSION_VOXEL_MAP_HPP
#define IESKF_FUSION_VOXEL_MAP_HPP

#include <pcl/filters/voxel_grid.h>

#include <pcl/filters/impl/voxel_grid.hpp>
#include <unordered_map>

#include "ieskf_fusion/math/SO3.hpp"
#include "ieskf_fusion/modules/map/unionfind.hpp"
#include "ieskf_fusion/modules/map/voxel_util.hpp"
#include "ieskf_fusion/type/pointcloud.hpp"
#include "ieskf_fusion/type/state.hpp"
#include "ros/publisher.h"

namespace ieskf_fusion {

class VoxelMap {
 public:
  VoxelMap(double resolution);
  ~VoxelMap();
  void reset();
  void addScan(pcl::PointCloud<PointType>::Ptr curr_scan, const State& state);
  pcl::PointCloud<PointType>::ConstPtr getLocalMap();
  pcl::PointCloud<PointType>::ConstPtr getMap();

  const std::unordered_map<VOXEL_LOC, UnionFind*>& getVoxelMap() const {
    return voxel_map;
  }

 private:
  std::unordered_map<VOXEL_LOC, UnionFind*> voxel_map;

  pcl::PointCloud<PointType>::Ptr local_map_ptr;
  pcl::PointCloud<PointType>::Ptr global_map_ptr;

  pcl::VoxelGrid<PointType> voxel_filter;
  double local_map_resolution;
  double voxel_size;
  bool first_frame = true;
  float range_cov;
  float angle_cov;

  void buildVoxelMap(const std::vector<PointWithCov>& input_points,
                     std::unordered_map<VOXEL_LOC, UnionFind*>& voxel_map);
  void updateVoxelMap(const std::vector<PointWithCov>& input_points,
                      std::unordered_map<VOXEL_LOC, UnionFind*>& voxel_map);
  void calculatePointCov(const pcl::PointCloud<PointType>::Ptr& cloud,
                         const State& state,
                         std::vector<PointWithCov>& points_with_cov);
};
}  // namespace ieskf_fusion

#endif