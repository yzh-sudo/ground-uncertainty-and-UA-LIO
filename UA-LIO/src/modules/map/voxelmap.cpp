#include "ieskf_fusion/modules/map/voxelmap.hpp"

#include <pcl/common/transforms.h>

#include <utility>

namespace ieskf_fusion {

VoxelMap::VoxelMap(double resolution) {
  local_map_resolution = resolution;
  std::cout << "map_resolution: " << local_map_resolution << std::endl;

  local_map_ptr = pcl::make_shared<pcl::PointCloud<PointType>>();
  global_map_ptr = pcl::make_shared<pcl::PointCloud<PointType>>();

  voxel_filter.setLeafSize(local_map_resolution, local_map_resolution,
                           local_map_resolution);

  //   voxel_map = std::unordered_map<VOXEL_LOC, UnionFind*>();
  range_cov = 0.04;
  angle_cov = 0.1;
  voxel_size = 0.5;
}

VoxelMap::~VoxelMap() = default;

void VoxelMap::addScan(pcl::PointCloud<PointType>::Ptr curr_scan,
                       const State& state) {
  pcl::PointCloud<PointType>::Ptr feats_down_scan(
      new pcl::PointCloud<PointType>());
  Eigen::Matrix4d trans = Eigen::Matrix4d::Identity();
  Eigen::Vector3d pos = state.rotation * state.offset_T_L_I + state.position;
  auto rot = Eigen::Quaterniond(state.rotation * state.offset_R_L_I);
  trans.block<3, 3>(0, 0) = rot.toRotationMatrix();
  trans.block<3, 1>(0, 3) = pos;
  voxel_filter.setInputCloud(curr_scan);
  voxel_filter.filter(*feats_down_scan);
  //   feats_down_scan = std::move(curr_scan);
  pcl::PointCloud<PointType>::Ptr world_scan(new pcl::PointCloud<PointType>());
  pcl::transformPointCloud(*feats_down_scan, *world_scan, trans.cast<float>());

  std::vector<PointWithCov> points_with_cov;
  calculatePointCov(feats_down_scan, state, points_with_cov);
  if (first_frame) {
    buildVoxelMap(points_with_cov, voxel_map);
    first_frame = false;
    return;
  }

  updateVoxelMap(points_with_cov, voxel_map);
}

void VoxelMap::reset() { local_map_ptr->clear(); }

pcl::PointCloud<PointType>::ConstPtr VoxelMap::getLocalMap() {
  return local_map_ptr;
}

pcl::PointCloud<PointType>::ConstPtr VoxelMap::getMap() {
  return global_map_ptr;
}

void VoxelMap::buildVoxelMap(
    const std::vector<PointWithCov>& input_points,
    std::unordered_map<VOXEL_LOC, UnionFind*>& voxel_map) {
  uint plsize = input_points.size();
  for (uint i = 0; i < plsize; i++) {
    const PointWithCov& p_v = input_points[i];
    double loc_xyz[3];
    for (int j = 0; j < 3; j++) {
      loc_xyz[j] = p_v.world_point[j] / voxel_size;
      if (loc_xyz[j] < 0) {
        loc_xyz[j] -= 1.0;
      }
    }
    VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
                       (int64_t)loc_xyz[2]);
    auto iter = voxel_map.find(position);
    if (iter != voxel_map.end()) {
      voxel_map[position]->all_points.push_back(p_v);
    } else {
      auto* unionfind_node = new UnionFind();
      voxel_map[position] = unionfind_node;
      voxel_map[position]->voxel_center[0] =
          (0.5 + static_cast<double>(position.x)) * voxel_size;
      voxel_map[position]->voxel_center[1] =
          (0.5 + static_cast<double>(position.y)) * voxel_size;
      voxel_map[position]->voxel_center[2] =
          (0.5 + static_cast<double>(position.z)) * voxel_size;
      voxel_map[position]->all_points.push_back(p_v);
    }
  }
  for (auto& iter : voxel_map) {
    iter.second->init_unionfind_node();
  }
}

void VoxelMap::updateVoxelMap(
    const std::vector<PointWithCov>& input_points,
    std::unordered_map<VOXEL_LOC, UnionFind*>& voxel_map) {
  uint plsize = input_points.size();
  for (uint i = 0; i < plsize; i++) {
    const PointWithCov& p_v = input_points[i];
    double loc_xyz[3];
    for (int j = 0; j < 3; j++) {
      loc_xyz[j] = p_v.world_point[j] / voxel_size;
      if (loc_xyz[j] < 0) {
        loc_xyz[j] -= 1.0;
      }
    }
    VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
                       (int64_t)loc_xyz[2]);
    auto iter = voxel_map.find(position);
    if (iter != voxel_map.end()) {
      voxel_map[position]->update_unionfind_node(p_v, position, voxel_map);
    } else {
      auto* unionfind_node = new UnionFind();
      voxel_map[position] = unionfind_node;
      voxel_map[position]->voxel_center[0] =
          (0.5 + static_cast<double>(position.x)) * voxel_size;
      voxel_map[position]->voxel_center[1] =
          (0.5 + static_cast<double>(position.y)) * voxel_size;
      voxel_map[position]->voxel_center[2] =
          (0.5 + static_cast<double>(position.z)) * voxel_size;
      voxel_map[position]->update_unionfind_node(p_v, position, voxel_map);
    }
  }
}

void VoxelMap::calculatePointCov(const pcl::PointCloud<PointType>::Ptr& cloud,
                                 const State& state,
                                 std::vector<PointWithCov>& points_with_cov) {
  int point_num = cloud->size();
  points_with_cov.resize(point_num);
  Eigen::Vector3d pos =
      state.rotation.toRotationMatrix() * state.offset_T_L_I + state.position;
  Eigen::Quaterniond rot =
      Eigen::Quaterniond(state.rotation.toRotationMatrix() *
                         state.offset_R_L_I.toRotationMatrix());

  for (int i = 0; i < point_num; i++) {
    PointWithCov& p_v = points_with_cov[i];
    p_v.point = Eigen::Vector3d(cloud->points[i].x, cloud->points[i].y,
                                cloud->points[i].z);
    if (p_v.point[2] == 0) {
      p_v.point[2] = 0.001;
    }
    calcBodyCov(p_v.point, range_cov, angle_cov, p_v.cov);
    // std::cout << "before cov:\n" << p_v.cov << std::endl;
    p_v.cov = state.rotation.toRotationMatrix() * p_v.cov *
                  state.rotation.toRotationMatrix().transpose() +
              (-skewSymmetric(p_v.point)) * state.cov.block<3, 3>(3, 3) *
                  (-skewSymmetric(p_v.point)).transpose() +
              state.cov.block<3, 3>(0, 0);
    p_v.world_point = rot * p_v.point + pos;
    // std::cout << "after cov:\n" << p_v.cov << std::endl;
  }
}

}  // namespace ieskf_fusion
