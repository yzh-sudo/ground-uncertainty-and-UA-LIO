#include "ieskf_fusion/modules/map/map_manager.hpp"

namespace ieskf_fusion {
MapManager::MapManager(double resolution) {
  local_map_resolution = resolution;
  std::cout << "map_resolution: " << local_map_resolution << std::endl;

  local_map_ptr = pcl::make_shared<pcl::PointCloud<PointType>>();
  global_map_ptr = pcl::make_shared<pcl::PointCloud<PointType>>();
  kdtree_ptr = pcl::make_shared<pcl::KdTreeFLANN<PointType>>();

  voxel_filter.setLeafSize(local_map_resolution, local_map_resolution,
                           local_map_resolution);
}

MapManager::~MapManager() = default;

void MapManager::addScan(pcl::PointCloud<PointType>::Ptr curr_scan,
                         const State& state) {
  pcl::PointCloud<PointType>::Ptr feats_down_scan(
      new pcl::PointCloud<PointType>());
  Eigen::Matrix4d trans = Eigen::Matrix4d::Identity();
  Eigen::Vector3d pos = state.rotation * state.offset_T_L_I + state.position;
  Eigen::Quaterniond rot =
      Eigen::Quaterniond(state.rotation * state.offset_R_L_I);
  trans.block<3, 3>(0, 0) = rot.toRotationMatrix();
  trans.block<3, 1>(0, 3) = pos;
  voxel_filter.setInputCloud(curr_scan);
  voxel_filter.filter(*feats_down_scan);
  pcl::PointCloud<PointType>::Ptr world_scan(new pcl::PointCloud<PointType>());
  pcl::transformPointCloud(*feats_down_scan, *world_scan, trans.cast<float>());
  *global_map_ptr += *world_scan;

  if (local_map_ptr->empty()) {
    *local_map_ptr = *world_scan;
  } else {
    for (auto&& point : world_scan->points) {
      std::vector<int> ind;
      std::vector<float> distance;
      kdtree_ptr->nearestKSearch(point, 5, ind, distance);
      if (distance[0] > local_map_resolution) {
        local_map_ptr->push_back(point);
      }
    }

    // int left = 0;
    // int right = local_map_ptr->size() - 1;
    // while (left < right) {
    //   while (left < right && !isInBoundingBox(local_map_ptr->points[right],
    //                                           state.position, 250)) {
    //     right--;
    //   }
    //   while (left < right && isInBoundingBox(local_map_ptr->points[left],
    //                                          state.position, 250)) {
    //     left++;
    //   }
    //   std::swap(local_map_ptr->points[left], local_map_ptr->points[right]);
    // }
    // local_map_ptr->resize(right + 1);
  }
  kdtree_ptr->setInputCloud(local_map_ptr);
}

void MapManager::reset() { local_map_ptr->clear(); }

pcl::PointCloud<PointType>::ConstPtr MapManager::getLocalMap() {
  return local_map_ptr;
}

pcl::PointCloud<PointType>::ConstPtr MapManager::getMap() {
  return global_map_ptr;
}

pcl::KdTreeFLANN<PointType>::ConstPtr MapManager::readKDtree() {
  return kdtree_ptr;
}

bool MapManager::isInBoundingBox(const PointType& point,
                                 const Eigen::Vector3d& pos_t,
                                 double half_side_length) {
  return std::abs(point.x - pos_t.x()) <= half_side_length &&
         std::abs(point.y - pos_t.y()) <= half_side_length &&
         std::abs(point.z - pos_t.z()) <= half_side_length;
}

}  // namespace ieskf_fusion