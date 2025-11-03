#ifndef IESKF_FUSION_FRONTEND_HPP
#define IESKF_FUSION_FRONTEND_HPP

#include <fstream>

#include "ieskf_fusion/ieskf/ieskf.hpp"
#include "ieskf_fusion/modules/map/map_manager.hpp"
#include "ieskf_fusion/modules/map/voxelmap.hpp"
#include "ieskf_fusion/modules/observationModel/Po2PlModel.hpp"
#include "ieskf_fusion/modules/observationModel/voxelmapModel.hpp"
#include "ieskf_fusion/modules/propagate/propagate.hpp"
#include "ieskf_fusion/type/measure_group.hpp"

namespace ieskf_fusion {
class FrontEnd {
 public:
  FrontEnd(ros::NodeHandle& nh);
  ~FrontEnd() = default;

  bool track(MeasureGroup& measure_group);
  pcl::PointCloud<PointType>::ConstPtr getLocalMap();
  pcl::PointCloud<PointType>::ConstPtr getMap();
  pcl::PointCloud<PointType>::Ptr getDistortCloud();
  Eigen::Matrix4f getPose();
  void write_planes_to_csv(
      const std::string& filename,
      const std::unordered_map<VOXEL_LOC, UnionFind*>& voxel_map);
  std::string csv_file_name;

 private:
  const double GRAVITY = 9.81;
  bool imu_inited = false;
  bool flg_first_scan = true;

  pcl::PointCloud<PointType> curr_scan;

  std::shared_ptr<IESKF> ieskf_ptr_;            // 滤波器
  std::shared_ptr<Propagate> propagate_ptr_;    // 前向传播
  std::shared_ptr<MapManager> map_ptr_;         // 地图管理
  std::shared_ptr<ObservationModel> obs_model;  // 观测模型
  std::shared_ptr<VoxelMapModel> voxel_model;   // 观测模型

  std::shared_ptr<VoxelMap> voxel_map_ptr_;  // 体素地图管理
  ros::Publisher voxel_plane_pub_;
  ros::Publisher voxel_near_pub_;
  ros::Publisher voxel_point_pub_;
  ros::Publisher text_pub;
  ros::Publisher near_text_pub;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
  int scan_idx = -1;

  std::shared_ptr<OdometryPublisher> odom_pub_ptr;

  void initState(MeasureGroup& measure_group);

  Eigen::Matrix4d offset_lidar_to_imu = Eigen::Matrix4d::Identity();
  pcl::VoxelGrid<PointType> voxel_filter;
};
}  // namespace ieskf_fusion

#endif