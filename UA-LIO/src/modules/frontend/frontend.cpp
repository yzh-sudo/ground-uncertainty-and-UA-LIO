#include "ieskf_fusion/modules/frontend/frontend.hpp"

#include <memory>

namespace ieskf_fusion {
FrontEnd::FrontEnd(ros::NodeHandle& nh) {
  // offset params set
  std::vector<double> offset_lidar_to_imu_T(3, 0.0);
  std::vector<double> offset_lidar_to_imu_R(9, 0.0);
  nh.param<std::vector<double>>("front_end/offset_lidar_to_imu_T",
                                offset_lidar_to_imu_T, std::vector<double>());
  nh.param<std::vector<double>>("front_end/offset_lidar_to_imu_R",
                                offset_lidar_to_imu_R, std::vector<double>());
  offset_lidar_to_imu.block<3, 1>(0, 3) =
      Eigen::Vector3d(offset_lidar_to_imu_T[0], offset_lidar_to_imu_T[1],
                      offset_lidar_to_imu_T[2]);
  offset_lidar_to_imu.block<3, 3>(0, 0) << offset_lidar_to_imu_R[0],
      offset_lidar_to_imu_R[1], offset_lidar_to_imu_R[2],
      offset_lidar_to_imu_R[3], offset_lidar_to_imu_R[4],
      offset_lidar_to_imu_R[5], offset_lidar_to_imu_R[6],
      offset_lidar_to_imu_R[7], offset_lidar_to_imu_R[8];
  std::cout << "set lidar imu External reference \n"
            << offset_lidar_to_imu << std::endl;

  // IESKF params
  double thresh;
  double cov_gyroscope;
  double cov_acceleration;
  double cov_bias_acceleration;
  double cov_bias_gyroscope;
  nh.param<double>("IESKF/thresh", thresh, 0.01);
  nh.param<double>("IESKF/cov_gyroscope", cov_gyroscope, 0.01);
  nh.param<double>("IESKF/cov_acceleration", cov_acceleration, 0.01);
  nh.param<double>("IESKF/cov_bias_acceleration", cov_bias_acceleration, 0.01);
  nh.param<double>("IESKF/cov_bias_gyroscope", cov_bias_gyroscope, 0.01);
  ieskf_ptr_ =
      std::make_shared<IESKF>(thresh, cov_gyroscope, cov_acceleration,
                              cov_bias_acceleration, cov_bias_gyroscope);

  propagate_ptr_ = std::make_shared<Propagate>();

  // map params
  double local_map_resolution;
  nh.param<double>("map_manager/local_map_resolution", local_map_resolution,
                   0.01);
  map_ptr_ = std::make_shared<MapManager>(local_map_resolution);
  voxel_map_ptr_ = std::make_shared<VoxelMap>(local_map_resolution);
  voxel_plane_pub_ =
      nh.advertise<visualization_msgs::MarkerArray>("plane", 1000, true);
  voxel_near_pub_ =
      nh.advertise<visualization_msgs::MarkerArray>("line", 1000, true);
  voxel_point_pub_ =
      nh.advertise<visualization_msgs::MarkerArray>("voxel_point", 1000, true);
  text_pub = nh.advertise<visualization_msgs::Marker>("voxel_text", 1000);
  near_text_pub =
      nh.advertise<visualization_msgs::Marker>("voxel_near_text", 1000);
  server = std::make_shared<interactive_markers::InteractiveMarkerServer>(
      "voxel_map_markers");

  // observation_model params
  double filter_size;
  std::string model_type;
  nh.param<std::string>("observationModel/model_type", model_type, "Po2Pl");
  nh.param<double>("observationModel/filter_size", filter_size, 0.1);
  obs_model = std::make_shared<Po2PlModel>(filter_size);
  voxel_model = std::make_shared<VoxelMapModel>(filter_size);

  // other todo
  std::string predict_odom_topic = "predict_odom";
  std::string predict_odom_frame_id = "odom";
  std::string predict_odom_child_frame_id = "base_footprint";
  odom_pub_ptr = std::make_shared<OdometryPublisher>(
      nh, predict_odom_topic, predict_odom_frame_id,
      predict_odom_child_frame_id, 10);

  voxel_filter.setLeafSize(filter_size, filter_size, filter_size);

  csv_file_name = "/home/myh/ieskf_ws/log/all_points.csv";

  const std::string binlog_file =
      "/home/myh/data_log_tool/log_data/plane_judge/plane_record.binlog";
  // CreateLog(binlog_file);
}

void FrontEnd::write_planes_to_csv(
    const std::string& filename,
    const std::unordered_map<VOXEL_LOC, UnionFind*>& voxel_map) {
  std::ofstream file;
  // 以追加模式写文件
  file.open(filename, std::ios::app);
  if (!file.is_open()) {
    std::cerr << "Error: cannot open " << filename << std::endl;
    return;
  }

  // 如果是新文件，写表头
  if (file.tellp() == 0) {
    file << "angel,d,n_cov_trace,q_cov_trace\n";
  }

  file << std::fixed << std::setprecision(6);

  for (const auto& iter : voxel_map) {
    UnionFind* curr_node = iter.second;
    if (curr_node->plane_ptr->plane_id == 124) {
      UnionFind* root = curr_node->findRoot(curr_node);

      Eigen::Vector3d nn1(0, 0, 1);
      Eigen::Vector3d nn2 = root->plane_ptr->normal.normalized();
      if (nn1.dot(nn2) < 0) {
        nn1 = -nn1;
      }

      // 1. 法向量夹角
      double cos_angle = std::abs(nn1.dot(nn2));
      double angle =
          std::acos(std::min(1.0, std::max(-1.0, cos_angle)));  // 数值安全
      file << angle * 180 / M_PI << "," << root->plane_ptr->d << ","
           << root->plane_ptr->plane_cov.block<3, 3>(0, 0).trace() << ","
           << root->plane_ptr->plane_cov.block<3, 3>(3, 3).trace() << "\n";
    }
  }
}

bool FrontEnd::track(MeasureGroup& measure_group) {
  scan_idx++;
  if (!imu_inited) {
    initState(measure_group);
    std::cout << "imu init finish!!!!!" << std::endl;
    return false;
  }
  if (flg_first_scan && imu_inited) {
    // map_ptr_->reset();
    // map_ptr_->addScan(measure_group.cloud.cloud_ptr, ieskf_ptr_->getX());
    // obs_model->setMap(map_ptr_->getLocalMap(), map_ptr_->readKDtree());
    voxel_model->setMap(voxel_map_ptr_->getVoxelMap());
    voxel_map_ptr_->reset();
    voxel_map_ptr_->addScan(measure_group.cloud.cloud_ptr, ieskf_ptr_->getX());
    flg_first_scan = false;
    std::cout << "get first scan!!!!!" << std::endl;
    return false;
  }

  pcl::PointCloud<PointType> origin_point;
  origin_point.points = measure_group.cloud.cloud_ptr->points;
  propagate_ptr_->Propagation(measure_group, ieskf_ptr_, odom_pub_ptr);
  voxel_filter.setInputCloud(measure_group.cloud.cloud_ptr);
  voxel_filter.filter(curr_scan);

  // curr_scan.points = measure_group.cloud.cloud_ptr->points;
  // obs_model->setCurrentPointCloud(measure_group.cloud.cloud_ptr);
  // ieskf_ptr_->update(obs_model);
  voxel_model->setCurrentPointCloud(measure_group.cloud.cloud_ptr);

  {
    auto t1 = std::chrono::high_resolution_clock::now();
    ieskf_ptr_->update(voxel_model);
    auto t2 = std::chrono::high_resolution_clock::now();
    double duration_ms =
        std::chrono::duration<double, std::milli>(t2 - t1).count();
    std::cout << "[Timing] ieskf update耗时: " << duration_ms << " ms"
              << std::endl;
  }

  // map_ptr_->addScan(measure_group.cloud.cloud_ptr, ieskf_ptr_->getX());
  {
    auto t1 = std::chrono::high_resolution_clock::now();
    voxel_map_ptr_->addScan(measure_group.cloud.cloud_ptr, ieskf_ptr_->getX());
    auto t2 = std::chrono::high_resolution_clock::now();
    double duration_ms =
        std::chrono::duration<double, std::milli>(t2 - t1).count();
    std::cout << "[Timing] voxel_map addScan耗时: " << duration_ms << " ms"
              << std::endl;
  }

  // if ((scan_idx % 10) == 0) {
  //   publishVoxelMap(voxel_plane_pub_, voxel_map_ptr_->getVoxelMap());
  //   publishNodeRelation(voxel_near_pub_, voxel_map_ptr_->getVoxelMap());
  //   pubVoxelPoints(voxel_point_pub_, voxel_map_ptr_->getVoxelMap());
  //   publishVoxelMarkers(text_pub, near_text_pub, server,
  //                       voxel_map_ptr_->getVoxelMap());
  // }

  return true;
}

void FrontEnd::initState(MeasureGroup& measure_group) {
  static int imu_count = 0;
  static Eigen::Vector3d mean_acc{0, 0, 0};
  static Eigen::Vector3d mean_bg{0, 0, 0};
  auto& ieskf = *ieskf_ptr_;
  if (imu_inited) {
    return;
  }
  for (auto& imu : measure_group.imus) {
    imu_count++;
    auto x = ieskf.getX();
    mean_acc += imu.acceleration;
    mean_bg += imu.gyroscope;
  }
  if (imu_count >= 5) {
    auto x = ieskf.getX();
    mean_acc /= double(imu_count);
    mean_bg /= double(imu_count);

    double imu_scale = GRAVITY / mean_acc.norm();
    propagate_ptr_->imu_scale = imu_scale;
    propagate_ptr_->last_imu = measure_group.imus.back();

    Eigen::Vector3d g_b = mean_acc.normalized();
    Eigen::Vector3d g_w(0, 0, 1.0);
    Eigen::Quaterniond q_wb = Eigen::Quaterniond::FromTwoVectors(g_b, g_w);

    x.gravity = -g_w * GRAVITY;
    x.rotation = q_wb;
    x.bg = mean_bg;
    x.offset_R_L_I = Eigen::Quaterniond(offset_lidar_to_imu.block<3, 3>(0, 0));
    x.offset_T_L_I = offset_lidar_to_imu.block<3, 1>(0, 3);
    ieskf.setX(x);
    imu_inited = true;

    std::cout << "state init finish!!!!!!\nbg : " << x.bg.transpose()
              << "\ngravity : " << x.gravity.transpose()
              << "\nimu scale: " << imu_scale << std::endl;
  }
}

pcl::PointCloud<PointType>::ConstPtr FrontEnd::getLocalMap() {
  return map_ptr_->getLocalMap();
}

pcl::PointCloud<PointType>::ConstPtr FrontEnd::getMap() {
  return map_ptr_->getMap();
}

pcl::PointCloud<PointType>::Ptr FrontEnd::getDistortCloud() {
  pcl::PointCloud<PointType>::Ptr distort_cloud_ptr(
      new pcl::PointCloud<PointType>(curr_scan));
  return distort_cloud_ptr;
}

Eigen::Matrix4f FrontEnd::getPose() {
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  auto state = ieskf_ptr_->getX();
  pose.block<3, 3>(0, 0) = state.rotation.toRotationMatrix().cast<float>();
  pose.block<3, 1>(0, 3) = state.position.cast<float>();

  return pose;
}

}  // namespace ieskf_fusion