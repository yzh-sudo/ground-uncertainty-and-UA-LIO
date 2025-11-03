#include "ieskf_fusion/ieskf_fusion_flow.hpp"

namespace ieskf_fusion {
IESKFFusionFlow::IESKFFusionFlow(ros::NodeHandle& nh) : node_(nh) {
  // node params
  std::string lidar_topic;
  std::string lidar_type;
  std::string imu_topic;
  node_.param<std::string>("ieskf_fusion_node/lidar_type", lidar_type,
                           "mid360");
  node_.param<std::string>("ieskf_fusion_node/lidar_topic", lidar_topic,
                           "/livox/lidar");
  node_.param<std::string>("ieskf_fusion_node/imu_topic", imu_topic,
                           "/livox/imu");
  lidar_sub_ptr_ =
      std::make_shared<LidarSubscriber>(node_, lidar_topic, 1000, lidar_type);
  imu_sub_ptr_ = std::make_shared<IMUSubscriber>(node_, imu_topic, 1000);

  front_end_ptr_ = std::make_shared<FrontEnd>(node_);

  std::string local_map_topic = "local_map";
  std::string map_frame_id = "odom";
  local_map_pub_ptr = std::make_shared<PointCloudPublisher>(
      node_, local_map_topic, map_frame_id, 1000);

  std::string map_topic = "map";
  map_pub_ptr = std::make_shared<PointCloudPublisher>(node_, map_topic,
                                                      map_frame_id, 1000);
  std::string current_scan_topic = "curr_scan";
  std::string current_scan_frame_id = "base_footprint";
  scan_pub_ptr = std::make_shared<PointCloudPublisher>(
      node_, current_scan_topic, current_scan_frame_id, 1000);

  std::string odom_topic = "ieskf_odom";
  std::string odom_frame_id = "odom";
  std::string odom_child_frame_id = "base_footprint";
  odom_pub_ptr = std::make_shared<OdometryPublisher>(
      nh, odom_topic, odom_frame_id, odom_child_frame_id, 10);

  tf_pub_ptr =
      std::make_shared<TFBroadCaster>(odom_frame_id, odom_child_frame_id);

  std::string path_topic = "path";
  path_pub_ptr =
      std::make_shared<PathPublisher>(nh, path_topic, odom_frame_id, 10);

  processing_thread_ = std::thread(&IESKFFusionFlow::process, this);
}

void IESKFFusionFlow::process() {
  while (ros::ok()) {
    MeasureGroup measure_group;

    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      cond_var_.wait(lock, [this] {
        return !measure_group_data_buff_.empty() || stop_flag_;
      });

      if (stop_flag_ && measure_group_data_buff_.empty()) {
        break;
      }

      measure_group = measure_group_data_buff_.front();
      measure_group_data_buff_.pop_front();
    }

    lidar_timestamp = measure_group.lidar_end_time;
    if (front_end_ptr_->track(measure_group)) {
      publishMsg();
    }
  }
}

void IESKFFusionFlow::run() {
  ros::Rate rate(500);
  while (ros::ok()) {
    ros::spinOnce();
    if (getSensorData() || !imu_data_buff_.empty() ||
        !lidar_data_buff_.empty()) {
      MeasureGroup measure_group;
      if (syncMeasureGroup(measure_group)) {
        {
          std::lock_guard<std::mutex> lock(queue_mutex_);
          measure_group_data_buff_.push_back(measure_group);
        }
        cond_var_.notify_one();
      }
    }
    rate.sleep();
  }
}

bool IESKFFusionFlow::syncMeasureGroup(MeasureGroup& measure_group) {
  measure_group.imus.clear();
  measure_group.cloud.cloud_ptr->clear();

  //点云数据队列/IMU数据队列为空就返回
  if (lidar_data_buff_.empty() || imu_data_buff_.empty()) {
    return false;
  }

  double imu_start_time = imu_data_buff_.front().time_stamp;
  double imu_end_time = imu_data_buff_.back().time_stamp;
  double laser_start_time = lidar_data_buff_.front().time_stamp;
  double laser_end_time = laser_start_time + lidar_data_buff_.front().scan_time;

  //若IMU队列结束时间小于雷达结束时间，说明IMU数据还不足
  if (imu_end_time < laser_end_time) {
    return false;
  }

  //若雷达结束时间，说明雷达数据滞后，将雷达数据弹出
  if (laser_end_time < imu_start_time) {
    lidar_data_buff_.pop_front();
    return false;
  }

  measure_group.cloud = lidar_data_buff_.front();

  lidar_data_buff_.pop_front();
  measure_group.lidar_beg_time = laser_start_time;
  measure_group.lidar_end_time = laser_end_time;
  while (!imu_data_buff_.empty()) {
    if (imu_data_buff_.front().time_stamp < measure_group.lidar_end_time) {
      measure_group.imus.push_back(imu_data_buff_.front());
      imu_data_buff_.pop_front();
    } else {
      break;
    }
  }

  return measure_group.imus.size() > 3;
}

bool IESKFFusionFlow::getSensorData() {
  return imu_sub_ptr_->AddIMUData(imu_data_buff_) &&
         lidar_sub_ptr_->AddLidarData(lidar_data_buff_);
}

void IESKFFusionFlow::publishMsg() {
  pcl::PointCloud<PointType>::ConstPtr local_map_cloud =
      front_end_ptr_->getLocalMap();
  local_map_pub_ptr->Publish(local_map_cloud, lidar_timestamp);

  // pcl::PointCloud<PointType>::ConstPtr map_cloud = front_end_ptr_->getMap();
  // map_pub_ptr->Publish(map_cloud, lidar_timestamp);

  pcl::PointCloud<PointType>::ConstPtr distort_cloud =
      front_end_ptr_->getDistortCloud();
  scan_pub_ptr->Publish(distort_cloud, lidar_timestamp);

  Eigen::Matrix4f pose = front_end_ptr_->getPose();
  odom_pub_ptr->Publish(pose, lidar_timestamp);
  tf_pub_ptr->SendTransform(pose, lidar_timestamp);
  path_pub_ptr->Publish(pose, lidar_timestamp);
}

}  // namespace ieskf_fusion