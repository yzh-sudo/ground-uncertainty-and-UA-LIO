#include "ieskf_fusion/modules/subscriber/lidar_subscriber.hpp"

namespace ieskf_fusion {
LidarSubscriber::LidarSubscriber(ros::NodeHandle& nh, std::string& topic_name,
                                 size_t buff_size, std::string& lidar_type)
    : node_(nh) {
  if (lidar_type == "pointcloud") {
    sub_ = node_.subscribe(topic_name, buff_size,
                           &LidarSubscriber::pointcloud_callback, this,
                           ros::TransportHints().tcpNoDelay());
  } else if (lidar_type == "mid360") {
    sub_ = node_.subscribe(topic_name, buff_size,
                           &LidarSubscriber::mid360_callback, this,
                           ros::TransportHints().tcpNoDelay());
  } else if (lidar_type == "ouster") {
    sub_ = node_.subscribe(topic_name, buff_size,
                           &LidarSubscriber::ouster_callback, this,
                           ros::TransportHints().tcpNoDelay());
  } else {
    std::cout << ">>>>>>>>>>>>>>>unknown lidar type<<<<<<<<<<<<<<<"
              << std::endl;
  }
}

void LidarSubscriber::ouster_callback(
    const sensor_msgs::PointCloud2::ConstPtr& lidar_msg_ptr) {
  std::lock_guard<std::mutex> lock(buff_mutex_);
  pcl::PointCloud<ousterPoint>::Ptr original_scan(
      boost::make_shared<pcl::PointCloud<ousterPoint>>());
  pcl::fromROSMsg(*lidar_msg_ptr, *original_scan);

  PointCloud cloud_data;
  cloud_data.time_stamp = lidar_msg_ptr->header.stamp.toSec();
  int point_nums = original_scan->size();
  for (int i = 0; i < point_nums; i++) {
    PointType added_pt;
    added_pt.x = original_scan->points[i].x;
    added_pt.y = original_scan->points[i].y;
    added_pt.z = original_scan->points[i].z;
    if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z +
            added_pt.z <
        blind * blind) {
      continue;
    }
    added_pt.intensity = original_scan->points[i].intensity;
    added_pt.timestamp =
        original_scan->points[i].t * 1.e-6f;  // timestamp unit: ms

    cloud_data.cloud_ptr->points.push_back(added_pt);
  }
  original_scan.reset();
  cloud_data.scan_time = cloud_data.cloud_ptr->points.back().timestamp / 1000;

  new_lidar_data_buff_.push_back(cloud_data);
}

void LidarSubscriber::pointcloud_callback(
    const sensor_msgs::PointCloud2::ConstPtr& lidar_msg_ptr) {
  std::lock_guard<std::mutex> lock(buff_mutex_);
  pcl::PointCloud<PointType>::Ptr original_scan(
      boost::make_shared<pcl::PointCloud<PointType>>());
  pcl::fromROSMsg(*lidar_msg_ptr, *original_scan);

  PointCloud cloud_data;
  cloud_data.time_stamp = lidar_msg_ptr->header.stamp.toSec();
  cloud_data.cloud_ptr = original_scan;
  original_scan.reset();

  new_lidar_data_buff_.push_back(cloud_data);
}

void LidarSubscriber::mid360_callback(
    const livox_ros_driver::CustomMsg::ConstPtr& lidar_msg_ptr) {
  std::lock_guard<std::mutex> lock(buff_mutex_);
  PointCloud cloud_data;
  cloud_data.time_stamp = lidar_msg_ptr->header.stamp.toSec();
  int point_nums = lidar_msg_ptr->point_num;
  for (uint i = 1; i < point_nums; i++) {
    PointType point;
    point.x = lidar_msg_ptr->points[i].x;
    point.y = lidar_msg_ptr->points[i].y;
    point.z = lidar_msg_ptr->points[i].z;
    if (point.x * point.x + point.y * point.y + point.z + point.z <
        blind * blind) {
      continue;
    }
    point.intensity = lidar_msg_ptr->points[i].reflectivity;
    point.timestamp = lidar_msg_ptr->points[i].offset_time / float(1000000);

    cloud_data.cloud_ptr->points.push_back(point);
  }

  cloud_data.scan_time = cloud_data.cloud_ptr->points.back().timestamp / 1000;

  new_lidar_data_buff_.push_back(cloud_data);
}

bool LidarSubscriber::AddLidarData(std::deque<PointCloud>& lidar_data_buff) {
  std::lock_guard<std::mutex> lock(buff_mutex_);
  if (!new_lidar_data_buff_.empty()) {
    lidar_data_buff.insert(lidar_data_buff.end(), new_lidar_data_buff_.begin(),
                           new_lidar_data_buff_.end());
    new_lidar_data_buff_.clear();
    return true;
  }
  return false;
}
}  // namespace ieskf_fusion
