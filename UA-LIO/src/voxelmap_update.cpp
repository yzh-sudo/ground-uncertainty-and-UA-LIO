#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "ieskf_fusion/modules/map/voxelmap.hpp"
#include "pcl_conversions/pcl_conversions.h"

using namespace ieskf_fusion;

std::deque<State> state_buf;
std::deque<pcl::PointCloud<PointType>::Ptr> cloud_buf;

void odomCallback(const nav_msgs::OdometryConstPtr& msg) {
  ROS_INFO_STREAM("Received Odom, position: ("
                  << msg->pose.pose.position.x << ", "
                  << msg->pose.pose.position.y << ", "
                  << msg->pose.pose.position.z << ")");
  // TODO: 这里处理里程计数据
  std::cout << "odom  time: " << std::setprecision(20)
            << msg->header.stamp.toSec() << std::endl;
  State curr_state;
  Eigen::Quaterniond q;
  q.x() = msg->pose.pose.orientation.x;
  q.y() = msg->pose.pose.orientation.y;
  q.z() = msg->pose.pose.orientation.z;
  q.w() = msg->pose.pose.orientation.w;
  curr_state.rotation = q;
  curr_state.position.x() = msg->pose.pose.position.x;
  curr_state.position.y() = msg->pose.pose.position.y;
  curr_state.position.z() = msg->pose.pose.position.z;
  curr_state.offset_T_L_I = Eigen::Vector3d(-0.011, -0.02329, 0.04412);
  curr_state.offset_R_L_I = Eigen::Matrix3d::Identity();

  state_buf.push_back(curr_state);

  std::cout << "state buf size: " << state_buf.size() << std::endl;
}

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  ROS_INFO_STREAM("Received PointCloud2, width: " << msg->width << ", height: "
                                                  << msg->height);
  std::cout << "cloud time: " << std::setprecision(20)
            << msg->header.stamp.toSec() << std::endl;
  // TODO: 这里处理点云数据
  pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
  pcl::fromROSMsg(*msg, *cloud);

  cloud_buf.push_back(cloud);

  std::cout << "cloud buf size: " << cloud_buf.size() << std::endl;
}

void pubPointCloud(const ros::Publisher& pub, const State& state,
                   const pcl::PointCloud<PointType>::Ptr& cloud) {
  sensor_msgs::PointCloud2 pointcloud;
  pcl::toROSMsg(*cloud, pointcloud);
  pointcloud.header.stamp = ros::Time::now();
  pointcloud.header.frame_id = "odom";
  pub.publish(pointcloud);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "data_subscriber_node");
  ros::NodeHandle nh;

  ros::Subscriber sub_odom_ =
      nh.subscribe("/aft_mapped_to_init", 10, odomCallback);
  ros::Subscriber sub_pointcloud_ =
      nh.subscribe("/cloud_registered_surf", 10, pointCloudCallback);
  ros::Publisher voxel_plane_pub =
      nh.advertise<visualization_msgs::MarkerArray>("plane", 1000, true);
  ros::Publisher voxel_near_pub =
      nh.advertise<visualization_msgs::MarkerArray>("line", 1000, true);
  ros::Publisher voxel_point_pub =
      nh.advertise<visualization_msgs::MarkerArray>("voxel_point", 1000, true);
  ros::Publisher point_pub =
      nh.advertise<sensor_msgs::PointCloud2>("point_cloud", 1000, true);

  std::shared_ptr<VoxelMap> voxel_map_ptr;
  voxel_map_ptr = std::make_shared<VoxelMap>(0.2);
  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();

    if (state_buf.empty()) {
      continue;
    }
    if (cloud_buf.empty()) {
      continue;
    }

    pcl::PointCloud<PointType>::Ptr new_cloud = cloud_buf.front();
    cloud_buf.pop_front();
    State new_state;
    state_buf.pop_front();
    voxel_map_ptr->addScan(new_cloud, new_state);

    publishVoxelMap(voxel_plane_pub, voxel_map_ptr->getVoxelMap());
    publishNodeRelation(voxel_near_pub, voxel_map_ptr->getVoxelMap());
    pubVoxelPoints(voxel_point_pub, voxel_map_ptr->getVoxelMap());
    pubPointCloud(point_pub, new_state, new_cloud);
    rate.sleep();
  }

  return 0;
}