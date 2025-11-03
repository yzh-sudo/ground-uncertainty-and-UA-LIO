#ifndef IESKF_FUSION_VOXEL_UTIL_HPP
#define IESKF_FUSION_VOXEL_UTIL_HPP

#include <interactive_markers/interactive_marker_server.h>
#include <pcl/pcl_macros.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>
#include <random>

#include "ieskf_fusion/modules/map/unionfind.hpp"
#include "ieskf_fusion/modules/map/voxel_feature.hpp"
#include "ieskf_fusion/modules/map/voxel_loc.hpp"
#include "ros/publisher.h"

namespace ieskf_fusion {

inline void calcBodyCov(Eigen::Vector3d& pb, const float range_inc,
                        const float degree_inc, Eigen::Matrix3d& cov) {
  float range = sqrt(pb[0] * pb[0] + pb[1] * pb[1] + pb[2] * pb[2]);
  float range_var = range_inc * range_inc;
  Eigen::Matrix2d direction_var;
  direction_var << pow(sin(DEG2RAD(degree_inc)), 2), 0, 0,
      pow(sin(DEG2RAD(degree_inc)), 2);
  Eigen::Vector3d direction(pb);
  if (direction(2) == 0) {
    direction(2) = 1e-6;
  }
  direction.normalize();
  Eigen::Matrix3d direction_hat;
  direction_hat << 0, -direction(2), direction(1), direction(2), 0,
      -direction(0), -direction(1), direction(0), 0;
  Eigen::Vector3d base_vector1(1, 1,
                               -(direction(0) + direction(1)) / direction(2));
  base_vector1.normalize();
  Eigen::Vector3d base_vector2 = base_vector1.cross(direction);
  base_vector2.normalize();
  Eigen::Matrix<double, 3, 2> N;
  N << base_vector1(0), base_vector2(0), base_vector1(1), base_vector2(1),
      base_vector1(2), base_vector2(2);
  Eigen::Matrix<double, 3, 2> A = range * direction_hat * N;
  cov = direction * range_var * direction.transpose() +
        A * direction_var * A.transpose();
}

inline void CalcVectQuation(const Eigen::Vector3d& x_vec,
                            const Eigen::Vector3d& y_vec,
                            const Eigen::Vector3d& z_vec,
                            geometry_msgs::Quaternion& q_out) {
  // 1) 三个方向归一化
  Eigen::Vector3d x = x_vec.normalized();
  Eigen::Vector3d y = y_vec.normalized();
  Eigen::Vector3d z = z_vec.normalized();

  if (!x.allFinite() || !y.allFinite() || !z.allFinite()) {
    return;  // 有非法数
  }

  // 2) 重新正交化，保证三轴正交
  //    优先保留 z 作为法向
  z.normalize();
  x = x - z * (z.dot(x));  // 投影到平面
  if (x.norm() < 1e-8) {
    // 如果退化，随便挑个基准轴
    x = (std::abs(z.z()) < 0.9 ? Eigen::Vector3d::UnitZ()
                               : Eigen::Vector3d::UnitY());
    x = x - z * (z.dot(x));
  }
  x.normalize();
  y = z.cross(x);  // 保证右手系
  y.normalize();

  // 3) 构造旋转矩阵（按列装填）
  Eigen::Matrix3d R;
  R.col(0) = x;
  R.col(1) = y;
  R.col(2) = z;

  // 4) 检查行列式是否为 +1（右手系）
  if (R.determinant() < 0) {
    R.col(1) = -R.col(1);  // 翻转 y，修正为右手系
  }

  // 5) 转四元数
  Eigen::Quaterniond q(R);
  q.normalize();

  q_out.w = q.w();
  q_out.x = q.x();
  q_out.y = q.y();
  q_out.z = q.z();
}

inline void pubSinglePlane(visualization_msgs::MarkerArray& plane_pub,
                           const std::string& plane_ns,
                           const Plane& single_plane, const float alpha,
                           const Eigen::Vector3d& rgb, int id) {
  visualization_msgs::Marker plane;
  plane.header.frame_id = "odom";
  plane.header.stamp = ros::Time();
  plane.ns = plane_ns;
  plane.id = id;
  if (single_plane.is_merged) {
    plane.type = visualization_msgs::Marker::CUBE;
  } else {
    plane.type = visualization_msgs::Marker::CYLINDER;
  }
  plane.action = visualization_msgs::Marker::ADD;
  plane.pose.position.x = single_plane.center[0];
  plane.pose.position.y = single_plane.center[1];
  plane.pose.position.z = single_plane.center[2];
  geometry_msgs::Quaternion q;
  CalcVectQuation(single_plane.x_normal, single_plane.y_normal,
                  single_plane.normal, q);
  plane.pose.orientation = q;
  plane.scale.x = 3 * sqrt(abs(single_plane.max_eigen_value)) == 0
                      ? 0.01
                      : 3 * sqrt(abs(single_plane.max_eigen_value));
  plane.scale.y = 3 * sqrt(abs(single_plane.mid_eigen_value)) == 0
                      ? 0.01
                      : 3 * sqrt(abs(single_plane.mid_eigen_value));
  plane.scale.z = 2 * sqrt(abs(single_plane.min_eigen_value)) == 0
                      ? 0.01
                      : 2 * sqrt(abs(single_plane.min_eigen_value));
  plane.color.a = alpha;
  plane.color.r = rgb(0);
  plane.color.g = rgb(1);
  plane.color.b = rgb(2);
  plane.lifetime = ros::Duration();
  plane_pub.markers.push_back(plane);
}

inline void mapJet(double v, double vmin, double vmax, uint8_t& r, uint8_t& g,
                   uint8_t& b) {
  r = 255;
  g = 255;
  b = 255;

  if (v < vmin) {
    v = vmin;
  }

  if (v > vmax) {
    v = vmax;
  }

  double dr, dg, db;

  if (v < 0.1242) {
    db = 0.504 + ((1. - 0.504) / 0.1242) * v;
    dg = dr = 0.;
  } else if (v < 0.3747) {
    db = 1.;
    dr = 0.;
    dg = (v - 0.1242) * (1. / (0.3747 - 0.1242));
  } else if (v < 0.6253) {
    db = (0.6253 - v) * (1. / (0.6253 - 0.3747));
    dg = 1.;
    dr = (v - 0.3747) * (1. / (0.6253 - 0.3747));
  } else if (v < 0.8758) {
    db = 0.;
    dr = 1.;
    dg = (0.8758 - v) * (1. / (0.8758 - 0.6253));
  } else {
    db = 0.;
    dg = 0.;
    dr = 1. - (v - 0.8758) * ((1. - 0.504) / (1. - 0.8758));
  }

  r = (uint8_t)(255 * dr);
  g = (uint8_t)(255 * dg);
  b = (uint8_t)(255 * db);
}

inline void publishVoxelMap(
    const ros::Publisher& plane_pub,
    const std::unordered_map<VOXEL_LOC, UnionFind*>& voxel_map) {
  float use_alpha = 1;
  double max_trace = 0.25;
  double pow_num = 0.2;
  visualization_msgs::MarkerArray voxel_plane;
  voxel_plane.markers.reserve(1000000);

  for (const auto& iter : voxel_map) {
    UnionFind* curr_node = iter.second;
    if (curr_node->update_enable) {
      continue;
    }
    if (!curr_node->plane_ptr->is_plane) {
      continue;
    }
    // if (curr_node->plane_ptr->plane_id == 124) {
    //   UnionFind* root = curr_node->findRoot(curr_node);

    //   Eigen::Vector3d nn1(0, 0, 1);
    //   Eigen::Vector3d nn2 = root->plane_ptr->normal.normalized();
    //   if (nn1.dot(nn2) < 0) {
    //     nn1 = -nn1;
    //   }

    //   // 1. 法向量夹角
    //   double cos_angle = std::abs(nn1.dot(nn2));
    //   double angle =
    //       std::acos(std::min(1.0, std::max(-1.0, cos_angle)));  // 数值安全
    //   // std::cout << "plane id " << root->plane_ptr->plane_id << " center "
    //   //           << root->plane_ptr->center.transpose() << " angel: " <<
    //   angle
    //   //           << std::endl;
    // }

    UnionFind* root_node = curr_node->findRoot(curr_node);
    // 根据点云不确定性着色
    Eigen::Vector3d plane_cov =
        root_node->plane_ptr->plane_cov.block<3, 3>(0, 0).diagonal();
    double trace = plane_cov.sum();
    if (trace >= max_trace) {
      trace = max_trace;
    }
    trace = trace * (1.0 / max_trace);
    trace = pow(trace, pow_num);
    uint8_t r, g, b;
    mapJet(trace, 0, 1, r, g, b);
    Eigen::Vector3d plane_rgb(r / 256.0, g / 256.0, b / 256.0);

    float alpha = use_alpha;

    if (root_node == curr_node) {
      Plane pub_plane = *curr_node->plane_ptr;
      pub_plane.center[0] = curr_node->voxel_center[0];
      pub_plane.center[1] = curr_node->voxel_center[1];
      pub_plane.center[2] = curr_node->voxel_center[2];
      pubSinglePlane(voxel_plane, "plane", pub_plane, alpha, plane_rgb,
                     curr_node->plane_ptr->plane_id);
    } else {
      Plane pub_plane = *root_node->plane_ptr;
      pub_plane.center[0] = curr_node->voxel_center[0];
      pub_plane.center[1] = curr_node->voxel_center[1];
      pub_plane.center[2] = curr_node->voxel_center[2];

      pubSinglePlane(voxel_plane, "plane", pub_plane, alpha, plane_rgb,
                     curr_node->plane_ptr->plane_id);
    }
  }
  // std::cout << "voxel_plane size:" << voxel_plane.markers.size() <<
  // std::endl;
  plane_pub.publish(voxel_plane);
}

inline void pubVoxelPoints(
    const ros::Publisher& point_pub,
    const std::unordered_map<VOXEL_LOC, UnionFind*>& voxel_map) {
  visualization_msgs::MarkerArray marker_array;
  int id_counter = 0;

  std::mt19937 gen(std::random_device{}());
  std::uniform_real_distribution<> dis(0.0, 1.0);
  for (const auto& iter : voxel_map) {
    UnionFind* curr_node = iter.second;
    if (curr_node->update_enable) {
      continue;
    }

    // 一个体素内的点放到一个 Marker 里
    visualization_msgs::Marker points_marker;
    points_marker.header.frame_id = "odom";
    points_marker.header.stamp = ros::Time();
    points_marker.ns = "voxel_points";
    points_marker.id = id_counter++;
    points_marker.type = visualization_msgs::Marker::POINTS;
    points_marker.action = visualization_msgs::Marker::ADD;

    points_marker.scale.x = 0.02;  // 点的宽度
    points_marker.scale.y = 0.02;  // 点的高度
    points_marker.color.a = 1.0;
    points_marker.color.r = dis(gen);
    points_marker.color.g = dis(gen);
    points_marker.color.b = dis(gen);

    // 加入体素内的点
    for (const auto& point_with_cov : curr_node->all_points) {
      geometry_msgs::Point p;
      p.x = point_with_cov.world_point.x();
      p.y = point_with_cov.world_point.y();
      p.z = point_with_cov.world_point.z();
      points_marker.points.push_back(p);
    }

    // 避免空点集合报错
    if (!points_marker.points.empty()) {
      marker_array.markers.push_back(points_marker);
    }
  }

  point_pub.publish(marker_array);
}

inline void publishNodeRelation(
    const ros::Publisher& line_pub,
    const std::unordered_map<VOXEL_LOC, UnionFind*>& voxel_map) {
  visualization_msgs::MarkerArray marker_array;
  int id_counter = 0;

  for (auto& kv : voxel_map) {
    UnionFind* node = kv.second;

    if (node == node->rootNode) {
      continue;  // 跳过自己是根的
    }
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = "odom";
    arrow.header.stamp = ros::Time();
    arrow.ns = "merge_arrows";
    arrow.id = id_counter++;
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;
    arrow.scale.x = 0.02;  // 线宽
    arrow.scale.y = 0.04;
    arrow.scale.z = 0.1;
    arrow.color.r = 0.0;
    arrow.color.g = 1.0;
    arrow.color.b = 0.0;
    arrow.color.a = 1.0;

    geometry_msgs::Point p1, p2;
    p1.x = kv.second->voxel_center[0];
    p1.y = kv.second->voxel_center[1];
    p1.z = kv.second->voxel_center[2];

    p2.x = node->rootNode->voxel_center[0];
    p2.y = node->rootNode->voxel_center[1];
    p2.z = node->rootNode->voxel_center[2];

    arrow.points.push_back(p1);
    arrow.points.push_back(p2);
    marker_array.markers.push_back(arrow);
  }

  line_pub.publish(marker_array);
}

inline void publishVoxelTextMarker(const ros::Publisher& text_pub,
                                   UnionFind* info) {
  visualization_msgs::Marker text_marker;
  text_marker.header.frame_id = "odom";
  text_marker.header.stamp = ros::Time::now();
  text_marker.ns = "voxel_text";
  text_marker.id = 0;  // 固定为0，保证只显示一个，新的会覆盖旧的
  text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text_marker.action = visualization_msgs::Marker::ADD;
  text_marker.pose.position.x = info->voxel_center[0];
  text_marker.pose.position.y = info->voxel_center[1];
  text_marker.pose.position.z = info->voxel_center[2];
  text_marker.scale.z = 0.2;
  text_marker.color.r = 1.0f;
  text_marker.color.g = 1.0f;
  text_marker.color.b = 1.0f;
  text_marker.color.a = 1.0f;

  UnionFind* root_node = info->findRoot(info);

  std::stringstream ss;
  ss << "Voxel: " << info->plane_ptr->plane_id
     << "\nMean: " << root_node->plane_ptr->center.transpose()
     << "\nIsPlane: " << (info->plane_ptr->is_plane ? "Yes" : "No");
  if (info->plane_ptr->is_plane) {
    ss << "\nNormal: " << root_node->plane_ptr->normal.transpose()
       << "\nRoot: " << root_node->plane_ptr->plane_id;
  }

  text_marker.text = ss.str();

  text_pub.publish(text_marker);
}

inline double angleBetween(const Eigen::Vector3d& v1,
                           const Eigen::Vector3d& v2) {
  Eigen::Vector3d n2 = v2;
  if (v1.dot(v2) < 0) {
    n2 = -v2;
  }
  double cos_theta = v1.normalized().dot(n2.normalized());
  cos_theta = std::min(1.0, std::max(-1.0, cos_theta));  // 避免数值误差
  return acos(cos_theta) * 180.0 / M_PI;
}

inline void publishVoxelNearTextMarker(const ros::Publisher& text_pub,
                                       UnionFind* curr_node,
                                       UnionFind* near_node) {
  visualization_msgs::Marker text_marker;
  text_marker.header.frame_id = "odom";
  text_marker.header.stamp = ros::Time::now();
  text_marker.ns = "voxel_near_text";
  text_marker.id = 0;  // 固定为0，保证只显示一个，新的会覆盖旧的
  text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text_marker.action = visualization_msgs::Marker::ADD;
  text_marker.pose.position.x = near_node->voxel_center[0];
  text_marker.pose.position.y = near_node->voxel_center[1];
  text_marker.pose.position.z = near_node->voxel_center[2];
  text_marker.scale.z = 0.2;
  text_marker.color.r = 1.0f;
  text_marker.color.g = 1.0f;
  text_marker.color.b = 1.0f;
  text_marker.color.a = 1.0f;

  UnionFind* curr_root_node = curr_node->findRoot(curr_node);
  UnionFind* near_root_node = near_node->findRoot(near_node);

  std::stringstream ss;
  ss << "Voxel_1: " << curr_node->plane_ptr->plane_id
     << "\nMean: " << curr_root_node->plane_ptr->center.transpose()
     << "\nVoxel_2: " << near_node->plane_ptr->plane_id
     << "\nMean: " << near_root_node->plane_ptr->center.transpose();
  if (curr_node->plane_ptr->is_plane && near_node->plane_ptr->is_plane) {
    ss << "\nnormal angel: "
       << angleBetween(curr_root_node->plane_ptr->normal,
                       near_root_node->plane_ptr->normal)
       << "\nplane distance: "
       << curr_root_node->plane_ptr->normal.dot(
              near_root_node->plane_ptr->center -
              curr_root_node->plane_ptr->center);
  }

  text_marker.text = ss.str();

  text_pub.publish(text_marker);
}

inline void publishVoxelMarkers(
    const ros::Publisher& text_pub, const ros::Publisher& near_text_pub,
    const std::shared_ptr<interactive_markers::InteractiveMarkerServer>& server,
    const std::unordered_map<VOXEL_LOC, UnionFind*>& voxel_map) {
  double max_trace = 0.25;
  double pow_num = 0.2;

  for (auto const& iter : voxel_map) {
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "odom";
    int_marker.name = std::to_string(iter.first.x) + "_" +
                      std::to_string(iter.first.y) + "_" +
                      std::to_string(iter.first.z);
    int_marker.description = "";
    int_marker.scale = 0.5;
    int_marker.pose.position.x = iter.second->voxel_center[0];
    int_marker.pose.position.y = iter.second->voxel_center[1];
    int_marker.pose.position.z = iter.second->voxel_center[2];

    visualization_msgs::InteractiveMarkerControl control;
    control.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::BUTTON;
    control.always_visible = true;

    UnionFind* root_node = iter.second->findRoot(iter.second);
    // 根据点云不确定性着色
    Eigen::Vector3d plane_cov =
        root_node->plane_ptr->plane_cov.block<3, 3>(0, 0).diagonal();
    double trace = plane_cov.sum();
    if (trace >= max_trace) {
      trace = max_trace;
    }
    trace = trace * (1.0 / max_trace);
    trace = pow(trace, pow_num);
    uint8_t r, g, b;
    mapJet(trace, 0, 1, r, g, b);
    Eigen::Vector3d plane_rgb(r / 256.0, g / 256.0, b / 256.0);

    // 用 cube 表示体素
    visualization_msgs::Marker cube;
    cube.type = visualization_msgs::Marker::CUBE;
    cube.scale.x = 0.4;
    cube.scale.y = 0.4;
    cube.scale.z = 0.4;
    cube.color.r = plane_rgb(0);
    cube.color.g = plane_rgb(1);
    cube.color.b = plane_rgb(2);
    cube.color.a = 0.8f;

    control.markers.push_back(cube);
    int_marker.controls.push_back(control);

    static std::vector<VOXEL_LOC> selected_voxels;

    server->insert(
        int_marker,
        [&voxel_map, &text_pub, &near_text_pub](
            const visualization_msgs::InteractiveMarkerFeedbackConstPtr&
                feedback) {
          if (feedback->event_type ==
              visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK) {
            ROS_INFO_STREAM("Clicked voxel: " << feedback->marker_name);
            std::string name = feedback->marker_name;
            std::istringstream ss(name);
            std::string token;
            std::vector<int64_t> coords;

            while (std::getline(ss, token, '_')) {
              coords.push_back(std::stoll(token));
            }

            if (coords.size() == 3) {
              VOXEL_LOC loc{coords[0], coords[1], coords[2]};
              auto it = voxel_map.find(loc);
              if (it != voxel_map.end()) {
                publishVoxelTextMarker(text_pub, it->second);
              }

              selected_voxels.push_back(loc);
              if (selected_voxels.size() == 2) {
                auto iter1 = voxel_map.find(selected_voxels[0]);
                auto iter2 = voxel_map.find(selected_voxels[1]);

                publishVoxelNearTextMarker(near_text_pub, iter1->second,
                                           iter2->second);

                selected_voxels.erase(selected_voxels.begin());
              }
            }
          }
        });
  }
  server->applyChanges();
}

}  // namespace ieskf_fusion

#endif