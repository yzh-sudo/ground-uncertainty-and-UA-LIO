#ifndef IESKF_FUSION_VOXELMAP_MODEL_HPP
#define IESKF_FUSION_VOXELMAP_MODEL_HPP

#include <pcl/filters/voxel_grid.h>

#include <Eigen/Dense>

#include "ieskf_fusion/math/SO3.hpp"
#include "ieskf_fusion/modules/map/voxelmap.hpp"
#include "ieskf_fusion/type/point.hpp"
#include "ieskf_fusion/type/state.hpp"

namespace ieskf_fusion {
class VoxelMapModel {
 public:
  struct residual {
    PointWithCov point;
    Eigen::Vector3d plane_normal;
    Eigen::Vector3d plane_center;
    Eigen::Matrix<double, 6, 6> plane_cov;
    double d;
    double distance;
  };

  VoxelMapModel(double filter_size) {
    std::cout << "filter_size: " << filter_size << std::endl;
    voxel_filter.setLeafSize(filter_size, filter_size, filter_size);

    current_cloud_ptr = pcl::make_shared<pcl::PointCloud<PointType>>();
  }

  void setCurrentPointCloud(pcl::PointCloud<PointType>::Ptr cloud) {
    voxel_filter.setInputCloud(cloud);
    voxel_filter.filter(*current_cloud_ptr);

    residual_set.clear();
    first = true;
  }

  void setMap(const std::unordered_map<VOXEL_LOC, UnionFind*>& input_map) {
    voxel_map = &input_map;
  }

  bool compute(const State& state, Eigen::MatrixXd& Z, Eigen::MatrixXd& H,
               Eigen::MatrixXd& R_inv) {
    std::vector<residual> residual_list;

    std::vector<PointWithCov> current_scan;

    {
      auto t1 = std::chrono::high_resolution_clock::now();
      calculatePointCov(current_cloud_ptr, state, current_scan);
      auto t2 = std::chrono::high_resolution_clock::now();
      double duration_ms =
          std::chrono::duration<double, std::milli>(t2 - t1).count();
      std::cout << "[Timing] voxel_map update calculatePointCov 耗时: "
                << duration_ms << " ms" << std::endl;
    }

    {
      auto t1 = std::chrono::high_resolution_clock::now();
      buildResidualList(*voxel_map, current_scan, residual_list);
      auto t2 = std::chrono::high_resolution_clock::now();
      double duration_ms =
          std::chrono::duration<double, std::milli>(t2 - t1).count();
      std::cout << "[Timing] voxel_map update buildResidualList 耗时: "
                << duration_ms << " ms" << std::endl;
    }

    int valid_points_num = residual_list.size();
    // std::cout << "valid_points_num " << valid_points_num << std::endl;
    H = Eigen::MatrixXd::Zero(valid_points_num, 24);
    R_inv = Eigen::MatrixXd::Zero(valid_points_num, valid_points_num);
    Z.resize(valid_points_num, 1);

    for (int vi = 0; vi < valid_points_num; vi++) {
      Eigen::Vector3d point_lidar = residual_list[vi].point.point;
      Eigen::Vector3d point_world = residual_list[vi].point.world_point;
      Eigen::Vector3d plane_normal = residual_list[vi].plane_normal;
      Eigen::Vector3d plane_center = residual_list[vi].plane_center;

      Eigen::Matrix3d dr =
          state.rotation.toRotationMatrix() *
          skewSymmetric(state.offset_R_L_I.toRotationMatrix() * point_lidar +
                        state.offset_T_L_I);
      Eigen::Vector3d direction = plane_normal;
      H.block<1, 3>(vi, 0) = direction.transpose();
      H.block<1, 3>(vi, 3) = -1 * plane_normal.transpose() * dr;
      //   H.block<1, 3>(vi, 6) =
      //       direction.transpose() * state.rotation.toRotationMatrix();
      //   H.block<1, 3>(vi, 9) =
      //       -1 * direction.transpose() * state.rotation.toRotationMatrix() *
      //       state.offset_R_L_I.toRotationMatrix() *
      //       skewSymmetric(point_lidar);

      Z(vi, 0) = residual_list[vi].distance;

      Eigen::Matrix<double, 6, 6> plane_cov = residual_list[vi].plane_cov;
      Eigen::Matrix3d cov;
      calcBodyCov(point_lidar, 0.04, 0.1, cov);  // 雷达系下点云的不确定性
      cov = state.rotation.toRotationMatrix() * cov *
            state.rotation.toRotationMatrix().transpose();
      Eigen::Matrix<double, 1, 6> jacobi_nq;
      jacobi_nq.block<1, 3>(0, 0) = (point_world - plane_center).transpose();
      jacobi_nq.block<1, 3>(0, 3) = -plane_normal.transpose();
      double sigma_l = jacobi_nq * plane_cov * jacobi_nq.transpose();
      R_inv(vi, vi) =
          1.0 / (sigma_l + plane_normal.transpose() * cov * plane_normal);
    }

    return true;
  }

  void buildResidualList(
      const std::unordered_map<VOXEL_LOC, UnionFind*>& voxel_map,
      const std::vector<PointWithCov>& input_points,
      std::vector<residual>& residual_list) {
    residual_list.clear();
    for (uint i = 0; i < input_points.size(); i++) {
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
        UnionFind* current_node = iter->second;
        UnionFind* root_node = current_node->findRoot(current_node);
        residual single_res;
        bool is_success = false;
        double prob = 0;
        buildSingleResidual(p_v, root_node, single_res, is_success, prob);
        // 邻域构建残差
        if (!is_success) {
          VOXEL_LOC near_position = position;
          if (loc_xyz[0] > (current_node->voxel_center[0] + voxel_size / 4)) {
            near_position.x = near_position.x + 1;
          } else if (loc_xyz[0] <
                     (current_node->voxel_center[0] - voxel_size / 4)) {
            near_position.x = near_position.x - 1;
          }

          if (loc_xyz[1] > (current_node->voxel_center[1] + voxel_size / 4)) {
            near_position.y = near_position.y + 1;
          } else if (loc_xyz[1] <
                     (current_node->voxel_center[1] - voxel_size / 4)) {
            near_position.y = near_position.y - 1;
          }

          if (loc_xyz[2] > (current_node->voxel_center[2] + voxel_size / 4)) {
            near_position.z = near_position.z + 1;
          } else if (loc_xyz[2] <
                     (current_node->voxel_center[2] - voxel_size / 4)) {
            near_position.z = near_position.z - 1;
          }

          auto iter_near = voxel_map.find(near_position);
          if (iter_near != voxel_map.end()) {
            UnionFind* near_node = iter_near->second;
            UnionFind* near_root_node = near_node->findRoot(near_node);
            buildSingleResidual(p_v, near_root_node, single_res, is_success,
                                prob);
          }
        }
        if (is_success) {
          residual_list.push_back(single_res);
        }
      }
    }
    first = false;
  }

  void buildSingleResidual(const PointWithCov& point,
                           const UnionFind* current_node,
                           residual& single_residual, bool& is_success,
                           double& prob, double sigma_num = 3) {
    Eigen::Vector3d point_world = point.world_point;
    if (current_node->plane_ptr->is_plane) {
      // 点的投影必须在拟合平面的范围内
      Plane curr_plane = *current_node->plane_ptr;
      Eigen::Vector3d point_to_plane_center = point_world - curr_plane.center;
      double proj_length =
          (point_to_plane_center -
           point_to_plane_center.dot(curr_plane.normal) * curr_plane.normal)
              .norm();
      double radius_k = 3;
      if (proj_length > radius_k * curr_plane.radius) {
        return;
      }

      // 3sigma准则
      Eigen::Matrix<double, 1, 6> jacobi_nq;
      jacobi_nq.block<1, 3>(0, 0) =
          (point_world - curr_plane.center).transpose();
      jacobi_nq.block<1, 3>(0, 3) = -curr_plane.normal.transpose();
      double sigma_l = jacobi_nq * curr_plane.plane_cov * jacobi_nq.transpose();
      sigma_l += curr_plane.normal.transpose() * point.cov * curr_plane.normal;
      double point_to_plane_distance =
          abs(curr_plane.normal.transpose() * point_world + curr_plane.d);
      if (point_to_plane_distance > sigma_num * sqrt(sigma_l)) {
        return;
      }

      is_success = true;
      double this_prob = 1.0 / (sqrt(sigma_l)) *
                         exp(-0.5 * point_to_plane_distance *
                             point_to_plane_distance / sigma_l);
      if (this_prob > prob) {
        prob = this_prob;
        single_residual.point = point;
        single_residual.plane_cov = curr_plane.plane_cov;
        single_residual.plane_normal = curr_plane.normal;
        single_residual.plane_center = curr_plane.center;
        single_residual.d = -curr_plane.normal.transpose() * curr_plane.center;
        single_residual.distance =
            point_world.transpose() * curr_plane.normal + curr_plane.d;
        if (first) {
          residual_set.insert(single_residual.distance);
        }
      }
    }
  }

  void calculatePointCov(const pcl::PointCloud<PointType>::Ptr& cloud,
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
      calcBodyCov(p_v.point, 0.04, 0.1, p_v.cov);
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

 private:
  const std::unordered_map<VOXEL_LOC, UnionFind*>* voxel_map;
  double voxel_size = 0.5;

  pcl::PointCloud<PointType>::Ptr current_cloud_ptr;
  pcl::VoxelGrid<PointType> voxel_filter;

  std::set<double> residual_set;
  bool first = true;
};
}  // namespace ieskf_fusion

#endif