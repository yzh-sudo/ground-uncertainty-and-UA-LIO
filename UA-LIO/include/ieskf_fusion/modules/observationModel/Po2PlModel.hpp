#ifndef IESKF_FUSION_PO2PL_OBSERVATION_MODEL_HPP
#define IESKF_FUSION_PO2PL_OBSERVATION_MODEL_HPP

#include "ieskf_fusion/modules/observationModel/observationModel.hpp"

namespace ieskf_fusion {
class Po2PlModel : public ObservationModel {
 public:
  struct Loss {
    double residual = 0.0;
    Eigen::Vector3d plane_normal = Eigen::Vector3d::Zero();
    Eigen::Vector3d point_lidar = Eigen::Vector3d::Zero();
  };

  Po2PlModel(double filter_size) {
    std::cout << "filter_size: " << filter_size << std::endl;
    voxel_filter.setLeafSize(filter_size, filter_size, filter_size);

    current_cloud_ptr = pcl::make_shared<pcl::PointCloud<PointType>>();
  }

  void setCurrentPointCloud(pcl::PointCloud<PointType>::Ptr cloud) override {
    voxel_filter.setInputCloud(cloud);
    voxel_filter.filter(*current_cloud_ptr);

    nearest_indices_vec.resize(current_cloud_ptr->size());
    is_effect_point.clear();
    is_effect_point.resize(current_cloud_ptr->size(), false);
  }

  void setMap(pcl::PointCloud<PointType>::ConstPtr map_cloud,
              pcl::KdTreeFLANN<PointType>::ConstPtr map_kdtree) override {
    local_map_ptr = map_cloud;
    map_kdtree_ptr = map_kdtree;
  }

  bool processPoint(size_t i, const PointType& point_lidar,
                    const PointType& point_world, bool need_research,
                    Loss& out_loss) {
    Eigen::Vector3d point3d(point_lidar.x, point_lidar.y, point_lidar.z);
    Eigen::Vector3d world_point3d(point_world.x, point_world.y, point_world.z);

    std::vector<int> indices;
    if (need_research) {
      std::vector<float> distances;
      if (map_kdtree_ptr->nearestKSearch(point_world, 5, indices, distances) ==
          0) {
        return false;
      }
      if (distances.size() < 5 || distances[4] > 5.0) {
        return false;
      }
      nearest_indices_vec[i] = indices;
    } else {
      indices = nearest_indices_vec[i];
    }

    std::vector<PointType> fit_plane_points;
    fit_plane_points.reserve(indices.size());
    for (int idx : indices) {
      fit_plane_points.push_back(local_map_ptr->points[idx]);
    }

    Eigen::Vector4d plane;
    if (!planarCheck(fit_plane_points, plane, 0.1f)) {
      return false;
    }

    Loss loss;
    loss.residual = world_point3d.dot(plane.head<3>()) + plane(3);
    loss.plane_normal = plane.head<3>();
    loss.point_lidar = point3d;

    if (!isValidLoss(loss, 0.9)) {
      return false;
    }

    out_loss = loss;

    return true;
  }

  bool compute(const State& state, Eigen::MatrixXd& Z, Eigen::MatrixXd& H,
               bool converge) override {
    Eigen::Vector3d pos =
        state.rotation.toRotationMatrix() * state.offset_T_L_I + state.position;
    Eigen::Quaterniond rot =
        Eigen::Quaterniond(state.rotation.toRotationMatrix() *
                           state.offset_R_L_I.toRotationMatrix());
    rot.normalize();

    std::vector<Loss> loss_candidates(current_cloud_ptr->size());
    std::vector<Loss> valid_losses;

    for (size_t i = 0; i < current_cloud_ptr->size(); ++i) {
      const PointType& point_lidar = current_cloud_ptr->points[i];
      const PointType& point_world = transformPoint(point_lidar, rot, pos);

      Loss loss;
      if (converge) {
        bool valid = processPoint(i, point_lidar, point_world, true, loss);
        is_effect_point[i] = valid;
        if (valid) {
          loss_candidates[i] = loss;
        }
      } else {
        if (!is_effect_point[i]) {
          continue;
        }
        bool valid = processPoint(i, point_lidar, point_world, false, loss);
        is_effect_point[i] = valid;
        if (valid) {
          loss_candidates[i] = loss;
        }
      }
    }

    for (size_t i = 0; i < current_cloud_ptr->size(); ++i) {
      if (is_effect_point[i]) {
        valid_losses.push_back(loss_candidates[i]);
      }
    }

    int valid_points_num = valid_losses.size();
    H = Eigen::MatrixXd::Zero(valid_points_num, 24);
    Z.resize(valid_points_num, 1);
    for (int vi = 0; vi < valid_points_num; vi++) {
      Eigen::Vector3d dr = -1 * valid_losses[vi].plane_normal.transpose() *
                           state.rotation.toRotationMatrix() *
                           skewSymmetric(state.offset_R_L_I.toRotationMatrix() *
                                             valid_losses[vi].point_lidar +
                                         state.offset_T_L_I);
      Eigen::Vector3d direction = valid_losses[vi].plane_normal;
      H.block<1, 3>(vi, 0) = direction.transpose();
      H.block<1, 3>(vi, 3) = dr.transpose();
      H.block<1, 3>(vi, 6) =
          direction.transpose() * state.rotation.toRotationMatrix();
      H.block<1, 3>(vi, 9) = -1 * direction.transpose() *
                             state.rotation.toRotationMatrix() *
                             state.offset_R_L_I.toRotationMatrix() *
                             skewSymmetric(valid_losses[vi].point_lidar);
      Z(vi, 0) = valid_losses[vi].residual;
    }

    return true;
  }

  static bool planarCheck(const std::vector<PointType>& points,
                          Eigen::Vector4d& pabcd, float threshold) {
    Eigen::Vector3d normal_vector;
    Eigen::MatrixXd A;
    Eigen::VectorXd B;
    int point_num = points.size();
    A.resize(point_num, 3);
    B.resize(point_num);
    A.setZero();
    B.setOnes();
    B *= -1.0f;
    for (int i = 0; i < point_num; i++) {
      A(i, 0) = points[i].x;
      A(i, 1) = points[i].y;
      A(i, 2) = points[i].z;
    }

    normal_vector = A.colPivHouseholderQr().solve(B);

    double normal = normal_vector.norm();
    pabcd(0) = normal_vector(0) / normal;
    pabcd(1) = normal_vector(1) / normal;
    pabcd(2) = normal_vector(2) / normal;
    pabcd(3) = 1.0 / normal;

    for (int j = 0; j < point_num; j++) {
      if (fabs(pabcd(0) * points[j].x + pabcd(1) * points[j].y +
               pabcd(2) * points[j].z + pabcd(3)) > threshold) {
        return false;
      }
    }

    return true;
  }

  static bool isValidLoss(const Loss& loss, double threshold) {
    if (std::isnan(loss.residual)) {
      return false;
    }
    if (!loss.plane_normal.allFinite()) {
      return false;
    }
    double weight =
        1 - 0.9 * std::abs(loss.residual) / std::sqrt(loss.point_lidar.norm());

    return weight > threshold;
  }

 private:
  std::vector<std::vector<int>> nearest_indices_vec;
  std::vector<bool> is_effect_point;

  pcl::VoxelGrid<PointType> voxel_filter;
};
}  // namespace ieskf_fusion

#endif