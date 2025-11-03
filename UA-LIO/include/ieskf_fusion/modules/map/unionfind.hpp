#ifndef IESKF_FUSION_UNIONFIND_HPP
#define IESKF_FUSION_UNIONFIND_HPP

#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>

#include "ieskf_fusion/modules/map/voxel_feature.hpp"
#include "ieskf_fusion/modules/map/voxel_loc.hpp"

namespace ieskf_fusion {

static int plane_id = 1;

class PlaneInformationFilter {
 public:
  PlaneInformationFilter(const Plane& init_plane) {
    Eigen::VectorXd x(6);
    x << init_plane.normal, init_plane.center;

    // 转换为信息矩阵形式
    Lambda_ = init_plane.plane_cov.inverse();
    eta_ = Lambda_ * x;
  }

  // 观测更新
  void update(const Plane& measure_plane) {
    Eigen::VectorXd z(6);
    z << measure_plane.normal, measure_plane.center;

    Eigen::MatrixXd Lambda_z = measure_plane.plane_cov.inverse();
    Eigen::VectorXd eta_z = Lambda_z * z;

    // 累加信息量
    Lambda_ += Lambda_z;
    eta_ += eta_z;
  }

  Plane getState() const {
    Eigen::VectorXd x = Lambda_.inverse() * eta_;
    Plane st;
    st.normal = x.segment<3>(0);
    st.center = x.segment<3>(3);
    st.plane_cov = Lambda_.inverse();
    return st;
  }

 private:
  Eigen::MatrixXd Lambda_;  // 信息矩阵 (6x6)
  Eigen::VectorXd eta_;     // 信息向量 (6x1)
};

class PlaneKalmanFilter {
 public:
  PlaneKalmanFilter(const Plane& init_plane) {
    plane_state_ = init_plane;
    normal_cov = init_plane.plane_cov.block<3, 3>(0, 0);
    center_cov = init_plane.plane_cov.block<3, 3>(3, 3);
    P_ = init_plane.plane_cov;
  }

  void update(const Plane& measure_plane) {
    update_normal(measure_plane.normal,
                  measure_plane.plane_cov.block<3, 3>(0, 0));
    // update_center(measure_plane.center,
    //               measure_plane.plane_cov.block<3, 3>(3, 3));

    // Eigen::VectorXd x(6);
    // x << plane_state_.normal, plane_state_.center;

    // Eigen::VectorXd z(6);
    // z << measure_plane.normal, measure_plane.center;

    // Eigen::MatrixXd R = measure_plane.plane_cov;
    // Eigen::MatrixXd K = P_ * (P_ + R).inverse();
    // x = x + K * (z - x);

    // P_ = (Eigen::MatrixXd::Identity(6, 6) - K) * P_;

    // plane_state_.normal = x.segment<3>(0);
    // plane_state_.center = x.segment<3>(3);
    // plane_state_.plane_cov = P_;
  }

  void update_normal(const Eigen::Vector3d& measure_normal,
                     const Eigen::Matrix3d& normal_R) {
    Eigen::Vector3d x = plane_state_.normal;
    Eigen::Vector3d z = measure_normal;

    Eigen::MatrixXd K = normal_cov * (normal_cov + normal_R).inverse();

    x = x + K * (z - x);
    normal_cov = (Eigen::MatrixXd::Identity(3, 3) - K) * normal_cov;

    plane_state_.normal = x;
    plane_state_.plane_cov.block<3, 3>(0, 0) = normal_cov;
  }

  void update_center(const Eigen::Vector3d& measure_center,
                     const Eigen::Matrix3d& center_R) {
    Eigen::Vector3d x = plane_state_.center;
    Eigen::Vector3d z = measure_center;

    Eigen::MatrixXd K = center_cov * (center_cov + center_R).inverse();

    x = x + K * (z - x);
    center_cov = (Eigen::MatrixXd::Identity(3, 3) - K) * center_cov;

    plane_state_.center = x;
    plane_state_.plane_cov.block<3, 3>(3, 3) = center_cov;
  }

  Plane getPlane() const { return plane_state_; }

 private:
  Plane plane_state_;
  Eigen::Matrix3d normal_cov;
  Eigen::Matrix3d center_cov;
  Eigen::MatrixXd P_;
};

class UnionFind {
 public:
  struct EigenValueResult {
    Eigen::Vector3d eigen_values;
    Eigen::Matrix<double, 3, 3> eigen_vectors;
    Eigen::Index min_index;
    Eigen::Index mid_index;
    Eigen::Index max_index;
  };

  std::vector<PointWithCov> all_points;     // 体素内存储的所有点
  std::vector<PointWithCov> recent_points;  // 体素内存储的最新点
  int all_points_num;
  int max_points_num;
  int new_points_num;  // 地图更新时，体素内新加入的点的个数，避免频繁更新
  int min_points_num;  // 地图更新时所需要点的最小个数

  Plane* plane_ptr;            // 体素内的平面
  float plane_min_eigenvalue;  // 平面判断阈值

  bool init_node;  // 并查集节点是否初始化
  bool update_enable;  // 体素是否能够更新，达到max_points_num后不更新
  bool update_cov_enable;
  std::array<double, 3> voxel_center;  // 并查集节点在体素地图上的中心坐标
  UnionFind* rootNode;                 // 并查集根节点

  std::string file_name;

  UnionFind() {
    plane_ptr = new Plane;
    init_node = false;
    plane_min_eigenvalue = 0.01;
    min_points_num = 5;
    max_points_num = 50;
    rootNode = this;
    update_enable = true;
    all_points.reserve(max_points_num);

    file_name = "/home/myh/ieskf_ws/log/re-calc/merge-plane-record-f.csv";
  }

  void write_planes_to_csv(const std::string& filename, const double para_a,
                           const double para_b, const double trace_a,
                           const double trace_b,
                           const Eigen::Vector3d& delta_normal,
                           const Eigen::Vector3d& normal, const double angel,
                           const double angel_z, const double near_angel_z,
                           const double local_angel_z) {
    std::ofstream file;
    // 以追加模式写文件
    file.open(filename, std::ios::app);
    if (!file.is_open()) {
      std::cerr << "Error: cannot open " << filename << std::endl;
      return;
    }

    // 如果是新文件，写表头
    if (file.tellp() == 0) {
      file << "large_para,small_para,large_trace,small_trace,normal_x,normal_"
              "y,"
              "normal_z,pn_x,pn_y,pn_"
              "z,angle,angel_z,near_angel_z,local_angel_z\n";
    }

    file << std::fixed << std::setprecision(6);

    file << para_b << "," << para_a << "," << trace_a << "," << trace_b << ","
         << delta_normal[0] << "," << delta_normal[1] << "," << delta_normal[2]
         << "," << abs(normal[0]) << "," << abs(normal[1]) << ","
         << abs(normal[2]) << "," << angel << "," << angel_z << ","
         << near_angel_z << "," << local_angel_z << "\n";
  }

  void write_to_csv(const std::string& filename, const double min_lambda,
                    const double planarity, const double rmse_distance,
                    const double angel_z) {
    std::ofstream file;
    // 以追加模式写文件
    file.open(filename, std::ios::app);
    if (!file.is_open()) {
      std::cerr << "Error: cannot open " << filename << std::endl;
      return;
    }

    // 如果是新文件，写表头
    if (file.tellp() == 0) {
      file << "min_eigen_value,angel_z,planarity,rmse_distance\n";
    }

    file << std::fixed << std::setprecision(6);

    file << min_lambda << "," << angel_z << "," << planarity << ","
         << rmse_distance << "\n";
  }

  void init_plane(const std::vector<PointWithCov>& points, Plane* plane) {
    if (!plane->is_inited) {
      plane->plane_id = plane_id++;
      plane->is_inited = true;
    }

    ComputeMeanCovariance(points, plane);

    // 对协方差矩阵做特征值分解，计算法向量等参数
    auto result = ComputeEigenValue(plane->covariance);

    if (result.eigen_values(result.min_index) < plane_min_eigenvalue) {
      // 计算平面噪声，参考BALM论文的公式
      ComputePlaneCov(result, points, plane);
      plane->is_plane = true;
      UpdatePlaneParams(result, plane);
    } else {
      plane->is_plane = false;
      UpdatePlaneParams(result, plane);
    }
  }

  static void ComputeMeanCovariance(const std::vector<PointWithCov>& points,
                                    Plane* plane) {
    plane->plane_cov = Eigen::Matrix<double, 6, 6>::Zero();
    plane->covariance = Eigen::Matrix3d::Zero();
    plane->center = Eigen::Vector3d::Zero();
    plane->normal = Eigen::Vector3d::Zero();

    // 计算平面中心和协方差矩阵
    for (const auto& point : points) {
      plane->center += point.world_point;
      plane->covariance += point.world_point * point.world_point.transpose();
    }
    plane->center /= points.size();
    plane->covariance = plane->covariance / points.size() -
                        plane->center * plane->center.transpose();
  }

  static EigenValueResult ComputeEigenValue(const Eigen::Matrix3d& covariance) {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(covariance);

    EigenValueResult result;
    result.eigen_values = eigen_solver.eigenvalues();
    result.eigen_vectors = eigen_solver.eigenvectors();

    result.eigen_values.minCoeff(&result.min_index);
    result.eigen_values.maxCoeff(&result.max_index);
    result.mid_index = 3 - result.min_index - result.max_index;

    return result;
  }

  static void ComputePlaneCov(const EigenValueResult& result,
                              const std::vector<PointWithCov>& points,
                              Plane* plane) {
    Eigen::Matrix3d jacobi_q;
    jacobi_q << 1.0 / points.size(), 0, 0, 0, 1.0 / points.size(), 0, 0, 0,
        1.0 / points.size();

    // 计算平面噪声，参考BALM论文的公式
    for (size_t i = 0; i < points.size(); i++) {
      Eigen::Matrix<double, 6, 3> jacobi_nq;
      Eigen::Matrix3d F;
      for (int m = 0; m < 3; m++) {
        if (m != (int)result.min_index) {
          Eigen::Matrix<double, 1, 3> F_m =
              (points[i].world_point - plane->center).transpose() /
              ((points.size()) * (result.eigen_values[result.min_index] -
                                  result.eigen_values[m])) *
              (result.eigen_vectors.col(m) *
                   result.eigen_vectors.col(result.min_index).transpose() +
               result.eigen_vectors.col(result.min_index) *
                   result.eigen_vectors.col(m).transpose());
          F.row(m) = F_m;
        } else {
          Eigen::Matrix<double, 1, 3> F_m;
          F_m << 0, 0, 0;
          F.row(m) = F_m;
        }
      }
      jacobi_nq.block<3, 3>(0, 0) = result.eigen_vectors * F;
      jacobi_nq.block<3, 3>(3, 0) = jacobi_q;
      plane->plane_cov += jacobi_nq * points[i].cov * jacobi_nq.transpose();
    }
  }

  static void UpdatePlaneParams(const EigenValueResult& result, Plane* plane) {
    plane->normal = result.eigen_vectors.col(result.min_index);
    plane->x_normal = result.eigen_vectors.col(result.max_index);
    plane->y_normal = result.eigen_vectors.col(result.mid_index);
    plane->min_eigen_value = result.eigen_values(result.min_index);
    plane->mid_eigen_value = result.eigen_values(result.mid_index);
    plane->max_eigen_value = result.eigen_values(result.max_index);
    plane->radius = sqrt(result.eigen_values(result.max_index));
    plane->d = -plane->normal.transpose() * plane->center;
  }

  void init_unionfind_node() {
    if (all_points.size() > min_points_num) {
      init_plane(all_points, plane_ptr);
      if (all_points.size() > max_points_num) {
        update_enable = false;
      }
      init_node = true;
    }
  }

  UnionFind* findRoot(UnionFind* node) {
    if (node->rootNode != node) {
      node->rootNode = findRoot(node->rootNode);  // 递归+路径压缩
    }
    return node->rootNode;
  }

  void update_unionfind_node(
      const PointWithCov& point_with_cov, VOXEL_LOC& position,
      std::unordered_map<VOXEL_LOC, UnionFind*>& voxel_map) {
    // 节点没有初始化就进行初始化
    if (!init_node) {
      all_points.push_back(point_with_cov);
      if (all_points.size() > min_points_num) {
        init_unionfind_node();
      }
      return;
    }

    // 已经初始化判断是否能够进行更新
    if (update_enable) {
      all_points.push_back(point_with_cov);
      new_points_num++;

      // 点数足够 拟合平面
      if (new_points_num > min_points_num) {
        init_plane(all_points, plane_ptr);
        new_points_num = 0;
      }

      // 点数超过 清空点云数据 不再更新
      if (all_points.size() > max_points_num) {
        new_points_num = 0;
        update_enable = false;
        // std::vector<PointWithCov>().swap(all_points);
        plane_ptr->points_num = all_points.size();
        all_points_num = all_points.size();
        new_points_num = 0;
      }
    } else if (plane_ptr->is_plane) {
      // 点数达到，进行合并
      plane_merge(position, voxel_map);
    }
  }

  void plane_merge(VOXEL_LOC& position,
                   std::unordered_map<VOXEL_LOC, UnionFind*>& voxel_map) {
    UnionFind* current_node = findRoot(this);

    VOXEL_LOC current_voxel = position;
    for (int k = 0; k < 6; k++) {
      VOXEL_LOC neighbor = current_voxel;
      switch (k) {
        case 0:  // 前
          neighbor.x = neighbor.x + 1;
          break;
        case 1:  // 后
          neighbor.x = neighbor.x - 1;
          break;
        case 2:  // 左
          neighbor.y = neighbor.y + 1;
          break;
        case 3:  // 右
          neighbor.y = neighbor.y - 1;
          break;
        case 4:  // 上
          neighbor.z = neighbor.z + 1;
          break;
        case 5:  // 下
          neighbor.z = neighbor.z - 1;
          break;
        default:
          break;
      }

      auto iter = voxel_map.find(neighbor);
      VOXEL_LOC temp_pose;
      temp_pose.x = 1;
      temp_pose.y = -3;
      temp_pose.z = -1;
      auto temp_node = voxel_map.find(temp_pose);
      int temp_id = 0;
      if (temp_node != voxel_map.end()) {
        UnionFind* temp_root_node = findRoot(temp_node->second);

        temp_id = temp_root_node->plane_ptr->plane_id;
      }
      if (iter != voxel_map.end()) {
        // neighbor_plane所在的octotree可能不是root,所以要找到它的根节点
        // 找邻居的根节点
        UnionFind* near_node = findRoot(iter->second);

        //邻居与当前平面可能是相同root或点数不够
        if (near_node == current_node || near_node->update_enable) {
          continue;
        }

        if (!near_node->plane_ptr->is_plane) {
          continue;
        }

        Plane* near_plane = near_node->plane_ptr;
        Plane* curr_plane = current_node->plane_ptr;
        Eigen::Vector3d old_curr_normal = curr_plane->normal;
        Eigen::Vector3d old_near_normal = near_plane->normal;
        /*** Plane Merging ***/
        if (!plane_similar_judge(curr_plane, near_plane, temp_id)) {
          continue;
        }

        if (plane_update_recalc(current_node, near_node, temp_id)) {
          near_node->rootNode = current_node;
        }

        // if (plane_similar_judge(curr_plane, near_plane, temp_id)) {
        //   // plane_update(curr_plane, near_plane, temp_id);
        //   // if (plane_update(curr_plane, near_plane, temp_id)) {

        //   if (plane_update_recalc(current_node, near_node, temp_id)) {
        //     curr_plane->is_merged = true;
        //     near_plane->is_merged = true;
        //     near_node->rootNode = current_node;
        //   }
        // }
      }
    }
  }

  bool plane_similar_judge(const Plane* curr_plane, const Plane* near_plane,
                           int temp_id, double angle_thresh_deg = 5.0,
                           double dist_thresh = 0.05) {
    Eigen::Vector3d nn1 = curr_plane->normal.normalized();
    Eigen::Vector3d nn2 = near_plane->normal.normalized();

    // 1. 法向量夹角
    double cos_angle = std::abs(nn1.dot(nn2));
    double angle =
        std::acos(std::min(1.0, std::max(-1.0, cos_angle)));  // 数值安全
    double angle_thresh = angle_thresh_deg * M_PI / 180.0;

    // if (curr_plane->plane_id == temp_id || near_plane->plane_id == temp_id) {
    //   std::cout << "angel: " << angle << std::endl;
    // }

    if (angle > angle_thresh) {
      return false;  // 法向量差异太大
    }

    // 2. 平面间的距离（用 q1 到平面2 的距离）
    // double dist = std::abs(nn2.dot(curr_plane->center - near_plane->center));
    // if (dist > dist_thresh) {
    //   return false;  // 两平面位置不一致
    // }

    return true;  // 满足条件，可以认为相似
  }

  bool plane_update_recalc(UnionFind* current_node, UnionFind* near_node,
                           int temp_id) {
    Eigen::Vector3d curr_center = current_node->plane_ptr->center;
    Eigen::Vector3d near_center = near_node->plane_ptr->center;
    int all_nums = current_node->all_points_num + near_node->all_points_num;
    Eigen::Vector3d new_center = (curr_center * current_node->all_points_num +
                                  near_center * near_node->all_points_num) /
                                 double(all_nums);

    Eigen::Matrix3d new_cov =
        current_node->all_points_num * current_node->plane_ptr->covariance +
        near_node->all_points_num * near_node->plane_ptr->covariance;
    Eigen::Vector3d da = curr_center - new_center;
    Eigen::Vector3d db = near_center - new_center;
    new_cov += current_node->all_points_num * (da * da.transpose());
    new_cov += near_node->all_points_num * (db * db.transpose());
    new_cov = new_cov / double(all_nums);

    Eigen::Vector3d new_normal = current_node->plane_ptr->normal;
    auto result = ComputeEigenValue(new_cov);

    if (result.eigen_values(result.min_index) < plane_min_eigenvalue) {
      new_normal = result.eigen_vectors.col(result.min_index);
    } else {
      return false;
    }

    if (current_node->plane_ptr->normal.dot(new_normal) < 0) {
      new_normal = -new_normal;
    }

    current_node->plane_ptr->normal = new_normal;
    current_node->plane_ptr->center = new_center;
    current_node->plane_ptr->d = -new_normal.transpose() * new_center;
    current_node->plane_ptr->covariance = new_cov;
    current_node->plane_ptr->radius =
        sqrt(result.eigen_values(result.max_index));

    // near_node->plane_ptr->normal = new_normal;
    // near_node->plane_ptr->center = new_center;
    // near_node->plane_ptr->d = -new_normal.transpose() * new_center;
    // near_node->plane_ptr->covariance = new_cov;

    current_node->all_points.insert(current_node->all_points.end(),
                                    near_node->all_points.begin(),
                                    near_node->all_points.end());
    current_node->all_points_num = current_node->all_points.size();
    std::vector<PointWithCov>().swap(near_node->all_points);

    double distance = rmse_distance(current_node);
    if (current_node->plane_ptr->plane_id == temp_id ||
        near_node->plane_ptr->plane_id == temp_id) {
      write_to_csv(file_name, result.eigen_values(result.min_index), 0,
                   distance, signed_angle_with_z(new_normal));
    }

    current_node->all_points_num =
        current_node->all_points_num + near_node->all_points_num;

    return true;
  }

  double rmse_distance(const UnionFind* node) {
    double rmse = -1.0;
    if (!node->all_points.empty()) {
      double sum_err2 = 0.0;
      for (const auto& p : node->all_points) {
        double d = node->plane_ptr->normal.dot(
            p.world_point - node->plane_ptr->center);  // 点到平面的距离
        sum_err2 += d * d;
      }
      rmse = std::sqrt(sum_err2 / node->all_points.size());
    }
    return rmse;
  }

  bool plane_update(UnionFind* current_node, UnionFind* near_node,
                    int temp_id) {
    if (current_node->plane_ptr->normal.dot(near_node->plane_ptr->normal) < 0) {
      current_node->plane_ptr->normal = -current_node->plane_ptr->normal;
      current_node->plane_ptr->center = -current_node->plane_ptr->center;
    }
    Eigen::Vector3d curr_center = current_node->plane_ptr->center;
    Eigen::Vector3d near_center = near_node->plane_ptr->center;
    int all_nums = current_node->all_points_num + near_node->all_points_num;
    Eigen::Vector3d new_center = (curr_center * current_node->all_points_num +
                                  near_center * near_node->all_points_num) /
                                 double(all_nums);

    // if (current_node->plane_ptr->plane_id == temp_id ||
    //     near_node->plane_ptr->plane_id == temp_id) {
    //   std::cout << "curr_num: " << current_node->all_points_num
    //             << " curr_center: " << curr_center.transpose()
    //             << "near_num: " << near_node->all_points_num
    //             << " near_center: " << near_center.transpose()
    //             << " new_center: " << new_center.transpose() << std::endl;
    // }

    Eigen::Matrix3d new_cov =
        current_node->all_points_num * current_node->plane_ptr->covariance +
        near_node->all_points_num * near_node->plane_ptr->covariance;
    Eigen::Vector3d da = curr_center - new_center;
    Eigen::Vector3d db = near_center - new_center;
    new_cov += current_node->all_points_num * (da * da.transpose());
    new_cov += near_node->all_points_num * (db * db.transpose());

    new_cov = new_cov / double(all_nums);

    Eigen::Vector3d new_normal;
    // 对协方差矩阵做特征值分解，计算法向量等参数
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(new_cov);
    Eigen::Vector3d eigen_values = eigen_solver.eigenvalues();
    Eigen::Matrix3d eigen_vectors = eigen_solver.eigenvectors();

    Eigen::Index min_value_index;
    Eigen::Index max_value_index;
    eigen_values.minCoeff(&min_value_index);
    eigen_values.maxCoeff(&max_value_index);
    Eigen::Index mid_value_index = 3 - min_value_index - max_value_index;
    Eigen::Vector3d min_value_vector =
        eigen_vectors.real().col(min_value_index);
    Eigen::Vector3d mid_value_vector =
        eigen_vectors.real().col(mid_value_index);
    Eigen::Vector3d max_value_vector =
        eigen_vectors.real().col(max_value_index);
    if (eigen_values(min_value_index) < plane_min_eigenvalue) {
      double curr_ratio = 1 - current_node->plane_ptr->min_eigen_value /
                                  (current_node->plane_ptr->mid_eigen_value +
                                   current_node->plane_ptr->max_eigen_value);
      double near_ratio = 1 - near_node->plane_ptr->min_eigen_value /
                                  (near_node->plane_ptr->mid_eigen_value +
                                   near_node->plane_ptr->max_eigen_value);

      double w_curr = current_node->all_points_num * curr_ratio;
      double w_near = near_node->all_points_num * near_ratio;

      double alpha = w_curr * w_curr / (w_curr * w_curr + w_near * w_near);
      double beta = w_near * w_near / (w_curr * w_curr + w_near * w_near);
      // double alpha = w_curr / (w_curr + w_near);
      // double beta = w_near / (w_curr + w_near);

      // if (current_node->plane_ptr->plane_id == temp_id ||
      //     near_node->plane_ptr->plane_id == temp_id) {
      //   std::cout << "curr weight: " << alpha << " near weight: " << beta
      //             << std::endl;
      // }

      new_normal = alpha * current_node->plane_ptr->normal +
                   beta * near_node->plane_ptr->normal;
      new_normal.normalize();
    } else {
      return false;
    }

    current_node->plane_ptr->normal = new_normal;
    current_node->plane_ptr->center = new_center;
    current_node->plane_ptr->d = -new_normal.transpose() * new_center;
    current_node->plane_ptr->covariance = new_cov;
    current_node->plane_ptr->min_eigen_value = eigen_values(min_value_index);
    current_node->plane_ptr->mid_eigen_value = eigen_values(mid_value_index);
    current_node->plane_ptr->max_eigen_value = eigen_values(max_value_index);

    near_node->plane_ptr->normal = new_normal;
    near_node->plane_ptr->center = new_center;
    near_node->plane_ptr->d = -new_normal.transpose() * new_center;
    near_node->plane_ptr->covariance = new_cov;
    near_node->plane_ptr->min_eigen_value = eigen_values(min_value_index);
    near_node->plane_ptr->mid_eigen_value = eigen_values(mid_value_index);
    near_node->plane_ptr->max_eigen_value = eigen_values(max_value_index);

    current_node->all_points_num =
        current_node->all_points_num + near_node->all_points_num;

    double para_a = near_node->plane_ptr->plane_cov.norm() /
                    (near_node->plane_ptr->plane_cov.norm() +
                     current_node->plane_ptr->plane_cov.norm());
    double para_b = current_node->plane_ptr->plane_cov.norm() /
                    (near_node->plane_ptr->plane_cov.norm() +
                     current_node->plane_ptr->plane_cov.norm());
    current_node->plane_ptr->plane_cov =
        para_a * para_a * current_node->plane_ptr->plane_cov +
        para_b * para_b * near_node->plane_ptr->plane_cov;

    return true;
  }

  // void plane_update(UnionFind* current_node, UnionFind* near_node,
  //                   int temp_id) {
  //   // if (current_node->plane_ptr->plane_id == temp_id ||
  //   //     near_node->plane_ptr->plane_id == temp_id) {
  //   //   std::cout << "curr node points size" <<
  //   //   current_node->all_points.size()
  //   //             << std::endl;
  //   // }

  //   std::vector<PointWithCov> merged_points;
  //   current_node->all_points.insert(current_node->all_points.end(),
  //                                   near_node->all_points.begin(),
  //                                   near_node->all_points.end());

  //   merged_points.insert(merged_points.end(),
  //   current_node->all_points.begin(),
  //                        current_node->all_points.end());
  //   merged_points.insert(merged_points.end(), near_node->all_points.begin(),
  //                        near_node->all_points.end());
  //   std::vector<PointWithCov>().swap(near_node->all_points);

  //   Plane merged_plane;
  //   merged_plane.plane_cov = Eigen::Matrix<double, 6, 6>::Zero();
  //   merged_plane.covariance = Eigen::Matrix3d::Zero();
  //   merged_plane.center = Eigen::Vector3d::Zero();
  //   merged_plane.normal = Eigen::Vector3d::Zero();

  //   // 计算平面中心和协方差矩阵
  //   for (const auto& point : merged_points) {
  //     merged_plane.center += point.world_point;
  //     merged_plane.covariance +=
  //         point.world_point * point.world_point.transpose();
  //   }
  //   merged_plane.center /= merged_points.size();
  //   merged_plane.covariance =
  //       merged_plane.covariance / merged_points.size() -
  //       merged_plane.center * merged_plane.center.transpose();

  //   // 对协方差矩阵做特征值分解，计算法向量等参数
  //   Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(
  //       merged_plane.covariance);
  //   Eigen::Vector3d eigen_values = eigen_solver.eigenvalues();
  //   Eigen::Matrix3d eigen_vectors = eigen_solver.eigenvectors();

  //   Eigen::Index min_value_index;
  //   Eigen::Index max_value_index;
  //   eigen_values.minCoeff(&min_value_index);
  //   eigen_values.maxCoeff(&max_value_index);
  //   Eigen::Index mid_value_index = 3 - min_value_index - max_value_index;
  //   Eigen::Vector3d min_value_vector =
  //       eigen_vectors.real().col(min_value_index);
  //   Eigen::Vector3d mid_value_vector =
  //       eigen_vectors.real().col(mid_value_index);
  //   Eigen::Vector3d max_value_vector =
  //       eigen_vectors.real().col(max_value_index);
  //   Eigen::Matrix3d jacobi_q;
  //   jacobi_q << 1.0 / merged_points.size(), 0, 0, 0, 1.0 /
  //   merged_points.size(),
  //       0, 0, 0, 1.0 / merged_points.size();
  //   if (eigen_values(min_value_index) < plane_min_eigenvalue) {
  //     merged_plane.is_plane = true;
  //     merged_plane.normal = min_value_vector;

  //   } else {
  //     merged_plane.is_plane = false;
  //     merged_plane.normal = min_value_vector;
  //   }

  //   current_node->plane_ptr->normal = merged_plane.normal;
  //   current_node->plane_ptr->center = merged_plane.center;
  //   current_node->plane_ptr->d =
  //       -merged_plane.normal.transpose() * merged_plane.center;

  //   near_node->plane_ptr->normal = merged_plane.normal;
  //   near_node->plane_ptr->center = merged_plane.center;
  //   near_node->plane_ptr->d =
  //       -merged_plane.normal.transpose() * merged_plane.center;
  // }

  double angle_between(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2) {
    Eigen::Vector3d n2 = v2;
    if (v1.dot(v2) < 0) {
      n2 = -v2;
    }
    double cos_theta = v1.normalized().dot(n2.normalized());
    cos_theta = std::min(1.0, std::max(-1.0, cos_theta));  // 避免数值误差
    return acos(cos_theta) * 180.0 / M_PI;
  }

  template <typename T>
  T clamp(T v, T lo, T hi) {
    return std::min(hi, std::max(lo, v));
  }

  double signed_angle_with_z(const Eigen::Vector3d& v_in) {
    Eigen::Vector3d z(0, 0, 1);
    Eigen::Vector3d v = v_in.normalized();

    // 如果和 z 反向，则翻转
    if (v.dot(z) < 0) v = -v;

    // 与 z 的夹角（0 ~ 90）
    double cos_theta = clamp(v.dot(z), -1.0, 1.0);
    double theta = std::acos(cos_theta);

    // 在 XY 平面上的投影
    double azimuth = std::atan2(v.y(), v.x());

    // 用方位角的正负来确定符号
    double sign = (azimuth >= 0 ? 1.0 : -1.0);

    return theta * sign * 180.0 / M_PI;  // [-90, 90]
  }

  bool plane_update(Plane* curr_plane, Plane* near_plane, int temp_id) {
    if (curr_plane->normal.dot(near_plane->normal) < 0) {
      curr_plane->normal = -curr_plane->normal;
      curr_plane->center = -curr_plane->center;
    }

    // 6 cov
    // double para_a =
    //     near_plane->plane_cov.norm() /
    //     (near_plane->plane_cov.norm() + curr_plane->plane_cov.norm());
    // double para_b =
    //     curr_plane->plane_cov.norm() /
    //     (near_plane->plane_cov.norm() + curr_plane->plane_cov.norm());

    // curr_plane->center =
    //     para_a * curr_plane->center + para_b * near_plane->center;
    // curr_plane->normal =
    //     para_a * curr_plane->normal + para_b * near_plane->normal;
    // curr_plane->normal.normalize();
    // curr_plane->plane_cov = para_a * para_a * curr_plane->plane_cov +
    //                         para_b * para_b * near_plane->plane_cov;
    // curr_plane->d = -curr_plane->normal.transpose() *curr_plane->center;

    // near_plane->normal = curr_plane->normal;
    // near_plane->plane_cov = curr_plane->plane_cov;
    // near_plane->center = curr_plane->center;
    // near_plane->d = curr_plane->d;

    Eigen::Vector3d old_curr_normal = curr_plane->normal;
    Eigen::Vector3d old_near_normal = near_plane->normal;
    double old_curr_trace = curr_plane->plane_cov.block<3, 3>(0, 0).trace();
    double old_near_trace = near_plane->plane_cov.block<3, 3>(0, 0).trace();
    // 3 cov
    double normal_para_a = near_plane->plane_cov.block<3, 3>(0, 0).trace() /
                           (near_plane->plane_cov.block<3, 3>(0, 0).trace() +
                            curr_plane->plane_cov.block<3, 3>(0, 0).trace());
    double normal_para_b = curr_plane->plane_cov.block<3, 3>(0, 0).trace() /
                           (near_plane->plane_cov.block<3, 3>(0, 0).trace() +
                            curr_plane->plane_cov.block<3, 3>(0, 0).trace());
    double center_para_a = near_plane->plane_cov.block<3, 3>(3, 3).trace() /
                           (near_plane->plane_cov.block<3, 3>(3, 3).trace() +
                            curr_plane->plane_cov.block<3, 3>(3, 3).trace());
    double center_para_b = curr_plane->plane_cov.block<3, 3>(3, 3).trace() /
                           (near_plane->plane_cov.block<3, 3>(3, 3).trace() +
                            curr_plane->plane_cov.block<3, 3>(3, 3).trace());

    curr_plane->normal =
        normal_para_a * curr_plane->normal + normal_para_b * near_plane->normal;
    curr_plane->normal.normalize();
    curr_plane->center =
        center_para_a * curr_plane->center + center_para_b * near_plane->normal;
    curr_plane->plane_cov.block<3, 3>(0, 0) =
        normal_para_a * normal_para_a *
            curr_plane->plane_cov.block<3, 3>(0, 0) +
        normal_para_b * normal_para_b * near_plane->plane_cov.block<3, 3>(0, 0);
    curr_plane->plane_cov.block<3, 3>(3, 3) =
        center_para_a * center_para_a *
            curr_plane->plane_cov.block<3, 3>(3, 3) +
        center_para_b * center_para_b * near_plane->plane_cov.block<3, 3>(3, 3);
    curr_plane->d = -curr_plane->normal.transpose() * curr_plane->center;

    near_plane->normal = curr_plane->normal;
    near_plane->plane_cov = curr_plane->plane_cov;
    near_plane->center = curr_plane->center;
    near_plane->d = curr_plane->d;

    if (curr_plane->plane_id == temp_id || near_plane->plane_id == temp_id) {
      if (curr_plane->points_num > near_plane->points_num) {
        write_planes_to_csv(file_name, normal_para_b, normal_para_a,
                            old_curr_trace, old_near_trace,
                            normal_para_b * near_plane->normal,
                            normal_para_a * curr_plane->normal,
                            angle_between(curr_plane->normal, old_curr_normal),
                            signed_angle_with_z(curr_plane->normal),
                            signed_angle_with_z(old_near_normal),
                            signed_angle_with_z(near_plane->local_normal));
      } else {
        write_planes_to_csv(file_name, normal_para_a, normal_para_b,
                            old_near_trace, old_curr_trace,
                            normal_para_a * curr_plane->normal,
                            normal_para_b * near_plane->normal,
                            angle_between(near_plane->normal, old_near_normal),
                            signed_angle_with_z(near_plane->normal),
                            signed_angle_with_z(old_curr_normal),
                            signed_angle_with_z(curr_plane->local_normal));
      }
    }

    curr_plane->points_num = curr_plane->points_num + near_plane->points_num;
    return true;
  }

  // void plane_update(Plane* curr_plane, Plane* near_plane, int temp_id) {
  //   if (curr_plane->normal.dot(near_plane->normal) < 0) {
  //     curr_plane->normal = -curr_plane->normal;
  //     curr_plane->center = -curr_plane->center;
  //   }

  //   PlaneKalmanFilter kf(*curr_plane);
  //   kf.update(*near_plane);

  //   Plane merge_plane = kf.getPlane();
  //   Eigen::Vector3d merge_center =
  //       (curr_plane->center * curr_plane->points_num +
  //        near_plane->center * near_plane->points_num) /
  //       (curr_plane->points_num + near_plane->points_num);

  //   curr_plane->normal = merge_plane.normal;
  //   curr_plane->normal.normalize();
  //   curr_plane->center = merge_center;
  //   curr_plane->plane_cov = merge_plane.plane_cov;
  //   curr_plane->d = -curr_plane->normal.transpose() * curr_plane->center;

  //   near_plane->normal = curr_plane->normal;
  //   near_plane->center = merge_center;
  //   near_plane->plane_cov = curr_plane->plane_cov;
  //   near_plane->d = -near_plane->normal.transpose() * near_plane->center;

  //   if (curr_plane->plane_id == temp_id || near_plane->plane_id == temp_id) {
  //     std::cout << "angel :" << signed_angle_with_z(curr_plane->normal)
  //               << std::endl;
  //     std::cout << "trace cov: " << curr_plane->plane_cov.trace() <<
  //     std::endl;
  //   }

  //   curr_plane->points_num = curr_plane->points_num + near_plane->points_num;
  // }
};  // namespace ieskf_fusion

}  // namespace ieskf_fusion

#endif