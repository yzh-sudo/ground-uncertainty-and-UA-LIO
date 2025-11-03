#ifndef IESKF_FUSION_STATE_HPP
#define IESKF_FUSION_STATE_HPP

#include <Eigen/Dense>

namespace ieskf_fusion {
struct State {
  Eigen::Vector3d position;
  Eigen::Quaterniond rotation;
  Eigen::Vector3d offset_T_L_I;
  Eigen::Quaterniond offset_R_L_I;
  Eigen::Vector3d velocity;
  Eigen::Vector3d bg;
  Eigen::Vector3d ba;
  Eigen::Vector3d gravity;
  Eigen::Matrix<double, 24, 24> cov;
  State() {
    position = Eigen::Vector3d::Zero();
    rotation = Eigen::Quaterniond::Identity();
    offset_T_L_I = Eigen::Vector3d::Zero();
    offset_R_L_I = Eigen::Quaterniond::Identity();
    velocity = Eigen::Vector3d::Zero();
    bg = Eigen::Vector3d::Zero();
    ba = Eigen::Vector3d::Zero();
    gravity = Eigen::Vector3d::Zero();
    cov = Eigen::Matrix<double, 24, 24>::Identity();
  }
};
};  // namespace ieskf_fusion

#endif