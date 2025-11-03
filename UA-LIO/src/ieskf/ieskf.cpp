#include "ieskf_fusion/ieskf/ieskf.hpp"

namespace ieskf_fusion {
IESKF::IESKF(double eps, double cov_gyroscope, double cov_acceleration,
             double cov_bias_acceleration, double cov_bias_gyroscope) {
  P.setIdentity();
  P(0, 0) = P(1, 1) = P(2, 2) = 0.0001;
  P(3, 3) = P(4, 4) = P(5, 5) = 0.0001;
  P(12, 12) = P(13, 13) = P(14, 14) = 0.0001;
  P(6, 6) = P(7, 7) = P(8, 8) = 0.00001;        // offset_T_L_I
  P(9, 9) = P(10, 10) = P(11, 11) = 0.00001;    // offset_R_L_I
  P(15, 15) = P(16, 16) = P(17, 17) = 0.0001;   // bg
  P(18, 18) = P(19, 19) = P(20, 20) = 0.0001;   // ba
  P(21, 21) = P(22, 22) = P(23, 23) = 0.00001;  // g

  // P(9, 9) = P(10, 10) = P(11, 11) = 0.0001;     // bg
  // P(12, 12) = P(13, 13) = P(14, 14) = 0.001;    // ba
  // P(15, 15) = P(16, 16) = P(17, 17) = 0.00001;  // g
  thresh = eps;
  std::cout << "set IESKF iterator converge eps:   " << thresh << std::endl;
  std::cout << "set IESKF process noise Q" << std::endl;
  std::cout << "cov_gyroscop:           " << cov_gyroscope << std::endl;
  std::cout << "cov_acceleration:       " << cov_acceleration << std::endl;
  std::cout << "cov_bias_acceleration:  " << cov_bias_acceleration << std::endl;
  std::cout << "cov_bias_gyroscope:     " << cov_bias_gyroscope << std::endl;
  Q(0, 0) = Q(1, 1) = Q(2, 2) = cov_gyroscope;              // gyr
  Q(3, 3) = Q(4, 4) = Q(5, 5) = cov_acceleration;           // acc
  Q(6, 6) = Q(7, 7) = Q(8, 8) = cov_bias_gyroscope;         // bg
  Q(9, 9) = Q(10, 10) = Q(11, 11) = cov_bias_acceleration;  // ba

  X.ba.setZero();
  X.bg.setZero();
  X.gravity.setZero();
  X.position.setZero();
  X.rotation.setIdentity();
  X.velocity.setZero();

  X.cov = P;
}

IESKF::~IESKF() = default;

void IESKF::predict(IMU& imu, double dt) {
  Eigen::Vector3d imu_acc = imu.acceleration - X.ba;
  Eigen::Vector3d imu_gyr = imu.gyroscope - X.bg;
  auto rotation = X.rotation.toRotationMatrix();
  X.rotation =
      Eigen::Quaterniond(X.rotation.toRotationMatrix() * so3Exp((imu_gyr)*dt));
  X.rotation.normalize();
  X.velocity += (rotation * (imu_acc) + X.gravity) * dt;
  X.position += X.velocity * dt;

  Eigen::Matrix<double, 24, 24> Fx;
  Eigen::Matrix<double, 24, 12> Fw;
  Fw.setZero();
  Fx.setIdentity();

  // 24 state p R Tp TR v bg ba g
  Fx.block<3, 3>(0, 12) = Eigen::Matrix3d::Identity() * dt;  // p->v
  // Fx.block<3, 3>(3, 3) =
  //     Eigen::Matrix3d::Identity() + skewSymmetric(-imu_gyr * dt);  // R->R
  Fx.block<3, 3>(3, 15) = -1 * Eigen::Matrix3d::Identity() * dt;  // R->bg
  // Fast-lio1 F
  Fx.block<3, 3>(3, 3) = so3Exp(-1 * imu_gyr * dt);  // R->R
  //  Fx.block<3, 3>(3, 15) = -1 * A_T(-imu_gyr * dt) * dt;      // R->bg
  Fx.block<3, 3>(12, 3) =
      rotation * skewSymmetric(imu_acc) * dt * (-1);          // v->R
  Fx.block<3, 3>(12, 18) = rotation * dt * (-1);              // v->ba
  Fx.block<3, 3>(12, 21) = Eigen::Matrix3d::Identity() * dt;  // v->g

  // 18state  R p v bg ba g
  //  Fx.block<3, 3>(0, 0) = so3Exp(-1 * imu_gyr * dt);         // R->R
  //  Fx.block<3, 3>(0, 9) = -1 * A_T(-imu_gyr * dt) * dt;      // R->bg
  //  Fx.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity() * dt;  // p->v
  //  Fx.block<3, 3>(6, 0) = rotation * skewSymmetric(imu_acc) * dt * (-1);  //
  //  v->R Fx.block<3, 3>(6, 12) = rotation * dt * (-1);              // v->ba
  //  Fx.block<3, 3>(6, 15) = Eigen::Matrix3d::Identity() * dt;  // v->g

  // Fw.block<3, 3>(3, 0) = -1 * A_T(-imu_gyr * dt) * dt;
  Fw.block<3, 3>(3, 0) = -1 * Eigen::Matrix3d::Identity() * dt;
  Fw.block<3, 3>(12, 3) = -1 * rotation * dt;
  Fw.block<3, 3>(15, 6) = Eigen::Matrix3d::Identity() * dt;
  Fw.block<3, 3>(18, 9) = Eigen::Matrix3d::Identity() * dt;

  P = Fx * P * Fx.transpose() + Fw * Q * Fw.transpose();
  X.cov = P;
}

bool IESKF::update(const std::shared_ptr<ObservationModel>& obs_model) {
  auto x_k_k = X;

  Eigen::MatrixXd K;
  Eigen::MatrixXd H_k;
  Eigen::Matrix<double, 24, 24> P_in_update;
  bool converge = true;
  int t = 0;
  for (int i = 0; i < iter_times; i++) {
    Eigen::Matrix<double, 24, 1> error_state = getErrorState(x_k_k, X);
    Eigen::Matrix<double, 24, 24> J_inv;
    J_inv.setIdentity();
    J_inv.block<3, 3>(3, 3) = A_T(error_state.block<3, 1>(3, 0));
    J_inv.block<3, 3>(9, 9) = A_T(error_state.block<3, 1>(9, 0));

    P_in_update = J_inv * P * J_inv.transpose();

    Eigen::MatrixXd z_k;
    Eigen::MatrixXd R_inv;

    obs_model->compute(x_k_k, z_k, H_k, converge);

    Eigen::MatrixXd H_kt = H_k.transpose();

    // clock_t begin = clock();
    R_inv = Eigen::MatrixXd::Identity(H_k.rows(), H_k.rows()) * 1000;
    K = (H_kt * R_inv * H_k + P_in_update.inverse()).inverse() * H_kt * R_inv;
    // K = (H_kt * H_k + (P_in_update / 0.001).inverse()).inverse() * H_kt;
    // clock_t end = clock();
    // std::cout << "calc K time: " << double(end - begin) / CLOCKS_PER_SEC << "
    // s"
    // << std::endl;

    Eigen::MatrixXd left = -1 * K * z_k;
    Eigen::MatrixXd right =
        -1 * (Eigen::Matrix<double, 24, 24>::Identity() - K * H_k) * J_inv *
        error_state;
    Eigen::MatrixXd update_x = left + right;

    converge = true;
    for (int idx = 0; idx < 24; idx++) {
      if (fabs(update_x(idx, 0)) > thresh) {
        converge = false;
        break;
      }
    }

    x_k_k.position = x_k_k.position + update_x.block<3, 1>(0, 0);
    x_k_k.rotation = Eigen::Quaterniond(x_k_k.rotation.toRotationMatrix() *
                                        so3Exp(update_x.block<3, 1>(3, 0)));
    x_k_k.rotation.normalize();
    x_k_k.offset_T_L_I = x_k_k.offset_T_L_I + update_x.block<3, 1>(6, 0);
    x_k_k.offset_R_L_I =
        Eigen::Quaterniond(x_k_k.offset_R_L_I.toRotationMatrix() *
                           so3Exp(update_x.block<3, 1>(9, 0)));
    x_k_k.offset_R_L_I.normalize();
    x_k_k.velocity = x_k_k.velocity + update_x.block<3, 1>(12, 0);
    x_k_k.bg = x_k_k.bg + update_x.block<3, 1>(15, 0);
    x_k_k.ba = x_k_k.ba + update_x.block<3, 1>(18, 0);
    x_k_k.gravity = x_k_k.gravity + update_x.block<3, 1>(21, 0);

    if (converge && t == 1) {
      break;
    }
    if (converge) {
      t++;
    }
  }
  X = x_k_k;
  P = (Eigen::Matrix<double, 24, 24>::Identity() - K * H_k) * P_in_update;
  X.cov = P;
  return converge;
}

bool IESKF::update(const std::shared_ptr<VoxelMapModel>& voxel_model) {
  auto x_k_k = X;

  Eigen::MatrixXd K;
  Eigen::MatrixXd H_k;
  Eigen::Matrix<double, 24, 24> P_in_update;
  bool converge = true;
  int t = 0;
  for (int i = 0; i < iter_times; i++) {
    Eigen::Matrix<double, 24, 1> error_state = getErrorState(x_k_k, X);
    Eigen::Matrix<double, 24, 24> J_inv;
    J_inv.setIdentity();
    J_inv.block<3, 3>(3, 3) = A_T(error_state.block<3, 1>(3, 0));
    J_inv.block<3, 3>(9, 9) = A_T(error_state.block<3, 1>(9, 0));

    P_in_update = J_inv * P * J_inv.transpose();

    Eigen::MatrixXd z_k;
    Eigen::MatrixXd R_inv;

    auto t1 = std::chrono::high_resolution_clock::now();
    voxel_model->compute(x_k_k, z_k, H_k, R_inv);
    auto t2 = std::chrono::high_resolution_clock::now();
    double duration_ms =
        std::chrono::duration<double, std::milli>(t2 - t1).count();
    std::cout << "[Timing] voxel_map update compute 耗时: " << duration_ms
              << " ms" << std::endl;

    Eigen::MatrixXd H_kt = H_k.transpose();

    // clock_t begin = clock();
    R_inv = Eigen::MatrixXd::Identity(H_k.rows(), H_k.rows()) * 1000;
    // K = (H_kt * R_inv * H_k + P_in_update.inverse()).inverse() * H_kt *
    // R_inv;
    K = (H_kt * H_k + (P_in_update / 0.001).inverse()).inverse() * H_kt;
    // clock_t end = clock();
    // std::cout << "calc K time: " << double(end - begin) / CLOCKS_PER_SEC << "
    // s"
    // << std::endl;

    Eigen::MatrixXd left = -1 * K * z_k;
    Eigen::MatrixXd right =
        -1 * (Eigen::Matrix<double, 24, 24>::Identity() - K * H_k) * J_inv *
        error_state;
    Eigen::MatrixXd update_x = left + right;

    converge = true;
    for (int idx = 0; idx < 24; idx++) {
      if (fabs(update_x(idx, 0)) > thresh) {
        converge = false;
        break;
      }
    }

    x_k_k.position = x_k_k.position + update_x.block<3, 1>(0, 0);
    x_k_k.rotation = Eigen::Quaterniond(x_k_k.rotation.toRotationMatrix() *
                                        so3Exp(update_x.block<3, 1>(3, 0)));
    x_k_k.rotation.normalize();
    x_k_k.offset_T_L_I = x_k_k.offset_T_L_I + update_x.block<3, 1>(6, 0);
    x_k_k.offset_R_L_I =
        Eigen::Quaterniond(x_k_k.offset_R_L_I.toRotationMatrix() *
                           so3Exp(update_x.block<3, 1>(9, 0)));
    x_k_k.offset_R_L_I.normalize();
    x_k_k.velocity = x_k_k.velocity + update_x.block<3, 1>(12, 0);
    x_k_k.bg = x_k_k.bg + update_x.block<3, 1>(15, 0);
    x_k_k.ba = x_k_k.ba + update_x.block<3, 1>(18, 0);
    x_k_k.gravity = x_k_k.gravity + update_x.block<3, 1>(21, 0);

    if (converge && t == 1) {
      break;
    }
    if (converge) {
      t++;
    }
  }
  X = x_k_k;
  P = (Eigen::Matrix<double, 24, 24>::Identity() - K * H_k) * P_in_update;
  X.cov = P;
  return converge;
}

Eigen::Matrix<double, 24, 1> IESKF::getErrorState(const State& state1,
                                                  const State& state2) {
  Eigen::Matrix<double, 24, 1> es;
  es.setZero();
  es.block<3, 1>(0, 0) = state1.position - state2.position;
  es.block<3, 1>(3, 0) = SO3Log(state2.rotation.toRotationMatrix().transpose() *
                                state1.rotation.toRotationMatrix());
  es.block<3, 1>(6, 0) = state1.offset_T_L_I - state2.offset_T_L_I;
  es.block<3, 1>(9, 0) =
      SO3Log(state2.offset_R_L_I.toRotationMatrix().transpose() *
             state1.offset_R_L_I.toRotationMatrix());
  es.block<3, 1>(12, 0) = state1.velocity - state2.velocity;
  es.block<3, 1>(15, 0) = state1.bg - state2.bg;
  es.block<3, 1>(18, 0) = state1.ba - state2.ba;
  es.block<3, 1>(21, 0) = state1.gravity - state2.gravity;
  return es;
}

const State& IESKF::getX() { return X; }

void IESKF::setX(const State& x_in) { X = x_in; }
}  // namespace ieskf_fusion