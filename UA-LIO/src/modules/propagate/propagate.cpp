#include "ieskf_fusion/modules/propagate/propagate.hpp"

namespace ieskf_fusion {
Propagate::Propagate() {}

Propagate::~Propagate() = default;

void Propagate::Propagation(
    MeasureGroup& measures, IESKF::Ptr ieskf_ptr,
    std::shared_ptr<OdometryPublisher> predict_odom_pub_pt) {
  ForwardPropagation(measures, std::move(ieskf_ptr), predict_odom_pub_pt);
}

void Propagate::ForwardPropagation(
    MeasureGroup& measures, IESKF::Ptr ieskf_ptr,
    std::shared_ptr<OdometryPublisher> predict_odom_pub_ptr) {
  auto imus = measures.imus;
  imus.push_front(last_imu);
  double dt = 0;
  IMU imu_in;
  State last_state = ieskf_ptr->getX();
  State imu_state = ieskf_ptr->getX();
  const double& lidar_beg_time = measures.lidar_beg_time;
  const double& lidar_end_time = measures.lidar_end_time;
  IMUpose.clear();
  // acc_last ang_last?
  IMUpose.emplace_back(0.0, acc_last, ang_last, imu_state.velocity,
                       imu_state.position, imu_state.rotation);
  for (auto it_imu = imus.begin(); it_imu < (imus.end() - 1); it_imu++) {
    const auto& head = it_imu;
    const auto& tail = it_imu + 1;
    Eigen::Vector3d gyr_avr = 0.5 * (head->gyroscope + tail->gyroscope);
    Eigen::Vector3d acc_avr = 0.5 * (head->acceleration + tail->acceleration);
    acc_avr = imu_scale * acc_avr;
    dt = tail->time_stamp - head->time_stamp;
    imu_in.acceleration = acc_avr;
    imu_in.gyroscope = gyr_avr;
    ieskf_ptr->predict(imu_in, dt);
    imu_state = ieskf_ptr->getX();
    Eigen::Matrix4f pre_pose = Eigen::Matrix4f::Identity();
    pre_pose.block<3, 3>(0, 0) =
        imu_state.rotation.toRotationMatrix().cast<float>();
    pre_pose.block<3, 1>(0, 3) = imu_state.position.cast<float>();
    double pre_time = 0.5 * (tail->time_stamp + head->time_stamp);
    predict_odom_pub_ptr->Publish(pre_pose, pre_time);

    ang_last = gyr_avr - imu_state.bg;
    acc_last = imu_state.rotation * (acc_avr - imu_state.ba);
    for (int i = 0; i < 3; i++) {
      acc_last[i] += imu_state.gravity[i];
    }
    double imu_time = tail->time_stamp - lidar_beg_time;
    IMUpose.emplace_back(imu_time, acc_last, ang_last, imu_state.velocity,
                         imu_state.position, imu_state.rotation);
  }
  dt = measures.lidar_end_time - imus.back().time_stamp;
  ieskf_ptr->predict(imu_in, dt);
  imu_state = ieskf_ptr->getX();

  last_imu = measures.imus.back();
  last_imu.time_stamp = measures.lidar_end_time;

  Eigen::Vector3d distance = imu_state.position - last_state.position;
  pre_dis = distance.norm();

  BackPropagation(measures, std::move(ieskf_ptr));
}

void Propagate::BackPropagation(MeasureGroup& measures, IESKF::Ptr ieskf_ptr) {
  State last_imu_state = ieskf_ptr->getX();
  std::sort(measures.cloud.cloud_ptr->begin(), measures.cloud.cloud_ptr->end(),
            [](PointType x, PointType y) { return x.timestamp < y.timestamp; });

  Eigen::Vector3d ang_avr, acc_avr, acc_imu, vel_imu, pos_imu;
  Eigen::Matrix3d rot_imu;
  auto& pcl_out = *measures.cloud.cloud_ptr;

  if (pcl_out.points.begin() == pcl_out.points.end()) {
    return;
  }
  auto it_pcl = pcl_out.points.end() - 1;
  int i = pcl_out.points.size();
  for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--) {
    auto head = it_kp - 1;
    auto tail = it_kp;
    rot_imu = head->rot.toRotationMatrix();
    vel_imu = head->vel;
    pos_imu = head->pos;
    acc_imu = tail->acc;
    ang_avr = tail->ang;

    for (; it_pcl->timestamp / double(1000) > head->time; it_pcl--) {
      double dt = it_pcl->timestamp / double(1000) - head->time;

      Eigen::Matrix3d R_i(rot_imu * so3Exp(ang_avr * dt));
      Eigen::Vector3d P_i(it_pcl->x, it_pcl->y, it_pcl->z);
      Eigen::Vector3d T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt -
                           last_imu_state.position);

      Eigen::Vector3d P_compensate =
          last_imu_state.offset_R_L_I.conjugate().toRotationMatrix() *
          (last_imu_state.rotation.conjugate().toRotationMatrix() *
               (R_i * (last_imu_state.offset_R_L_I.toRotationMatrix() * P_i +
                       last_imu_state.offset_T_L_I) +
                T_ei) -
           last_imu_state.offset_T_L_I);

      it_pcl->x = P_compensate(0);
      it_pcl->y = P_compensate(1);
      it_pcl->z = P_compensate(2);

      if (it_pcl == pcl_out.points.begin()) {
        break;
      }
    }
  }
}

double Propagate::getPredictDis() { return pre_dis; }
}  // namespace ieskf_fusion