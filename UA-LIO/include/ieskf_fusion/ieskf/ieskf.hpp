#ifndef IESKF_FUSION_IESKF_HPP
#define IESKF_FUSION_IESKF_HPP

#include "ieskf_fusion/math/SO3.hpp"
#include "ieskf_fusion/modules/observationModel/Po2PlModel.hpp"
#include "ieskf_fusion/modules/observationModel/voxelmapModel.hpp"
#include "ieskf_fusion/type/imu.hpp"
#include "ieskf_fusion/type/state.hpp"

namespace ieskf_fusion {
class IESKF {
 private:
  State X;
  Eigen::Matrix<double, 24, 24> P;
  Eigen::Matrix<double, 12, 12> Q;
  int iter_times = 4;
  double thresh;

 public:
  using Ptr = std::shared_ptr<IESKF>;
  IESKF(double eps, double cov_gyroscope, double cov_acceleration,
        double cov_bias_acceleration, double cov_bias_gyroscope);
  ~IESKF();

  void predict(IMU& imu, double dt);
  bool update(const std::shared_ptr<ObservationModel>& obs_model);
  bool update(const std::shared_ptr<VoxelMapModel>& voxel_model);
  const State& getX();
  void setX(const State& x_in);
  Eigen::Matrix<double, 24, 1> getErrorState(const State& state1,
                                             const State& state2);
};
}  // namespace ieskf_fusion

#endif