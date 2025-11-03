#ifndef IESKF_FUSION_MEASURE_GROUP_HPP
#define IESKF_FUSION_MEASURE_GROUP_HPP

#include <deque>

#include "ieskf_fusion/type/imu.hpp"
#include "ieskf_fusion/type/pointcloud.hpp"

namespace ieskf_fusion {
struct MeasureGroup {
  double lidar_beg_time;
  double lidar_end_time;
  std::deque<IMU> imus;
  PointCloud cloud;
};
}  // namespace ieskf_fusion

#endif