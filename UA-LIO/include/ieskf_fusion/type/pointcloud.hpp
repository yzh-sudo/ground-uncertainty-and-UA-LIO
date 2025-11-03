#ifndef IESKF_FUSION_POINT_CLOUD_HPP
#define IESKF_FUSION_POINT_CLOUD_HPP

#include "ieskf_fusion/type/point.hpp"

namespace ieskf_fusion {
struct PointCloud {
  pcl::PointCloud<PointType>::Ptr cloud_ptr;
  double time_stamp;
  double scan_time;
  PointCloud() {
    cloud_ptr = pcl::make_shared<pcl::PointCloud<PointType>>();
    time_stamp = 0.0;
    scan_time = 0.0;
  }
};
}  // namespace ieskf_fusion

#endif