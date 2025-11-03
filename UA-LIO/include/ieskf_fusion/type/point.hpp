#ifndef IESKF_FUSION_POINT_HPP
#define IESKF_FUSION_POINT_HPP

#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/impl/pcl_base.hpp>

namespace ieskf_fusion {
struct EIGEN_ALIGN16 PointType {
  PCL_ADD_POINT4D;
  float intensity;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct EIGEN_ALIGN16 ousterPoint {
  PCL_ADD_POINT4D;
  float intensity;
  uint32_t t;
  uint16_t reflectivity;
  uint8_t ring;
  uint16_t ambient;
  uint32_t range;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace ieskf_fusion

POINT_CLOUD_REGISTER_POINT_STRUCT(ieskf_fusion::PointType, (float, x, x)  // x
                                  (float, y, y)                           // y
                                  (float, z, z)                           // z
                                  (float, intensity, intensity)    // intensity
                                  (double, timestamp, timestamp))  // timestamp

POINT_CLOUD_REGISTER_POINT_STRUCT(
    ieskf_fusion::ousterPoint, (float, x, x)(float, y, y)(float, z, z)  // x y z
    (float, intensity, intensity)                                // intensity
    (std::uint32_t, t, t)                                        // t
    (std::uint16_t, reflectivity, reflectivity)                  // reflectivity
    (std::uint8_t, ring, ring)(std::uint16_t, ambient, ambient)  // ring ambient
    (std::uint32_t, range, range))                               // range

#endif