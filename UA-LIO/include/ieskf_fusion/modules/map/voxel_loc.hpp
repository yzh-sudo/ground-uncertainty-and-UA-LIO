#ifndef IESKF_FUSION_VOXEL_LOC_HPP
#define IESKF_FUSION_VOXEL_LOC_HPP

#include <cstdint>
#include <functional>

#define HASH_P 116101
#define MAX_N 10000000000

class VOXEL_LOC {
 public:
  int64_t x, y, z;

  VOXEL_LOC(int64_t pose_x = 0, int64_t pose_y = 0, int64_t pose_z = 0)
      : x(pose_x), y(pose_y), z(pose_z) {}

  bool operator==(const VOXEL_LOC& other) const {
    return (x == other.x && y == other.y && z == other.z);
  }
};

// Hash value
namespace std {
template <>
struct hash<VOXEL_LOC> {
  int64_t operator()(const VOXEL_LOC& pose) const {
    using std::hash;
    using std::size_t;
    return ((((pose.z) * HASH_P) % MAX_N + (pose.y)) * HASH_P) % MAX_N +
           (pose.x);
  }
};
}  // namespace std

#endif