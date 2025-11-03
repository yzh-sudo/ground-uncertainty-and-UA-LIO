#ifndef IESKF_FUSION_IESKF_FUSION_FLOW_HPP
#define IESKF_FUSION_IESKF_FUSION_FLOW_HPP

#include <condition_variable>
#include <deque>
#include <thread>

#include "ieskf_fusion/modules/frontend/frontend.hpp"
#include "ieskf_fusion/modules/publisher/path_publisher.hpp"
#include "ieskf_fusion/modules/publisher/pointcloud2_publisher.hpp"
#include "ieskf_fusion/modules/publisher/tf_broadcaster.hpp"
#include "ieskf_fusion/modules/subscriber/imu_subscriber.hpp"
#include "ieskf_fusion/modules/subscriber/lidar_subscriber.hpp"
#include "ieskf_fusion/type/measure_group.hpp"

namespace ieskf_fusion {
class IESKFFusionFlow {
 public:
  IESKFFusionFlow(ros::NodeHandle& nh);
  ~IESKFFusionFlow() {
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      stop_flag_ = true;
    }
    cond_var_.notify_all();
    if (processing_thread_.joinable()) {
      processing_thread_.join();
    }
  }

  void run();

 private:
  ros::NodeHandle node_;

  std::shared_ptr<FrontEnd> front_end_ptr_;

  std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
  std::shared_ptr<LidarSubscriber> lidar_sub_ptr_;

  std::shared_ptr<PointCloudPublisher> local_map_pub_ptr;
  std::shared_ptr<PointCloudPublisher> map_pub_ptr;
  std::shared_ptr<PointCloudPublisher> scan_pub_ptr;
  std::shared_ptr<OdometryPublisher> odom_pub_ptr;
  std::shared_ptr<PathPublisher> path_pub_ptr;
  std::shared_ptr<TFBroadCaster> tf_pub_ptr;

  std::deque<IMU> imu_data_buff_;
  std::deque<PointCloud> lidar_data_buff_;
  std::deque<MeasureGroup> measure_group_data_buff_;

  std::mutex queue_mutex_;
  std::condition_variable cond_var_;
  std::thread processing_thread_;
  bool stop_flag_ = false;
  double lidar_timestamp = 0.0;

  void process();
  void publishMsg();
  bool getSensorData();
  bool syncMeasureGroup(MeasureGroup& measure_group);
};
}  // namespace ieskf_fusion

#endif