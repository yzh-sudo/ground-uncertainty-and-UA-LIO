#include "ieskf_fusion/ieskf_fusion_flow.hpp"

using namespace ieskf_fusion;

int main(int argc, char** argv) {
  ros::init(argc, argv, "ieskf_fusion_node");

  ros::NodeHandle nh;

  std::shared_ptr<IESKFFusionFlow> ieskf_slam =
      std::make_shared<IESKFFusionFlow>(nh);
  ieskf_slam->run();

  return 0;
}