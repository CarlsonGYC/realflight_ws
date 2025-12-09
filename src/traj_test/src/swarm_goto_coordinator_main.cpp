#include "traj_test/swarm_goto_coordinator_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto param_reader = std::make_shared<rclcpp::Node>("swarm_goto_param_reader");
  const int total_drones = param_reader->declare_parameter<int>("total_drones", 1);

  auto node = std::make_shared<SwarmGotoCoordinator>(total_drones);
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
