#include "traj_test/traj_test_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  // Get drone ID from command line or use default
  int drone_id = 0;
  if (argc > 1) {
    drone_id = std::atoi(argv[1]);
  }
  
  auto node = std::make_shared<TrajTestNode>(drone_id);
  
  RCLCPP_INFO(node->get_logger(), 
              "Starting trajectory test node for drone %d", drone_id);
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  
  return 0;
}