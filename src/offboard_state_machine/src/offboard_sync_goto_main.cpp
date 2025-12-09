#include "offboard_state_machine/offboard_sync_goto_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto param_reader = std::make_shared<rclcpp::Node>("sync_goto_param_reader");
  const int drone_id = param_reader->declare_parameter<int>("drone_id", 0);

  RCLCPP_INFO(param_reader->get_logger(),
              "Starting Offboard Sync GOTO node for drone %d", drone_id);

  auto node = std::make_shared<OffboardSyncGoto>(drone_id);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
