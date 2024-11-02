#include <memory>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <moveit/move_group_interface/move_group_interface.h>

class CartersianController : public rclcpp::Node
{
public:
  CartersianController(rclcpp::Node::SharedPtr node)
  : Node("cartersian_control"), moveGroupInterface_(node, GROUP_NAME)
  {
    RCLCPP_ERROR(this->get_logger(), "You are in the constructor");

    robotState_ = moveGroupInterface_.getCurrentState();

    RCLCPP_INFO(this->get_logger(), "Planning success!");

    auto targetPose = moveGroupInterface_.getCurrentPose().pose;
    targetPose.position.z -= 0.1;
    moveGroupInterface_.setPoseTarget(targetPose);

    // Create a plan to that target pose
    auto const [success, plan] = [this] {
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(moveGroupInterface_.plan(msg));
      return std::make_pair(ok, msg);
    }();

    // Execute the plan
    if(success) {
      moveGroupInterface_.execute(plan);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Planning failed!");
    }

    auto currentPose = moveGroupInterface_.getCurrentPose();
    double error = pow(targetPose.position.x - currentPose.pose.position.x, 2) + 
                   pow(targetPose.position.y - currentPose.pose.position.y, 2) + 
                   pow(targetPose.position.z - currentPose.pose.position.z, 2);
    std::cout << "error: " << sqrt(error) * 1000 << "mm" << std::endl;
  }

private:
  const std::string GROUP_NAME = "arm";
  moveit::planning_interface::MoveGroupInterface moveGroupInterface_;
  moveit::core::RobotStatePtr robotState_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "arm_moveit_controller",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() {
      executor.spin();
  }).detach();

  CartersianController controller(node);

  rclcpp::shutdown();
  return 0;
}