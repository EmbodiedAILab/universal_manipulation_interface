#include <memory>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Geometry>
#include <umi_control/tf2_helper.hpp>

using namespace std;

class CartersianController : public rclcpp::Node
{
public:
  CartersianController(rclcpp::Node::SharedPtr node)
      : Node("cartersian_control"),
        robotModelLoader_(node),
        robotModel_(robotModelLoader_.getModel()),
        robotState_(new moveit::core::RobotState(robotModel_)),
        jointModelGroup_(robotModel_->getJointModelGroup(GROUP_NAME_)),
        moveGroupInterface_(node, GROUP_NAME_)
  {
    robotState_->setToDefaultValues();
    baseFrame_ = moveGroupInterface_.getPoseReferenceFrame();
    eefFrame_ = moveGroupInterface_.getEndEffectorLink();
    RCLCPP_INFO(this->get_logger(), "base frame %s", baseFrame_.c_str());
    RCLCPP_INFO(this->get_logger(), "eef frame %s", eefFrame_.c_str());

    jointCommandPub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("joint_servo_controller", 10);
    eefCommandSub_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "eef_servo_controller", 10, std::bind(&CartersianController::EEFCommandCallback, this, std::placeholders::_1));
    preTime_ = this->get_clock()->now();
    RCLCPP_WARN(this->get_logger(), "Ready to take new end effector command!");
  }

private:
  const std::string GROUP_NAME_ = "arm";  // 机械臂的规划组
  const std::string BASE_FRAME_ = "base"; // 传过来的位姿一定是相对该坐标系的
  robot_model_loader::RobotModelLoader robotModelLoader_;
  const moveit::core::RobotModelPtr &robotModel_;
  moveit::core::RobotStatePtr robotState_;
  const moveit::core::JointModelGroup *jointModelGroup_;
  moveit::planning_interface::MoveGroupInterface moveGroupInterface_;

  Eigen::Isometry3d base2WorldMatrix_;
  string eefFrame_;
  string baseFrame_;
  bool initialized_ = false;
  rclcpp::Time preTime_;
  const double MAX_TIMEOUT_ = 0.2;

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr eefCommandSub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr jointCommandPub_;

  void EEFCommandCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    if (initialized_ == false)
    {
      robotState_ = moveGroupInterface_.getCurrentState();
      base2WorldMatrix_ = robotState_->getFrameTransform(BASE_FRAME_);
      tf2_helper::PrintIsometry(base2WorldMatrix_);
      initialized_ = true;
      RCLCPP_INFO(this->get_logger(), "Initialize robot state");
    }

    double duration = (this->get_clock()->now() - preTime_).seconds();
    if (duration > MAX_TIMEOUT_)
    {
      RCLCPP_INFO(this->get_logger(), "Set duration to maximum duration");
      duration = MAX_TIMEOUT_;
    }

    // 测试下来发现，在moveit2中，直接计算IK也比较稳定，不会出现IK跳变的问题
    // 但是在ROS1中，会出现跳变的问题。因此在ROS1中推荐使用setFromDiffIK
    // 此外，在ROS2中也测试过setFromDiffIK，但是因为O3DE中的机器人关节运动会存在扰动，因此控制不稳定。
    // 因此，如果未来有使用setFromDiffIK，还需要详细的调试
    const auto toolLink2World = base2WorldMatrix_ * tf2_helper::PoseToMatrix(*msg);
    robotState_->setFromIK(jointModelGroup_, tf2_helper::MatrixToPose(toolLink2World));
    // robotState_->setFromIK(jointModelGroup_, *msg);

    std_msgs::msg::Float64MultiArray jointCommandMsg;
    robotState_->copyJointGroupPositions(jointModelGroup_, jointCommandMsg.data);
    jointCommandPub_->publish(jointCommandMsg);

    preTime_ = this->get_clock()->now();
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "arm_eef_controller",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  auto controller = std::make_shared<CartersianController>(node);
  executor.add_node(controller);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}