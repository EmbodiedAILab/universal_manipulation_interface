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
#include <geometry_msgs/msg/twist.hpp>

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
    RCLCPP_ERROR(this->get_logger(), "base frame %s", baseFrame_.c_str());
    RCLCPP_ERROR(this->get_logger(), "eef frame %s", eefFrame_.c_str());

    jointCommandPub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("joint_servo_controller", 10);
    eefCommandSub_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "eef_servo_controller", 10, std::bind(&CartersianController::EEFCommandCallback, this, std::placeholders::_1));
    preTime_ = this->get_clock()->now();
  }

private:
  const std::string GROUP_NAME_ = "arm";
  robot_model_loader::RobotModelLoader robotModelLoader_;
  const moveit::core::RobotModelPtr &robotModel_;
  moveit::core::RobotStatePtr robotState_;
  const moveit::core::JointModelGroup *jointModelGroup_;
  moveit::planning_interface::MoveGroupInterface moveGroupInterface_;

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
      initialized_ = true;
      RCLCPP_INFO(this->get_logger(), "Initialize robot state");
    }

    double duration = (this->get_clock()->now() - preTime_).seconds();
    if (duration > MAX_TIMEOUT_)
    {
      RCLCPP_INFO(this->get_logger(), "Set duration to maximum duration");
      duration = MAX_TIMEOUT_;
    }

    auto currentPose = moveGroupInterface_.getCurrentPose().pose;
    geometry_msgs::msg::Twist twist = GetTwist(currentPose, *msg, duration);
    TransformTwist(twist);
    if (!robotState_->setFromDiffIK(jointModelGroup_, twist, eefFrame_, duration))
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to get IK");
    }

    std_msgs::msg::Float64MultiArray jointCommandMsg;
    robotState_->copyJointGroupPositions(jointModelGroup_, jointCommandMsg.data);
    jointCommandPub_->publish(jointCommandMsg);

    preTime_ = this->get_clock()->now();
  }

  void TransformTwist(geometry_msgs::msg::Twist& twist)
  {
    std::vector<double> currentJointValues = moveGroupInterface_.getCurrentJointValues();

    robotState_->setJointGroupPositions(jointModelGroup_, currentJointValues);
    Eigen::Matrix3d eef2BaseRotationMatrix = robotState_->getFrameTransform(eefFrame_).rotation();
    Eigen::Vector3d linear, angular;
    linear.x() = twist.linear.x;
    linear.y() = twist.linear.y;
    linear.z() = twist.linear.z;
    angular.x() = twist.angular.x;
    angular.y() = twist.angular.y;
    angular.z() = twist.angular.z;
    linear = eef2BaseRotationMatrix.transpose() * linear;
    angular = eef2BaseRotationMatrix.transpose() * angular;
    twist.linear.x = linear.x();
    twist.linear.y = linear.y();
    twist.linear.z = linear.z();
    twist.angular.x = angular.x();
    twist.angular.y = angular.y();
    twist.angular.z = angular.z();   
  }

  geometry_msgs::msg::Twist GetTwist(const geometry_msgs::msg::Pose &startPose, const geometry_msgs::msg::Pose &targetPose, const double duration)
  {
    geometry_msgs::msg::Twist twist;

    // 计算位置差
    double dx = targetPose.position.x - startPose.position.x;
    double dy = targetPose.position.y - startPose.position.y;
    double dz = targetPose.position.z - startPose.position.z;

    // 计算线速度（x, y, z 方向）
    twist.linear.x = dx / duration; // x 方向线速度
    twist.linear.y = dy / duration; // y 方向线速度
    twist.linear.z = dz / duration; // z 方向线速度

    // 从四元数计算起始和目标的朝向（roll, pitch, yaw）
    double startRoll, startPitch, startYaw;
    tf2::Quaternion startQuat(
        startPose.orientation.x,
        startPose.orientation.y,
        startPose.orientation.z,
        startPose.orientation.w);
    tf2::Matrix3x3(startQuat).getRPY(startRoll, startPitch, startYaw);

    double targetRoll, targetPitch, targetYaw;
    tf2::Quaternion targetQuat(
        targetPose.orientation.x,
        targetPose.orientation.y,
        targetPose.orientation.z,
        targetPose.orientation.w);
    tf2::Matrix3x3(targetQuat).getRPY(targetRoll, targetPitch, targetYaw);

    // 计算角速度（roll, pitch, yaw）
    double rollDiff = targetRoll - startRoll;
    double pitchDiff = targetPitch - startPitch;
    double yawDiff = targetYaw - startYaw;

    // 确保角度在 -π 到 π 之间
    while (yawDiff > M_PI)
      yawDiff -= 2 * M_PI;
    while (yawDiff < -M_PI)
      yawDiff += 2 * M_PI;

    while (pitchDiff > M_PI)
      pitchDiff -= 2 * M_PI;
    while (pitchDiff < -M_PI)
      pitchDiff += 2 * M_PI;

    while (rollDiff > M_PI)
      rollDiff -= 2 * M_PI;
    while (rollDiff < -M_PI)
      rollDiff += 2 * M_PI;

    // 计算角速度
    twist.angular.x = rollDiff / duration;  // roll 角速度
    twist.angular.y = pitchDiff / duration; // pitch 角速度
    twist.angular.z = yawDiff / duration;   // yaw 角速度

    return twist;
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