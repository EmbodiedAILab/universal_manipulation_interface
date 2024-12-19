#include <memory>
#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class GripperController : public rclcpp::Node
{
public:
    GripperController()
    : Node("gripper_control")
    {
        gripperActionClient_ = rclcpp_action::create_client<control_msgs::action::GripperCommand>(this, "hand_controller/gripper_cmd");

        // 等待 Action 服务器可用
        while (!gripperActionClient_->wait_for_action_server(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for the action server...");
        }

        gripperCmdSub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/gripper_cmd", 10, std::bind(&GripperController::GripperCmdCallback, this, std::placeholders::_1));

        RCLCPP_WARN(this->get_logger(), "Ready to take new gripper command!");
    }

private:
    void GripperCmdCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // 创建 gripper 命令消息
        auto goal_msg = control_msgs::action::GripperCommand::Goal();
        auto width = msg->position[0] / 2;
        goal_msg.command.position = width;
        
        // 发送 gripper 命令
        gripperActionClient_->async_send_goal(goal_msg);
        RCLCPP_INFO(this->get_logger(), "Sending gripper command: %f", width);
    }

    rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr gripperActionClient_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr gripperCmdSub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GripperController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}