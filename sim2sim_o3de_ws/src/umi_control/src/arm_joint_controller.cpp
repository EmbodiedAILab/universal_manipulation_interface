#include <memory>
#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class ActionController : public rclcpp::Node
{
public:
    ActionController()
    : Node("action_control")
    {
        currentJointStateMsg_.name = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
        actionClient_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(this, "arm_controller/follow_joint_trajectory");

        // 等待 Action 服务器可用
        while (!actionClient_->wait_for_action_server(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for the action server...");
        }

        // 订阅joint_states
        jointStateSub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, std::bind(&ActionController::JointStateCallback, this, std::placeholders::_1));
        
        // 订阅控制接口
        jointCommandSub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "joint_servo_controller", 10, std::bind(&ActionController::JointCommandCallback, this, std::placeholders::_1));

        preTime_ = this->get_clock()->now();
        RCLCPP_WARN(this->get_logger(), "Ready to take new joint command!");
    }

private:
    void JointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::vector<double> jointValues;
        for (size_t i = 0; i < currentJointStateMsg_.name.size(); ++i)
        {
            for (size_t j = 0; j < msg->name.size(); ++j)
            {
                if (currentJointStateMsg_.name[i] == msg->name[j])
                {
                    jointValues.push_back(msg->position[j]);
                }
            }
        }
        currentJointStateMsg_.position = jointValues;

        if (!jointStateFlag_)
        {
            RCLCPP_INFO(this->get_logger(), "Received joint states:");
            jointStateFlag_ = true;
        }
    }

    void JointCommandCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (jointStateFlag_ == false)
        {
            RCLCPP_ERROR(this->get_logger(), "Didn't receive joint states!");
            return;
        }
        if (msg->data.size() != currentJointStateMsg_.name.size())
        {
            RCLCPP_ERROR(this->get_logger(), "Joint size is wrong!");
            return;  
        }
        double duration = (this->get_clock()->now() - preTime_).seconds();
        if (duration > MAX_TIMEOUT_)
        {
          duration = MAX_TIMEOUT_;
        }
        // 创建目标轨迹消息
        auto goal_msg = control_msgs::action::FollowJointTrajectory::Goal();
        goal_msg.trajectory = constructTrajectory(msg->data, duration);
        actionClient_->async_send_goal(goal_msg);

        preTime_ = this->get_clock()->now();
    }
    
    trajectory_msgs::msg::JointTrajectory constructTrajectory(const std::vector<double>& targetJoints_, const double deltaT)
    {
        trajectory_msgs::msg::JointTrajectory res;
        res.joint_names = currentJointStateMsg_.name;

        res.points.resize(2);
        res.points[0].positions = currentJointStateMsg_.position;
        res.points[0].velocities.resize(currentJointStateMsg_.name.size(), 0);
        res.points[0].time_from_start = rclcpp::Duration(0, 0);

        res.points[1].positions = targetJoints_;
        res.points[1].velocities.resize(currentJointStateMsg_.name.size(), 0);
        res.points[1].time_from_start = rclcpp::Duration::from_seconds(deltaT);
        return res;
    }

    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr actionClient_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointStateSub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr jointCommandSub_;
    rclcpp::Time preTime_;
    bool jointStateFlag_ = false;
    sensor_msgs::msg::JointState currentJointStateMsg_;
    const double MAX_TIMEOUT_ = 0.2;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ActionController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}