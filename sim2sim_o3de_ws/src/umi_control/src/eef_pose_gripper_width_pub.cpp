#include <memory>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <umi_control/tf2_helper.hpp>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto const node = std::make_shared<rclcpp::Node>(
        "eef_pose_gripper_width_pub",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() {
        executor.spin();
    }).detach();

    auto const logger = rclcpp::get_logger("eef_pose_gripper_width_pub");

    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    const std::string ARM_GROUP = "arm";
    const std::string HAND_GROUP = "hand";
    auto moveGroupInterface = MoveGroupInterface(node, ARM_GROUP);
    auto robotState = moveGroupInterface.getCurrentState();

    std::vector<double> handJointValues;
    std_msgs::msg::Float64 gripperWidth;
    


    Eigen::Isometry3d base2WorldMatrix = robotState->getFrameTransform("base");

    auto eefPosePub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("eef_pose", 10);
    auto gripperWidthPub_ = node->create_publisher<std_msgs::msg::Float64>("gripper_width", 10);

    rclcpp::Rate loop_rate(50); // 设置循环频率为 50 Hz
    while (rclcpp::ok()) 
    {
        robotState = moveGroupInterface.getCurrentState();              // 更新robotState
        const auto currentPose = moveGroupInterface.getCurrentPose().pose;    // 获取末端相对于基座(world)的变换   
        const auto toolLink2BasePose = tf2_helper::MatrixToPose(base2WorldMatrix.inverse() * tf2_helper::PoseToMatrix(currentPose));

        robotState->copyJointGroupPositions(HAND_GROUP, handJointValues);
        if (handJointValues.size() != 2)
        {
            RCLCPP_INFO(logger, "Gripper joint size is wrong");
            return -1;
        }
        gripperWidth.data = handJointValues[0] + handJointValues[1];
        
        geometry_msgs::msg::PoseStamped currentPoseStamped;
        currentPoseStamped.header.pose = toolLink2BasePose;
        eefPosePub_->publish(currentPoseStamped);
        gripperWidthPub_->publish(gripperWidth);
        loop_rate.sleep(); // 控制发布频率
    }

    rclcpp::shutdown();
    return 0;
}