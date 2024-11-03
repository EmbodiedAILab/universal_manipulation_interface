#include <memory>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <umi_control/tf2_helper.hpp>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto const node = std::make_shared<rclcpp::Node>(
        "arm_eef_controller_test",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() {
        executor.spin();
    }).detach();

    auto const logger = rclcpp::get_logger("arm_eef_controller_test");

    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto moveGroupInterface = MoveGroupInterface(node, "arm");
    auto robotState = moveGroupInterface.getCurrentState();

    Eigen::Isometry3d base2WorldMatrix = robotState->getFrameTransform("base");

    auto eefCommandPub_ = node->create_publisher<geometry_msgs::msg::Pose>("eef_servo_controller", 10);
    auto refPose = moveGroupInterface.getCurrentPose().pose;
    rclcpp::Time initTime = node->get_clock()->now();

    geometry_msgs::msg::Pose targetPose = refPose;
    rclcpp::Rate loop_rate(30); // 设置循环频率为 30 Hz
    while (rclcpp::ok()) // 检查节点是否仍然运行
    {
        double duration = (node->get_clock()->now() - initTime).seconds();
        targetPose.position.y -= 0.01 * sin(2 * duration);
        // 确保发送的位姿是末端相对于base的
        // 后期可以将参数改成可配置的
        const auto toolLink2BasePose = tf2_helper::MatrixToPose(base2WorldMatrix.inverse() * tf2_helper::PoseToMatrix(targetPose));
        eefCommandPub_->publish(toolLink2BasePose);
        loop_rate.sleep(); // 控制发布频率
    }

    rclcpp::shutdown();
    return 0;
}