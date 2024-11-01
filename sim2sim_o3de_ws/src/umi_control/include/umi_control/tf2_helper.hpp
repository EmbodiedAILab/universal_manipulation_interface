#include <cmath>
#include <string>
#include <geometry_msgs/msg/pose.hpp>
#include <Eigen/Geometry>

namespace tf2_helper
{
    void PrintIsometry(const Eigen::Isometry3d &matrix)
    {
        // 提取平移部分
        Eigen::Vector3d translation = matrix.translation();

        // 提取旋转部分（四元数）
        Eigen::Quaterniond rotation(matrix.rotation());

        // 使用 AngleAxis 计算欧拉角
        Eigen::Vector3d euler_angles = rotation.toRotationMatrix().eulerAngles(0, 1, 2); // ZYX顺序

        // 打印结果
        std::cout << "Translation: ["
                  << translation.x() << ", "
                  << translation.y() << ", "
                  << translation.z() << "]" << std::endl;

        std::cout << "Rotation (Quaternion): ["
                  << rotation.x() << ", "
                  << rotation.y() << ", "
                  << rotation.z() << ", "
                  << rotation.w() << "]" << std::endl;

        std::cout << "Rotation (Euler angles): ["
                  << euler_angles.x() << ", "
                  << euler_angles.y() << ", "
                  << euler_angles.z() << "]" << std::endl;
    }

    geometry_msgs::msg::Pose MatrixToPose(const Eigen::Isometry3d &matrix)
    {
        geometry_msgs::msg::Pose pose;

        pose.position.x = matrix.translation().x();
        pose.position.y = matrix.translation().y();
        pose.position.z = matrix.translation().z();

        Eigen::Quaterniond rotation(matrix.rotation());
        pose.orientation.x = rotation.x();
        pose.orientation.y = rotation.y();
        pose.orientation.z = rotation.z();
        pose.orientation.w = rotation.w();

        return pose;
    }

    Eigen::Isometry3d PoseToMatrix(const geometry_msgs::msg::Pose &pose)
    {
        Eigen::Isometry3d matrix = Eigen::Isometry3d::Identity();

        // 设置平移
        matrix.translation() << pose.position.x, pose.position.y, pose.position.z;

        // 从四元数创建旋转矩阵
        Eigen::Quaterniond rotation(
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z);

        // 直接设置旋转部分
        matrix.linear() = rotation.toRotationMatrix();

        return matrix;
    }
} // namespace tf_geo
