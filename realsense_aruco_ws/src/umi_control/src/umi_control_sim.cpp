#include <iostream>
#include <vector>
#include <string>
#include <cmath>

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Core>

#include <ros/ros.h>
#include <tf2/convert.h>
#include <ros/package.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <fstream>
#include <iomanip>
#include <sstream>

#include <netinet/in.h>
#include <arpa/inet.h>
#include <boost/filesystem.hpp>
#include <regex>
#include <unordered_map>
#include <thread>

namespace fs = boost::filesystem;
using namespace std;

struct O3DELinkInfo
{
    string linkName;
    double x;
    double y;
    double z;
    double qw;
    double qx;
    double qy;
    double qz;

    void print()
    {
        cout << "link name " << this->linkName << endl;
        cout << "x: " << this->x << endl;
        cout << "y: " << this->y << endl;
        cout << "z: " << this->z << endl;
        cout << "qw: " << this->qw << endl;
        cout << "qx: " << this->qx << endl;
        cout << "qy: " << this->qy << endl;
        cout << "qz: " << this->qz << endl
             << endl;
    }
};

class UmiController
{
public:
    UmiController() : robotModelLoader_("robot_description"),
                      robotModel_(robotModelLoader_.getModel()),
                      robotState_(new robot_state::RobotState(robotModel_)),
                      jointModelGroup_(robotState_->getJointModelGroup("arm")),
                      moveGroup_("arm"),
                      frameCount_(0.0)
    {
        currentJointValues_.resize(armJointNumber_);
        robotStateSub_ = nh_.subscribe("joint_states", 1, &UmiController::jointStatesCallback, this);
        jointPosPub_ = nh_.advertise<sensor_msgs::JointState>("move_group/fake_controller_joint_states", 1);
        ros::Duration(0.5).sleep();
        while (ros::ok() && !jointStatesFlag_)
        {
            ROS_ERROR_STREAM("Waiting for joint states, and the topic name you specified is " << robotStateSub_.getTopic());
            ros::Duration(1.0).sleep();
        }

        baseFrame_ = moveGroup_.getPoseReferenceFrame();
        std::cout << "base effector: " << baseFrame_ << std::endl;
        eefFrame_ = moveGroup_.getEndEffectorLink();
        std::cout << "end effector: " << eefFrame_ << std::endl;
        jointNames_ = moveGroup_.getVariableNames();
        for (const auto& jointName : jointNames_)
        {
            std::cout << "Joint Name: " << jointName << std::endl;
        }
        linkWithGeos_ = robotModel_->getLinkModelNamesWithCollisionGeometry();
        robotState_ = moveGroup_.getCurrentState();
        moveGroup_.setNamedTarget("ready");
        moveGroup_.move();

        // initalize UDP Server
        setupUDPServer();

        servoLSub_ = nh_.subscribe("servoL_cmd", 1, &UmiController::servoLCallback, this);
        moveJSub_ = nh_.subscribe("moveJ_cmd", 1, &UmiController::moveJCallback, this);
        gripperSub_ = nh_.subscribe("gripper_cmd", 1, &UmiController::gripperCallback, this);

        std::thread commandThread(&UmiController::CommandProcessor, this);
        commandThread.detach();
    }

    void CommandProcessor()
    {
        std::unordered_map<char, std::string> keyToMessage = {
            {'c', "START"},
            {'s', "STOP"},
            {'r', "RESET"}
        };

        while (true)
        {
            char key = getchar();
            auto it = keyToMessage.find(key);
            if (it != keyToMessage.end())
            {
                sendDataToClients(it->second);
                ROS_INFO("%s sent", it->second.c_str());
                std::this_thread::sleep_for(std::chrono::seconds(1)); // 1秒内只发送一次
            }
        }
    }

private:
    // moveit variables
    robot_model_loader::RobotModelLoader robotModelLoader_;
    robot_model::RobotModelPtr robotModel_;
    robot_state::RobotStatePtr robotState_;
    const robot_state::JointModelGroup *jointModelGroup_;
    moveit::planning_interface::MoveGroupInterface moveGroup_;

    // ros communication variables
    ros::NodeHandle nh_;
    ros::Publisher jointPosPub_;
    ros::Subscriber robotStateSub_;
    ros::Subscriber servoLSub_;
    ros::Subscriber moveJSub_;
    ros::Subscriber gripperSub_;
    bool jointStatesFlag_ = false;
    vector<double> currentJointValues_; // 机械臂6个关节角度
    sensor_msgs::JointState jointStatesMsgs_;

    // space mouse control variables
    ros::Subscriber spaceMouseSub_;
    bool spaceMouseFlag_ = false;
    ros::Time lastSpaceMouseTime_;
    geometry_msgs::Twist eefTwist_;
    
    const double transVel_ = 0.15;      // 末端的线速度映射系数
    const double rotatVel_ = 0.3;       // 末端的角速度映射系数
    const double maxTimeout_ = 0.2;     // spaceMouse最大的超时时间，因此注意spaceMouse的数据频率不能太低
    const int armJointNumber_ = 6;      // 机械臂关节数量

    // socket
    int udpSocket_;
    struct sockaddr_in serverAddr_;
    struct sockaddr_in clientAddr_;
    socklen_t clientAddrLen_ = sizeof(clientAddr_);
    
    string baseFrame_;
    string eefFrame_;
    vector<string> jointNames_;
    vector<string> linkWithGeos_;
    double frameCount_;

    double leftFingerPosition_;
    double rightFingerPosition_;
    const double maxFingerPosition_ = 0;     // 最大开合值
    const double minFingerPosition_ = -0.04; // 最小开合值
    const double fingerStep_ = 0.001;         // 每次移动的步长
    double currentLeftFingerJoint_;
    double currentRightFingerJoint_;
    const int frequency=30;
    const double lookahead_time=0;
    const int gain=1;
    int listen_count = 0;

    void setupUDPServer()   
    {
        udpSocket_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (udpSocket_ < 0)
        {
            perror("socket creation failed");
            exit(EXIT_FAILURE);
        }

        int broadcastEnable = 1;
        if (setsockopt(udpSocket_, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable)) < 0)
        {
            perror("setsockopt(SO_BROADCAST) failed");
            exit(EXIT_FAILURE);
        }

        memset(&serverAddr_, 0, sizeof(serverAddr_));
        serverAddr_.sin_family = AF_INET;
        serverAddr_.sin_addr.s_addr = inet_addr("10.78.114.255"); // Subnet broadcast address
        serverAddr_.sin_port = htons(65433);

        ROS_INFO("UDP server set up and broadcasting on port 65433");
    }

    void sendDataToClients(const std::string &data)
    {
        ssize_t sentBytes = sendto(udpSocket_, data.c_str(), data.size(), MSG_CONFIRM,
                                (const struct sockaddr *)&serverAddr_, sizeof(serverAddr_));
        if (sentBytes < 0)
        {
            perror("sendto failed");
        }
    }

    void updateRobotState()
    {
        robotState_->setJointGroupPositions(jointModelGroup_, currentJointValues_);
        robotState_->setJointPositions("left_finger_joint", &currentLeftFingerJoint_);
        robotState_->setJointPositions("right_finger_joint", &currentRightFingerJoint_);
    }

    void jointStatesCallback(const sensor_msgs::JointStateConstPtr &jointStatePtr)
    {
        updateRobotState();

        if (jointStatesFlag_ == false)
        {
            jointStatesFlag_ = true;
            return;
        }
        if (jointStatePtr->position.size() != 8)
        {
            ROS_ERROR("The size of the joint state is wrong");
            return;
        }
        for (int i = 0; i < armJointNumber_; ++i)
        {
            currentJointValues_[i] = jointStatePtr->position[i];
        }
        currentLeftFingerJoint_ = jointStatePtr->position[6];
        currentRightFingerJoint_ = jointStatePtr->position[7];
    }

    // Listen to servoL command and execute
    void servoLCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        // 打印接收到的 pose 信息
        ROS_INFO("Received PoseStamped:");
        ROS_INFO("  Position - x: %f, y: %f, z: %f", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        ROS_INFO("  Orientation - x: %f, y: %f, z: %f, w: %f", msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);

        if (!updateTargetJointPositionsFromPose(msg->pose))
        {
            return;
        }

        jointPosPub_.publish(jointStatesMsgs_);
        sendO3DELinksInfo();
        listen_count++;
        std::cout << "count: " << listen_count << std::endl;
    }
    
    bool updateTargetJointPositionsFromPose(const geometry_msgs::Pose& target_pose)
    {
        const Eigen::Affine3d &current_pose = robotState_->getGlobalLinkTransform(eefFrame_);
        // std::cout << "current_pose: " << current_pose.translation().x() << " " << current_pose.translation().y() << " " << current_pose.translation().z() << std::endl;
        // std::cout << "current_pose rotation: " << current_pose.rotation() << std::endl;

        Eigen::Affine3d desired_pose;
        tf2::fromMsg(target_pose, desired_pose);
        // std::cout << "desired_pose: " << desired_pose.translation().x() << " " << desired_pose.translation().y() << " " << desired_pose.translation().z() << std::endl;
        // std::cout << "desired_pose rotation: " << desired_pose.rotation() << std::endl;

        Eigen::Affine3d pose_diff = current_pose.inverse() * desired_pose;
        // std::cout << "pose diff: " << pose_diff.translation().x() << " " << pose_diff.translation().y() << " " << pose_diff.translation().z() << std::endl;
        // std::cout << "pose diff rotation: " << pose_diff.rotation() << std::endl;

        double duration = 1.0 / this->frequency;
        // std::cout << "duration: " << duration << std::endl;

        // 使用lookahead_time来调整速度，确保在未来的lookahead_time时间内达到目标姿态
        double effective_duration = duration + this->lookahead_time;
        // std::cout << "effective_duration: " << effective_duration << std::endl;

        // 线性速度：将 Eigen::Vector3d 转换为 geometry_msgs::Vector3
        Eigen::Vector3d linear_velocity = pose_diff.translation() / effective_duration;
        eefTwist_.linear.x = linear_velocity.x();
        eefTwist_.linear.y = linear_velocity.y();
        eefTwist_.linear.z = linear_velocity.z();
        // std::cout << "= linear: " << eefTwist_.linear.x << " " << eefTwist_.linear.y << " " << eefTwist_.linear.z << std::endl;

        // 角速度：将 Eigen::Vector3d 转换为 geometry_msgs::Vector3
        Eigen::AngleAxisd angle_axis(pose_diff.rotation());
        Eigen::Vector3d angular_velocity = angle_axis.axis() * angle_axis.angle() / effective_duration;
        eefTwist_.angular.x = angular_velocity.x();
        eefTwist_.angular.y = angular_velocity.y();
        eefTwist_.angular.z = angular_velocity.z();
        // std::cout << "= angular: " << eefTwist_.angular.x << " " << eefTwist_.angular.y << " " << eefTwist_.angular.z << std::endl;

        // 通过增益调整Twist
        eefTwist_.linear.x *= this->gain;
        eefTwist_.linear.y *= this->gain;
        eefTwist_.linear.z *= this->gain;
        eefTwist_.angular.x *= this->gain;
        eefTwist_.angular.y *= this->gain;
        eefTwist_.angular.z *= this->gain;
        // std::cout << "=== linear with gain: " << eefTwist_.linear.x << " " << eefTwist_.linear.y << " " << eefTwist_.linear.z << std::endl;
        // std::cout << "=== angular with gain: " << eefTwist_.angular.x << " " << eefTwist_.angular.y << " " << eefTwist_.angular.z << std::endl;

        // 将速度映射为相对末端
        transformTwist(eefTwist_);
        std::cout << "=== linear transform: " << eefTwist_.linear.x << " " << eefTwist_.linear.y << " " << eefTwist_.linear.z << std::endl;
        std::cout << "=== angular transform: " << eefTwist_.angular.x << " " << eefTwist_.angular.y << " " << eefTwist_.angular.z << std::endl;

        // 更新MoveIt中的机器人状态
        if (!robotState_->setFromDiffIK(jointModelGroup_, eefTwist_, eefFrame_, 1.0 / this->frequency))
        {
            ROS_ERROR("Failed to get IK");
            return false;
        }

        jointStatesMsgs_.header.stamp = ros::Time::now();
        jointStatesMsgs_.header.frame_id = baseFrame_;
        jointStatesMsgs_.name.resize(armJointNumber_);
        jointStatesMsgs_.position.resize(armJointNumber_);
        jointStatesMsgs_.velocity.resize(armJointNumber_);
        jointStatesMsgs_.effort.resize(armJointNumber_);
        jointStatesMsgs_.name = jointNames_;
        // std::cout << "ok" << std::endl;
        robotState_->copyJointGroupPositions(jointModelGroup_, jointStatesMsgs_.position);
        return true;
    }

    void transformTwist(geometry_msgs::Twist& twist)
    {
        robotState_->setJointGroupPositions(jointModelGroup_, currentJointValues_);
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

    void moveToPose(const geometry_msgs::PoseStamped& msg)
    {
        // Extract the pose from the message
        geometry_msgs::Pose target_pose = msg.pose;

        // Set the pose target for the MoveGroupInterface
        moveGroup_.setPoseTarget(target_pose);

        // Plan the motion
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (moveGroup_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        // If planning is successful, execute the motion
        if (success) {
            moveGroup_.move();
            std::cout << "move finish" << std::endl;
        } else {
            ROS_WARN("Failed to plan to the specified target pose.");
        }
    }

    // Listen to moveJ command and execute
    void moveJCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        // 创建一个目标关节值向量
        std::vector<double> target_joint_values;

        // 将接收到的关节状态消息的关节位置添加到目标关节值向量中
        for (size_t i = 0; i < msg->position.size(); ++i)
        {
            target_joint_values.push_back(msg->position[i]);
        }

        // 设置机器人关节目标值
        moveGroup_.setJointValueTarget(target_joint_values);

        // 规划并执行运动
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (moveGroup_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (success)
        {
            moveGroup_.move();
            ROS_INFO("Robot moved to the target joint positions.");
        }
        else
        {
            ROS_WARN("Failed to plan to the target joint positions.");
        }
    }

    // Listen to gripper command and execute
    void gripperCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        leftFingerPosition_ = msg->position[0];
        rightFingerPosition_ = msg->position[1];
        // std::cout << "cmd received: left " << leftFingerPosition_ << " right: " << rightFingerPosition_ << std::endl; 
        updateGripper();
    }

    void openGripper()
    {
        jointStatesMsgs_.name.push_back("left_finger_joint");
        jointStatesMsgs_.name.push_back("right_finger_joint");
        jointStatesMsgs_.position.push_back(minFingerPosition_);
        jointStatesMsgs_.position.push_back(minFingerPosition_);
        jointStatesMsgs_.velocity.push_back(0);
        jointStatesMsgs_.velocity.push_back(0);
        jointStatesMsgs_.effort.push_back(0);
        jointStatesMsgs_.effort.push_back(0);
    }

    void closeGripper()
    {
        jointStatesMsgs_.name.push_back("left_finger_joint");
        jointStatesMsgs_.name.push_back("right_finger_joint");
        jointStatesMsgs_.position.push_back(0);
        jointStatesMsgs_.position.push_back(0);
        jointStatesMsgs_.velocity.push_back(0);
        jointStatesMsgs_.velocity.push_back(0);
        jointStatesMsgs_.effort.push_back(0);
        jointStatesMsgs_.effort.push_back(0);
    }

    void openGripperOneStep()
    {
        leftFingerPosition_ = std::max(currentLeftFingerJoint_ - fingerStep_, minFingerPosition_);
        rightFingerPosition_ = std::max(currentRightFingerJoint_ - fingerStep_, minFingerPosition_);
        updateGripper();
    }

    void closeGripperOneStep()
    {
        leftFingerPosition_ = std::min(currentLeftFingerJoint_ + fingerStep_, maxFingerPosition_);
        rightFingerPosition_ = std::min(currentRightFingerJoint_ + fingerStep_, maxFingerPosition_);
        updateGripper();
    }

    void updateGripper()
    {
        jointStatesMsgs_.name.push_back("left_finger_joint");
        jointStatesMsgs_.name.push_back("right_finger_joint");
        jointStatesMsgs_.position.push_back(leftFingerPosition_);
        jointStatesMsgs_.position.push_back(rightFingerPosition_);
        jointStatesMsgs_.velocity.push_back(0);
        jointStatesMsgs_.velocity.push_back(0);
        jointStatesMsgs_.effort.push_back(0);
        jointStatesMsgs_.effort.push_back(0);
        jointPosPub_.publish(jointStatesMsgs_);
        sendO3DELinksInfo();
    }

    vector<O3DELinkInfo> getO3DELinksInfo()
    {
        vector<O3DELinkInfo> o3deLinksInfo;

        for (auto link : linkWithGeos_)
        {
            O3DELinkInfo info = createO3DELinkInfo(link, robotState_->getFrameTransform(link));
            o3deLinksInfo.push_back(info);
        }

        return o3deLinksInfo;
    }

    vector<O3DELinkInfo> sendO3DELinksInfo()
    {
        vector<O3DELinkInfo> o3deLinksInfo;
        std::string dataToSend;

        for (auto link : linkWithGeos_)
        {
            O3DELinkInfo info = createO3DELinkInfo(link, robotState_->getFrameTransform(link));
            o3deLinksInfo.push_back(info);
            std::string infoString = info.linkName + ":" +
                                    std::to_string(info.x) + "," + std::to_string(info.y) + "," + std::to_string(info.z) + "," +
                                    std::to_string(info.qw) + "," + std::to_string(info.qx) + "," + std::to_string(info.qy) + "," + std::to_string(info.qz);
            // ROS_INFO("Info for %s: %s", info.linkName.c_str(), infoString.c_str());
            dataToSend += infoString + " ";
        }
        // ROS_INFO("Complete dataToSend: %s", dataToSend.c_str());

        // ROS_INFO("%s", dataToSend.c_str());
        sendDataToClients(dataToSend);
        // ROS_INFO("send finish");

        return o3deLinksInfo;
    }

    O3DELinkInfo createO3DELinkInfo(const string &linkName, const Eigen::Affine3d &matrix)
    {
        O3DELinkInfo linkInfo;
        geometry_msgs::Pose pose;
        tf2::convert(matrix, pose);
        linkInfo.linkName = linkName;
        linkInfo.x = pose.position.x;
        linkInfo.y = pose.position.y;
        linkInfo.z = pose.position.z;
        linkInfo.qw = pose.orientation.w;
        linkInfo.qx = pose.orientation.x;
        linkInfo.qy = pose.orientation.y;
        linkInfo.qz = pose.orientation.z;
        // linkInfo.print();
        return linkInfo;
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "umi_controller");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    UmiController umiController;
    ros::waitForShutdown();
    return 0;
}