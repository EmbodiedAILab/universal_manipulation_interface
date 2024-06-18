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
#include "httplib.h"

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
        eefFrame_ = moveGroup_.getEndEffectorLink();
        jointNames_ = moveGroup_.getVariableNames();
        linkWithGeos_ = robotModel_->getLinkModelNamesWithCollisionGeometry();
        robotState_ = moveGroup_.getCurrentState();
        // change this path to the desired one
        outputPath_ = ros::package::getPath("umi_control") + "/output_link_traj/";

        // initalize UDP Server
        // setupUDPServer();
        // clear output directory
        // clearOutputDirectory();

        bool isTest = false;
        if (isTest == false)
        {
            spaceMouseSub_ = nh_.subscribe("spacenav/joy", 1, &UmiController::spaceMouseCallback, this);
            ROS_INFO("Subscribed to spacenav/joy");
        }
        else
        {
            double controlFrequency = 30;
            ros::Time startTime = ros::Time::now();
            ros::Rate loopRate(controlFrequency);
            while (ros::ok())
            {
                double duration = (ros::Time::now() - startTime).toSec();
                updateRobotState();
                
                eefTwist_.linear.x = eefTwist_.linear.y = eefTwist_.linear.z = 0;
                eefTwist_.angular.x = eefTwist_.angular.y = eefTwist_.angular.z = 0;
                eefTwist_.linear.x = -0.08 * sin(duration);
                transformTwist(eefTwist_);

                if (!robotState_->setFromDiffIK(jointModelGroup_, eefTwist_, eefFrame_, 1.0 / controlFrequency))
                {
                    ROS_ERROR("Failed to get IK");
                }

                jointStatesMsgs_.header.stamp = ros::Time::now();
                jointStatesMsgs_.header.frame_id = baseFrame_;
                jointStatesMsgs_.name.resize(armJointNumber_);
                jointStatesMsgs_.position.resize(armJointNumber_);
                jointStatesMsgs_.velocity.resize(armJointNumber_);
                jointStatesMsgs_.effort.resize(armJointNumber_);
                jointStatesMsgs_.name = jointNames_;
                robotState_->copyJointGroupPositions(jointModelGroup_, jointStatesMsgs_.position);

                if (int(duration) % 10 < 5)
                {
                    openGripperOneStep();
                } else
                {
                    closeGripperOneStep();
                }
                saveO3DELinksInfo();
                jointPosPub_.publish(jointStatesMsgs_);

                ros::spinOnce();
                loopRate.sleep();
            }
        }

        // Start HTTP server thread
        std::thread server_thread(&UmiController::startHttpServer, this);
        server_thread.detach();
    }

    void startHttpServer() {
        httplib::Server svr;

        // 处理GET请求，返回机器人link位姿信息
        svr.Get("/robot_pose", [&](const httplib::Request& /*req*/, httplib::Response& res) {
            vector<O3DELinkInfo> linksInfo = getO3DELinksInfo();
            std::stringstream ss;
            for (const auto& link : linksInfo) {
                ss << link.linkName << ": "
                   << link.x << "," << link.y << "," << link.z << ","
                   << link.qw << "," << link.qx << "," << link.qy << "," << link.qz << "\n";
            }
            res.set_content(ss.str(), "text/plain");
        });

        // 启动HTTP服务器，监听8081端口
        svr.listen("0.0.0.0", 65432);
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
    std::string outputPath_;
    double leftFingerPosition_;
    double rightFingerPosition_;
    const double maxFingerPosition_ = 0;     // 最大开合值
    const double minFingerPosition_ = -0.04; // 最小开合值
    const double fingerStep_ = 0.001;         // 每次移动的步长
    double currentLeftFingerJoint_;
    double currentRightFingerJoint_;

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
        serverAddr_.sin_port = htons(8080);

        ROS_INFO("UDP server set up and broadcasting on port 8080");
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

    void spaceMouseCallback(const sensor_msgs::JoyConstPtr &joyPtr)
    {
        // ROS_INFO("SpaceMouse callback triggered");
        if (spaceMouseFlag_ == false)
        {
            spaceMouseFlag_ = true;
            lastSpaceMouseTime_ = joyPtr->header.stamp;
            return;
        }
        // 假设joyPtr->axes的数据是6维的
        if (joyPtr->axes.size() != 6)
        {
            ROS_ERROR("Data size from space mouse is wrong");
        }

        if (!updateTargetJointPositionsFromJoy(joyPtr))
        {
            return;
        }
        // 根据按钮决定夹爪的开关
        if (joyPtr->buttons[1] > 0)
        {
            openGripperOneStep();
        }
        if (joyPtr->buttons[0] > 0)
        {
            closeGripperOneStep();
        }

        // saveO3DELinksInfo();
        jointPosPub_.publish(jointStatesMsgs_);

        lastSpaceMouseTime_ = joyPtr->header.stamp;
    }

    bool updateTargetJointPositionsFromJoy(const sensor_msgs::JoyConstPtr &joyPtr, bool twistRelative2Base = true)
    {
        double duration = (joyPtr->header.stamp - lastSpaceMouseTime_).toSec();
        duration = duration > maxTimeout_ ? maxTimeout_ : duration;

        eefTwist_.linear.x = eefTwist_.linear.y = eefTwist_.linear.z = 0;
        eefTwist_.angular.x = eefTwist_.angular.y = eefTwist_.angular.z = 0;

        // 将joy映射成twist，以下是一个个基础的示例。
        // 为了能够让运动更加直观，下面的对应关系可能需要调整
        eefTwist_.linear.x = joyPtr->axes[0] * transVel_;
        eefTwist_.linear.y = joyPtr->axes[1] * transVel_;
        eefTwist_.linear.z = joyPtr->axes[2] * transVel_;
        eefTwist_.angular.x = joyPtr->axes[3] * rotatVel_;
        eefTwist_.angular.y = joyPtr->axes[4] * rotatVel_;
        eefTwist_.angular.z = joyPtr->axes[5] * rotatVel_;

        // 如果是相对基座坐标系运动，则需要将速度映射成末端。因为在setFromDiffIK中，eefTwist_是末端相对末端的速度
        if (twistRelative2Base)
        {
            transformTwist(eefTwist_);
        }

        if (!robotState_->setFromDiffIK(jointModelGroup_, eefTwist_, eefFrame_, duration))
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

        ROS_INFO("%s", dataToSend.c_str());
        sendDataToClients(dataToSend);
        ROS_INFO("send finish");

        return o3deLinksInfo;
    }

    vector<O3DELinkInfo> saveO3DELinksInfo() {
        vector<O3DELinkInfo> o3deLinksInfo;

        maintainFileCount(outputPath_, 30);

        // 生成文件名
        std::stringstream ss;
        ss << outputPath_ << "data_" << std::fixed << std::setprecision(1) << frameCount_++ << ".txt";
        std::ofstream file(ss.str());

        if (!file.is_open()) {
            ROS_ERROR("Failed to open file.");
            return o3deLinksInfo;
        }

        // 写入每个link的信息到文件
        for (auto link : linkWithGeos_) {
            O3DELinkInfo info = createO3DELinkInfo(link, robotState_->getFrameTransform(link));
            o3deLinksInfo.push_back(info);
            file << info.linkName << ":" 
                << info.x << "," << info.y << "," << info.z << "," 
                << info.qw << "," << info.qx << "," << info.qy << "," << info.qz << " ";
        }

        file.close();
        return o3deLinksInfo;
    }   

    static bool compareFiles(const fs::path& a, const fs::path& b) {
        // 使用正则表达式提取文件名中的数字部分
        std::regex re("data_(\\d+)\\.0.txt");
        std::smatch matchA, matchB;

        std::string fileA = a.filename().string();
        std::string fileB = b.filename().string();

        if (std::regex_search(fileA, matchA, re) && std::regex_search(fileB, matchB, re)) {
            int numA = std::stoi(matchA[1].str());
            int numB = std::stoi(matchB[1].str());
            return numA < numB;
        }
        // 如果正则表达式匹配失败，按字母顺序排序
        return fileA < fileB;
    }

    void maintainFileCount(const string& path, size_t maxFiles) {
        // 获取目录下的所有文件
        vector<fs::path> files;
        for (const auto& entry : fs::directory_iterator(path)) {
            if (fs::is_regular_file(entry.status())) {
                files.push_back(entry.path());
            }
        }

        // 如果文件数量超过指定值，删除最老的文件
        if (files.size() > maxFiles) {
            sort(files.begin(), files.end(), compareFiles);
            // cout << "Removing old file: " << files.front().string() << endl;
            fs::remove(files.front());
        }
    }

    // 清除输出目录
    void clearOutputDirectory() {
        for (const auto& entry : fs::directory_iterator(outputPath_)) {
            if (fs::is_regular_file(entry.status())) {
                fs::remove(entry.path());
            }
        }
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