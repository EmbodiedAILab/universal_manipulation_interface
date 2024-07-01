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
#include <unordered_map>
#include <thread>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <mutex>
#include <sys/socket.h>
#include <unistd.h>

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

class UmiMoveitController
{
public:
    UmiMoveitController() : robotModelLoader_("robot_description"),
                            robotModel_(robotModelLoader_.getModel()),
                            robotState_(new robot_state::RobotState(robotModel_)),
                            armJointModelGroup_(robotState_->getJointModelGroup("arm")),
                            gripperJointModelGroup_(robotState_->getJointModelGroup("gripper")),
                            armMoveGroup_("arm"),
                            gripperMoveGroup_("gripper")
    {
        currentJointValues_.resize(armJointNumber_);
        setupUDPServer();
        baseFrame_ = armMoveGroup_.getPoseReferenceFrame();
        eefFrame_ = armMoveGroup_.getEndEffectorLink();
        jointNames_ = armMoveGroup_.getVariableNames();
        linkWithGeos_ = robotModel_->getLinkModelNamesWithCollisionGeometry();
        robotState_ = armMoveGroup_.getCurrentState();

        robotStateSub_ = nh_.subscribe("joint_states", 1, &UmiMoveitController::jointStatesCallback, this);
        ros::Duration(0.5).sleep();
        while (ros::ok() && !jointStatesFlag_)
        {
            ROS_ERROR_STREAM("Waiting for joint states, and the topic name you specified is " << robotStateSub_.getTopic());
            ros::Duration(1.0).sleep();
        }

        std::thread commandThread(&UmiMoveitController::CommandProcessor, this);
        commandThread.detach();

        // Start the thread to receive data from clients
        std::thread receiveThread(&UmiMoveitController::receiveDataFromClients, this);
        receiveThread.detach();

        // make sure robot is at home and the gripper is opened
        armMoveGroup_.setNamedTarget("home");
        armMoveGroup_.move();
        gripperMoveGroup_.setNamedTarget("open");
        gripperMoveGroup_.move();

        objectMatrix_ = robotState_->getFrameTransform(eefFrame_);
        objectMatrix_.translation().z() -= 0.2;
        // retreatMatrix = objectMatrix;
        // retreatMatrix(2,3) += 0.1;
        placeMatrix_ = objectMatrix_;
        placeMatrix_.translation().y() += 0.1;
        
        while (ros::ok())
        {
            // Retrieve the latest object and place matrices
            {
                std::lock_guard<std::mutex> lock(matrixMutex_);
                retreatMatrix_ = objectMatrix_;
                retreatMatrix_.translation().z() += 0.2;
            }

            geometry_msgs::Pose objectPose, retreatPose, placePose;
            tf2::convert(objectMatrix_, objectPose);
            tf2::convert(retreatMatrix_, retreatPose);
            tf2::convert(placeMatrix_, placePose);

            armMoveGroup_.setPoseTarget(retreatPose);
            armMoveGroup_.move();
            ROS_INFO("Approach");

            armMoveGroup_.setPoseTarget(objectPose);
            armMoveGroup_.move();
            ROS_INFO("Pregrasp");

            gripperMoveGroup_.setJointValueTarget(getGripperJointsFromWidth(0.02));
            gripperMoveGroup_.move();
            ROS_INFO("Grasp");

            armMoveGroup_.setPoseTarget(retreatPose);
            armMoveGroup_.move();
            ROS_INFO("Retreat");

            armMoveGroup_.setPoseTarget(placePose);
            armMoveGroup_.move();
            ROS_INFO("PrePlace");

            gripperMoveGroup_.setNamedTarget("open");
            gripperMoveGroup_.move();
            ROS_INFO("Place");

            armMoveGroup_.setNamedTarget("home");
            armMoveGroup_.move();
            ROS_INFO("Home");
        }
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
    const robot_state::JointModelGroup *armJointModelGroup_;
    const robot_state::JointModelGroup *gripperJointModelGroup_;
    moveit::planning_interface::MoveGroupInterface armMoveGroup_;
    moveit::planning_interface::MoveGroupInterface gripperMoveGroup_;

    // ros communication variables
    ros::NodeHandle nh_;
    ros::Subscriber robotStateSub_;
    bool jointStatesFlag_ = false;
    vector<double> currentJointValues_;         // 机械臂6个关节角度
    sensor_msgs::JointState jointStatesMsgs_;   // 机器人最新的数据
    const int armJointNumber_ = 6;              // 机械臂关节数量

    // socket
    int udpSocket_;
    struct sockaddr_in serverAddr_;
    struct sockaddr_in clientAddr_;
    socklen_t clientAddrLen_ = sizeof(clientAddr_);
    
    string baseFrame_;
    string eefFrame_;
    vector<string> jointNames_;
    vector<string> linkWithGeos_;
    double currentLeftFingerJoint_;
    double currentRightFingerJoint_;
    const double maxWidth_ = 0.08;
    const double minWidth_ = 0.0;

    // Matrices
    Eigen::Affine3d objectMatrix_;
    Eigen::Affine3d retreatMatrix_;
    Eigen::Affine3d placeMatrix_;
    std::mutex matrixMutex_;

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
        robotState_->setJointGroupPositions(armJointModelGroup_, currentJointValues_);
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

        updateRobotState();
        sendO3DELinksInfo();
    }

    vector<double> getGripperJointsFromWidth(double width)
    {
        if (width < minWidth_)
        {
            ROS_ERROR_STREAM("The width must > 0, but the width specified is " << width);
            width = minWidth_;
        }
        if (width > maxWidth_)
        {
            ROS_ERROR_STREAM("The width must <= 0.08m, but the width specified is " << width);
            width = maxWidth_;
        }

        double leftPos = width / 2.0;
        double rightPos = width / 2.0;
        return vector<double>{leftPos, rightPos};
    }

    void sendO3DELinksInfo()
    {
        string baseFrame = baseFrame_;
        std::vector<O3DELinkInfo> o3deLinksInfo;

        for (int i = 0; i < linkWithGeos_.size(); ++i)
        {
            O3DELinkInfo info;
            info.linkName = linkWithGeos_[i];
            Eigen::Affine3d linkMatrix = robotState_->getFrameTransform(info.linkName);
            Eigen::Quaterniond quat(linkMatrix.rotation());

            info.x = linkMatrix(0, 3);
            info.y = linkMatrix(1, 3);
            info.z = linkMatrix(2, 3);

            info.qw = quat.w();
            info.qx = quat.x();
            info.qy = quat.y();
            info.qz = quat.z();
            o3deLinksInfo.push_back(info);
        }

        std::string message = convertO3DELinksInfoToJson(o3deLinksInfo);
        sendDataToClients(message);
    }

    std::string convertO3DELinksInfoToJson(const std::vector<O3DELinkInfo> &o3deLinksInfo)
    {
        std::ostringstream oss;
        oss << "[";
        for (size_t i = 0; i < o3deLinksInfo.size(); ++i)
        {
            const O3DELinkInfo &info = o3deLinksInfo[i];
            oss << "{"
                << "\"linkName\":\"" << info.linkName << "\","
                << "\"x\":" << info.x << ","
                << "\"y\":" << info.y << ","
                << "\"z\":" << info.z << ","
                << "\"qw\":" << info.qw << ","
                << "\"qx\":" << info.qx << ","
                << "\"qy\":" << info.qy << ","
                << "\"qz\":" << info.qz
                << "}";
            if (i < o3deLinksInfo.size() - 1)
            {
                oss << ",";
            }
        }
        oss << "]";
        return oss.str();
    }

    // Private function to receive data from clients
    void receiveDataFromClients()
    {
        char buffer[1024];
        while (true)
        {
            ssize_t recvBytes = recvfrom(udpSocket_, buffer, sizeof(buffer) - 1, 0,
                                         (struct sockaddr *)&clientAddr_, &clientAddrLen_);
            if (recvBytes < 0)
            {
                perror("recvfrom failed");
            }
            else
            {
                buffer[recvBytes] = '\0'; // Null-terminate the received data
                std::string receivedData(buffer);
                ROS_INFO("Received data: %s", receivedData.c_str());

                // Parse the JSON data
                std::istringstream iss(receivedData);
                boost::property_tree::ptree pt;
                boost::property_tree::read_json(iss, pt);

                // Update the object and place matrices
                {
                    std::lock_guard<std::mutex> lock(matrixMutex_);
                    updateMatrixFromJSON(pt, "cup", objectMatrix_);
                    updateMatrixFromJSON(pt, "dish", placeMatrix_);
                }
            }
        }
    }

    void updateMatrixFromJSON(const boost::property_tree::ptree &pt, const std::string &key, Eigen::Affine3d &matrix)
    {
        double x = pt.get<double>(key + ".x");
        double y = pt.get<double>(key + ".y");
        double z = pt.get<double>(key + ".z");
        double qx = pt.get<double>(key + ".q_x");
        double qy = pt.get<double>(key + ".q_y");
        double qz = pt.get<double>(key + ".q_z");
        double qw = pt.get<double>(key + ".q_w");

        matrix = Eigen::Translation3d(x, y, z) * Eigen::Quaterniond(qw, qx, qy, qz);
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "umi_moveit_controller");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    UmiMoveitController umiController;
    ros::waitForShutdown();
    return 0;
}
