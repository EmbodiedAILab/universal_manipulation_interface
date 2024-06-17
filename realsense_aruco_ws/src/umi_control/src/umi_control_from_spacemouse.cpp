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

        setupUDPServer();

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
                createO3DELinksInfo();
                jointPosPub_.publish(jointStatesMsgs_);

                ros::spinOnce();
                loopRate.sleep();
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

        memset(&serverAddr_, 0, sizeof(serverAddr_));
        serverAddr_.sin_family = AF_INET;
        serverAddr_.sin_addr.s_addr = INADDR_ANY;
        serverAddr_.sin_port = htons(8080);

        if (bind(udpSocket_, (const struct sockaddr *)&serverAddr_, sizeof(serverAddr_)) < 0)
        {
            perror("bind failed");
            close(udpSocket_);
            exit(EXIT_FAILURE);
        }

        ROS_INFO("UDP server set up and listening on port 8080");
    }

    void sendDataToClient(const std::string &data, const std::string &clientIP, int clientPort)
    {
        memset(&clientAddr_, 0, sizeof(clientAddr_));
        clientAddr_.sin_family = AF_INET;
        clientAddr_.sin_port = htons(clientPort);
        inet_pton(AF_INET, clientIP.c_str(), &clientAddr_.sin_addr);

        ssize_t sentBytes = sendto(udpSocket_, data.c_str(), data.size(), MSG_CONFIRM,
                                   (const struct sockaddr *)&clientAddr_, clientAddrLen_);
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

        createO3DELinksInfo();
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

    vector<O3DELinkInfo> createO3DELinksInfo()
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
        sendDataToClient(dataToSend, "127.0.0.1", 8080);
        ROS_INFO("send finish");

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