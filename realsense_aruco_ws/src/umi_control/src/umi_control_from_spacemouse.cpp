#include <iostream>
#include <vector>
#include <string>
#include <cmath>

#include <Eigen/Eigen>  
#include <Eigen/Geometry> 
#include <Eigen/Core>

#include <ros/ros.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include "moveit/robot_model_loader/robot_model_loader.h"

using namespace std;

struct O3DELinkInfo {
    string linkName;
    double x;
    double y;
    double z;
    double qw;
    double qx;
    double qy;
    double qz;

    void print() {
        cout << "link name " << this->linkName << endl;
        cout << "x: " << this->x << endl;
        cout << "y: " << this->y << endl;
        cout << "z: " << this->z << endl;
        cout << "qw: " << this->qw << endl;
        cout << "qx: " << this->qx << endl;
        cout << "qy: " << this->qy << endl;
        cout << "qz: " << this->qz << endl << endl;
    }
};

class UmiController
{
public:
    UmiController() : robotModelLoader_("robot_description"),
                      robotModel_(robotModelLoader_.getModel()),
                      robotState_(new robot_state::RobotState(robotModel_)),
                      jointModelGroup_(robotState_->getJointModelGroup("arm")),
                      moveGroup_("arm")
    {
        jointPosPub_ = nh_.advertise<sensor_msgs::JointState>("move_group/fake_controller_joint_states", 1);
        baseFrame_ = moveGroup_.getPoseReferenceFrame();
        eefFrame_ = moveGroup_.getEndEffectorLink();
        jointNames_ = moveGroup_.getVariableNames();
        linkWithGeos_ = robotModel_->getLinkModelNamesWithCollisionGeometry();
        
        bool isTest = true;
        if (isTest == false) {
            spaceMouseSub_ = nh_.subscribe("spacenav/joy", 1, &UmiController::spaceMouseCallback, this);
            while (ros::ok() && !spaceMouseFlag_) {
                ROS_ERROR_STREAM("Waiting for space mouse, and the topic name you specified is " << spaceMouseSub_.getTopic());
                ros::Duration(1.0).sleep();
            }
            ROS_WARN("Space Mouse Message Detected");
        } else {
            double controlFrequency = 30;
            ros::Time startTime = ros::Time::now();
            ros::Rate loopRate(controlFrequency);
            while(ros::ok()) {
                double duration = (ros::Time::now() - startTime).toSec();
                robotState_ = moveGroup_.getCurrentState();
                createO3DELinksInfo();
                eefTwist_.linear.x = eefTwist_.linear.y = eefTwist_.linear.z = 0;
                eefTwist_.angular.x = eefTwist_.angular.y = eefTwist_.angular.z = 0;
                eefTwist_.linear.x = -0.3 * sin(duration);

                Eigen::Matrix3d eef2BaseRotationMatrix = robotState_->getFrameTransform(eefFrame_).rotation();
                Eigen::Vector3d linear, angular;
                linear.x() = eefTwist_.linear.x;
                linear.y() = eefTwist_.linear.y;
                linear.z() = eefTwist_.linear.z;
                angular.x() = eefTwist_.angular.x;
                angular.y() = eefTwist_.angular.y;
                angular.z() = eefTwist_.angular.z;

                linear = eef2BaseRotationMatrix.transpose() * linear;
                angular = eef2BaseRotationMatrix.transpose() * angular;

                eefTwist_.linear.x = linear.x();
                eefTwist_.linear.y = linear.y();
                eefTwist_.linear.z = linear.z();
                eefTwist_.angular.x = angular.x();
                eefTwist_.angular.y = angular.y();
                eefTwist_.angular.z = angular.z();

                if (!robotState_->setFromDiffIK(jointModelGroup_, eefTwist_, eefFrame_, 1.0 / controlFrequency)) {
                    ROS_ERROR("Failed to get IK");
                }
                
                jointStatesMsgs_.header.stamp = ros::Time::now();
                jointStatesMsgs_.header.frame_id = baseFrame_;
                jointStatesMsgs_.name.resize(6);
                jointStatesMsgs_.position.resize(6);
                jointStatesMsgs_.velocity.resize(6);
                jointStatesMsgs_.effort.resize(6);
                jointStatesMsgs_.name = jointNames_;
                robotState_->copyJointGroupPositions(jointModelGroup_, jointStatesMsgs_.position);

                if (int(duration) % 10 == 0) {
                    openGripper();
                }
                if (int(duration) % 10 == 5) {
                    closeGripper();
                }
                
                jointPosPub_.publish(jointStatesMsgs_);

                ros::spinOnce();
                loopRate.sleep();
            }
        }
    }

private:
    robot_model_loader::RobotModelLoader robotModelLoader_;
    robot_model::RobotModelPtr robotModel_;
    robot_state::RobotStatePtr robotState_;
    const robot_state::JointModelGroup* jointModelGroup_;
    moveit::planning_interface::MoveGroupInterface moveGroup_;
    ros::NodeHandle nh_;
    ros::Publisher jointPosPub_;
    ros::Subscriber spaceMouseSub_;
    bool spaceMouseFlag_ = false;
    ros::Time lastSpaceMouseTime_;
    geometry_msgs::Twist eefTwist_;
    const double TRANS_VEL = 0.5;   // 末端的线速度映射系数
    const double ROTAT_VEL = 0.5;   // 末端的角速度映射系数
    const double MAX_TIMEOUT = 0.2; // spaceMouse最大的超时时间，因此注意spaceMouse的数据频率不能太低
    sensor_msgs::JointState jointStatesMsgs_;
    string baseFrame_;
    string eefFrame_;
    vector<string> jointNames_;
    vector<string> linkWithGeos_;

    void spaceMouseCallback(const sensor_msgs::JoyConstPtr& joyPtr) {
        if (spaceMouseFlag_ == false) {
            spaceMouseFlag_ = true;
            lastSpaceMouseTime_ = joyPtr->header.stamp;
            return;     // 丢弃第一帧
        }
        // 假设joyPtr->axes的数据是6维的
        if (joyPtr->axes.size() != 6) {
            ROS_ERROR("Data size from space mouse is wrong");
        }
        if (!getTargetJointPositionsFromJoy(joyPtr)) {
            return;
        }
        // 根据按钮决定夹爪的开关
        if (joyPtr->axes[0] > 0)    // 夹爪打开的条件，按需修改
        {
            openGripper();
        }
        if (joyPtr->axes[1] > 0)    // 夹爪关闭的条件，按需修改
        {
            closeGripper();
        }

        lastSpaceMouseTime_ = joyPtr->header.stamp;
    }

    bool getTargetJointPositionsFromJoy(const sensor_msgs::JoyConstPtr& joyPtr, bool twistRelative2Base = true) {
        double duration = (joyPtr->header.stamp - lastSpaceMouseTime_).toSec();
        duration = duration > MAX_TIMEOUT ? MAX_TIMEOUT : duration;

        eefTwist_.linear.x = eefTwist_.linear.y = eefTwist_.linear.z = 0;
        eefTwist_.angular.x = eefTwist_.angular.y = eefTwist_.angular.z = 0;
        
        // 将joy映射成twist，以下是一个个基础的示例。
        // 为了能够让运动更加直观，下面的对应关系可能需要调整
        eefTwist_.linear.x = joyPtr->axes[0] * TRANS_VEL;
        eefTwist_.linear.y = joyPtr->axes[1] * TRANS_VEL;
        eefTwist_.linear.z = joyPtr->axes[2] * TRANS_VEL;
        eefTwist_.angular.x = joyPtr->axes[0] * ROTAT_VEL;
        eefTwist_.angular.y = joyPtr->axes[1] * ROTAT_VEL;
        eefTwist_.angular.z = joyPtr->axes[2] * ROTAT_VEL;

        // 如果是相对基座坐标系运动，则需要将速度映射成末端。因为在setFromDiffIK中，eefTwist_是末端相对末端的速度
        if (twistRelative2Base) {
            robotState_ = moveGroup_.getCurrentState();
            Eigen::Matrix3d eef2BaseRotationMatrix = robotState_->getFrameTransform(eefFrame_).rotation();
            Eigen::Vector3d linear, angular;
            linear.x() = eefTwist_.linear.x;
            linear.y() = eefTwist_.linear.y;
            linear.z() = eefTwist_.linear.z;
            angular.x() = eefTwist_.angular.x;
            angular.y() = eefTwist_.angular.y;
            angular.z() = eefTwist_.angular.z;

            linear = eef2BaseRotationMatrix.transpose() * linear;
            angular = eef2BaseRotationMatrix.transpose() * angular;

            eefTwist_.linear.x = linear.x();
            eefTwist_.linear.y = linear.y();
            eefTwist_.linear.z = linear.z();
            eefTwist_.angular.x = angular.x();
            eefTwist_.angular.y = angular.y();
            eefTwist_.angular.z = angular.z();
        }

        if (!robotState_->setFromDiffIK(jointModelGroup_, eefTwist_, eefFrame_, duration)) {
            ROS_ERROR("Failed to get IK");
            return false;
        }
        
        jointStatesMsgs_.header.stamp = ros::Time::now();
        jointStatesMsgs_.header.frame_id = baseFrame_;
        jointStatesMsgs_.name.resize(6);
        jointStatesMsgs_.position.resize(6);
        jointStatesMsgs_.velocity.resize(6);
        jointStatesMsgs_.effort.resize(6);
        jointStatesMsgs_.name = jointNames_;
        robotState_->copyJointGroupPositions(jointModelGroup_, jointStatesMsgs_.position); 
        return true;
    }

    // 现在夹爪的开关还比较粗暴，后期可以优化
    void openGripper() {
        jointStatesMsgs_.name.push_back("left_finger_joint");
        jointStatesMsgs_.name.push_back("right_finger_joint");
        jointStatesMsgs_.position.push_back(-0.04);
        jointStatesMsgs_.position.push_back(-0.04);
        jointStatesMsgs_.velocity.push_back(0);
        jointStatesMsgs_.velocity.push_back(0);
        jointStatesMsgs_.effort.push_back(0);
        jointStatesMsgs_.effort.push_back(0);
    }

    void closeGripper() {
        jointStatesMsgs_.name.push_back("left_finger_joint");
        jointStatesMsgs_.name.push_back("right_finger_joint");
        jointStatesMsgs_.position.push_back(0);
        jointStatesMsgs_.position.push_back(0);
        jointStatesMsgs_.velocity.push_back(0);
        jointStatesMsgs_.velocity.push_back(0);
        jointStatesMsgs_.effort.push_back(0);
        jointStatesMsgs_.effort.push_back(0);
    }

    vector<O3DELinkInfo> createO3DELinksInfo() {
        vector<O3DELinkInfo> o3deLinksInfo;
        for (auto link : linkWithGeos_) {
            o3deLinksInfo.push_back(createO3DELinkInfo(link, robotState_->getFrameTransform(link)));
        }
        return o3deLinksInfo;
    }

    O3DELinkInfo createO3DELinkInfo(const string& linkName, const Eigen::Affine3d& matrix) {
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
    ros::AsyncSpinner spinner(1);
    spinner.start();
    UmiController umiController;
    spinner.stop();
    ros::shutdown();
    return 0;
}