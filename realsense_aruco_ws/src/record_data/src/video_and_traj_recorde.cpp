#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <aruco_msgs/Marker.h>

using namespace std;

class VideoAndTrajRecorder
{
public:
    VideoAndTrajRecorder(const string& egoCameraName, const string& extCameraName) :
        EGO_CAMERA_NAME(egoCameraName), EXT_CAMERA_NAME(extCameraName) {
        egoCameraSub_ = nh_.subscribe(EGO_CAMERA_NAME + "/camera_color/image_raw", 1, &VideoAndTrajRecorder::egoCameraCallback, this);
        extCameraSub_ = nh_.subscribe(EXT_CAMERA_NAME + "/camera_color/image_raw", 1, &VideoAndTrajRecorder::extCameraCallback, this);
        arucoPoseSub_ = nh_.subscribe(ARUCO_RES_NAME, 1, &VideoAndTrajRecorder::arucoPoseCallback, this);
        while (ros::ok() && egoCameraFlag_) {
            ROS_ERROR("Waiting for ego camera");
            ros::Duration(0.5).sleep();
        }
        while (ros::ok() && extCameraFlag_) {
            ROS_ERROR("Waiting for ext camera");
            ros::Duration(0.5).sleep();
        }
        while (ros::ok() && arucoPoseFlag_) {
            ROS_ERROR("Waiting for aruco pose");
            ros::Duration(0.5).sleep();
        }
        ROS_INFO("Ready to record video");
        
        ros::Rate loopRate(30);
        while (ros::ok()) {
            // record video
            ros::spinOnce();
            loopRate.sleep();
        } 
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber egoCameraSub_;
    ros::Subscriber extCameraSub_;
    ros::Subscriber arucoPoseSub_;

    bool egoCameraFlag_ = false;
    bool extCameraFlag_ = false;
    bool arucoPoseFlag_ = false;

    const string EGO_CAMERA_NAME;
    const string EXT_CAMERA_NAME;
    const string ARUCO_RES_NAME = "aruco_single/pose";
    
    // 回调函数
    void egoCameraCallback(const sensor_msgs::ImageConstPtr& imagePtr) {
        
        if (egoCameraFlag_ == false) {
            egoCameraFlag_ = true;
        }
    }
    void extCameraCallback(const sensor_msgs::ImageConstPtr& imagePtr) {

        if (extCameraFlag_ == false) {
            extCameraFlag_ = true;
        }
    }
    void arucoPoseCallback(const aruco_msgs::MarkerConstPtr& posePtr) {


        if (arucoPoseFlag_ == false) {
            arucoPoseFlag_ = true;
        }
    }
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "VideoAndTrajRecorder");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    VideoAndTrajRecorder recorder("camera", "camera2");
    ros::waitForShutdown();
    return 0;
}