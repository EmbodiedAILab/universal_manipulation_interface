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
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber egoCameraSub_;
    ros::Subscriber extCameraSub_;
    ros::Subscriber arucoPoseSub_;

    const string EGO_CAMERA_NAME;
    const string EXT_CAMERA_NAME;
    const string ARUCO_RES_NAME = "aruco_single/pose";
    
    // 回调函数
    void egoCameraCallback(const sensor_msgs::ImageConstPtr& imagePtr) {

    }
    void extCameraCallback(const sensor_msgs::ImageConstPtr& imagePtr) {

    }
    void arucoPoseCallback(const aruco_msgs::MarkerConstPtr& posePtr) {

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