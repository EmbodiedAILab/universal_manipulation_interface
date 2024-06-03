#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <aruco_msgs/Marker.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sstream>
#include <iomanip>

using namespace std;

class VideoAndTrajRecorder
{
public:
    VideoAndTrajRecorder(const string& egoCameraName, const string& extCameraName) :
        EGO_CAMERA_NAME(egoCameraName), EXT_CAMERA_NAME(extCameraName), it_(nh_) {
        egoCameraSub_ = it_.subscribe(EGO_CAMERA_NAME + "/color/image_raw", 1, &VideoAndTrajRecorder::egoCameraCallback, this);
        extCameraSub_ = it_.subscribe(EXT_CAMERA_NAME + "/color/image_raw", 1, &VideoAndTrajRecorder::extCameraCallback, this);
        arucoPoseSub_ = nh_.subscribe(ARUCO_RES_NAME, 1, &VideoAndTrajRecorder::arucoPoseCallback, this);
        
        ros::Rate loopRate(30);
        while (ros::ok() && !egoCameraFlag_) {
            ROS_ERROR("Waiting for ego camera");
            ros::spinOnce();
            ros::Duration(0.5).sleep();
        }
        while (ros::ok() && !extCameraFlag_) {
            ROS_ERROR("Waiting for ext camera");
            ros::spinOnce();
            ros::Duration(0.5).sleep();
        }
        // while (ros::ok() && !arucoPoseFlag_) {
        //     ROS_ERROR("Waiting for aruco pose");
        //     ros::spinOnce();
        //     ros::Duration(0.5).sleep();
        // }
        ROS_INFO("Ready to record video");
        videoWriter_.open("raw_video.mp4", cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 30, cv::Size(1280, 720), true);
        if (!videoWriter_.isOpened())
        {
            ROS_ERROR("Failed to open video writer");
            ros::shutdown();
        }

        int frameCount_ = 0;
        while (ros::ok()) {
            try
            {
                ROS_INFO("Saving Video");
                cv_bridge::CvImagePtr cvPtr = cv_bridge::toCvCopy(egoImagePtr_, sensor_msgs::image_encodings::BGR8);
                cv::Mat image = cvPtr->image;
                videoWriter_ << image;

                // 设置视频文件的元数据
                updateVideoMetadata(egoImagePtr_, frameCount_);
                frameCount_++;
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
            }
            ros::spinOnce();
            loopRate.sleep();
        } 
    }

    ~VideoAndTrajRecorder() {
        videoWriter_.release();
    }

private:
    ros::NodeHandle nh_;
    image_transport::Subscriber egoCameraSub_;
    image_transport::Subscriber extCameraSub_;
    ros::Subscriber arucoPoseSub_;

    bool egoCameraFlag_ = false;
    bool extCameraFlag_ = false;
    bool arucoPoseFlag_ = false;

    image_transport::ImageTransport it_;
    cv::VideoWriter videoWriter_;
    
    sensor_msgs::ImageConstPtr egoImagePtr_;
    sensor_msgs::ImageConstPtr extImagePtr_;

    const string EGO_CAMERA_NAME;
    const string EXT_CAMERA_NAME;
    const string ARUCO_RES_NAME = "aruco_single/pose";
    const string CAMERA_ID = "C123456";
    
    // 回调函数
    void egoCameraCallback(const sensor_msgs::ImageConstPtr& imagePtr) {
        egoImagePtr_ = imagePtr;
        if (egoCameraFlag_ == false) {
            egoCameraFlag_ = true;
            ROS_WARN("Received Ego Camera");
        }
    }
    void extCameraCallback(const sensor_msgs::ImageConstPtr& imagePtr) {
        extImagePtr_ = imagePtr;
        if (extCameraFlag_ == false) {
            extCameraFlag_ = true;
            ROS_WARN("Received Ext Camera");
        }
    }
    void arucoPoseCallback(const aruco_msgs::MarkerConstPtr& posePtr) {

        if (arucoPoseFlag_ == false) {
            arucoPoseFlag_ = true;
        }
    }

    void updateVideoMetadata(sensor_msgs::ImageConstPtr& imagePtr, int frameCount) {
        std::ostringstream metadata;
        metadata << "Camera ID: " << CAMERA_ID << " | ";
        metadata << "Frame Time: " << std::fixed << std::setprecision(6) << imagePtr->header.stamp.toSec() << " | ";
        metadata << "Frame Count: " << frameCount;
        cv::Mat metadata_mat(1, metadata.str().length(), CV_8UC1, (void*)metadata.str().c_str());
        videoWriter_.write(metadata_mat);
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