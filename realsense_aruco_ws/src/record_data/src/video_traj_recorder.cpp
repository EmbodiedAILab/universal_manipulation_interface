#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <chrono>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

using namespace std;

class VideoTrajRecorder
{
public:
    VideoTrajRecorder() : it_(nh_) {
        egoCameraSub_ = it_.subscribe("ego_camera_topic", 1, &VideoTrajRecorder::egoCameraCallback, this);
        extCameraSub_ = it_.subscribe("ext_camera_topic", 1, &VideoTrajRecorder::extCameraCallback, this);
        arucoPoseSub_ = nh_.subscribe("aruco_pose_topic", 1, &VideoTrajRecorder::arucoPoseCallback, this);
        ros::Duration(1.0).sleep();

        ros::Rate loopRate(30);
        // 检测是否收到第一视角图像
        while (ros::ok() && !egoCameraFlag_) {
            ROS_ERROR_STREAM("Waiting for ego camera, and the topic name you specified is " << egoCameraSub_.getTopic());
            ros::spinOnce();
            ros::Duration(1.0).sleep();
        }
        // 检测是否收到第三视角图像
        while (ros::ok() && !extCameraFlag_) {
            ROS_ERROR_STREAM("Waiting for ext camera, and the topic name you specified is " << extCameraSub_.getTopic());
            ros::spinOnce();
            ros::Duration(1.0).sleep();
        }
        // 检测是否收到aruco的检测结果
        startRecord_ = true;
        while (ros::ok() && !arucoPoseFlag_) {
            ROS_ERROR_STREAM("Waiting for aruco pose, and the topic name you specified is " << arucoPoseSub_.getTopic());
            ros::spinOnce();
            ros::Duration(1.0).sleep();
        }
        startRecord_ = false;

        // 通过键盘控制开始或者结束
        ROS_INFO("Ready to record video and trajectory");
        while(ros::ok()) {
            if (kbhit()) {
                char c = getchar();
                // 开始记录，使用按键s(start)
                if (c == 's') {
                    ROS_WARN("Record start");
                    startRecord_ = true;
                    arucoPoseFlag_ = false;

                    startTime_ = std::chrono::system_clock::now();
                    std::time_t currentTime = std::chrono::system_clock::to_time_t(startTime_);
                    std::stringstream ss;
                    ss << std::put_time(std::localtime(&currentTime), "%Y-%m-%d_%H-%M-%S");

                    videoPath_ = ros::package::getPath("record_data") + "/data/ego_" + ss.str() + ".mp4";
                    videoWriter_.open(videoPath_, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), imageFrequency, cv::Size(egoImagePtr_->width, egoImagePtr_->height), true);
                    if (!videoWriter_.isOpened()) {
                        ROS_ERROR("Failed to open video writer");
                        ros::shutdown();
                    }

                    csvFilePath_ = ros::package::getPath("record_data") + "/data/" + ss.str() + ".csv";
                    csvFile_.open(csvFilePath_);
                    if (!csvFile_.is_open()) {
                        ROS_ERROR("Failed to open file: %s", csvFilePath_.c_str());
                        ros::shutdown();
                    }
                    csvFile_ << "frame_idx,timestamp,state,is_lost,is_keyframe,x,y,z,q_x,q_y,q_z,q_w\n";
                }
                // 结束记录，使用按键e(end)
                if (c == 'e') {
                    startRecord_ = false;
                    ROS_WARN("Record finished");
                    ROS_WARN_STREAM("Video file has been saved to " << videoPath_);
                    ROS_WARN_STREAM("CSV file has been saved to " << csvFilePath_);
                    videoWriter_.release();
                    if (csvFile_.is_open()) {
                        csvFile_.close();
                    }
                }
            }
            ros::spinOnce();
            loopRate.sleep();
        }
    }

    ~VideoTrajRecorder() {
        videoWriter_.release();
        if (csvFile_.is_open()) {
            csvFile_.close();
        }
    }

private:
    ros::NodeHandle nh_;
    image_transport::Subscriber egoCameraSub_;
    image_transport::Subscriber extCameraSub_;
    ros::Subscriber arucoPoseSub_;

    bool egoCameraFlag_ = false;
    bool extCameraFlag_ = false;
    bool arucoPoseFlag_ = false;
    bool startRecord_ = false;
    double imageFrequency = 30;

    double firstTimestamp_;
    int frameIdx_;
    std::chrono::system_clock::time_point startTime_;

    image_transport::ImageTransport it_;
    cv::VideoWriter videoWriter_;
    std::string videoPath_;
    std::ofstream csvFile_;
    std::string csvFilePath_;
    
    sensor_msgs::ImageConstPtr egoImagePtr_;
    sensor_msgs::ImageConstPtr extImagePtr_;

    const string CAMERA_ID = "C123456";

    int kbhit()
    {
        struct termios oldt, newt;
        int ch;
        int oldf;

        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

        ch = getchar();

        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        fcntl(STDIN_FILENO, F_SETFL, oldf);

        if (ch != EOF) {
            ungetc(ch, stdin);
            return 1;
        }

        return 0;
    }

    // 第一视角回调函数
    void egoCameraCallback(const sensor_msgs::ImageConstPtr& imagePtr) {
        egoImagePtr_ = imagePtr;
        if (egoCameraFlag_ == false) {
            egoCameraFlag_ = true;
            ROS_WARN("Received Ego Camera");
        }
    }

    // 第三视角回调函数
    void extCameraCallback(const sensor_msgs::ImageConstPtr& imagePtr) {
        extImagePtr_ = imagePtr;
        if (extCameraFlag_ == false) {
            extCameraFlag_ = true;
            ROS_WARN("Received Ext Camera");
        }
    }

    // aruco位姿的回调函数
    void arucoPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        if (startRecord_ == false) {
            return;
        }
        if (!arucoPoseFlag_) {
            firstTimestamp_ = msg->header.stamp.toSec();
            frameIdx_ = 0;
            arucoPoseFlag_ = true;
        }

        frameIdx_++;
        double timestamp = msg->header.stamp.toSec() - firstTimestamp_;
        double x = msg->pose.position.x;
        double y = msg->pose.position.y;
        double z = msg->pose.position.z;
        double q_x = msg->pose.orientation.x;
        double q_y = msg->pose.orientation.y;
        double q_z = msg->pose.orientation.z;
        double q_w = msg->pose.orientation.w;

        csvFile_ << frameIdx_ << ","
                  << std::fixed << std::setprecision(6) << timestamp << ","
                  << 2 << "," << "FALSE" << "," << "TRUE" << ","
                  << x << "," << y << "," << z << ","
                  << q_x << "," << q_y << "," << q_z << "," << q_w << "\n";

        csvFile_.flush();

        // ROS_INFO("Frame: %d, Time: %.6f, Position: [%.6f, %.6f, %.6f], Orientation: [%.6f, %.6f, %.6f, %.6f]",
        //          frameIdx_, timestamp, x, y, z, q_x, q_y, q_z, q_w);

        try
        {
            cv_bridge::CvImagePtr cvPtr = cv_bridge::toCvCopy(egoImagePtr_, sensor_msgs::image_encodings::BGR8);
            cv::Mat image = cvPtr->image;
            videoWriter_ << image;

            // 设置视频文件的元数据
            // updateVideoMetadata(egoImagePtr_, frameCount_);
            // frameCount_++;
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    // 更新视频的元信息，可能需要调整
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
    ros::init(argc, argv, "VideoTrajRecorder");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    VideoTrajRecorder recorder;
    ros::waitForShutdown();
    return 0;
}