#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

using namespace std;

class VideoAndTrajRecorder
{
public:
    VideoAndTrajRecorder() {

    }

private:
    ros::NodeHandle nh_;
    string amrNs_;
    ros::Subscriber jointStateSub_;
    bool jontStateFlag_ = false;
    
    // 回调函数
    // void jointStatesCallback(const sensor_msgs::JointStateConstPtr& joint_states);
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "VideoAndTrajRecorder");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    VideoAndTrajRecorder recorder;
    ros::waitForShutdown();
    return 0;
}