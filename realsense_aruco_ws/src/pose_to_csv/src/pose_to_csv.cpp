#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <iomanip>

class PoseToCSV
{
public:
    PoseToCSV(ros::NodeHandle& nh) : first_timestamp_(0.0), first_message_received_(false)
    {
        nh.param<std::string>("pose_topic", pose_topic_, "/pose");
        nh.param<std::string>("csv_file_path", csv_file_path_, "pose_output.csv");

        csv_file_.open(csv_file_path_);
        if (!csv_file_.is_open()) {
            ROS_ERROR("Failed to open file: %s", csv_file_path_.c_str());
            ros::shutdown();
        }
        csv_file_ << "frame_idx,timestamp,x,y,z,q_x,q_y,q_z,q_w\n";
        frame_idx_ = 0;

        pose_sub_ = nh.subscribe(pose_topic_, 10, &PoseToCSV::poseCallback, this);

        ROS_INFO("Subscribed to topic: %s", pose_topic_.c_str());
        ROS_INFO("Writing to file: %s", csv_file_path_.c_str());
    }

    ~PoseToCSV()
    {
        if (csv_file_.is_open()) {
            csv_file_.close();
        }
    }

private:
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        if (!first_message_received_) {
            first_timestamp_ = msg->header.stamp.toSec();
            first_message_received_ = true;
        }

        frame_idx_++;
        double timestamp = msg->header.stamp.toSec() - first_timestamp_; 

        double x = msg->pose.position.x;
        double y = msg->pose.position.y;
        double z = msg->pose.position.z;
        double q_x = msg->pose.orientation.x;
        double q_y = msg->pose.orientation.y;
        double q_z = msg->pose.orientation.z;
        double q_w = msg->pose.orientation.w;

        csv_file_ << frame_idx_ << ","
                  << std::fixed << std::setprecision(6) << timestamp << ","
                  << x << "," << y << "," << z << ","
                  << q_x << "," << q_y << "," << q_z << "," << q_w << "\n";

        csv_file_.flush();

        ROS_INFO("Frame: %d, Time: %.6f, Position: [%.6f, %.6f, %.6f], Orientation: [%.6f, %.6f, %.6f, %.6f]",
                 frame_idx_, timestamp, x, y, z, q_x, q_y, q_z, q_w);
    }

    ros::Subscriber pose_sub_;
    std::ofstream csv_file_;
    std::string pose_topic_;
    std::string csv_file_path_;
    int frame_idx_;
    double first_timestamp_;
    bool first_message_received_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_to_csv");
    ros::NodeHandle nh("~");

    ROS_INFO("Starting pose_to_csv node");

    PoseToCSV pose_to_csv(nh);

    ros::spin();
    return 0;
}
