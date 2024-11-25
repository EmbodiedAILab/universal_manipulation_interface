#include <cmath>
#include <fstream>
#include <chrono>

#include <boost/filesystem.hpp>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf/transform_listener.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseStamped.h>

extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/opt.h>
#include <libavutil/timecode.h>
#include <libavutil/dict.h>
#include <libswscale/swscale.h>
}

using namespace std;

class VideoWriter{
public:
    VideoWriter(){

    }
    ~VideoWriter(){

    }

    // static void my_logoutput(void* ptr, int level, const char* fmt, va_list vl) {
    //     ROS_INFO("Hello Log Output! Content = %s", fmt);
    // }

    void open(const char* output_filename) {
        // 初始化FFmpeg库
        av_log_set_level(AV_LOG_DEBUG);
        // av_log_set_callback(my_logoutput);  // 设置自定义的日志输出方法
        av_register_all();

//        AVFormatContext *fmt_ctx = nullptr;
//        AVStream *video_stream = nullptr;
//        AVCodecContext *codec_ctx = nullptr;
//        AVCodec *codec = nullptr;
//        AVFrame *frame = nullptr;

//        struct SwsContext *sws_ctx = nullptr;
       ROS_INFO("start");

        const int width = 640;
        const int height = 480;
        const int fps = 30;

        filename=output_filename;

        // 创建输出格式上下文
        avformat_alloc_output_context2(&fmt_ctx, nullptr, nullptr, output_filename);
        if (!fmt_ctx) {
            ROS_ERROR("Could not create output context");
            return ;
        }

        // 查找编码器
        codec = avcodec_find_encoder(AV_CODEC_ID_MPEG4);

        // 创建视频流
        video_stream = avformat_new_stream(fmt_ctx, codec);
        if (!video_stream) {
            ROS_ERROR("Could not create video stream");
            return ;
        }

        codec_ctx = avcodec_alloc_context3(codec);
        if (!codec_ctx) {
            ROS_ERROR("Could not allocate video codec context");
            return ;
        }

        codec_ctx->codec_id = codec->id;
        codec_ctx->bit_rate = 11080647;
        //codec_ctx->bit_rate = 400000;
        codec_ctx->width = width;
        codec_ctx->height = height;
        codec_ctx->time_base = (AVRational) {1, fps};
        codec_ctx->framerate = (AVRational) {fps, 1};
        codec_ctx->gop_size = 10;
        codec_ctx->max_b_frames = 0;
        codec_ctx->coder_type = AVMEDIA_TYPE_VIDEO;
        codec_ctx->pix_fmt = AV_PIX_FMT_YUV420P;
        ROS_INFO("codec_id: %d, pix_fmt: %d", codec_ctx->codec_id, codec_ctx->pix_fmt);
        codec_ctx->max_b_frames = 1;
        codec_ctx->pix_fmt = AV_PIX_FMT_YUVJ420P;
        AVDictionary *meta = nullptr;
        av_dict_alloc(meta);
        video_stream->metadata
        if (fmt_ctx->oformat->flags & AVFMT_GLOBALHEADER)
            codec_ctx->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;

        // video_stream->codecpar->codec_id = fmt_ctx->oformat->video_codec;
        // video_stream->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
        // video_stream->codecpar->width = codec_ctx->width;
        // video_stream->codecpar->height = codec_ctx->height;
        // video_stream->codecpar->format = codec_ctx->pix_fmt;
        // video_stream->time_base = codec_ctx->time_base;
        
        ROS_INFO("codec_type: %d, delay: %d", video_stream->codecpar->codec_type, video_stream->codecpar->video_delay);

        // 打开编码器
        if (avcodec_open2(codec_ctx, codec, nullptr) < 0) {
            ROS_ERROR("Could not open codec");
            return ;
        }

        int ret = avcodec_parameters_from_context(video_stream->codecpar, codec_ctx);
        if (ret < 0) {
            ROS_ERROR("Could not copy the stream parameters");
            return ;
        }
        avcodec_parameters_from_context(video_stream->codecpar, codecCtx);

        video_stream->codecpar->codec_id = fmt_ctx->oformat->video_codec;
        video_stream->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
        video_stream->codecpar->width = codec_ctx->width;
        video_stream->codecpar->height = codec_ctx->height;
        video_stream->codecpar->format = codec_ctx->pix_fmt;
        video_stream->time_base = codec_ctx->time_base;

        // 打开输出文件
        if (!(fmt_ctx->oformat->flags & AVFMT_NOFILE)) {
            if (avio_open(&fmt_ctx->pb, output_filename, AVIO_FLAG_WRITE) < 0) {
                ROS_ERROR("Could not open output file");
                return ;
            }
        }

        // 添加元数据
        ROS_INFO("dict: %d, start: %s", av_dict_count(video_stream->metadata), "");
        // av_dict_set(&fmt_ctx->metadata, "title", "My Video Title", 0);
        AVDictionary *vmetadata = nullptr;
        //av_dict_alloc(vmetadata);
        av_dict_set(&fmt_ctx->metadata, "timecode", "21:00:00:01", 0);
        // video_stream->metadata = vmetadata;
        // av_dict_set(&fmt_ctx->metadata, "timecode", "00:00:00:00", 0);
        // AVRational rate=(AVRational) {fps, 1};
        // int flag = 0;
        // int frame_start = 0;
        // AVTimecode tc;
        // if(av_timecode_init(&tc, rate, flag, frame_start, video_stream)<0){
        //     ROS_ERROR("Could not init timecode");
        //     return ;
        // }
        // char tc_char[AV_TIMECODE_STR_SIZE];
        // av_timecode_make_string(&tc, tc_char, tc.start);
        av_dict_set(&(video_stream->metadata) , "title", "My Stream Title", 0);
        ROS_INFO("dict: %d, start: %s", av_dict_count(video_stream->metadata), "");
        // av_dict_set(&video_stream->metadata, "timecode", tc_char, 0);
        // av_dict_set(&fmt_ctx->metadata, "timecode", tc_char, 0);
        // av_dict_set(&video_stream->metadata, "title", "My Video Stream Title", 0);
        // av_dict_set(&fmt_ctx->metadata, "creation_time", "2024-08-08T21:00:00.00Z", 0);
        // av_dict_set(&fmt_ctx->metadata, "title", "My Video Title", 0);
        ROS_INFO("dict: %d, start: %s", av_dict_count(fmt_ctx->streams[0]->metadata), "");
        // 写入文件头
        if (avformat_write_header(fmt_ctx, nullptr) < 0) {
            ROS_ERROR("Error occurred when writing header");
            return ;
        }

        // av_dict_set(&video_stream->metadata , "timecode", "00:00:00:00", 0);

        av_dump_format(fmt_ctx, 0, output_filename, 1);

        frame = av_frame_alloc();
        if (!frame) {
            ROS_ERROR("Could not allocate video frame");
            return ;
        }
        frame->format = codec_ctx->pix_fmt;
        frame->width = codec_ctx->width;
        frame->height = codec_ctx->height;

        if (av_frame_get_buffer(frame, 32) < 0) {
            ROS_ERROR("Could not allocate the video frame data");
            return ;
        }

        sws_ctx = sws_getContext(width, height, AV_PIX_FMT_BGR24,
                                 width, height, AV_PIX_FMT_YUV420P,
                                 SWS_BILINEAR, nullptr, nullptr, nullptr);

        ROS_INFO("end");
    }

    bool write(const sensor_msgs::ImageConstPtr& msg) {
        // 转换BGR到YUV
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return false;
        }
        cv::Mat img = cv_ptr->image;
        const int stride[] = {static_cast<int>(img.step[0])};
        sws_scale(sws_ctx, &img.data, stride, 0, codec_ctx->height, frame->data, frame->linesize);

        // frame->pts = pts++;
        frame->pts = av_rescale_q(frame->pts, codec_ctx->time_base, video_stream->time_base);
        // ROS_INFO("pts: %d, time_base: %d, vtime_base: %d",frame->pts, codec_ctx->time_base, video_stream->time_base);

        // 发送帧到编码器
        if (avcodec_send_frame(codec_ctx, frame) < 0) {
            ROS_ERROR("Error sending frame to encoder");
            return false;
        }

        // 接收编码后的包
        while (avcodec_receive_packet(codec_ctx, pkt) == 0) {
            av_packet_rescale_ts(pkt, codec_ctx->time_base, video_stream->time_base);
            pkt->stream_index = video_stream->index;
            ROS_INFO(" -- %s] packet = %6ld, pts: %6ld, dts: %6ld, duration: %5ld\n",
                av_get_media_type_string(fmt_ctx->streams[pkt->stream_index]->codecpar->codec_type),
                frame_number++, pkt->pts, pkt->dts, pkt->duration);

            // 写入包到输出文件
            if (av_interleaved_write_frame(fmt_ctx, pkt) < 0) {
                ROS_ERROR("Error while writing video frame");
                return false;
            }
            av_packet_unref(pkt);
        }
        return true;
    }

    void close(){
        av_dump_format(fmt_ctx, 0, filename, 1);

        // 写入文件尾
        av_write_trailer(fmt_ctx);
        if (fmt_ctx && !(fmt_ctx->oformat->flags & AVFMT_NOFILE)) {
            avio_closep(&fmt_ctx->pb);
        }
        ROS_INFO("write trailer end");
        // 释放资源
        av_frame_free(&frame);
        av_packet_free(&pkt);
        avformat_close_input(&fmt_ctx);
        avcodec_free_context(&codec_ctx);
        avformat_free_context(fmt_ctx);
        sws_freeContext(sws_ctx);
    }
private:
    AVFormatContext *fmt_ctx = nullptr;
    AVStream *video_stream = nullptr;
    AVCodec *codec = nullptr;
    AVCodecContext *codec_ctx = nullptr;
    AVFrame *frame = nullptr;
    AVFrame *pFrameRGB = nullptr;
    struct SwsContext *sws_ctx = nullptr;
    AVPacket* pkt= av_packet_alloc();
    int frame_number=0;
    int64_t pts=0;
    const char* filename;
};

class VideoTrajRecorder
{
public:
    VideoTrajRecorder() : it_(nh_) {
        geometry_msgs::Pose egoCamera2ArucoPose;
        // 根据自己的硬件来调整该参数
        egoCamera2ArucoPose.position.x = 0;
        egoCamera2ArucoPose.position.y = -0.195;
        egoCamera2ArucoPose.position.z = 0.230;
        egoCamera2ArucoPose.orientation = tf::createQuaternionMsgFromRollPitchYaw(-M_PI_2 - 0.5, 0, 0);
        tf2::convert(egoCamera2ArucoPose, egoCamera2ArucoMatrix_);

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
        while (ros::ok() && !arucoPoseFlag_) {
            ROS_ERROR_STREAM("Waiting for aruco pose, and the topic name you specified is " << arucoPoseSub_.getTopic());
            ros::spinOnce();
            ros::Duration(1.0).sleep();
        }

        // 查看data_folder是否存在
        boost::filesystem::path dataFolderPath = ros::package::getPath("record_data") + "/data_folder";
        if (!boost::filesystem::exists(dataFolderPath)) {
            ROS_WARN_STREAM(dataFolderPath << "doesn't exist, create it");
            // 创建 data_folder 文件夹
            if (boost::filesystem::create_directory(dataFolderPath)) {
                ROS_WARN_STREAM(dataFolderPath << "has been created successfully.");
            } else {
                ROS_WARN_STREAM("Failed to create the" << dataFolderPath);
                ros::shutdown();
            }
        }

        // 通过键盘控制开始或者结束
        ROS_INFO("Ready to record video and trajectory");
        while(ros::ok()) {
            if (kbhit()) {
                char c = getchar();
                // 开始记录，使用按键s(start)
                if (c == 's') {
                    startTime_ = std::chrono::system_clock::now();
                    std::time_t currentTime = std::chrono::system_clock::to_time_t(startTime_);
                    std::stringstream ss;
                    ss << std::put_time(std::localtime(&currentTime), "%Y-%m-%d_%H-%M-%S");

                    videoPath_ = ros::package::getPath("record_data") + "/data_folder/ego_" + ss.str() + ".mp4";
                    videoWriter_.open(videoPath_, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), imageFrequency, cv::Size(egoImagePtr_->width, egoImagePtr_->height), true);
                    // videoWriter_.open(videoPath_.c_str());
                   if (!videoWriter_.isOpened()) {
                       ROS_ERROR("Failed to open video writer");
                       ros::shutdown();
                   }

                    csvFilePath_ = ros::package::getPath("record_data") + "/data_folder/" + ss.str() + ".csv";
                    csvFile_.open(csvFilePath_);
                    if (!csvFile_.is_open()) {
                        ROS_ERROR("Failed to open file: %s", csvFilePath_.c_str());
                        ros::shutdown();
                    }
                    csvFile_ << "frame_idx,timestamp,state,is_lost,is_keyframe,x,y,z,q_x,q_y,q_z,q_w\n";
                    startRecord_ = true;
                    ROS_WARN("Record start");
                }
                // 结束记录，使用按键e(end)
                if (c == 'e') {
                    startRecord_ = false;
                    frameIdx_=0;
                    videoWriter_.release(); 
                    //videoWriter_.close();
                    ROS_WARN_STREAM("Video file has been saved to " << videoPath_);
                    if (csvFile_.is_open()) {
                        csvFile_.close();
                    }
                    ROS_WARN_STREAM("CSV file has been saved to " << csvFilePath_);
                    ROS_WARN("Record finished");
                }
            }
            ros::spinOnce();
            loopRate.sleep();
        }
    }

    ~VideoTrajRecorder() {
        videoWriter_.release();
        // videoWriter_.close();
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
    int frameIdx_=0;
    std::chrono::system_clock::time_point startTime_;

    image_transport::ImageTransport it_;
   cv::VideoWriter videoWriter_;
   // VideoWriter videoWriter_;
    std::string videoPath_;
    std::ofstream csvFile_;
    std::string csvFilePath_;
    
    sensor_msgs::ImageConstPtr egoImagePtr_;
    sensor_msgs::ImageConstPtr extImagePtr_;

    Eigen::Affine3d egoCamera2ArucoMatrix_;
    Eigen::Affine3d aruco2ExtCameraMatrix_;
    Eigen::Affine3d egoCamera2ExtCameraMatrix_;
    geometry_msgs::Pose egoCamera2ExtCameraPose_;

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
        if (!arucoPoseFlag_) {
            arucoPoseFlag_ = true;
            ROS_WARN("Received aruco pose");
        }
        if (startRecord_ == false) {
            return;
        }
        if (frameIdx_ == 0){
            firstTimestamp_ = msg->header.stamp.toSec();
        }

        // 记录ego camera相对于ext camera的位姿，而不是aruco相对于ext camera的位姿
        tf2::convert(msg->pose, aruco2ExtCameraMatrix_);
        egoCamera2ExtCameraMatrix_ = aruco2ExtCameraMatrix_ * egoCamera2ArucoMatrix_;
        tf2::convert(egoCamera2ExtCameraMatrix_, egoCamera2ExtCameraPose_);
        
        double timestamp = msg->header.stamp.toSec() - firstTimestamp_;
        double x = egoCamera2ExtCameraPose_.position.x;
        double y = egoCamera2ExtCameraPose_.position.y;
        double z = egoCamera2ExtCameraPose_.position.z;
        double q_x = egoCamera2ExtCameraPose_.orientation.x;
        double q_y = egoCamera2ExtCameraPose_.orientation.y;
        double q_z = egoCamera2ExtCameraPose_.orientation.z;
        double q_w = egoCamera2ExtCameraPose_.orientation.w;

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
        // try{
        //     videoWriter_.write(egoImagePtr_);
        // }catch (std::exception& e){
        //     ROS_ERROR("video write exception: %s", e.what());
        // }
        frameIdx_++;
    }

    // 更新视频的元信息，可能需要调整
    void updateVideoMetadata(sensor_msgs::ImageConstPtr& imagePtr, int frameCount) {
        std::ostringstream metadata;
        metadata << "Camera ID: " << CAMERA_ID << " | ";
        metadata << "Frame Time: " << std::fixed << std::setprecision(6) << imagePtr->header.stamp.toSec() << " | ";
        metadata << "Frame Count: " << frameCount;
        cv::Mat metadata_mat(1, metadata.str().length(), CV_8UC1, (void*)metadata.str().c_str());
//        videoWriter_.write(metadata_mat);
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