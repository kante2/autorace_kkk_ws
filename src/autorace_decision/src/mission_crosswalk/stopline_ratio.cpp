// ROI -> BEV -> Binary: bev_binary

// 횡단보도 차선 검출 로직
// 흰색 영역 추출ROI
// >>> ROI 내 흰색 pixel 비율 지정 <<<

// pub:: mission/stopline (bool type)
// 횡단보도 검출: True / 미검출: False
// True 받으면 
// >> True 던져주고 7초 후 False 던져주기 <<

// stopline_node.cpp
// Subscribe: /stopline/bev_binary (mono8 BEV binary)
// Publish:   /mission/stopline (std_msgs::Bool)

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

// == 전역 ==
ros::Publisher g_pub_stop;

bool   g_show_window          = true;
double g_stop_duration        = 7.0;   // [sec]
double g_white_ratio_threshold = 0.2; // [0~1] 전체 픽셀 중 흰 픽셀 비율

bool      g_stop_active       = false;
ros::Time g_stop_start_time   = ros::Time(0);

// ===== 전체 이미지에서 white ratio 계산 =====
double computeWhiteRatio(const cv::Mat& binary, cv::Mat& debug_img)
{
    const int h = binary.rows;
    const int w = binary.cols;

    if (w <= 0 || h <= 0)
    {
        debug_img = cv::Mat();
        return 0.0;
    }

    const int total_pixels = w * h;
    const int total_white  = cv::countNonZero(binary);
    const double white_ratio = static_cast<double>(total_white) /
                               static_cast<double>(total_pixels);

    cv::cvtColor(binary, debug_img, cv::COLOR_GRAY2BGR);

    char text[128];
    std::snprintf(text, sizeof(text), "white_ratio=%.3f thr=%.3f",
                  white_ratio, g_white_ratio_threshold);
    cv::putText(debug_img, text,
                cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(0, 255, 0), 2);

    return white_ratio;
}

// ===== stop 상태 업데이트 =====
void updateStopState(double white_ratio, const ros::Time& now)
{
    const bool can_start  = (!g_stop_active) &&
                            (white_ratio > g_white_ratio_threshold);
    const bool can_finish = g_stop_active &&
                            !g_stop_start_time.isZero() &&
                            (now - g_stop_start_time).toSec() >= g_stop_duration;

    if (can_start)
    {
        g_stop_active     = true;
        g_stop_start_time = now;
        ROS_INFO("[stopline_node] STOP TRIGGERED (white_ratio=%.3f)", white_ratio);
    }

    if (can_finish)
    {
        g_stop_active     = false;
        g_stop_start_time = ros::Time(0);
        ROS_INFO("[stopline_node] STOP RELEASED (duration done)");
    }
}

// ===== 콜백 =====
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        // /stopline/bev_binary: mono8
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (const cv_bridge::Exception& e)
    {
        ROS_WARN("[stopline_node] cv_bridge exception: %s", e.what());
        return;
    }

    const cv::Mat& binary = cv_ptr->image;
    if (binary.empty())
        return;

    // 1) 전체 이미지에서 white ratio 계산
    cv::Mat debug_img;
    double white_ratio = computeWhiteRatio(binary, debug_img);

    // 2) stop 상태 갱신
    ros::Time now = ros::Time::now();
    updateStopState(white_ratio, now);

    // 3) Bool 토픽 퍼블리시
    std_msgs::Bool stop_msg;
    stop_msg.data = g_stop_active;
    g_pub_stop.publish(stop_msg);

    // 4) 디버그 화면
    if (g_show_window)
    {
        cv::Mat vis_bin, vis_dbg;
        cv::resize(binary,   vis_bin, cv::Size(640, 480));
        cv::resize(debug_img, vis_dbg, cv::Size(640, 480));

        cv::imshow("stopline_binary", vis_bin);
        cv::imshow("stopline_debug",  vis_dbg);
        cv::waitKey(1);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "stopline_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // 파라미터 로드
    std::string image_topic = "/stopline/bev_binary";
    pnh.param<std::string>("image_topic", image_topic, image_topic);
    pnh.param("show_window",           g_show_window,          true);
    pnh.param("stop_duration",         g_stop_duration,        7.0);
    pnh.param("white_ratio_threshold", g_white_ratio_threshold, 0.2);

    // 퍼블리셔 / 서브스크라이버
    g_pub_stop = nh.advertise<std_msgs::Bool>("/mission/stopline", 1);
    ros::Subscriber sub_image = nh.subscribe(image_topic, 2, imageCallback);
    

    if (g_show_window)
    {
        cv::namedWindow("stopline_binary", cv::WINDOW_NORMAL);
        cv::namedWindow("stopline_debug",  cv::WINDOW_NORMAL);
        cv::resizeWindow("stopline_binary", 640, 480);
        cv::resizeWindow("stopline_debug",  640, 480);
    }

    ROS_INFO("stopline_node (white_ratio) started, subscribing: %s", image_topic.c_str());
    ros::spin();
    return 0;
}
