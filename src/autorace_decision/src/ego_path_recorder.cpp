// src/ego_csv_recorder_kante.cpp
#include <ros/ros.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <fstream>
#include <iomanip>   // setprecision

static std::ofstream g_csv;
static std::ofstream g_log;
static bool g_open_csv_flag = false;
static bool g_wrote_header_flag = false;

void CB_ego(const morai_msgs::EgoVehicleStatus::ConstPtr& ego_msg)
{
    if(!g_open_csv_flag){
        g_log << "[FATAL] CSV file not open!\n";
        return;
    }

    if(!g_wrote_header_flag){
        g_csv << "x,y,yaw\n";
        g_wrote_header_flag = true;
        g_log << "[INFO] CSV header written.\n";
    }

    // double corrected_heading = ego_msg->heading - 61.0 * M_PI / 180.0;
    // double corrected_heading = ego_msg->heading - 61.0;

    g_csv << std::fixed << std::setprecision(6)
          << ego_msg->position.x << ","
          << ego_msg->position.y << ","
          << ego_msg->heading - 61.0 << "\n";
        //   << ego_msg->heading << "\n";

    g_log << "[INFO] Recorded: x=" << ego_msg->position.x
          << ", y=" << ego_msg->position.y
          << ", yaw=" << ego_msg->heading - 61.0<< "\n";
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ego_csv_recorder_kante");
    ros::NodeHandle nh("~");

    // 경로 수정 필요 
    // CSV 파일 경로
    std::string csv_file = "/home/autonav/aim_ws/src/roscpp_morai/data/ego_topic.csv";

    // 로그 파일 경로
    std::string log_file = "/home/autonav/aim_ws/src/roscpp_morai/data/ego_topic_log.txt";

    // CSV 파일 열기
    g_csv.open(csv_file, std::ios::out | std::ios::trunc);
    if(!g_csv){
        ROS_FATAL("Cannot open csv_file: %s", csv_file.c_str());
        return 1;
    }
    g_open_csv_flag = true;

    // 로그 파일 열기
    g_log.open(log_file, std::ios::out | std::ios::trunc);
    if(!g_log){
        ROS_FATAL("Cannot open log_file: %s", log_file.c_str());
        return 1;
    }

    g_log << "[INFO] Recording /Ego_topic to CSV: " << csv_file << "\n";

    // /Ego_topic 구독
    ros::Subscriber sub = nh.subscribe("/Ego_topic", 1, CB_ego);

    ROS_INFO_STREAM("[ego_csv_recorder_kante] Started recording to " << csv_file);

    ros::spin();

    // 종료 시 파일 닫기
    g_csv.close();
    g_log.close();

    return 0;
}
