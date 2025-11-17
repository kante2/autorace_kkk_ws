// src/autorace_control/src/mission_gate_control.cpp

#include <ros/ros.h>
#include <std_msgs/Float64.h>

// ===== 전역 ROS I/O =====
ros::Subscriber sub_distance_;
ros::Publisher  pub_speed_;

// ===== 전역 파라미터 및 상태 =====
double basic_speed_    = 1500.0;  // 게이트 없거나 멀리 있을 때 속도
double stop_speed_     = 0.0;     // 정지 속도
double stop_distance_  = 0.20;    // [m] 이 거리 안이면 정지

int gate_down_thresh_  = 3;       // 연속 n프레임 이하 거리일 때만 DOWN 확정
int gate_up_thresh_    = 3;       // 연속 n프레임 이상 위 거리일 때만 UP 확정

int gate_down_count_ = 0;
int gate_up_count_   = 0;

// ================== Callback ==================
void distanceCallback(const std_msgs::Float64ConstPtr& msg)
{
    double gate_dist = msg->data;

    bool down_candidate = false;

    if (gate_dist >= 0.0 && gate_dist <= stop_distance_)
    {
        down_candidate = true;
        ROS_INFO_THROTTLE(0.5,
            "[control] Gate distance = %.3f m <= %.3f m -> DOWN candidate",
            gate_dist, stop_distance_);
    }
    else
    {
        ROS_INFO_THROTTLE(0.5,
            "[control] Gate distance = %.3f m -> UP candidate (no gate or far)",
            gate_dist);
    }

    if (down_candidate)
    {
        gate_down_count_++;
        gate_up_count_ = 0;
    }
    else
    {
        gate_up_count_++;
        gate_down_count_ = 0;
    }

    bool gate_down_final = false;
    if (gate_down_count_ >= gate_down_thresh_)
        gate_down_final = true;
    if (gate_up_count_ >= gate_up_thresh_)
        gate_down_final = false;

    std_msgs::Float64 speed_msg;

    if (gate_down_final)
    {
        speed_msg.data = stop_speed_;
        ROS_INFO_THROTTLE(1.0,
            "→ [control] GATE DOWN (final) : STOP");
    }
    else
    {
        speed_msg.data = basic_speed_;
        ROS_INFO_THROTTLE(1.0,
            "→ [control] GATE UP (final) : MOVE");
    }

    pub_speed_.publish(speed_msg);
}

// ================== main ==================
int main(int argc, char** argv)
{
    ros::init(argc, argv, "mission7_gate_control");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.param("basic_speed",     basic_speed_,     basic_speed_);
    pnh.param("stop_speed",      stop_speed_,      stop_speed_);
    pnh.param("stop_distance",   stop_distance_,   stop_distance_);
    pnh.param("gate_down_thresh", gate_down_thresh_, gate_down_thresh_);
    pnh.param("gate_up_thresh",   gate_up_thresh_,   gate_up_thresh_);

    sub_distance_ = nh.subscribe<std_msgs::Float64>(
        "/mission7/gate_distance", 1, distanceCallback);
    pub_speed_ = nh.advertise<std_msgs::Float64>(
        "/commands/motor/speed", 1);

    ROS_INFO("mission7_gate_control started.");
    ros::spin();
    return 0;
}
