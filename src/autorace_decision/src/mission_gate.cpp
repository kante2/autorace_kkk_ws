// src/autorace_decision/src/mission_gate.cpp

#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <autorace_perception/GateCluster.h>

// ===== 전역 ROS I/O =====
ros::Subscriber sub_cluster_;
ros::Publisher  pub_distance_;

// ===== 전역 파라미터 =====
double bar_ratio_ = 3.0;   // var_y > bar_ratio_ * var_x 이면 가로 막대기로 판단

// ================== Callback ==================
void clusterCallback(const autorace_perception::GateClusterConstPtr& msg)
{
    std_msgs::Float64 dist_msg;
    dist_msg.data = -1.0;  // 기본값: 게이트 없음

    if (!msg->valid || msg->size < 1)
    {
        ROS_INFO_THROTTLE(1.0,
            "[decision] No valid cluster -> gate_distance = -1");
        pub_distance_.publish(dist_msg);
        return;
    }

    double vx = msg->var_x;
    double vy = msg->var_y;

    bool is_horizontal = (vy > bar_ratio_ * vx);

    if (is_horizontal)
    {
        // 게이트로 판단 → gate까지 거리 = mean_x
        dist_msg.data = msg->mean_x;
        ROS_INFO_THROTTLE(0.5,
            "[decision] Gate detected: distance x = %.3f m (var_y=%.4f, var_x=%.4f)",
            msg->mean_x, vy, vx);
    }
    else
    {
        // 클러스터는 있지만 가로 막대기가 아니면 gate로 안 본다
        ROS_INFO_THROTTLE(0.5,
            "[decision] Non-horizontal cluster (var_y=%.4f, var_x=%.4f) -> gate_distance = -1",
            vy, vx);
    }

    pub_distance_.publish(dist_msg);
}

// ================== main ==================
int main(int argc, char** argv)
{
    ros::init(argc, argv, "mission7_gate_decision");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.param("bar_ratio", bar_ratio_, bar_ratio_);

    sub_cluster_ = nh.subscribe<autorace_perception::GateCluster>(
        "/mission7/gate_cluster", 1, clusterCallback);
    pub_distance_ = nh.advertise<std_msgs::Float64>(
        "/mission7/gate_distance", 1);

    ROS_INFO("mission7_gate_decision started.");
    ros::spin();
    return 0;
}
