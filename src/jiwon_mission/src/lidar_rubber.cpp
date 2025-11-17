// 동혁오빠 코드

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64.h>
#include <laser_geometry/laser_geometry.h>

#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>

struct Point2D
{
    double x;
    double y;
};

class ClusterFollower
{
public:
    ClusterFollower(ros::NodeHandle& nh)
    {
        sub_scan_  = nh.subscribe("/scan", 1, &ClusterFollower::scanCallback, this);
        marker_pub_ = nh.advertise<visualization_msgs::Marker>("/dbscan_lines", 300);
        pub_cloud_  = nh.advertise<sensor_msgs::PointCloud2>("/scan_points", 300);
        pub_speed_  = nh.advertise<std_msgs::Float64>("/commands/motor/speed", 1);
        pub_servo_  = nh.advertise<std_msgs::Float64>("/commands/servo/position", 1);

        // 파라미터 (필요하면 rosparam 으로 뽑아도 됨)
        eps_          = 0.2;   // 클러스터 간 거리 (20~25cm)
        min_samples_  = 2;     // 최소 점 수
        follow_speed_ = 1500;  // 기본 주행 속도
        min_speed_    = 1100;  // 인식 안 될 때 속도
        k_yaw_        = 1.2;   // 조향 비율
    }

private:
    // ROS I/O
    ros::Subscriber sub_scan_;
    ros::Publisher  marker_pub_;
    ros::Publisher  pub_cloud_;
    ros::Publisher  pub_speed_;
    ros::Publisher  pub_servo_;

    laser_geometry::LaserProjection projector_;

    // 파라미터
    double eps_;
    int    min_samples_;
    double follow_speed_;
    double min_speed_;
    double k_yaw_;

    // ================== DBSCAN ==================
    std::vector<int> dbscan(const std::vector<Point2D>& points)
    {
        const int n = static_cast<int>(points.size());
        std::vector<int> labels(n, -1);
        int cluster_id = 0;

        auto dist = [](const Point2D& a, const Point2D& b)
        {
            return std::hypot(a.x - b.x, a.y - b.y);
        };

        for (int i = 0; i < n; ++i)
        {
            if (labels[i] != -1)
                continue;

            std::vector<int> neighbors;
            neighbors.reserve(n);

            for (int j = 0; j < n; ++j)
            {
                if (dist(points[i], points[j]) <= eps_)
                    neighbors.push_back(j);
            }

            if (static_cast<int>(neighbors.size()) < min_samples_)
                continue;

            ++cluster_id;
            labels[i] = cluster_id;

            std::vector<int> seed_set = neighbors;
            for (size_t k = 0; k < seed_set.size(); ++k)
            {
                int j = seed_set[k];
                if (labels[j] != -1)
                    continue;

                labels[j] = cluster_id;

                std::vector<int> j_neighbors;
                j_neighbors.reserve(n);
                for (int m = 0; m < n; ++m)
                {
                    if (dist(points[j], points[m]) <= eps_)
                        j_neighbors.push_back(m);
                }

                if (static_cast<int>(j_neighbors.size()) >= min_samples_)
                {
                    seed_set.insert(seed_set.end(),
                                    j_neighbors.begin(),
                                    j_neighbors.end());
                }
            }
        }

        return labels;
    }

    // ================== Scan Callback ==================
    void scanCallback(const sensor_msgs::LaserScanConstPtr& scan)
    {
        std::vector<Point2D> points;
        points.reserve(scan->ranges.size());

        double angle = scan->angle_min;

        // --- 포인트 필터링: 13cm ~ 90cm, 전방 기준 ±120도 ---
        for (const auto& r : scan->ranges)
        {
            if (!std::isfinite(r) || r < 0.13 || r > 0.9)
            {
                angle += scan->angle_increment;
                continue;
            }

            // 라디안 -> 도
            double angle_deg = angle * 180.0 / M_PI;
            if (angle_deg < 0.0)
                angle_deg += 360.0;

            // 60도 ~ 300도 사이만 사용 (±120도)
            if (angle_deg < 60.0 || angle_deg > 300.0)
            {
                angle += scan->angle_increment;
                continue;
            }

            double x = r * std::cos(angle);
            double y = r * std::sin(angle);

            points.push_back({x, y});
            angle += scan->angle_increment;
        }

        if (points.empty())
        {
            ROS_WARN_THROTTLE(1.0, "No valid points detected — moving straight slowly");
            publishMinimalSpeed(true);
            return;
        }

        // --- PointCloud2 퍼블리시 ---
        sensor_msgs::PointCloud2 cloud;
        projector_.projectLaser(*scan, cloud);
        pub_cloud_.publish(cloud);

        // --- DBSCAN ---
        std::vector<int> labels = dbscan(points);
        int max_label = 0;
        if (!labels.empty())
            max_label = *std::max_element(labels.begin(), labels.end());

        ROS_INFO_THROTTLE(1.0, "Clusters detected: %d", max_label);

        // --- 좌/우 클러스터 중심 계산 ---
        double left_x_sum  = 0.0;
        double left_y_sum  = 0.0;
        double right_x_sum = 0.0;
        double right_y_sum = 0.0;
        int left_cnt       = 0;
        int right_cnt      = 0;

        // --- 클러스터 중심 + Marker ---
        for (int c = 1; c <= max_label; ++c)
        {
            double cx = 0.0;
            double cy = 0.0;
            int count = 0;

            for (size_t i = 0; i < points.size(); ++i)
            {
                if (labels[i] == c)
                {
                    cx += points[i].x;
                    cy += points[i].y;
                    ++count;
                }
            }

            if (count <= 0)
                continue;

            cx /= static_cast<double>(count);
            cy /= static_cast<double>(count);

            if (cy > 0.0)
            {
                ++left_cnt;
                left_x_sum += cx;
                left_y_sum += cy;
            }
            else
            {
                ++right_cnt;
                right_x_sum += cx;
                right_y_sum += cy;
            }

            visualization_msgs::Marker marker;
            marker.header.frame_id = scan->header.frame_id;
            marker.header.stamp    = ros::Time::now();
            marker.ns              = "cluster";
            marker.id              = c;
            marker.type            = visualization_msgs::Marker::SPHERE;
            marker.action          = visualization_msgs::Marker::ADD;
            marker.pose.position.x = cx;
            marker.pose.position.y = cy;
            marker.pose.position.z = 0.0;
            marker.scale.x = marker.scale.y = marker.scale.z = 0.1;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            marker.lifetime = ros::Duration(0.1);

            marker_pub_.publish(marker);
        }

        // --- 목표 좌표 계산 ---
        double target_x  = 0.0;
        double target_y  = 0.0;
        double cmd_speed = follow_speed_;

        if (left_cnt > 0 && right_cnt > 0)
        {
            double left_x  = left_x_sum  / left_cnt;
            double left_y  = left_y_sum  / left_cnt;
            double right_x = right_x_sum / right_cnt;
            double right_y = right_y_sum / right_cnt;

            int total_cnt      = left_cnt + right_cnt;
            double weight_left  = static_cast<double>(right_cnt) / total_cnt;
            double weight_right = static_cast<double>(left_cnt)  / total_cnt;

            target_x = weight_left * left_x  + weight_right * right_x;
            target_y = weight_left * left_y  + weight_right * right_y;
        }
        else if (left_cnt > 0)
        {
            target_x = left_x_sum  / left_cnt;
            target_y = (left_y_sum / left_cnt) - 0.12; // 왼쪽 → 오른쪽 피하기
        }
        else if (right_cnt > 0)
        {
            target_x = right_x_sum  / right_cnt;
            target_y = (right_y_sum / right_cnt) + 0.12; // 오른쪽 → 왼쪽 피하기
        }
        else
        {
            ROS_WARN_THROTTLE(1.0, "No clusters detected — moving straight slowly");
            publishMinimalSpeed(true);
            return;
        }

        // --- Yaw → 조향 ---
        double yaw = std::atan2(target_y, target_x);
        yaw *= k_yaw_;

        double yaw_deg = yaw * 180.0 / M_PI;
        double steering;

        if (yaw_deg < 0.0)
        {
            // 왼쪽
            steering = 0.545 + (yaw_deg / 25.0) * 0.545;
        }
        else
        {
            // 오른쪽
            steering = 0.545 + (yaw_deg / 20.0) * (1.0 - 0.545);
        }

        steering = std::clamp(steering, 0.0, 1.0);

        // --- 퍼블리시 ---
        std_msgs::Float64 speed_msg;
        std_msgs::Float64 servo_msg;

        speed_msg.data = cmd_speed;
        servo_msg.data = steering;

        pub_speed_.publish(speed_msg);
        pub_servo_.publish(servo_msg);

        ROS_INFO_THROTTLE(1.0, "Speed=%.2f, Steering=%.2f", cmd_speed, steering);
    }

    // ================== 속도 제어 ==================
    void publishMinimalSpeed(bool move_forward = true)
    {
        std_msgs::Float64 speed_msg;
        std_msgs::Float64 servo_msg;

        if (move_forward)
        {
            speed_msg.data = min_speed_; // 천천히 전진
            servo_msg.data = 0.545;      // 직진 (원래 값 0.57165)
            ROS_INFO_THROTTLE(1.0, "→ No target, moving forward slowly");
        }
        else
        {
            speed_msg.data = 0.0;        // 완전 정지
            servo_msg.data = 0.545;
            ROS_INFO_THROTTLE(1.0, "→ Stopping due to no target");
        }

        pub_speed_.publish(speed_msg);
        pub_servo_.publish(servo_msg);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cluster_follower");
    ros::NodeHandle nh;

    ClusterFollower node(nh);
    ros::spin();

    return 0;
}
