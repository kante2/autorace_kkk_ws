// src/autorace_perception/src/dbscan_gate.cpp

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <laser_geometry/laser_geometry.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Point.h>

#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <autorace_perception/GateCluster.h>

struct Point2D
{
    double x;
    double y;
};

// ===== 전역 ROS I/O =====
ros::Subscriber            sub_scan_;
ros::Publisher             pub_cluster_;
ros::Publisher             pub_cloud_;
ros::Publisher             marker_pub_;
laser_geometry::LaserProjection projector_;

// ===== 전역 파라미터 =====
double eps_         = 0.2;  // DBSCAN eps
int    min_samples_ = 5;    // DBSCAN 최소 점 수

// ================== DBSCAN (PCL KD-tree) ==================
std::vector<int> dbscan(const std::vector<Point2D>& points)
{
    const int n = static_cast<int>(points.size());
    std::vector<int> labels(n, -1); // -1: unallocated 또는 noise

    if (n == 0) return labels;

    using PointT = pcl::PointXYZ;

    // 1) Point2D -> PCL PointCloud 변환 (z=0)
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    cloud->points.reserve(n);

    for (const auto& p : points)
    {
        PointT pt;
        pt.x = static_cast<float>(p.x);
        pt.y = static_cast<float>(p.y);
        pt.z = 0.0f;
        cloud->points.push_back(pt);
    }
    cloud->width    = cloud->points.size();
    cloud->height   = 1;
    cloud->is_dense = true;

    // 2) KD-tree 구성
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(cloud);

    int cluster_id = 0;

    std::vector<int>   neighbor_indices;
    std::vector<float> neighbor_sq_dists;

    // 3) DBSCAN main loop
    for (int i = 0; i < n; ++i)
    {
        // 이미 어떤 클러스터에 속해 있으면 스킵
        if (labels[i] != -1)
            continue;

        PointT searchPoint = cloud->points[i];

        neighbor_indices.clear();
        neighbor_sq_dists.clear();

        int found = kdtree.radiusSearch(
            searchPoint,
            static_cast<float>(eps_),
            neighbor_indices,
            neighbor_sq_dists,
            0  // max_nn=0 -> 제한 없음
        );

        // core point 조건 미달: noise (label -1 유지)
        if (found < min_samples_)
            continue;

        // 새 클러스터 id
        ++cluster_id;
        labels[i] = cluster_id;

        // seed_set: 앞으로 확장할 point 인덱스
        std::vector<int> seed_set;
        seed_set.reserve(found);

        for (int idx : neighbor_indices)
        {
            if (idx == i) continue;
            seed_set.push_back(idx);
        }

        // BFS로 클러스터 확장
        for (size_t k = 0; k < seed_set.size(); ++k)
        {
            int j = seed_set[k];

            if (labels[j] != -1)
                continue;

            labels[j] = cluster_id;

            // j가 core point인지 확인
            searchPoint = cloud->points[j];

            neighbor_indices.clear();
            neighbor_sq_dists.clear();

            int found_j = kdtree.radiusSearch(
                searchPoint,
                static_cast<float>(eps_),
                neighbor_indices,
                neighbor_sq_dists,
                0
            );

            if (found_j >= min_samples_)
            {
                for (int m : neighbor_indices)
                {
                    if (labels[m] == -1)
                        seed_set.push_back(m);
                }
            }
        }
    }

    return labels;
}

// ================== Marker 퍼블리시 (옵션) ==================
void publishClusterMarker(const std::vector<Point2D>& cluster,
                          const std_msgs::Header& header)
{
    if (cluster.empty())
        return;

    visualization_msgs::Marker marker;
    marker.header = header;
    marker.ns = "gate_cluster";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = 0.02; // line width

    marker.color.r = 1.0;
    marker.color.g = 0.5;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker.pose.orientation.w = 1.0;

    for (const auto& p : cluster)
    {
        geometry_msgs::Point gp;
        gp.x = p.x;
        gp.y = p.y;
        gp.z = 0.0;
        marker.points.push_back(gp);
    }

    marker_pub_.publish(marker);
}

// ================== Scan Callback ==================
void scanCallback(const sensor_msgs::LaserScanConstPtr& scan)
{
    std::vector<Point2D> points;
    points.reserve(scan->ranges.size());

    double angle = scan->angle_min;

    // --- 포인트 필터링: 13cm ~ 40cm, 전방 기준 ±120도 ---
    for (const auto& r : scan->ranges)
    {
        if (!std::isfinite(r) || r < 0.13 || r > 0.4)
        {
            angle += scan->angle_increment;
            continue;
        }

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

    // 디버그용 PointCloud2
    sensor_msgs::PointCloud2 cloud;
    projector_.projectLaser(*scan, cloud);
    pub_cloud_.publish(cloud);

    autorace_perception::GateCluster msg;
    msg.header = scan->header;
    msg.valid  = false;
    msg.size   = 0;

    if (points.empty())
    {
        ROS_INFO_THROTTLE(1.0, "[perception] No points in ROI");
        pub_cluster_.publish(msg);
        return;
    }

    // --- DBSCAN ---
    std::vector<int> labels = dbscan(points);
    int max_label = 0;
    if (!labels.empty())
        max_label = *std::max_element(labels.begin(), labels.end());

    ROS_INFO_THROTTLE(1.0, "[perception] Clusters detected: %d", max_label);

    if (max_label <= 0)
    {
        pub_cluster_.publish(msg);
        return;
    }

    // 클러스터별 포인트 모으기
    std::vector<std::vector<Point2D>> clusters(max_label + 1);
    for (size_t i = 0; i < points.size(); ++i)
    {
        int cid = labels[i];
        if (cid <= 0) continue;
        clusters[cid].push_back(points[i]);
    }

    // 가장 가까운 클러스터 선택 (mean_x 최소)
    int    best_id = -1;
    double best_x  = std::numeric_limits<double>::max();

    for (int cid = 1; cid <= max_label; ++cid)
    {
        const auto& c = clusters[cid];
        if (c.size() < static_cast<size_t>(min_samples_))
            continue;

        double mean_x = 0.0;
        for (const auto& p : c) mean_x += p.x;
        mean_x /= c.size();

        if (mean_x < best_x)
        {
            best_x  = mean_x;
            best_id = cid;
        }
    }

    if (best_id < 0)
    {
        pub_cluster_.publish(msg);
        return;
    }

    // 선택된 클러스터에 대해 통계량 계산
    const auto& cluster = clusters[best_id];

    double mx = 0.0, my = 0.0;
    double min_x =  1e9, max_x = -1e9;
    double min_y =  1e9, max_y = -1e9;

    for (const auto& p : cluster)
    {
        mx += p.x;
        my += p.y;
        min_x = std::min(min_x, p.x);
        max_x = std::max(max_x, p.x);
        min_y = std::min(min_y, p.y);
        max_y = std::max(max_y, p.y);
    }
    mx /= cluster.size();
    my /= cluster.size();

    double var_x = 0.0, var_y = 0.0;
    for (const auto& p : cluster)
    {
        double dx = p.x - mx;
        double dy = p.y - my;
        var_x += dx * dx;
        var_y += dy * dy;
    }
    var_x /= cluster.size();
    var_y /= cluster.size();

    msg.valid  = true;
    msg.size   = static_cast<int32_t>(cluster.size());
    msg.mean_x = static_cast<float>(mx);
    msg.mean_y = static_cast<float>(my);
    msg.var_x  = static_cast<float>(var_x);
    msg.var_y  = static_cast<float>(var_y);
    msg.min_x  = static_cast<float>(min_x);
    msg.max_x  = static_cast<float>(max_x);
    msg.min_y  = static_cast<float>(min_y);
    msg.max_y  = static_cast<float>(max_y);

    pub_cluster_.publish(msg);

    // Marker 시각화 (선택된 클러스터)
    publishClusterMarker(cluster, scan->header);
}

// ================== main ==================
int main(int argc, char** argv)
{
    ros::init(argc, argv, "mission7_gate_perception_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.param("eps",         eps_,         eps_);
    pnh.param("min_samples", min_samples_, min_samples_);

    sub_scan_    = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, scanCallback);
    pub_cluster_ = nh.advertise<autorace_perception::GateCluster>("/mission7/gate_cluster", 1);
    pub_cloud_   = nh.advertise<sensor_msgs::PointCloud2>("/scan_points", 1);
    marker_pub_  = nh.advertise<visualization_msgs::Marker>("/gate_cluster_marker", 1);

    ROS_INFO("mission7_gate_perception_node started.");
    ros::spin();
    return 0;
}
