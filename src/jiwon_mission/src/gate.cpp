// mission 7: 차단기 미션
// 차단기 앞에서 정지 후 차단기가 올라가면 주행하는 미션

// Lidar (laser/scan) 전처리
// 외벽 구분을 위해 전방 기준 가까운 거리 내 Lidar Point 처리
// 특정 각도 내에 연속적으로 유효한 값 나타나면 정지 flag (지속)
// 해당 Lidar Points가 사라지면(올라가면) 주행 flag
// lidar point 가져올 때 로봇과 차단기 사이의 거리 처리

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

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

struct Point2D
{
    double x;
    double y;
};

// ===== 전역 ROS I/O =====
ros::Subscriber sub_scan_;
ros::Publisher  marker_pub_;
ros::Publisher  pub_cloud_;
ros::Publisher  pub_speed_;
laser_geometry::LaserProjection projector_;

// ===== 전역 파라미터 =====
double eps_          = 0.2;    // 클러스터 간 거리 [m]
int    min_samples_  = 5;      // DBSCAN 최소 점 수
double basic_speed_  = 1500.0; // 게이트 올라가 있을 때 기본 속도
double stop_speed_   = 0.0;    // 정지 속도

// 게이트 상태 히스테리시스
int gate_down_count_   = 0;
int gate_up_count_     = 0;
int gate_down_thresh_  = 3; // 연속 n프레임 이상 DOWN일 때 stop
int gate_up_thresh_    = 3; // 연속 n프레임 이상 UP일 때 go

enum GateState {
    GATE_UNKNOWN = 0,
    GATE_DOWN,
    GATE_UP
};

// ====== DBSCAN PCL KD-tree ======
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
        pt.z = 0.0f;  // 2D 라이다라서 0
        cloud->points.push_back(pt);
    }
    cloud->width    = cloud->points.size();
    cloud->height   = 1;
    cloud->is_dense = true;

    // 2) KD-tree 구성
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(cloud);

    int cluster_id = 0;

    // radiusSearch 결과 버퍼
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

        // eps_ 반경 안의 이웃 검색
        int found = kdtree.radiusSearch(
            searchPoint,
            static_cast<float>(eps_),
            neighbor_indices,
            neighbor_sq_dists,
            0 // max_nn=0 -> 제한 없음
        );

        // core point 조건 미달: noise (임시로 label -1 유지)
        if (found < min_samples_)
            continue;

        // 새 클러스터 생성
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

            if (labels[j] != -1) continue;

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
                    {
                        seed_set.push_back(m);
                    }
                }
            }
        }
    }

    return labels;
}

// ===== 게이트 상태 추정 =====
GateState estimateGateState(const std::vector<Point2D>& points,
                            const std::vector<int>& labels)
{
    if (points.empty() || labels.empty()) {
        // 아무것도 안 보이면 게이트가 없거나 올라가 있다고 가정
        return GATE_UP;
    }

    int max_label = *std::max_element(labels.begin(), labels.end());
    if (max_label <= 0)
        return GATE_UP;

    // 클러스터별 포인트 모으기
    std::vector<std::vector<Point2D>> clusters(max_label + 1);
    for (size_t i = 0; i < points.size(); ++i) {
        int cid = labels[i];
        if (cid <= 0) continue; // noise or unallocated
        clusters[cid].push_back(points[i]);
    }

    // 가장 가까운 클러스터 선택 (mean_x 최소)
    int best_id = -1;
    double best_x = std::numeric_limits<double>::max();

    for (int cid = 1; cid <= max_label; ++cid) {
        const auto& c = clusters[cid];
        if (c.size() < static_cast<size_t>(min_samples_))
            continue;

        double mean_x = 0.0;
        for (const auto& p : c) mean_x += p.x;
        mean_x /= c.size();

        if (mean_x < best_x) {
            best_x = mean_x;
            best_id = cid;
        }
    }

    // 유효 클러스터 없음 -> 게이트 안 보인다고 간주
    if (best_id < 0) {
        return GATE_UP;
    }

    const auto& cluster = clusters[best_id];

    // x, y variance -> orientation 판단
    double mx = 0.0;
    double my = 0.0;
    for (const auto& p : cluster) {
        mx += p.x;
        my += p.y;
    }
    mx /= cluster.size();
    my /= cluster.size();

    double var_x = 0.0;
    double var_y = 0.0;
    for (const auto& p : cluster) {
        double dx = p.x - mx;
        double dy = p.y - my;
        var_x += dx * dx;
        var_y += dy * dy;
    }
    var_x /= cluster.size();
    var_y /= cluster.size();

    // x: 전/후방, y: 좌우
    // var_y >> var_x -> 좌우로 누운 막대기 (게이트 내려온 상태)
    const double ratio = 3.0;
    if (var_y > ratio * var_x) {
        ROS_INFO_THROTTLE(0.5, "[gate_state] Gate DOWN (horizontal cluster detected)");
        return GATE_DOWN;
    } else {
        ROS_INFO_THROTTLE(0.5, "[gate_state] Gate UP or vertical object");
        return GATE_UP;
    }
}

// ===== 제어 =====
void pubGateSign(bool gate_down)
{
    std_msgs::Float64 speed_msg;

    if (gate_down) {
        speed_msg.data = stop_speed_; // 정지
        ROS_INFO_THROTTLE(1.0, "→ GATE DOWN: Stopping");
    } else {
        speed_msg.data = basic_speed_; // 전진
        ROS_INFO_THROTTLE(1.0, "→ GATE UP / No target, Moving forward");
    }

    pub_speed_.publish(speed_msg);
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

    // --- PointCloud2 퍼블리시 (디버그용) ---
    sensor_msgs::PointCloud2 cloud;
    projector_.projectLaser(*scan, cloud);
    pub_cloud_.publish(cloud);

    // --- DBSCAN ---
    std::vector<int> labels = dbscan(points);
    int max_label = 0;
    if (!labels.empty())
        max_label = *std::max_element(labels.begin(), labels.end());

    ROS_INFO_THROTTLE(1.0, "Clusters detected: %d", max_label);

    // --- 게이트 상태 추정 ---
    GateState gs = estimateGateState(points, labels);

    // 히스테리시스 (연속 프레임 카운트)
    bool gate_down_final = false;

    if (gs == GATE_DOWN) {
        gate_down_count_++;
        gate_up_count_ = 0;
    } else { // GATE_UP
        gate_up_count_++;
        gate_down_count_ = 0;
    }

    if (gate_down_count_ >= gate_down_thresh_) {
        gate_down_final = true;
    }
    if (gate_up_count_ >= gate_up_thresh_) {
        gate_down_final = false;
    }

    // --- 속도 명령 퍼블리시 ---
    pubGateSign(gate_down_final);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mission7_gate_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // 파라미터 로드 (없으면 기본값 사용)
    pnh.param("eps",              eps_,              eps_);
    pnh.param("min_samples",      min_samples_,      min_samples_);
    pnh.param("basic_speed",      basic_speed_,      basic_speed_);
    pnh.param("stop_speed",       stop_speed_,       stop_speed_);
    pnh.param("gate_down_thresh", gate_down_thresh_, gate_down_thresh_);
    pnh.param("gate_up_thresh",   gate_up_thresh_,   gate_up_thresh_);

    // Pub/Sub
    sub_scan_   = nh.subscribe<sensor_msgs::LaserScan>(
        "/scan", 1, scanCallback);
    pub_cloud_  = nh.advertise<sensor_msgs::PointCloud2>(
        "/scan_points", 1);
    pub_speed_  = nh.advertise<std_msgs::Float64>(
        "/commands/motor/speed", 1);
    marker_pub_ = nh.advertise<visualization_msgs::Marker>(
        "/gate_cluster_marker", 1);

    ROS_INFO("mission7_gate_node started.");
    ros::spin();
    return 0;
}
