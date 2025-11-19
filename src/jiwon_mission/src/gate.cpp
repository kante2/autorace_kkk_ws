// mission 7: 차단기 미션
// 차단기 앞에서 정지 후 차단기가 올라가면 주행하는 미션

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

// ================== 기본 타입 ==================
struct Point2D
{
    double x;
    double y;
};

// 게이트 상태
enum GateState {
    GATE_UNKNOWN = 0,
    GATE_DOWN,
    GATE_UP
};

// ===== 전역 ROS I/O =====
ros::Subscriber sub_scan_;
ros::Publisher  marker_pub_;
ros::Publisher  pub_cloud_;
ros::Publisher  pub_speed_;
laser_geometry::LaserProjection projector_;

// ===== 전역 파라미터 =====
double eps_            = 0.2;    // 클러스터 간 거리 [m]
int    min_samples_    = 10;      // DBSCAN 최소 점 수
double basic_speed_    = 1000.0; // 게이트 올라가 있을 때 기본 속도
double stop_speed_     = 0.0;    // 정지 속도
double gate_stop_dist_ = 2.0;    // 로봇-게이트 중심 거리 임계값 [m]

// 게이트 상태 히스테리시스
int gate_down_count_   = 0;
int gate_up_count_     = 0;
int gate_down_thresh_  = 3; // 연속 n프레임 이상 DOWN일 때 stop
int gate_up_thresh_    = 3; // 연속 n프레임 이상 UP일 때 go

// Lidar Clustering (DBSCAN)
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

// 클러스터 중심점 계산 함수
bool compute_CenterPoint(const std::vector<Point2D>& cluster,
                         double& mx, double& my)
{
    if (cluster.empty()) {
        mx = my = 0.0;
        return false;
    }

    mx = 0.0;
    my = 0.0;
    for (const auto& p : cluster) {
        mx += p.x;
        my += p.y;
    }
    mx /= cluster.size();
    my /= cluster.size();
    return true;
}

// 가장 가까운 게이트 클러스터 선택 
bool compute_GateCluster(const std::vector<Point2D>& points,
                         const std::vector<int>& labels,
                         std::vector<Point2D>& gate_cluster,
                         double& mx, double& my)
{
    gate_cluster.clear();
    mx = my = 0.0;

    if (points.empty() || labels.empty())
        return false;

    int max_label = *std::max_element(labels.begin(), labels.end());
    if (max_label <= 0)
        return false;

    // 클러스터별 포인트 모으기
    std::vector<std::vector<Point2D>> clusters(max_label + 1);
    for (size_t i = 0; i < points.size(); ++i) {
        int cid = labels[i];
        if (cid <= 0) continue; // noise or unallocated
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

        double cx, cy;
        if (!compute_CenterPoint(c, cx, cy))
            continue;

        if (cx < best_x) {
            best_x = cx;
            best_id = cid;
        }
    }

    if (best_id < 0)
        return false;

    gate_cluster = clusters[best_id];
    compute_CenterPoint(gate_cluster, mx, my);

    return true;
}

// 로봇-게이트 거리 계산
double compute_Distance(double x, double y)
{
    return std::sqrt(x * x + y * y);
}

// 게이트 상태 계산 (가로로 긴 막대형 인지)
GateState compute_GateState(const std::vector<Point2D>& gate_cluster)
{
    if (gate_cluster.empty())
        return GATE_UP;  // 아무것도 안 보이면 UP 가정

    double mx, my;
    if (!compute_CenterPoint(gate_cluster, mx, my))
        return GATE_UP;

    double var_x = 0.0;
    double var_y = 0.0;
    for (const auto& p : gate_cluster) {
        double dx = p.x - mx;
        double dy = p.y - my;
        var_x += dx * dx;
        var_y += dy * dy;
    }
    var_x /= gate_cluster.size();
    var_y /= gate_cluster.size();

    double ratio = 2.0;  // var_y >> var_x 이면 좌우로 긴 막대기
    if (var_y > ratio * var_x) {
        ROS_INFO_THROTTLE(0.5, "[gate_state] Gate DOWN (horizontal cluster detected)");
        return GATE_DOWN;
    } else {
        ROS_INFO_THROTTLE(0.5, "[gate_state] Gate UP or vertical object");
        return GATE_UP;
    }
}

// 속도 제어 (거리 + 게이트 상태)
void pubGateSign(bool stop)
{
    std_msgs::Float64 speed_msg;

    if (stop) {
        speed_msg.data = stop_speed_; // 정지
        ROS_INFO_THROTTLE(1.0, "→ GATE DOWN & CLOSE: Stopping");
    } else {
        speed_msg.data = basic_speed_; // 전진
        ROS_INFO_THROTTLE(1.0, "→ GATE UP or FAR: Moving forward");
    }

    pub_speed_.publish(speed_msg);
}

bool SpeedControl(bool gate_down_final, double gate_dist)
{
    bool should_stop = false;

    if (gate_down_final &&
        std::isfinite(gate_dist) &&
        gate_dist <= gate_stop_dist_)
    {
        should_stop = true;
    }

    ROS_INFO_THROTTLE(0.5,
        "[mission7] gate_down_final=%d, dist=%.3f, stop_dist=%.3f -> %s",
        gate_down_final ? 1 : 0,
        gate_dist,
        gate_stop_dist_,
        should_stop ? "STOP" : "GO");

    pubGateSign(should_stop);
    return should_stop;
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
        if (!std::isfinite(r) || r < 0.10 || r > 0.5)
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

    // --- (1) Lidar Clustering ---
    std::vector<int> labels = dbscan(points);
    int max_label = 0;
    if (!labels.empty())
        max_label = *std::max_element(labels.begin(), labels.end());

    ROS_INFO_THROTTLE(1.0, "Clusters detected: %d", max_label);

    // --- (2) 게이트 클러스터 선택 + 중심점 계산 ---
    std::vector<Point2D> gate_cluster;
    double gate_cx = 0.0, gate_cy = 0.0;
    bool has_gate_cluster =
        compute_GateCluster(points, labels, gate_cluster, gate_cx, gate_cy);

    // --- (3) 거리 & 게이트 상태 처리 ---
    double   gate_dist = std::numeric_limits<double>::infinity();
    GateState gs       = GATE_UP;

    if (has_gate_cluster) {
        gate_dist = compute_Distance(gate_cx, gate_cy);
        ROS_INFO_THROTTLE(0.5,
            "[gate] center=(%.3f, %.3f), dist=%.3f",
            gate_cx, gate_cy, gate_dist);

        gs = compute_GateState(gate_cluster);
    } else {
        ROS_INFO_THROTTLE(0.5, "[gate] no valid gate cluster -> assume UP");
        gs = GATE_UP;
    }

    // --- (4) 히스테리시스 적용 ---
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

    // --- (5) 속도 제어 (거리 + 게이트 상태) ---
    SpeedControl(gate_down_final, gate_dist);
}

// ================== main ==================
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
    pnh.param("gate_stop_dist",   gate_stop_dist_,   gate_stop_dist_);

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
