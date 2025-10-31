#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <morai_msgs/CtrlCmd.h>
#include <tf/transform_datatypes.h>

#include <opencv2/opencv.hpp>

#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <limits>

class PPRoiGapFollow {
public:
  PPRoiGapFollow()
  : nh_("~")
  {
    // ---- Params (라이다 ROI / 조향 / 속도 / 종료 판정 / 시각화) ----
    nh_.param<double>("wheelbase", wheelbase_L_, 2.6);
    nh_.param<double>("lookahead", lfd_, 3.5);                 // Pure Pursuit Ld
    nh_.param<double>("target_kmh", target_vel_kmh_, 10.0);     // 고정 속도 (km/h)

    nh_.param<double>("roi_depth", roi_depth_, 8.0);            // 전방 ROI 깊이 [m]
    nh_.param<double>("roi_width", roi_width_, 4.0);            // 전방 ROI 너비(좌우 합) [m]
    nh_.param<double>("front_fov_deg", front_fov_deg_, 120.0);  // 정면에서 좌/우 총 FOV
    nh_.param<double>("point_max_range", pt_max_range_, 10.0);  // 유효 포인트 최대 거리 [m]
    nh_.param<int>("min_points_threshold", min_pts_thr_, 12);   // 좌/우 각각 최소 포인트 개수
    nh_.param<double>("stop_timeout", stop_timeout_sec_, 1.0);  // ROI 내 재검출 없으면 정지 판정 시간[s]

    nh_.param<bool>("show_debug", show_debug_, true);           // OpenCV 디버그 창
    nh_.param<double>("laser_yaw_offset_deg", laser_yaw_offset_deg_, 0.0);
    laser_yaw_offset_rad_ = laser_yaw_offset_deg_ * M_PI / 180.0;
    // ---- Pubs/Subs ----
    scan_sub_ = nh_.subscribe("/scan", 1, &PPRoiGapFollow::scanCB, this);
    imu_sub_  = nh_.subscribe("/imu", 1, &PPRoiGapFollow::imuCB, this); // yaw가 꼭 필요하진 않지만 남겨둠

    cmd_pub_      = nh_.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd", 2);
    local_path_pub_= nh_.advertise<nav_msgs::Path>("/local_path", 1, true);
    stop_pub_     = nh_.advertise<std_msgs::Bool>("/stop_flag", 1, true);

    // MORAI 제어 모드: velocity(2)
    cmd_.longlCmdType = 2;
    cmd_.accel  = 0.0;
    cmd_.brake  = 0.0;
    cmd_.velocity = 0.0;

    last_seen_obstacle_ = ros::Time::now(); // 초기에는 바로 정지되지 않도록 현재시각으로
    makeEmptyPathMsg();                     // RVIZ 확인용 빈 Path 초기화

    ROS_INFO("[pp_roi] started. ROI depth=%.1f width=%.1f fov=%.0f deg",
             roi_depth_, roi_width_, front_fov_deg_);
  }

  void spin() {
    ros::Rate rate(20);
    while (ros::ok()) {
      ros::spinOnce();

      if (!have_scan_) {
        ROS_INFO_THROTTLE(1.0, "[pp_roi] waiting /scan ...");
        rate.sleep();
        continue;
      }

      // 1) 라이다 포인트를 전방 ROI로 필터 → 좌/우로 분할
      std::vector<cv::Point2f> left_pts, right_pts;
      extractRoiPoints(last_scan_, left_pts, right_pts);

      const int left_n  = (int)left_pts.size();
      const int right_n = (int)right_pts.size();
      const int total_n = left_n + right_n;

      const ros::Time now = ros::Time::now();
      bool obstacle_in_roi = total_n > 0;

      if (obstacle_in_roi) last_seen_obstacle_ = now;

      // 2) 1초 이상 장애물 미검출이면 정지 플래그 ON + 차량 정지
      const bool stop_condition = ((now - last_seen_obstacle_).toSec() > stop_timeout_sec_);
      publishStopFlag(stop_condition);
      if (stop_condition) {
        publishStopCmd();
        drawDebug(left_pts, right_pts, /*has_wp=*/false, cv::Point2f(), "STOP (no obstacle > 1s)");
        rate.sleep();
        continue;
      }

      // 3) 좌/우 최소 포인트 개수 만족 시 중앙 갭의 waypoint 계산
      bool has_wp = false;
      cv::Point2f wp_local(0.f, 0.f); // 차량 로컬(base_link) 기준 (x:전방+, y:좌+)
      if (left_n >= min_pts_thr_ && right_n >= min_pts_thr_) {
        has_wp = computeGapWaypoint(left_pts, right_pts, wp_local);
      }

      // 좌/우 포인트가 부족하면, lookahead 거리에서 가운데(y=0)로 안전 폴백
      if (!has_wp) {
        wp_local.x = std::max(1.0, lfd_);
        wp_local.y = 0.0;
        has_wp = true;
      }

      // 4) Pure Pursuit 조향 (바디 프레임 목표점 사용)
      double Ld = std::hypot(wp_local.x, wp_local.y);
      if (Ld < 0.5) Ld = 0.5; // 너무 가까우면 수치 불안정 방지
      const double theta = std::atan2(wp_local.y, wp_local.x);
      const double delta = std::atan2(2.0 * wheelbase_L_ * std::sin(theta), Ld);

      cmd_.steering = -(delta * 10);
      cmd_.velocity = target_vel_kmh_; // MORAI velocity 모드는 [m/s]가 아니라 [km/h] 필드로 쓰는 케이스가 많아 사용자 코드에 맞춤
      cmd_.accel = 0.0;
      cmd_.brake = 0.0;
      cmd_pub_.publish(cmd_);

      // 5) RVIZ path(한 점짜리 wayPoint를 Path로 표시)
      publishSingleWaypointPath(wp_local);

      // 6) 디버그 시각화
      char msgbuf[128];
      std::snprintf(msgbuf, sizeof(msgbuf), "steer=%.3frad, L/R=%d/%d", delta, left_n, right_n);
      drawDebug(left_pts, right_pts, /*has_wp=*/true, wp_local, msgbuf);

      ROS_INFO_THROTTLE(0.5, "[pp_roi] wp(%.2f, %.2f) steer=%.3f L/R=%d/%d",
                        wp_local.x, wp_local.y, delta, left_n, right_n);

      rate.sleep();
    }
  }

private:
  // ------------------------- Callbacks -------------------------
  void scanCB(const sensor_msgs::LaserScan& msg) {
    last_scan_ = msg;
    have_scan_ = true;
  }

  void imuCB(const sensor_msgs::Imu& msg) {
    // yaw가 꼭 필요하진 않지만 남겨둠 (필요시 사용)
    const auto &q = msg.orientation;
    tf::Quaternion tfq(q.x, q.y, q.z, q.w);
    yaw_ = tf::getYaw(tfq);
    have_yaw_ = true;
  }

  // ------------------------- ROI 처리 -------------------------
    // void extractRoiPoints(const sensor_msgs::LaserScan& scan,
    //                         std::vector<cv::Point2f>& left_pts,
    //                         std::vector<cv::Point2f>& right_pts)
    // {
    //     left_pts.clear(); right_pts.clear();

    //     const double half_width = roi_width_ * 0.5;
    //     const double min_x = 0.0;
    //     const double max_x = roi_depth_;

    //     const double fov_rad = front_fov_deg_ * M_PI / 180.0;
    //     const double half_fov = 0.5 * fov_rad;

    //     // LaserScan 기준: angle=0이 전방(+x), +각은 좌측(+y)
    //     const int N = (int)scan.ranges.size();
    //     for (int i=0; i<N; ++i) {
    //     const double ang = scan.angle_min + i * scan.angle_increment;

    //     // 전방 FOV 제한
    //     if (ang < -half_fov || ang > half_fov) continue;

    //     const float r = scan.ranges[i];
    //     if (!std::isfinite(r) || r <= scan.range_min || r > scan.range_max) continue;
    //     if (r > pt_max_range_) continue;

    //     const double x = r * std::cos(ang);
    //     const double y = r * std::sin(ang);

    //     // 전방 사각형 ROI: x∈[0, roi_depth], |y| ≤ roi_width/2
    //     if (x < min_x || x > max_x) continue;
    //     if (std::fabs(y) > half_width) continue;

    //     if (y >= 0.0) left_pts.emplace_back((float)x, (float)y);
    //     else          right_pts.emplace_back((float)x, (float)y);
    //     }
    // }
    // void extractRoiPoints(const sensor_msgs::LaserScan& scan,
    //                     std::vector<cv::Point2f>& left_pts,
    //                     std::vector<cv::Point2f>& right_pts)
    // {
    //     left_pts.clear(); right_pts.clear();

    //     const double half_width = roi_width_ * 0.5;
    //     const double min_x = 0.0;
    //     const double max_x = roi_depth_;

    //     const double fov_rad  = front_fov_deg_ * M_PI / 180.0;
    //     const double half_fov = 0.5 * fov_rad;

    //     // LaserScan 기준: angle=0이 라이다 프레임 전방. 오프셋으로 바디 프레임 전방(+x) 정렬
    //     const int N = static_cast<int>(scan.ranges.size());
    //     for (int i = 0; i < N; ++i) {
    //         const double ang_raw = scan.angle_min + i * scan.angle_increment;

    //         // ★ 라이다→바디 yaw 오프셋 적용 (예: 뒤로 달렸다면 +pi)
    //         double ang = ang_raw + laser_yaw_offset_rad_;

    //         // [-pi, pi] 정규화(권장)
    //         if (ang >  M_PI) ang -= 2.0 * M_PI;
    //         if (ang < -M_PI) ang += 2.0 * M_PI;

    //         // 전방 FOV 제한은 '보정된 ang' 기준
    //         if (ang < -half_fov || ang > half_fov) continue;

    //         const float r = scan.ranges[i];
    //         if (!std::isfinite(r) || r <= scan.range_min || r > scan.range_max) continue;
    //         if (r > pt_max_range_) continue;

    //         // 바디 프레임(+x 전방, +y 좌측) 좌표
    //         const double x = -(r * std::cos(ang));
    //         const double y = -(r * std::sin(ang));

    //         // 전방 사각형 ROI: x∈[0, roi_depth], |y| ≤ roi_width/2
    //         if (x < min_x || x > max_x) continue;
    //         if (std::fabs(y) > half_width) continue;

    //         if (y >= 0.0) left_pts.emplace_back(static_cast<float>(x), static_cast<float>(y));
    //         else          right_pts.emplace_back(static_cast<float>(x), static_cast<float>(y));
    //     }
    // }
    void extractRoiPoints(const sensor_msgs::LaserScan& scan,
                        std::vector<cv::Point2f>& left_pts,
                        std::vector<cv::Point2f>& right_pts)
    {
        left_pts.clear(); right_pts.clear();

        const double half_width = roi_width_ * 0.5;
        const double min_x = 0.0;
        const double max_x = roi_depth_;

        const int N = static_cast<int>(scan.ranges.size());
        if (N == 0) return;

        const double ang_min = scan.angle_min;
        const double d_ang   = scan.angle_increment;

        // ---- 1) 인덱스 영역으로 FOV 정의 (정면=0rad 기준) ----
        const double fov_rad  = front_fov_deg_ * M_PI / 180.0;   // 총 FOV
        const int    k_half   = std::max(0, (int)std::floor(0.5 * fov_rad / d_ang));

        // 정면(바디 프레임 0rad)이 되는 스캔 인덱스 (라이다->바디 yaw 오프셋 반영)
        // angle[i] (바디기준) = angle_min + i*d_ang + laser_yaw_offset_rad_
        // 0 = angle_min + i0*d_ang + offset  ->  i0 = -(angle_min + offset)/d_ang
        int i0 = (int)std::llround( -(ang_min + laser_yaw_offset_rad_) / d_ang );
        // 원형 버퍼 보정
        auto wrap = [N](int i){ i %= N; if (i < 0) i += N; return i; };
        i0 = wrap(i0);

        // ---- 2) 전방 FOV에 해당하는 인덱스들만 순회 (랩어라운드 지원) ----
        const int i_start = i0 - k_half;
        const int i_end   = i0 + k_half;

        auto process_index = [&](int i)
        {
            i = wrap(i);
            const float r = scan.ranges[i];
            if (!std::isfinite(r) || r <= scan.range_min || r > scan.range_max) return;

            // ---- 3) 각도는 인덱스로부터 재계산 (오프셋 포함) ----
            const double ang = ang_min + i * d_ang + laser_yaw_offset_rad_;
            // (필요하면 정규화) // if (ang > M_PI) ang -= 2*M_PI; if (ang < -M_PI) ang += 2*M_PI;

            if (r > pt_max_range_) return;

            // ---- 4) 바디 프레임 좌표 (+x 전방, +y 좌측) ----
            const double x = r * std::cos(ang);
            const double y = r * std::sin(ang);

            // ---- 5) 전방 사각형 ROI ----
            if (x < min_x || x > max_x) return;
            if (std::fabs(y) > half_width) return;

            if (y >= 0.0) left_pts.emplace_back((float)x, (float)y);
            else          right_pts.emplace_back((float)x, (float)y);
        };

        // i_start..i_end를 모두 처리 (랩어라운드 포함)
        for (int i = i_start; i <= i_end; ++i) process_index(i);
    }

  // 좌/우 각각 “센터라인에 가장 가까운” 장애물 경계를 찾아 그 사이 중앙을 waypoint로 설정
  bool computeGapWaypoint(const std::vector<cv::Point2f>& left_pts,
                          const std::vector<cv::Point2f>& right_pts,
                          cv::Point2f& wp_out)
  {
    // x-슬랩(전방 띠)을 하나 잡고(예: x in [2m, roi_depth]), 그 구간에서 좌/우 경계(y최솟값/최댓값)에 가까운 점 선택
    const float x_min = std::max<float>(1.5f, (float)std::min(lfd_, roi_depth_*0.25)); // 너무 가까운 x는 제외
    const float x_max = (float)roi_depth_;

    // 왼쪽( y>0 ): 센터라인 y=0 에 가장 가까운(= y가 최소) 점
    float best_left_y  = std::numeric_limits<float>::infinity();
    float best_left_x  = 0.f;
    for (auto &p : left_pts) {
      if (p.x < x_min || p.x > x_max) continue;
      if (p.y < best_left_y) { best_left_y = p.y; best_left_x = p.x; }
    }

    // 오른쪽( y<0 ): 센터라인에 가장 가까운(= y가 최대) 점 (y는 음수이므로 값이 큰 쪽이 0에 가깝다)
    float best_right_y = -std::numeric_limits<float>::infinity();
    float best_right_x = 0.f;
    for (auto &p : right_pts) {
      if (p.x < x_min || p.x > x_max) continue;
      if (p.y > best_right_y) { best_right_y = p.y; best_right_x = p.x; }
    }

    if (!std::isfinite(best_left_y) || !std::isfinite(best_right_y)) {
      return false;
    }

    // 양쪽 경계가 너무 가까우면(차선 폭이 너무 좁음) 안전하게 가운데로 가되 x를 줄임
    const float lane_width = best_left_y - best_right_y; // (양수 기대)
    float x_wp = std::min(best_left_x, best_right_x);
    if (lane_width < 0.6f) x_wp = std::max(1.0f, (float)lfd_);

    // 최종 waypoint: 두 경계의 가운데
    const float y_wp = 0.5f * (best_left_y + best_right_y);
    // lookahead와 조화: x_wp가 너무 작으면 고정 Ld 사용
    if (x_wp < 0.5f) x_wp = std::max(1.0f, (float)lfd_);

    wp_out = cv::Point2f(x_wp, y_wp);
    return true;
  }

  // ------------------------- Outputs -------------------------
  void publishStopFlag(bool stop_on) {
    std_msgs::Bool b; b.data = stop_on;
    stop_pub_.publish(b);
  }

  void publishStopCmd() {
    cmd_.steering = 0.0;
    cmd_.velocity = 0.0;
    cmd_.accel = 0.0;
    cmd_.brake = 0.0;
    cmd_pub_.publish(cmd_);
  }

  void makeEmptyPathMsg() {
    nav_msgs::Path p;
    p.header.frame_id = "base_link"; // 바디 프레임 기준 waypoint를 보이기 위함
    p.header.stamp = ros::Time::now();
    local_path_pub_.publish(p);
  }

  void publishSingleWaypointPath(const cv::Point2f& wp_local) {
    nav_msgs::Path p;
    p.header.frame_id = "base_link";
    p.header.stamp = ros::Time::now();

    geometry_msgs::PoseStamped ps;
    ps.header = p.header;
    ps.pose.position.x = wp_local.x;
    ps.pose.position.y = wp_local.y;
    ps.pose.orientation.w = 1.0;

    p.poses.push_back(ps);
    local_path_pub_.publish(p);
  }

  // ------------------------- Debug Draw -------------------------
  void drawDebug(const std::vector<cv::Point2f>& left_pts,
                 const std::vector<cv::Point2f>& right_pts,
                 bool has_wp, const cv::Point2f& wp,
                 const std::string& status_text)
  {
    if (!show_debug_) return;

    // 간단한 탑뷰 스케치 (pixel-per-meter)
    const int W = 600, H = 600;           // 12m x 12m 캔버스
    const float ppm = 50.0f;              // 1 m = 50 px
    cv::Mat img(H, W, CV_8UC3, cv::Scalar(30,30,30));

    auto toPix = [&](float x, float y){
      // 차량을 하단 중앙 근처에 두어 전방을 위로 보이게: (x:전방+)를 -y화, (y:좌+)를 +x화
      float X = W*0.5f + y*ppm;
      float Y = H - x*ppm - 50.0f; // 아래쪽 여백 50 px
      return cv::Point((int)std::round(X), (int)std::round(Y));
    };

    // 좌/우 ROI 영역 채우기
    const float half_w = roi_width_/2.f;
    cv::Point p1 = toPix(0.f, -half_w);
    cv::Point p2 = toPix(roi_depth_, -half_w);
    cv::Point p3 = toPix(roi_depth_,  0.f);
    cv::Point p4 = toPix(0.f,  0.f);
    std::vector<cv::Point> right_poly = {p1,p2,p3,p4}; // 오른쪽( y<0 ) 영역 (빨강)
    cv::fillConvexPoly(img, right_poly, cv::Scalar(60,60,180), cv::LINE_AA);

    cv::Point p5 = toPix(0.f, 0.f);
    cv::Point p6 = toPix(roi_depth_, 0.f);
    cv::Point p7 = toPix(roi_depth_,  half_w);
    cv::Point p8 = toPix(0.f,  half_w);
    std::vector<cv::Point> left_poly = {p5,p6,p7,p8};  // 왼쪽( y>0 ) 영역 (파랑)
    cv::fillConvexPoly(img, left_poly, cv::Scalar(180,60,60), cv::LINE_AA);

    // 좌/우 포인트 찍기
    for (auto &p : left_pts)  cv::circle(img, toPix(p.x, p.y), 2, cv::Scalar(255,120,120), -1, cv::LINE_AA);
    for (auto &p : right_pts) cv::circle(img, toPix(p.x, p.y), 2, cv::Scalar(120,120,255), -1, cv::LINE_AA);

    // waypoint 표시
    if (has_wp) {
      cv::circle(img, toPix(wp.x, wp.y), 5, cv::Scalar(120,255,120), -1, cv::LINE_AA);
      cv::putText(img, "WP", toPix(wp.x, wp.y - 0.2f), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(120,255,120), 1, cv::LINE_AA);
    }

    // 차량 위치/방향(간단 아이콘)
    cv::arrowedLine(img, toPix(0.f,0.f), toPix(1.0f,0.f), cv::Scalar(200,200,200), 2, cv::LINE_AA, 0, 0.2);

    // 상태 텍스트
    cv::putText(img, status_text, cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(220,220,220), 1, cv::LINE_AA);

    cv::imshow("roi_debug", img);
    cv::waitKey(1);
  }

private:
  ros::NodeHandle nh_;

  // pubs/subs
  ros::Subscriber scan_sub_;
  ros::Subscriber imu_sub_;
  ros::Publisher  cmd_pub_;
  ros::Publisher  local_path_pub_;
  ros::Publisher  stop_pub_;

  // states
  sensor_msgs::LaserScan last_scan_;
  bool have_scan_{false};

  bool  have_yaw_{false};
  double yaw_{0.0};

  // control params
  double wheelbase_L_;
  double lfd_;
  double target_vel_kmh_;

  // ROI params
  double roi_depth_;
  double roi_width_;
  double front_fov_deg_;
  double pt_max_range_;
  int    min_pts_thr_;

  // 멤버 변수 - 라이다 뒤집기
  double laser_yaw_offset_deg_{0.0};
  double laser_yaw_offset_rad_{0.0};


  // stop condition
  double     stop_timeout_sec_;
  ros::Time  last_seen_obstacle_;

  // cmd msg
  morai_msgs::CtrlCmd cmd_;

  // debug
  bool show_debug_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pp_roi_gap_follow");
  PPRoiGapFollow node;
  node.spin();
  return 0;
}
