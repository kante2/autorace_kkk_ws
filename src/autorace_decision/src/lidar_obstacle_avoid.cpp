#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <vector>
#include <string>
#include <cmath>
#include <limits>
#include <algorithm>

struct Candidate {
  std::string name;
  std::vector<std::pair<double,double>> segments_deg_len; // (heading_deg, length_m)
  float color[4]; // r,g,b,a
};

static inline double deg2rad(double d){ return d * M_PI / 180.0; }
static inline double clamp(double v, double lo, double hi){ return std::max(lo, std::min(hi, v)); }

class ScanAvoidPlanner {
public:
  ScanAvoidPlanner(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh)
  {
    frame_id_ = pnh_.param<std::string>("frame_id", "base_link");
    forward_length_ = pnh_.param("forward_length", 3.0);
    seg1_length_    = pnh_.param("seg1_length", 1.5);
    seg2_length_    = pnh_.param("seg2_length", 1.5);
    sample_step_    = pnh_.param("sample_step", 0.05);
    footprint_r_    = pnh_.param("footprint_radius", 0.25);
    safety_margin_  = pnh_.param("safety_margin", 0.10);
    beam_downsample_= std::max(1, pnh_.param("beam_downsample", 2));
    straight_bias_  = pnh_.param("straight_bias", 0.2);
    enable_follow_  = pnh_.param("follow", false);
    follow_v_       = pnh_.param("follow_linear", 0.6);
    follow_k_yaw_   = pnh_.param("follow_k_yaw", 1.5);

    // 후보 경로 정의
    candidates_.push_back(Candidate{"FWD", {{0.0, forward_length_}}, {0.7f,0.7f,0.7f,1.0f}});
    candidates_.push_back(Candidate{"L30", {{30.0, seg1_length_}, {0.0, seg2_length_}}, {0.2f,0.8f,0.2f,1.0f}});
    candidates_.push_back(Candidate{"L60", {{60.0, seg1_length_}, {0.0, seg2_length_}}, {0.0f,0.5f,0.0f,1.0f}});
    candidates_.push_back(Candidate{"R30", {{-30.0,seg1_length_}, {0.0, seg2_length_}}, {0.2f,0.6f,1.0f,1.0f}});
    candidates_.push_back(Candidate{"R60", {{-60.0,seg1_length_}, {0.0, seg2_length_}}, {0.0f,0.2f,0.8f,1.0f}});

    // publishers
    for(size_t i=0;i<candidates_.size();++i){
      path_pubs_.push_back(pnh_.advertise<nav_msgs::Path>("path_"+candidates_[i].name, 1, true));
    }
    sel_pub_  = pnh_.advertise<nav_msgs::Path>("selected_path", 1, true);
    mark_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("markers", 1, true);
    if(enable_follow_) cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    sub_ = nh_.subscribe("scan", 1, &ScanAvoidPlanner::scanCb, this);

    ROS_INFO_STREAM("scan_avoid_paths(C++): frame_id="<<frame_id_<<", follow="<<(enable_follow_?"true":"false"));
  }

private:
  // ===== 콜백 =====
  void scanCb(const sensor_msgs::LaserScanConstPtr& scan){
    // 1) 후보 경로 생성 (각 경로: base_link 기준 (x,y) 포인트 배열)
    std::vector<std::vector<std::pair<double,double>>> paths;
    paths.reserve(candidates_.size());
    for(const auto& c : candidates_) paths.push_back(buildPath(c.segments_deg_len));

    // 2) 라이다 포인트로 변환(다운샘플)
    std::vector<std::pair<double,double>> obs = scanToPoints(*scan);

    // 3) 각 경로 충돌/스코어 평가
    std::vector<double> scores(paths.size(), -1e9);
    std::vector<double> min_dists(paths.size(), std::numeric_limits<double>::infinity());
    std::vector<bool> feasible(paths.size(), false);

    const double inflate = footprint_r_ + safety_margin_;
    for(size_t i=0;i<paths.size();++i){
      double min_d = std::numeric_limits<double>::infinity();
      if(obs.empty()){
        feasible[i] = true;
      }else{
        // 경로 포인트별 최근접 장애물 거리
        for(const auto& pt : paths[i]){
          double px = pt.first, py = pt.second;
          double local_min = std::numeric_limits<double>::infinity();
          for(const auto& ob : obs){
            double dx = px - ob.first;
            double dy = py - ob.second;
            double d  = std::hypot(dx, dy);
            if(d < local_min) local_min = d;
          }
          if(local_min < min_d) min_d = local_min;
        }
        feasible[i] = (min_d > inflate);
      }
      min_dists[i] = obs.empty() ? 1e6 : min_d;

      double s = (std::isfinite(min_dists[i]) ? min_dists[i] : 1e6) * 10.0;
      if(candidates_[i].name == "FWD") s += straight_bias_;
      scores[i] = s;
    }

    // 4) 최적 경로 선택
    int best_idx = -1;
    double best_score = -1e9;
    for(size_t i=0;i<scores.size();++i){
      if(!feasible[i]) continue;
      if(scores[i] > best_score){
        best_score = scores[i];
        best_idx = static_cast<int>(i);
      }
    }

    // 5) 시각화 발행
    publishPaths(paths, best_idx);
    publishMarkers(paths, best_idx);

    // 6) (옵션) 간단 추종
    if(enable_follow_) follow(paths, best_idx);
  }

  // ===== 경로 생성 =====
  std::vector<std::pair<double,double>> buildPath(const std::vector<std::pair<double,double>>& segs_deg_len){
    std::vector<std::pair<double,double>> pts;
    pts.reserve(256);
    double x=0.0, y=0.0;
    for(const auto& seg : segs_deg_len){
      const double hdg = deg2rad(seg.first);
      const double len = seg.second;
      double s = 0.0;
      while(s < len){
        double px = x + s*std::cos(hdg);
        double py = y + s*std::sin(hdg);
        pts.emplace_back(px, py);
        s += sample_step_;
      }
      x += len*std::cos(hdg);
      y += len*std::sin(hdg);
    }
    pts.emplace_back(x,y); // 끝점 보장
    return pts;
  }

  // ===== LaserScan → 장애물 점(로봇 기준) =====
  std::vector<std::pair<double,double>> scanToPoints(const sensor_msgs::LaserScan& scan){
    std::vector<std::pair<double,double>> pts;
    pts.reserve(scan.ranges.size()/beam_downsample_ + 4);
    const double a0 = scan.angle_min;
    const double inc = scan.angle_increment;
    const double rmin = scan.range_min, rmax = scan.range_max;
    for(size_t i=0;i<scan.ranges.size(); i += static_cast<size_t>(beam_downsample_)){
      const float r = scan.ranges[i];
      if(std::isfinite(r) && r > rmin && r < rmax){
        const double ang = a0 + inc*static_cast<double>(i);
        pts.emplace_back(r*std::cos(ang), r*std::sin(ang));
      }
    }
    return pts;
  }

  // ===== Path/Marker 발행 =====
  nav_msgs::Path toPathMsg(const std::vector<std::pair<double,double>>& pts){
    nav_msgs::Path p;
    p.header.frame_id = frame_id_;
    p.header.stamp = ros::Time::now();
    p.poses.reserve(pts.size());
    for(const auto& xy : pts){
      geometry_msgs::PoseStamped ps;
      ps.header = p.header;
      ps.pose.position.x = xy.first;
      ps.pose.position.y = xy.second;
      p.poses.push_back(ps);
    }
    return p;
  }

  void publishPaths(const std::vector<std::vector<std::pair<double,double>>>& paths, int best_idx){
    for(size_t i=0;i<paths.size();++i){
      path_pubs_[i].publish(toPathMsg(paths[i]));
    }
    if(best_idx >= 0) sel_pub_.publish(toPathMsg(paths[best_idx]));
    else              sel_pub_.publish(nav_msgs::Path()); // 빈 경로
  }

  void publishMarkers(const std::vector<std::vector<std::pair<double,double>>>& paths, int best_idx){
    visualization_msgs::MarkerArray arr;
    ros::Time now = ros::Time::now();

    for(size_t i=0;i<candidates_.size();++i){
      // 라인
      visualization_msgs::Marker line;
      line.header.frame_id = frame_id_;
      line.header.stamp = now;
      line.ns = "avoid_paths";
      line.id = static_cast<int>(i);
      line.type = visualization_msgs::Marker::LINE_STRIP;
      line.action = visualization_msgs::Marker::ADD;
      line.scale.x = (static_cast<int>(i)==best_idx) ? 0.06 : 0.03;

      float r=candidates_[i].color[0], g=candidates_[i].color[1], b=candidates_[i].color[2], a=candidates_[i].color[3];
      if(static_cast<int>(i)==best_idx){
        r = clamp(r+0.2f,0.f,1.f);
        g = clamp(g+0.2f,0.f,1.f);
        b = clamp(b+0.2f,0.f,1.f);
      }
      line.color.r=r; line.color.g=g; line.color.b=b; line.color.a=a;

      for(const auto& pt : paths[i]){
        geometry_msgs::Point p;
        p.x = pt.first; p.y = pt.second; p.z = 0.0;
        line.points.push_back(p);
      }
      arr.markers.push_back(line);

      // 텍스트
      visualization_msgs::Marker text;
      text.header.frame_id = frame_id_;
      text.header.stamp = now;
      text.ns = "avoid_labels";
      text.id = 100 + static_cast<int>(i);
      text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::Marker::ADD;
      text.scale.z = 0.18;
      text.color.r = 1.0; text.color.g = 1.0; text.color.b = 1.0; text.color.a = 0.9;
      text.text = candidates_[i].name;
      if(!paths[i].empty()){
        text.pose.position.x = paths[i].front().first + 0.1;
        text.pose.position.y = paths[i].front().second+ 0.1;
        text.pose.position.z = 0.1;
      }
      arr.markers.push_back(text);
    }
    mark_pub_.publish(arr);
  }

  // ===== 간단 추종 (/cmd_vel) =====
  void follow(const std::vector<std::vector<std::pair<double,double>>>& paths, int best_idx){
    geometry_msgs::Twist cmd;
    if(best_idx < 0 || paths[best_idx].empty()){
      cmd_pub_.publish(geometry_msgs::Twist());
      return;
    }
    // 약간 앞쪽 포인트를 목표로
    size_t idx = std::min<size_t>(5, paths[best_idx].size()-1);
    double tx = paths[best_idx][idx].first;
    double ty = paths[best_idx][idx].second;
    double yaw = std::atan2(ty, tx); // base_link 기준
    cmd.linear.x  = follow_v_;
    cmd.angular.z = clamp(follow_k_yaw_ * yaw, -1.2, 1.2);
    cmd_pub_.publish(cmd);
  }

private:
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_;
  std::vector<ros::Publisher> path_pubs_;
  ros::Publisher sel_pub_, mark_pub_, cmd_pub_;

  std::string frame_id_;
  double forward_length_, seg1_length_, seg2_length_;
  double sample_step_;
  double footprint_r_, safety_margin_;
  int    beam_downsample_;
  double straight_bias_;
  bool   enable_follow_;
  double follow_v_, follow_k_yaw_;

  std::vector<Candidate> candidates_;
};

int main(int argc, char** argv){
  ros::init(argc, argv, "scan_avoid_paths_cpp");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ScanAvoidPlanner node(nh, pnh);
  ros::spin();
  return 0;
}
