#include <ros/ros.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <morai_msgs/CtrlCmd.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <algorithm>

// 구조체: 웨이포인트
struct Waypoint { double x, y, yaw; };

// 전역 변수
std::vector<Waypoint> g_waypoints;
morai_msgs::EgoVehicleStatus g_ego;
bool g_have_ego = false;
ros::Publisher g_pub_cmd;

// 파라미터
double g_wheelbase = 2.7;  // m
double g_v_set     = 40.0;  // m/s
double g_lookahead = 20.0;  // LD [m]

// 가장 가까운 인덱스 찾기
int nearestIndex(const std::vector<Waypoint>& wps, double x, double y){
    if(wps.empty()) return -1;
    int best = 0;
    double bd = 1e18;
    for(size_t i=0;i<wps.size();++i){
        double dx = wps[i].x - x;
        double dy = wps[i].y - y;
        double d2 = dx*dx + dy*dy;
        if(d2 < bd){ bd = d2; best = (int)i; }
    }
    return best;
}

void runPurePursuit(){
    if(!g_have_ego || g_waypoints.size() < 2) return;

    double ex = g_ego.position.x;
    double ey = g_ego.position.y;
    double eyaw = g_ego.heading; // rad assumed

    int idx = nearestIndex(g_waypoints, ex, ey);
    if(idx < 0 || idx >= (int)g_waypoints.size()-1) return;

    // lookahead 거리 이상 떨어진 첫 번째 웨이포인트를 찾음
    int tgt_idx = idx;
    while (tgt_idx < (int)g_waypoints.size()) {
        double dx = g_waypoints[tgt_idx].x - ex;
        double dy = g_waypoints[tgt_idx].y - ey;
        double dist = std::sqrt(dx*dx + dy*dy);
        if(dist >= g_lookahead) break;
        tgt_idx++;
    }
    if(tgt_idx >= (int)g_waypoints.size()) {
        tgt_idx = (int)g_waypoints.size() - 1;
    }

    const auto& tgt = g_waypoints[tgt_idx];
    ROS_INFO_STREAM("[DEBUG] idx=" << idx << ", tgt_idx=" << tgt_idx);

    // ego 좌표계 변환 (yaw 보정 제거)
    // (ex, ey) 는 차량의 위치
    // (tgt.x, tgt.y)는 목표지점의 위치
    // double eyaw = g_ego.heading; // rad assumed
    //  차량의 앞이 x축이 되도록 윌드 좌표를 ego로 회전한 값.
    double dx = tgt.x - ex;
    double dy = tgt.y - ey;
    double x_e =  std::cos(eyaw)*dx + std::sin(eyaw)*dy;
    double y_e = -std::sin(eyaw)*dx + std::cos(eyaw)*dy;

    // Pure Pursuit 조향각 계산
    // 타겟까지의 실제 직선거리 
    double Ld = std::hypot(x_e, y_e);
    if (Ld < 1e-6) return;

    // alpha는 타겟까지의 방향각 오차
    //  직진이면 0, 왼쪽이면 + 오른쪽이면 -
    double alpha = std::atan2(y_e, x_e);
    double delta = std::atan2(2.0 * g_wheelbase * std::sin(alpha), Ld);
    

    // morai steering 제한 범위 맞추기 (±0.35 rad 예상)
    const double max_steering = 0.35;  // 모라이 스티어링 최대값
    const double max_delta = 0.7;      // 실제 최대 조향각 (라디안, 예시)
    // double steering_cmd = delta / max_delta * max_steering;
    double steering_cmd = delta;

    // steering 값 범위 제한
    if (steering_cmd > max_steering) steering_cmd = max_steering;
    if (steering_cmd < -max_steering) steering_cmd = -max_steering;

    ROS_INFO("delta=%.4f, steering_cmd=%.4f", delta, steering_cmd);
    
    /*//     morai_msgs::CtrlCmd cmd;
//     cmd.longlCmdType = 2;  // velocity control
//     cmd.velocity     = g_v_set;
//     cmd.steering     = steering_cmd;
//     // cmd.steering     = -0.1;
//     // 0.5일떄 좌회전 크게
//     // 0.1일때 약한 좌회전
//     // -0.35일때 steering angle이 -20으로 우회전 약하게
//     // -0.1일때 steering angle이 -5.7
//     cmd.accel        = 10.0;
//     cmd.brake        = 0.0;*/

    // CtrlCmd 메시지 구성 및 발행
    morai_msgs::CtrlCmd cmd;
    cmd.longlCmdType = 2; // velocity control
    cmd.velocity     = g_v_set;
    cmd.steering     = steering_cmd; //이값에  //radian,, 스케일링 안해도 됨, 
    cmd.accel        = 10.0;
    cmd.brake        = 0.0;

    g_pub_cmd.publish(cmd);
}


// 콜백: EGO 상태
void CB_Ego(const morai_msgs::EgoVehicleStatus::ConstPtr& ego){
    g_ego = *ego;
    g_have_ego = true;

    // runPurePursuit();
}

// CSV 읽기
bool loadCSV(const std::string& csv_file){
    std::ifstream ifs(csv_file);
    if(!ifs){
        ROS_FATAL("Cannot open CSV: %s", csv_file.c_str());
        return false;
    }

    std::string line;
    std::getline(ifs, line); // 헤더 스킵

    while(std::getline(ifs, line)){
        if(line.empty()) continue;
        std::stringstream ss(line);
        std::string sx, sy, syaw;
        if(!std::getline(ss, sx, ',')) continue;
        if(!std::getline(ss, sy, ',')) continue;
        std::getline(ss, syaw, ',');

        Waypoint wp;
        wp.x = std::stod(sx);
        wp.y = std::stod(sy);
        wp.yaw = std::stod(syaw);
        g_waypoints.push_back(wp);
    }
    ROS_INFO("Loaded %zu waypoints from CSV", g_waypoints.size());
    return true;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "pp_from_csv_node");
    ros::NodeHandle nh("~");

    // 파라미터
    std::string csv_file;
    nh.param<std::string>("csv_file", csv_file, "/home/autonav/aim_ws/src/roscpp_morai/data/ego_topic.csv");
    nh.param<double>("wheelbase", g_wheelbase, 2.7);
    nh.param<double>("v_set", g_v_set, 4.0);
    nh.param<double>("lookahead", g_lookahead, 10.0);

    if(!loadCSV(csv_file)) return 1;

    g_pub_cmd = nh.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd", 1);
    ros::Subscriber sub_ego = nh.subscribe("/Ego_topic", 10, CB_Ego);

    ROS_INFO("[PP CSV Node] running with %zu waypoints", g_waypoints.size());

    // csv읽어서 배열로 한번 들어오게 해서 그걸 사용하는게 나아보임 

    ros::Rate loop_rate(7); // 10Hz로 실행 빈도 제한 (필요시 조절 가능)

    while(ros::ok()){
        ros::spinOnce(); // spinonce 는 메인 루프 안에서 원하는 타이밍에 콜백을 처리하고, 그 직후의 최신 상태로 제어할 수 있게 해줌
        if(g_have_ego){
            runPurePursuit();
        }
        loop_rate.sleep();
    }
    return 0;
}



/*[INFO] [1759346627.293870725]: [DEBUG] idx=476, tgt_idx=542
[INFO] [1759346627.311109030]: [DEBUG] idx=476, tgt_idx=542
[INFO] [1759346627.326246945]: [DEBUG] idx=476, tgt_idx=542
[INFO] [1759346627.341026391]: [DEBUG] idx=476, tgt_idx=542
[INFO] [1759346627.361322182]: [DEBUG] idx=477, tgt_idx=542
[INFO] [1759346627.394254573]: [DEBUG] idx=477, tgt_idx=542
[INFO] [1759346627.409215485]: [DEBUG] idx=477, tgt_idx=542
[INFO] [1759346627.425575530]: [DEBUG] idx=477, tgt_idx=542
[INFO] [1759346627.443427885]: [DEBUG] idx=477, tgt_idx=542


[INFO] [1759347098.327397600]: [DEBUG] idx=483, tgt_idx=553
[INFO] [1759347098.327467759]: delta=-0.0133, steering_cmd=-0.0066
[INFO] [1759347138.684175939]: [DEBUG] idx=84, tgt_idx=186
[INFO] [1759347138.684290369]: delta=0.2912, steering_cmd=0.1456


좌표계는 enu이용, 
 */