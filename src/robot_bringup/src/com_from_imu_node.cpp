#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Dense>

static const Eigen::Matrix3d R_B_Imu = (Eigen::Matrix3d() <<
    1,  0,  0,
    0, -1,  0,
    0,  0, -1).finished();

// COM → IMU (Body 기준) [m]  (네가 준 값)
static const Eigen::Vector3d s_B(0.16, 0.0, 0.09);

class ComFromImuNode {
public:
  ComFromImuNode(ros::NodeHandle& nh) : nh_(nh) {
    // 프레임 이름 (RViz에서 base_link 원점에 보이게)
    nh_.param<std::string>("base_frame", base_frame_, "base_link");

    sub_ = nh_.subscribe("imu/data", 100, &ComFromImuNode::imuCb, this);

    // COM 선형가속도(중력 제거) 전체
    pub_accel_ = nh_.advertise<geometry_msgs::AccelStamped>("com/accel", 50);

    // 성분/크기 분리
    pub_x_    = nh_.advertise<std_msgs::Float64>("com/accel_x", 50);
    pub_y_    = nh_.advertise<std_msgs::Float64>("com/accel_y", 50);
    pub_z_    = nh_.advertise<std_msgs::Float64>("com/accel_z", 50);
    pub_mag_  = nh_.advertise<std_msgs::Float64>("com/accel_mag", 50);
    pub_vec_  = nh_.advertise<geometry_msgs::Vector3Stamped>("com/accel_vec", 50);

    // RViz 화살표 시각화 (base_link에서 보이게)
    pub_marker_ = nh_.advertise<visualization_msgs::Marker>("com/accel_marker", 10);
  }

private:
  void imuCb(const sensor_msgs::ImuConstPtr& m){
    // 센서축 값
    const Eigen::Vector3d accI(m->linear_acceleration.x,
                               m->linear_acceleration.y,
                               m->linear_acceleration.z);
    const Eigen::Vector3d omgI(m->angular_velocity.x,
                               m->angular_velocity.y,
                               m->angular_velocity.z);

    // 각가속도(센서축) 추정
    Eigen::Vector3d alpI = Eigen::Vector3d::Zero();
    if (has_prev_) {
      const double dt = (m->header.stamp - prev_t_).toSec();
      if (dt > 1e-4) alpI = (omgI - prev_omgI_) / dt;
    }
    prev_omgI_ = omgI; prev_t_ = m->header.stamp; has_prev_ = true;

    // I->IMU, I->Body (자세는 화살표 방향에만 쓰이지만 여기선 frame_id=base_link로 고정할 것)
    const Eigen::Quaterniond qI2Imu(m->orientation.w, m->orientation.x,
                                    m->orientation.y, m->orientation.z);
    const Eigen::Matrix3d R_ImuI = qI2Imu.toRotationMatrix(); // I→IMU
    // const Eigen::Matrix3d R_BI   = R_B_Imu * R_ImuI;        // I→Body (linear 모드에선 중력 안 씀)

    // 센서축→Body축 회전
    const Eigen::Vector3d a_lin_IMU = accI;                    // 입력이 이미 linear accel이라 가정
    const Eigen::Vector3d a_lin_B   = R_B_Imu * a_lin_IMU;     // Body 기준 선형가속도 (중력 없음)
    const Eigen::Vector3d omega_B   = R_B_Imu * omgI;
    const Eigen::Vector3d alpha_B   = R_B_Imu * alpI;

    // COM 선형가속도 (linear 모드: +g_B 없이)
    const Eigen::Vector3d aB = a_lin_B
                             - alpha_B.cross(s_B)
                             - omega_B.cross(omega_B.cross(s_B));

    // ---------------- Publish: AccelStamped (frame=base_link) ----------------
    geometry_msgs::AccelStamped acc_msg;
    acc_msg.header = m->header;
    acc_msg.header.frame_id = base_frame_;        // ← RViz에서 base_link에 그려지게!
    acc_msg.accel.linear.x = aB.x();
    acc_msg.accel.linear.y = aB.y();
    acc_msg.accel.linear.z = aB.z();
    pub_accel_.publish(acc_msg);

    // ---------------- 성분/벡터/크기 ----------------
    std_msgs::Float64 fx, fy, fz, fm;
    fx.data = aB.x(); fy.data = aB.y(); fz.data = aB.z(); fm.data = aB.norm();
    pub_x_.publish(fx); pub_y_.publish(fy); pub_z_.publish(fz); pub_mag_.publish(fm);

    geometry_msgs::Vector3Stamped vec;
    vec.header = acc_msg.header;                   // frame_id = base_link
    vec.vector.x = aB.x(); vec.vector.y = aB.y(); vec.vector.z = aB.z();
    pub_vec_.publish(vec);

    // ---------------- RViz Marker (ARROW) at base_link ----------------
    visualization_msgs::Marker mk;
    mk.header = acc_msg.header;     // frame_id = base_link
    mk.ns = "com_accel";
    mk.id = 0;
    mk.type = visualization_msgs::Marker::ARROW;
    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.orientation.w = 1.0;    // base_link 원점에서 시작
    // 화살표 길이/두께 (길이는 가속도 크기에 비례, 너무 길면 clamp)
    const double len = std::min(0.5, 0.05 * aB.norm()); // 0~0.5 m
    mk.scale.x = std::max(0.02, len);   // shaft length
    mk.scale.y = 0.02;                  // shaft diameter
    mk.scale.z = 0.02;                  // head diameter (RViz ARROW에서 z는 head dia로 사용됨)
    mk.color.r = 0.1f; mk.color.g = 0.8f; mk.color.b = 0.2f; mk.color.a = 0.9f;

    // 방향을 벡터로 반영: Marker는 pose만 주므로, 방향은 points로 지정
    geometry_msgs::Point p0, p1;
    p0.x = p0.y = p0.z = 0.0;          // base_link 원점
    // 벡터 방향 단위화 후 길이 len만큼
    Eigen::Vector3d dir = (aB.norm() > 1e-6) ? (aB.normalized()*len) : Eigen::Vector3d::Zero();
    p1.x = dir.x(); p1.y = dir.y(); p1.z = dir.z();
    mk.points.clear(); mk.points.push_back(p0); mk.points.push_back(p1);

    pub_marker_.publish(mk);
  }

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_accel_;
  ros::Publisher pub_x_, pub_y_, pub_z_, pub_mag_, pub_vec_;
  ros::Publisher pub_marker_;
  std::string base_frame_{"base_link"};

  bool has_prev_ = false;
  ros::Time prev_t_;
  Eigen::Vector3d prev_omgI_ = Eigen::Vector3d::Zero();
};

int main(int argc, char** argv){
  ros::init(argc, argv, "com_from_imu_node");
  ros::NodeHandle nh("~");
  ComFromImuNode node(nh);
  ros::spin();
  return 0;
}
