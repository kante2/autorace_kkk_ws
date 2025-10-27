#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/AccelStamped.h>
#include <Eigen/Dense>

static constexpr double g = 9.80665;

// IMU(센서축) -> Body(FLU) 회전: NED 비슷(+Z Down)일 때 Y,Z 뒤집기
static const Eigen::Matrix3d R_B_Imu = (Eigen::Matrix3d() <<
    1,  0,  0,
    0, -1,  0,
    0,  0, -1).finished();

// COM -> IMU 위치벡터 (Body 기준) [m]
static const Eigen::Vector3d s_B(0.16, 0.0, 0.09);

class ComFromImuNode {
public:
  ComFromImuNode(ros::NodeHandle& nh) : nh_(nh) {
    sub_ = nh_.subscribe("/imu", 100, &ComFromImuNode::imuCb, this);
    pub_ = nh_.advertise<geometry_msgs::AccelStamped>("com/accel", 100);
  }

private:
  void imuCb(const sensor_msgs::ImuConstPtr& msg){
    // 1) IMU 원시값(센서축)
    const Eigen::Vector3d accI(msg->linear_acceleration.x,
                               msg->linear_acceleration.y,
                               msg->linear_acceleration.z);
    const Eigen::Vector3d omgI(msg->angular_velocity.x,
                               msg->angular_velocity.y,
                               msg->angular_velocity.z);

    // 2) 각가속도(센서축) 추정 (단순 차분)
    Eigen::Vector3d alpI = Eigen::Vector3d::Zero();
    if (has_prev_) {
      const double dt = (msg->header.stamp - prev_t_).toSec();
      if (dt > 1e-4) alpI = (omgI - prev_omgI_) / dt;
    }
    prev_omgI_ = omgI; prev_t_ = msg->header.stamp; has_prev_ = true;

    // 3) I->IMU 자세행렬, I->Body = R_B_Imu * R_ImuI
    const Eigen::Quaterniond qI2Imu(msg->orientation.w, msg->orientation.x,
                                    msg->orientation.y, msg->orientation.z);
    const Eigen::Matrix3d R_ImuI = qI2Imu.toRotationMatrix();   // I -> IMU
    const Eigen::Matrix3d R_BI   = R_B_Imu * R_ImuI;            // I -> Body

    // 4) 센서축 -> Body축 회전
    const Eigen::Vector3d f_B     = R_B_Imu * accI;   // specific force (중력 포함 측정이라면 그대로)
    const Eigen::Vector3d omega_B = R_B_Imu * omgI;
    const Eigen::Vector3d alpha_B = R_B_Imu * alpI;

    // 5) 중력의 Body 표현 (I에서 [0,0,-g])
    const Eigen::Vector3d g_B = R_BI * Eigen::Vector3d(0,0,-g);

    // 6) COM 가속도 (Body 기준)
    // a_COM^B = f_B + g_B - (alpha x s) - omega x (omega x s)
    const Eigen::Vector3d aB =
        f_B + g_B - alpha_B.cross(s_B) - omega_B.cross(omega_B.cross(s_B));

    // 7) Publish
    geometry_msgs::AccelStamped out;
    out.header = msg->header;
    out.accel.linear.x = aB.x();
    out.accel.linear.y = aB.y();
    out.accel.linear.z = aB.z();
    pub_.publish(out);
  }

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
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
