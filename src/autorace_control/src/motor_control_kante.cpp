// motor_servo_ctrl_node.cpp
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <algorithm>

// 두 토픽 퍼블리셔(모터/서보) + 간단 유틸 함수
class MotorServoCtrl {
public:
  // latched 기본값 true: rostopic pub 과 동일하게 마지막 메시지를 신규 구독자에게 즉시 전달
  MotorServoCtrl(ros::NodeHandle& nh, const std::string& motor_topic="/commands/motor/speed",
                 const std::string& servo_topic="/commands/servo/position", bool latched=true)
  {
    motor_pub_ = nh.advertise<std_msgs::Float64>(motor_topic, 1, latched);
    servo_pub_ = nh.advertise<std_msgs::Float64>(servo_topic, 1, latched);
  }

  // 전진/후진 속도 명령 (장비 스케일에 맞게 범위 클램프)
  void setMotorSpeed(double cmd, double min_cmd=0.0, double max_cmd=2000.0) {
    std_msgs::Float64 m;
    m.data = clamp(cmd, min_cmd, max_cmd);
    motor_pub_.publish(m);
  }

  // 조향 명령 (예: 0.0~1.0 스케일)
  // 0은 좌측 끝, 1은 우측 끝, 0.5는 중립
  void setSteering(double pos, double min_pos=0.0, double max_pos=1.0) {
    std_msgs::Float64 s;
    s.data = clamp(pos, min_pos, max_pos);
    servo_pub_.publish(s);
  }

  // 안전 정지(모터 0, 서보 중립)
  void stop(double motor_min=0.0, double servo_center=0.5) {
    setMotorSpeed(motor_min);
    setSteering(servo_center);
  }

private:
  static inline double clamp(double v, double lo, double hi) {
    return std::max(lo, std::min(v, hi));
  }

  ros::Publisher motor_pub_;
  ros::Publisher servo_pub_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "motor_servo_ctrl_node");
  ros::NodeHandle nh;

  // 파라미터(필요 시 런치에서 조정)
  std::string motor_topic = "/commands/motor/speed";
  std::string servo_topic = "/commands/servo/position";
  bool latched = true;
  nh.param<std::string>("motor_topic", motor_topic, motor_topic);
  nh.param<std::string>("servo_topic", servo_topic, servo_topic);
  nh.param<bool>("latched", latched, latched);

  MotorServoCtrl motor_ctrl(nh, motor_topic, servo_topic, latched);

  // --- 데모 시퀀스: 1) 전진 2) 좌로 조향 3) 정지 ---
  // motor_ctrl 객체 사용
  // motor_ctrl 객체란, MotorServoCtrl 클래스의 인스턴스(객체)로, motor/servo 제어 기능을 제공
  ROS_INFO("Go forward");
  motor_ctrl.setMotorSpeed(2000.0);   // 2000 (장비에 맞게 조정)
  motor_ctrl.setSteering(0.5);        // 중립(직진 가정)
  ros::Duration(1.5).sleep();

  ROS_INFO("Steer left");
  motor_ctrl.setSteering(0.35);       // 장비에 맞게 조정
  ros::Duration(1.0).sleep();

  ROS_INFO("Stop");
  motor_ctrl.stop();                  // motor=0.0, servo=0.5

  // 필요하면 루프 유지
  // ros::Rate rate(10);
  // while (ros::ok()) { ros::spinOnce(); rate.sleep(); }

  return 0;
}
