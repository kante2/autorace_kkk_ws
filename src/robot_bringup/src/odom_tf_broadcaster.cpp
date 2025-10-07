#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

static tf2_ros::TransformBroadcaster* g_br = nullptr;  // 전역 포인터(간단 버전)

// ODOM -> BASELINK
/*
해당 노드는 들어오는 /odom 메시지의 포즈(= odom 기준 base_link의 위치·자세)를 그대로 TF(odom-> base_link)로 중계.
즉, 로봇이 움직여서 /odom의 pose가 변하면, 그 값이 곧바로 tfm.transform.translation/rotation에 복사돼서 퍼블리시됨.
*/
geometry_msgs::TransformStamped makeTf(const nav_msgs::Odometry& odom){
  geometry_msgs::TransformStamped tfm;
  tfm.header = odom.header; // 보통 frame_id="odom"
  tfm.child_frame_id = !odom.child_frame_id.empty() ? odom.child_frame_id : "base_link";
  //  tx, ty, tz
  tfm.transform.translation.x = odom.pose.pose.position.x;
  tfm.transform.translation.y = odom.pose.pose.position.y;
  tfm.transform.translation.z = odom.pose.pose.position.z;
  tfm.transform.rotation      = odom.pose.pose.orientation;
  return tfm;
}

void odomCb(const nav_msgs::Odometry::ConstPtr& msg){
  if(!g_br) return;
  // -> : 포인터를 통해 멤버에 접근(포인터 g_br로 객체의 sendTransform 호출).
  g_br->sendTransform(makeTf(*msg));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odom_broadcaster");
  ros::NodeHandle nh;

  tf2_ros::TransformBroadcaster br;
  g_br = &br;  // 콜백에서 접근할 수 있게 등록

  // odom은 로봇이 시작할때의 위치에 해당
  ros::Subscriber sub = nh.subscribe("/odom", 10, odomCb);

  ros::spin();
  g_br = nullptr;
  return 0;
}
