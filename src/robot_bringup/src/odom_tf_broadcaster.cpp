#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

/**
 * Simple TF broadcaster:
 * - Subscribes: /wheel_odom (nav_msgs/Odometry) or param ~odom_topic
 * - Publishes TF: odom -> base_link using msg.pose.pose
 * - Republishes Odometry to /odom (optional, enabled by ~republish_odom)
 */
class OdomTFBroadcaster {
public:
  OdomTFBroadcaster(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    pnh.param<std::string>("odom_topic", odom_topic_, "/wheel_odom");
    pnh.param<std::string>("odom_frame", odom_frame_, "odom");
    pnh.param<std::string>("base_frame", base_frame_, "base_link");
    pnh.param<bool>("republish_odom", republish_odom_, true);
    sub_ = nh.subscribe(odom_topic_, 50, &OdomTFBroadcaster::cbOdom, this);
    if (republish_odom_) pub_ = nh.advertise<nav_msgs::Odometry>("/odom", 10);
    ROS_INFO_STREAM("[odom_tf_broadcaster] listening to " << odom_topic_
                    << ", TF: " << odom_frame_ << " -> " << base_frame_
                    << ", republish_odom=" << (republish_odom_ ? "true" : "false"));
  }

private:
  void cbOdom(const nav_msgs::Odometry::ConstPtr& msg) {
    // Publish TF using pose in Odometry
    geometry_msgs::TransformStamped ts;
    ts.header.stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    ts.header.frame_id = odom_frame_;
    ts.child_frame_id  = base_frame_;
    ts.transform.translation.x = msg->pose.pose.position.x;
    ts.transform.translation.y = msg->pose.pose.position.y;
    ts.transform.translation.z = msg->pose.pose.position.z;
    ts.transform.rotation = msg->pose.pose.orientation;
    br_.sendTransform(ts);

    if (republish_odom_) {
      nav_msgs::Odometry out = *msg;
      out.header.frame_id = odom_frame_;
      out.child_frame_id  = base_frame_;
      pub_.publish(out);
    }
  }

  std::string odom_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  bool republish_odom_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  tf::TransformBroadcaster br_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "odom_tf_broadcaster");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  OdomTFBroadcaster node(nh, pnh);
  ros::spin();
  return 0;
}
