// rubbercone_main.cpp
#include <ros/ros.h>
// forward declarations (mission functions are in mission_labacorn.cpp)
void mission_labacorn_init(ros::NodeHandle& nh, ros::NodeHandle& pnh);
void mission_labacorn_step();
int main(int argc, char** argv){  
	ros::init(argc, argv, "labacorn_cluster");  
	ros::NodeHandle nh;  
	ros::NodeHandle pnh("~");
	
  mission_labacorn_init(nh, pnh);
  
  ros::Rate rate(15);  
  while (ros::ok())  {    
  ros::spinOnce();    
  mission_labacorn_step();    
  rate.sleep();  
  }  
  return 0;
}
