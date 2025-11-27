// save_usb_cam_images.cpp
// 이미지 저장용 임시 데이터 

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <sstream>
#include <iomanip>   // std::setw, std::setfill
#include <string>

// 전역 카운터
int g_img_count = 0;
// 저장할 폴더 (파라미터로도 바꿀 수 있게 할 거지만, 기본값)
std::string g_save_dir = "/tmp/usb_cam_images";

void imageCB(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    // BGR8 포맷으로 변환 (일반 컬러 이미지)
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    const cv::Mat& img = cv_ptr->image;

    // 파일 이름 만들기: img_0001.png, img_0002.png, ...
    std::ostringstream oss;
    oss << g_save_dir << "/img_"
        << std::setw(4) << std::setfill('0') << g_img_count++
        << ".png";
    std::string filename = oss.str();

    // 이미지 저장
    if (cv::imwrite(filename, img))
    {
      ROS_INFO_STREAM("Saved image: " << filename);
    }
    else
    {
      ROS_WARN_STREAM("Failed to save image: " << filename);
    }
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "save_usb_cam_images");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // private 파라미터로 저장 폴더 변경 가능: ~save_dir
  pnh.param<std::string>("save_dir", g_save_dir, g_save_dir);
  ROS_INFO_STREAM("Image save directory: " << g_save_dir);

  // 토픽 구독
  ros::Subscriber img_sub =
      nh.subscribe("/usb_cam/image_rect_color", 1, imageCB);

  ros::spin();
  return 0;
}
