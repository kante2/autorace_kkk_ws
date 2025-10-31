#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <std_msgs/String.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <cstdint>
#include <sstream>

#include "lane_detection/feature_extraction.hpp"
#include "lane_detection/preprocessing.hpp"

namespace lane_detection {

namespace {

cv::Mat decodeCompressedImage(const sensor_msgs::CompressedImageConstPtr& msg) {
    if (!msg) {
        return cv::Mat();
    }
    cv::Mat buffer(1,
                   static_cast<int>(msg->data.size()),
                   CV_8UC1,
                   const_cast<uint8_t*>(msg->data.data()));
    return cv::imdecode(buffer, cv::IMREAD_COLOR);
}

}  // namespace

class PerCameraRotaryNode {
public:
    PerCameraRotaryNode()
        : nh_(),
          preprocessor_(),
          feature_extractor_(),
          img_x_(0),
          img_y_(0) {
        sub_ = nh_.subscribe("/image_jpeg/compressed", 1, &PerCameraRotaryNode::imageCallback, this);
        pub_ = nh_.advertise<std_msgs::String>("/perception/camera/rotary", 1);
        ROS_INFO("PerCameraRotary C++ node started");
    }

    void spin() const {
        ros::spin();
    }

private:
    void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg) {
        try {
            img_ = decodeCompressedImage(msg);
            if (img_.empty()) {
                ROS_WARN_THROTTLE(1.0, "Failed to decode compressed image");
                return;
            }
            processing();
        } catch (const cv::Exception& e) {
            ROS_ERROR_STREAM("OpenCV exception: " << e.what());
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM("Exception in rotary image callback: " << e.what());
        }
    }

    void processing() {
        if (img_.empty()) {
            return;
        }

        try {
            img_y_ = img_.rows;
            img_x_ = img_.cols;

            cv::Mat warped_img = preprocessor_.bevWarpRotary(img_, img_y_, img_x_);
            if (warped_img.empty()) {
                return;
            }

            cv::Mat warped_hsv;
            cv::cvtColor(warped_img, warped_hsv, cv::COLOR_BGR2HSV);

            cv::Mat yellow_filtered;
            cv::Mat white_filtered;
            preprocessor_.detectColorYellowAndWhite(warped_img,
                                                    warped_hsv,
                                                    yellow_filtered,
                                                    white_filtered);

            preprocessor_.binaryImagesYellowAndWhite(yellow_filtered,
                                                     white_filtered,
                                                     yellow_bin_,
                                                     white_bin_);

            bool is_out_rotary = feature_extractor_.detectOutRotary(yellow_bin_);

            publishRotaryInfo(is_out_rotary);
            viewCamera();
        } catch (const cv::Exception& e) {
            ROS_ERROR_STREAM_THROTTLE(1.0, "Rotary processing OpenCV exception: " << e.what());
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM_THROTTLE(1.0, "Rotary processing exception: " << e.what());
        }
    }

    void publishRotaryInfo(bool is_out_rotary) {
        std::ostringstream oss;
        oss << "[" << (is_out_rotary ? "true" : "false") << "]";

        std_msgs::String msg;
        msg.data = oss.str();
        pub_.publish(msg);
    }

    void viewCamera() {
        if (yellow_bin_.empty()) {
            return;
        }

        cv::imshow("yellow_bin_img", yellow_bin_);
        cv::waitKey(1);
    }

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;

    CameraPreprocessor preprocessor_;
    LaneFeatureExtractor feature_extractor_;

    cv::Mat img_;
    cv::Mat yellow_bin_;
    cv::Mat white_bin_;

    int img_x_;
    int img_y_;
};

}  // namespace lane_detection

int main(int argc, char** argv) {
    ros::init(argc, argv, "per_camera_rotary_node");
    lane_detection::PerCameraRotaryNode node;
    node.spin();
    return 0;
}
