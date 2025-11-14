#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

#include "lane_detection/feature_extraction.hpp"
#include "lane_detection/preprocessing.hpp"
#include "lane_detection/sliding_window.hpp"

namespace lane_detection {

namespace {

// 주기 측정/디버깅용 더미 클래스 (현재는 기능 비활성화 상태)
class CheckTimer {
public:
    explicit CheckTimer(const std::string& /*name*/) {}
    void tick(const std::string& /*phase*/ = std::string()) {}
};

std::string pointsToJson(const std::vector<cv::Point>& points) {
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < points.size(); ++i) {
        oss << "[" << points[i].x << "," << points[i].y << "]";
        if (i + 1 < points.size()) {
            oss << ",";
        }
    }
    oss << "]";
    return oss.str();
}

std::string stopLineToJson(const std::vector<int>& stop_line) {
    if (stop_line.size() != 2) {
        return "[]";
    }
    std::ostringstream oss;
    oss << "[" << stop_line[0] << "," << stop_line[1] << "]";
    return oss.str();
}

cv::Mat toCvMat(const sensor_msgs::ImageConstPtr& msg) {
    if (!msg) {
        return cv::Mat();
    }
    try {
        auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        return cv_ptr->image;
    } catch (const cv_bridge::Exception& e) {
        ROS_ERROR_STREAM_THROTTLE(1.0, "cv_bridge exception: " << e.what());
        return cv::Mat();
    }
}

}  // namespace

// 실시간 카메라 토픽을 구독해 BEV 변환 → 전처리 → 차선/정지선 검출 → 결과 퍼블리시까지 수행하는 ROS 노드
class PerCameraNode {
public:
    PerCameraNode()
        : nh_(),
          preprocessor_(),
          feature_extractor_(),
          sliding_window_(),
          check_timer_("per_camera_node"),
          img_x_(0),
          img_y_(0) {
        sub_ = nh_.subscribe("/usb_cam/image_rect_color", 1, &PerCameraNode::imageCallback, this);
        pub_ = nh_.advertise<std_msgs::String>("/perception/camera", 1);
        ROS_INFO("PerCamera C++ node started");
    }

    void spin() const {
        ros::spin();
    }

private:
    // USB 카메라 토픽으로부터 이미지가 들어올 때마다 호출
    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            img_ = toCvMat(msg);
            if (img_.empty()) {
                ROS_WARN_THROTTLE(1.0, "Failed to convert image");
                return;
            }
            processing();
        } catch (const cv::Exception& e) {
            ROS_ERROR_STREAM("OpenCV exception: " << e.what());
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM("Exception in image callback: " << e.what());
        }
    }

    // 단일 프레임에 대해 BEV 변환→색상 분리→이진화→정지선·차선 추출까지 수행
    void processing() {
        if (img_.empty()) {
            return;
        }

        try {
            img_y_ = img_.rows;
            img_x_ = img_.cols;

            cv::Mat warped_img = preprocessor_.bevWarp(img_, img_y_, img_x_);
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

            cv::Mat yellow_bin;
            cv::Mat white_bin;
            preprocessor_.binaryImagesYellowAndWhite(yellow_filtered,
                                                     white_filtered,
                                                     yellow_bin,
                                                     white_bin);

            sliding_window_.setImageHeight(img_y_);

            stop_line_ = feature_extractor_.estimateStopLine(white_bin, img_y_);
            lane_mode_ = feature_extractor_.estimateLaneMode(warped_img);

            auto yellow_result = sliding_window_.slidingWindowAdaptive(yellow_bin);
            auto white_result = sliding_window_.slidingWindowAdaptive(white_bin);

            yellow_lane_img_ = yellow_result.visualized;
            white_lane_img_ = white_result.visualized;
            yellow_left_lane_ = yellow_result.left_lane;
            yellow_right_lane_ = yellow_result.right_lane;
            white_left_lane_ = white_result.left_lane;
            white_right_lane_ = white_result.right_lane;

            publishCameraInfo();
            viewCamera();
        } catch (const cv::Exception& e) {
            ROS_ERROR_STREAM_THROTTLE(1.0, "Processing OpenCV exception: " << e.what());
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM_THROTTLE(1.0, "Processing exception: " << e.what());
        }
    }

    void publishCameraInfo() {
        std::ostringstream oss;
        oss << "["
            << stopLineToJson(stop_line_) << ","
            << pointsToJson(yellow_left_lane_) << ","
            << pointsToJson(yellow_right_lane_) << ","
            << pointsToJson(white_left_lane_) << ","
            << pointsToJson(white_right_lane_) << ","
            << "\"" << lane_mode_ << "\""
            << "]";

        std_msgs::String msg;
        msg.data = oss.str();
        pub_.publish(msg);
    }

    void viewCamera() {
        if (yellow_lane_img_.empty() && white_lane_img_.empty()) {
            return;
        }

        cv::Mat combined;
        if (!yellow_lane_img_.empty() && !white_lane_img_.empty()) {
            cv::bitwise_or(yellow_lane_img_, white_lane_img_, combined);
        } else if (!yellow_lane_img_.empty()) {
            combined = yellow_lane_img_.clone();
        } else {
            combined = white_lane_img_.clone();
        }

        if (stop_line_.size() == 2) {
            int min_y = stop_line_[0];
            int max_y = stop_line_[1];
            int cross_diff = max_y - min_y;
            if (cross_diff > 35) {
                cv::rectangle(combined,
                              cv::Point(0, min_y),
                              cv::Point(std::max(0, img_x_ - 1), max_y),
                              cv::Scalar(0, 0, 255),
                              3);
            }
        }
        
        cv::imshow("lane_img", combined);
        cv::waitKey(1);
    }

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;

    CameraPreprocessor preprocessor_;
    LaneFeatureExtractor feature_extractor_;
    SlidingWindow sliding_window_;
    CheckTimer check_timer_;

    cv::Mat img_;
    cv::Mat yellow_lane_img_;
    cv::Mat white_lane_img_;

    std::vector<int> stop_line_;
    std::vector<cv::Point> yellow_left_lane_;
    std::vector<cv::Point> yellow_right_lane_;
    std::vector<cv::Point> white_left_lane_;
    std::vector<cv::Point> white_right_lane_;
    std::string lane_mode_;

    int img_x_;
    int img_y_;
};

}  // namespace lane_detection

int main(int argc, char** argv) {
    ros::init(argc, argv, "per_camera_node");  // 노드 초기화
    lane_detection::PerCameraNode node;        // 주요 처리 클래스 생성
    node.spin();                               // ROS 이벤트 루프 실행
    return 0;
}