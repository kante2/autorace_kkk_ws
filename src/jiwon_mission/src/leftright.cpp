// KAZE 기반 좌/우 표지판 매칭 온라인 버전 (ROI 사용 X)
// /usb_cam/image_rect_color 구독 → left/right/none 퍼블리시

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ros/package.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

// ================== 전역 변수 ==================

// 특징 추출기 + 매처
cv::Ptr<cv::Feature2D> g_feat;
cv::BFMatcher g_matcher(cv::NORM_L2, false);

// 템플릿 이미지
cv::Mat g_left_gray, g_right_gray;
cv::Mat g_desc_left, g_desc_right;
std::vector<cv::KeyPoint> g_kp_left, g_kp_right;

// 파라미터
double g_ratio_thresh        = 0.80; // 매칭 ratio test
int    g_min_good_matches    = 3;    // good match 최소 개수

// 퍼블리셔
ros::Publisher g_pub_label;
std::string g_pub_topic = "/leftright_sign";

// 디버그용 윈도우 이름
const std::string WIN_NAME = "leftright_kaze_online";

// ================== 유틸 함수 ==================

// 템플릿 로드
bool loadTemplates(const std::string& left_path,
                   const std::string& right_path)
{
    cv::Mat left_color  = cv::imread(left_path,  cv::IMREAD_COLOR);
    cv::Mat right_color = cv::imread(right_path, cv::IMREAD_COLOR);

    if (left_color.empty() || right_color.empty())
    {
        ROS_ERROR_STREAM("Failed to load template images: "
                         << left_path << ", " << right_path);
        return false;
    }

    cv::cvtColor(left_color,  g_left_gray,  cv::COLOR_BGR2GRAY);
    cv::cvtColor(right_color, g_right_gray, cv::COLOR_BGR2GRAY);

    g_feat->detectAndCompute(g_left_gray,  cv::noArray(), g_kp_left,  g_desc_left);
    g_feat->detectAndCompute(g_right_gray, cv::noArray(), g_kp_right, g_desc_right);

    if (g_desc_left.empty() || g_desc_right.empty())
    {
        ROS_ERROR("Template descriptors are empty.");
        return false;
    }

    ROS_INFO("Templates loaded: left_kp=%d, right_kp=%d",
             (int)g_kp_left.size(), (int)g_kp_right.size());

    cv::imshow("template_left",  g_left_gray);
    cv::imshow("template_right", g_right_gray);
    cv::waitKey(1);

    return true;
}

// 매칭 점수 계산
int computeMatchScore(const cv::Mat& desc_query, const cv::Mat& desc_train)
{
    if (desc_query.empty() || desc_train.empty())
        return 0;

    std::vector<std::vector<cv::DMatch>> knn_matches;
    g_matcher.knnMatch(desc_query, desc_train, knn_matches, 2);

    int good = 0;
    for (const auto& m : knn_matches)
    {
        if (m.size() < 2) continue;
        const cv::DMatch& best   = m[0];
        const cv::DMatch& second = m[1];
        if (best.distance < g_ratio_thresh * second.distance)
            good++;
    }
    return good;
}

// (참고용) 파란 표지판 ROI 함수 - 지금은 사용 안 함
bool detectBlueSignROI(const cv::Mat& bgr, cv::Rect& roi_rect)
{
    if (bgr.empty()) return false;

    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

    cv::Mat mask1, mask2, mask;
    cv::inRange(hsv, cv::Scalar(90,  80, 80), cv::Scalar(130,255,255), mask1);
    cv::inRange(hsv, cv::Scalar(100, 80, 80), cv::Scalar(140,255,255), mask2);
    mask = mask1 | mask2;

    cv::erode(mask,  mask, cv::Mat(), cv::Point(-1,-1), 1);
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1,-1), 2);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) return false;

    double best_area = 0.0;
    size_t best_idx  = 0;

    for (size_t i = 0; i < contours.size(); ++i)
    {
        double area = cv::contourArea(contours[i]);
        if (area > best_area)
        {
            best_area = area;
            best_idx  = i;
        }
    }

    if (best_area < 1000.0)
        return false;

    roi_rect = cv::boundingRect(contours[best_idx]);

    int pad = 10;
    roi_rect.x      = std::max(0, roi_rect.x - pad);
    roi_rect.y      = std::max(0, roi_rect.y - pad);
    roi_rect.width  = std::min(bgr.cols - roi_rect.x, roi_rect.width + 2*pad);
    roi_rect.height = std::min(bgr.rows - roi_rect.y, roi_rect.height + 2*pad);

    return true;
}

// ================== 콜백: 이미지 한 장 처리 ==================

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_WARN("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat frame = cv_ptr->image;
    if (frame.empty()) return;

    // --- 전체 이미지 사용 (ROI X) ---
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    std::vector<cv::KeyPoint> kp;
    cv::Mat desc;
    g_feat->detectAndCompute(gray, cv::noArray(), kp, desc);

    if (desc.empty())
    {
        ROS_WARN_THROTTLE(1.0, "[INFO] No KAZE features in frame");
        std_msgs::String out;
        out.data = "none";
        g_pub_label.publish(out);
        return;
    }

    int score_left  = computeMatchScore(desc, g_desc_left);
    int score_right = computeMatchScore(desc, g_desc_right);

    std::string label = "none";

    if (score_left > g_min_good_matches || score_right > g_min_good_matches)
    {
        label = (score_left > score_right) ? "left" : "right";
    }

    ROS_INFO_THROTTLE(1.0,
        "scoreL=%d scoreR=%d -> %s",
        score_left, score_right, label.c_str());

    // 퍼블리시
    std_msgs::String out;
    out.data = label;
    g_pub_label.publish(out);

    // 디버그용 텍스트만 화면 좌상단에 표시
    cv::putText(frame, label,
                cv::Point(10, 40),
                cv::FONT_HERSHEY_SIMPLEX, 1.0,
                cv::Scalar(0,255,0), 2);

    cv::imshow(WIN_NAME, frame);
    cv::waitKey(1);
}

// ================== main ==================

int main(int argc, char** argv)
{
    ros::init(argc, argv, "leftright_kaze_online");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // === KAZE 생성 ===
    g_feat    = cv::KAZE::create();
    g_matcher = cv::BFMatcher(cv::NORM_L2);

    // 윈도우
    cv::namedWindow(WIN_NAME, cv::WINDOW_NORMAL);
    cv::resizeWindow(WIN_NAME, 800, 600);

    // 템플릿 경로 (패키지 내부)
    std::string pkg_path = ros::package::getPath("jiwon_mission");
    std::string default_left_tpl  = pkg_path + "/data/ab_truth_left.png";
    std::string default_right_tpl = pkg_path + "/data/ab_truth_right.png";

    std::string left_tpl, right_tpl;
    pnh.param("left_template_path",  left_tpl,  default_left_tpl);
    pnh.param("right_template_path", right_tpl, default_right_tpl);

    pnh.param("ratio_thresh",       g_ratio_thresh,      0.80);
    pnh.param("min_good_matches",   g_min_good_matches,  3);
    pnh.param("pub_topic",          g_pub_topic,         std::string("/leftright_sign"));

    ROS_INFO("Templates: %s / %s", left_tpl.c_str(), right_tpl.c_str());
    if (!loadTemplates(left_tpl, right_tpl))
        return -1;

    // 퍼블리셔 & 서브스크라이버
    g_pub_label = nh.advertise<std_msgs::String>(g_pub_topic, 1);

    std::string image_topic = "/usb_cam/image_rect_color";
    pnh.param("image_topic", image_topic, image_topic);
    ros::Subscriber sub_img = nh.subscribe(image_topic, 1, imageCallback);
    ROS_INFO("Subscribe image: %s, publish label: %s",
             image_topic.c_str(), g_pub_topic.c_str());

    ros::spin();

    cv::destroyAllWindows();
    return 0;
}
