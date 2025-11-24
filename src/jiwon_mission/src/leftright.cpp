//ORB 특징점 검출

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

// ================== 전역 변수 ==================
ros::Publisher g_pub_label;

// ORB 특징 추출기 + 매처
cv::Ptr<cv::Feature2D> g_feat;
cv::BFMatcher g_matcher(cv::NORM_HAMMING, false);

// 템플릿 이미지 (그레이)
cv::Mat g_left_gray, g_right_gray;
cv::Mat g_desc_left, g_desc_right;
std::vector<cv::KeyPoint> g_kp_left, g_kp_right;

// 파라미터
double g_ratio_thresh     = 0.75;  // ORB 매칭 ratio test
int    g_min_good_matches = 5;     // good match 최소 개수
bool   g_use_circle_roi   = true;  // 원 검출 사용할지 여부

// ================== 유틸 함수 ==================

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

    // 그레이 변환
    cv::cvtColor(left_color,  g_left_gray,  cv::COLOR_BGR2GRAY);
    cv::cvtColor(right_color, g_right_gray, cv::COLOR_BGR2GRAY);

    // ORB 디스크립터 계산
    g_feat->detectAndCompute(g_left_gray,  cv::noArray(), g_kp_left,  g_desc_left);
    g_feat->detectAndCompute(g_right_gray, cv::noArray(), g_kp_right, g_desc_right);

    if (g_desc_left.empty() || g_desc_right.empty())
    {
        ROS_ERROR("Template descriptors are empty.");
        return false;
    }

    ROS_INFO("Loaded templates: left_kp=%d, right_kp=%d",
             (int)g_kp_left.size(), (int)g_kp_right.size());
    return true;
}

// ORB 매칭 점수 계산 (good match 개수)
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

// 회색 영상에서 원(표지판) ROI 찾기
bool detectCircleROI(const cv::Mat& gray, cv::Rect& roi_rect)
{
    cv::Mat blur;
    cv::medianBlur(gray, blur, 5);

    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(blur, circles, cv::HOUGH_GRADIENT,
                     1.2,                    // dp
                     gray.rows / 4,         // minDist
                     80,                    // param1 (Canny high threshold)
                     30,                    // param2 (center detection threshold)
                     20,                    // minRadius (조정 가능)
                     200);                  // maxRadius (조정 가능)

    if (circles.empty())
        return false;

    // 가장 큰 원 하나 선택
    cv::Vec3f best = circles[0];
    for (const auto& c : circles)
    {
        if (c[2] > best[2]) best = c;
    }

    int x = (int)std::round(best[0]);
    int y = (int)std::round(best[1]);
    int r = (int)std::round(best[2]);

    int x0 = std::max(0, x - r);
    int y0 = std::max(0, y - r);
    int x1 = std::min(gray.cols, x + r);
    int y1 = std::min(gray.rows, y + r);

    roi_rect = cv::Rect(cv::Point(x0, y0), cv::Point(x1, y1));
    return true;
}

void publishLabel(const std::string& label)
{
    std_msgs::String msg;
    msg.data = label;
    g_pub_label.publish(msg);
}

// ================== 콜백 ==================

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat frame = cv_ptr->image;
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    cv::Mat roi_gray = gray;
    cv::Rect roi_rect(0, 0, gray.cols, gray.rows);

    if (g_use_circle_roi)
    {
        if (detectCircleROI(gray, roi_rect))
        {
            roi_gray = gray(roi_rect);
        }
        else
        {
            // 원 못 찾으면 전체 이미지 사용
            roi_gray = gray;
        }
    }

    // ORB 특징 추출
    std::vector<cv::KeyPoint> kp_roi;
    cv::Mat desc_roi;
    g_feat->detectAndCompute(roi_gray, cv::noArray(), kp_roi, desc_roi);

    if (desc_roi.empty())
    {
        publishLabel("unknown");
        return;
    }

    // 템플릿과 매칭
    int score_left  = computeMatchScore(desc_roi, g_desc_left);
    int score_right = computeMatchScore(desc_roi, g_desc_right);

    std::string label = "unknown";
    if (score_left < g_min_good_matches && score_right < g_min_good_matches)
    {
        label = "unknown";
    }
    else if (score_left > score_right)
    {
        label = "left";
    }
    else if (score_right > score_left)
    {
        label = "right";
    }
    else
    {
        label = "unknown";
    }

    ROS_INFO_THROTTLE(1.0,
        "ORB scores -> left: %d, right: %d, label: %s",
        score_left, score_right, label.c_str());

    publishLabel(label);

    // 디버그용으로 보고 싶으면 주석 해제
    
    cv::rectangle(frame, roi_rect, cv::Scalar(0,255,0), 2);
    cv::putText(frame, label, roi_rect.tl() + cv::Point(0,-10),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,255,0), 2);
    cv::imshow("leftright_debug", frame);
    cv::waitKey(1);
    
}

// ================== main ==================

int main(int argc, char** argv)
{
    ros::init(argc, argv, "leftright");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // ORB 생성 (원하면 maxFeatures 조절)
    g_feat = cv::ORB::create(800);
    g_matcher = cv::BFMatcher(cv::NORM_HAMMING, false);

    // 파라미터
    std::string image_topic;
    std::string left_template_path;
    std::string right_template_path;

    pnh.param<std::string>("image_topic",        image_topic,        std::string("/camera/image_rect_color"));
    pnh.param<std::string>("left_template_path", left_template_path, std::string("left.png"));
    pnh.param<std::string>("right_template_path", right_template_path, std::string("right.png"));
    pnh.param("ratio_thresh",     g_ratio_thresh,     0.75);
    pnh.param("min_good_matches", g_min_good_matches, 5);
    pnh.param("use_circle_roi",   g_use_circle_roi,   true);

    // 템플릿 로딩
    if (!loadTemplates(left_template_path, right_template_path))
    {
        ROS_WARN("Template load failed. Node will run but output may be 'unknown' only.");
    }

    // 퍼블리셔 / 서브스크라이버
    g_pub_label = pnh.advertise<std_msgs::String>("sign_label", 1);
    ros::Subscriber sub_img = nh.subscribe(image_topic, 1, imageCallback);

    ROS_INFO("leftright (ORB) node started.");
    ros::spin();
    return 0;
}
