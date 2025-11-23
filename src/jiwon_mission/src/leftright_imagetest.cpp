// SIFT 기반 좌/우 표지판 매칭 오프라인 테스트 (ROS + launch, 토픽 X)
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ros/package.h>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

// ================== 전역 변수 ==================

// SIFT 특징 추출기 + 매처
cv::Ptr<cv::Feature2D> g_feat;              // SIFT
cv::BFMatcher g_matcher(cv::NORM_L2, false); // SIFT는 float → L2 distance

// 템플릿 이미지 (그레이)
cv::Mat g_left_gray, g_right_gray;
cv::Mat g_desc_left, g_desc_right;
std::vector<cv::KeyPoint> g_kp_left, g_kp_right;

// 파라미터
double g_ratio_thresh      = 0.80; // SIFT 매칭 ratio test
int    g_min_good_matches  = 3;    // good match 최소 개수
bool   g_use_roi           = false; // ROI(파란 표지판) 사용 여부

// 디버그용 윈도우 이름
const std::string WIN_NAME = "leftright_debug";

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

    // 그레이 변환
    cv::cvtColor(left_color,  g_left_gray,  cv::COLOR_BGR2GRAY);
    cv::cvtColor(right_color, g_right_gray, cv::COLOR_BGR2GRAY);

    // SIFT 디스크립터 계산
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

// SIFT 매칭 점수 계산 (good match 개수)
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

// BGR 이미지에서 파란 표지판 ROI 찾기
bool detectBlueSignROI(const cv::Mat& bgr, cv::Rect& roi_rect)
{
    if (bgr.empty()) return false;

    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

    cv::Mat mask1, mask2, mask;
    cv::inRange(hsv, cv::Scalar(90, 80, 80),  cv::Scalar(130,255,255), mask1);
    cv::inRange(hsv, cv::Scalar(100,80,80),  cv::Scalar(140,255,255), mask2);
    mask = mask1 | mask2;

    cv::erode(mask,  mask, cv::Mat(), cv::Point(-1,-1), 1);
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1,-1), 2);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) return false;

    double best_area = 0;
    size_t best_idx = 0;

    for (size_t i = 0; i < contours.size(); ++i)
    {
        double area = cv::contourArea(contours[i]);
        if (area > best_area)
        {
            best_area = area;
            best_idx = i;
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

// 한 장에 대해 좌/우 매칭 테스트
void testOneImage(const std::string& img_path)
{
    cv::Mat frame = cv::imread(img_path, cv::IMREAD_COLOR);
    if (frame.empty())
    {
        ROS_ERROR_STREAM("[ERR] Failed to load test image: " << img_path);
        return;
    }

    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    cv::Mat roi_gray = gray;
    cv::Rect roi_rect(0, 0, gray.cols, gray.rows);

    if (g_use_roi)
    {
        if (detectBlueSignROI(frame, roi_rect))
        {
            roi_gray = gray(roi_rect);
            ROS_INFO("Blue ROI: x=%d y=%d w=%d h=%d",
                     roi_rect.x, roi_rect.y, roi_rect.width, roi_rect.height);
        }
        else
        {
            ROS_WARN("Blue ROI not detected, using full image");
        }
    }

    std::vector<cv::KeyPoint> kp_roi;
    cv::Mat desc_roi;
    g_feat->detectAndCompute(roi_gray, cv::noArray(), kp_roi, desc_roi);

    if (desc_roi.empty())
    {
        ROS_WARN("[INFO] No SIFT features in ROI for %s", img_path.c_str());
        return;
    }

    int score_left  = computeMatchScore(desc_roi, g_desc_left);
    int score_right = computeMatchScore(desc_roi, g_desc_right);

    std::string label = "unknown";
    if (score_left > g_min_good_matches || score_right > g_min_good_matches)
    {
        label = (score_left > score_right) ? "left" : "right";
    }

    ROS_INFO("=== Test: %s ===", img_path.c_str());
    ROS_INFO(" SIFT scores ⇒ left: %d, right: %d → %s",
             score_left, score_right, label.c_str());

    cv::rectangle(frame, roi_rect, cv::Scalar(0,255,0), 2);
    cv::putText(frame, label,
                roi_rect.tl() + cv::Point(0,-10),
                cv::FONT_HERSHEY_SIMPLEX, 0.8,
                cv::Scalar(0,255,0), 2);

    cv::imshow(WIN_NAME, frame);
    cv::waitKey(0);
}

// ================== main ==================

int main(int argc, char** argv)
{
    ros::init(argc, argv, "leftright_offline_sift");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // === SIFT 생성 ===
    //g_feat = cv::xfeatures2d::SIFT::create(800);   // 800개 특징점
    //g_matcher = cv::BFMatcher(cv::NORM_L2);        // SIFT는 L2 distance

    g_feat = cv::KAZE::create();
    g_matcher = cv::BFMatcher(cv::NORM_L2);

    cv::namedWindow(WIN_NAME, cv::WINDOW_NORMAL);
    cv::resizeWindow(WIN_NAME, 800, 600);

    std::string pkg_path = ros::package::getPath("jiwon_mission");
    std::string default_left_tpl  = pkg_path + "/data/left.png";
    std::string default_right_tpl = pkg_path + "/data/right.png";
    std::string default_test1     = pkg_path + "/data/autorace_left.jpg";
    std::string default_test2     = pkg_path + "/data/autorace_right.jpg";

    std::string left_tpl, right_tpl, test1, test2;
    pnh.param("left_template_path",  left_tpl, default_left_tpl);
    pnh.param("right_template_path", right_tpl, default_right_tpl);
    pnh.param("test_image1_path",    test1,    default_test1);
    pnh.param("test_image2_path",    test2,    default_test2);

    pnh.param("use_roi",            g_use_roi, true);
    pnh.param("ratio_thresh",       g_ratio_thresh, 0.80);
    pnh.param("min_good_matches",   g_min_good_matches, 3);

    ROS_INFO("Templates: %s / %s", left_tpl.c_str(), right_tpl.c_str());

    if (!loadTemplates(left_tpl, right_tpl))
        return -1;

    if (!test1.empty()) testOneImage(test1);
    if (!test2.empty()) testOneImage(test2);

    ROS_INFO("SIFT offline test finished.");
    return 0;
}

"""
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>

using namespace cv;
using namespace std;

// ====================== 전역 =========================

Ptr<Feature2D> g_feat;        // KAZE extractor
BFMatcher g_matcher(NORM_L2); // KAZE는 L2 distance

Mat g_left_gray, g_right_gray;
Mat g_desc_left, g_desc_right;
vector<KeyPoint> g_kp_left, g_kp_right;

double g_ratio_thresh = 0.80;
int g_min_good_matches = 3;

// ====================== 함수 =========================

// 템플릿 로드
bool loadTemplate(const string& left_path, const string& right_path)
{
    Mat left = imread(left_path, IMREAD_COLOR);
    Mat right = imread(right_path, IMREAD_COLOR);

    if (left.empty() || right.empty())
    {
        cout << "[ERR] Failed to load templates." << endl;
        return false;
    }

    cvtColor(left, g_left_gray, COLOR_BGR2GRAY);
    cvtColor(right, g_right_gray, COLOR_BGR2GRAY);

    g_feat->detectAndCompute(g_left_gray, noArray(), g_kp_left, g_desc_left);
    g_feat->detectAndCompute(g_right_gray, noArray(), g_kp_right, g_desc_right);

    if (g_desc_left.empty() || g_desc_right.empty())
    {
        cout << "[ERR] Template descriptors empty." << endl;
        return false;
    }

    return true;
}

// good match 계산
int computeScore(const Mat& desc_query, const Mat& desc_train)
{
    if (desc_query.empty() || desc_train.empty()) return 0;

    vector<vector<DMatch>> knn_matches;
    g_matcher.knnMatch(desc_query, desc_train, knn_matches, 2);

    int good = 0;
    for (auto& m : knn_matches)
    {
        if (m.size() < 2) continue;
        if (m[0].distance < g_ratio_thresh * m[1].distance) good++;
    }
    return good;
}

// 파란 표지판 ROI
bool detectBlueROI(const Mat& bgr, Rect& rect)
{
    Mat hsv;
    cvtColor(bgr, hsv, COLOR_BGR2HSV);

    Mat mask;
    inRange(hsv, Scalar(90,80,80), Scalar(140,255,255), mask);

    erode(mask, mask, Mat(), Point(-1,-1), 1);
    dilate(mask, mask, Mat(), Point(-1,-1), 2);

    vector<vector<Point>> cs;
    findContours(mask, cs, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    if (cs.empty()) return false;

    double best = 0;
    int idx = -1;
    for (int i=0; i<cs.size(); i++)
    {
        double a = contourArea(cs[i]);
        if (a > best) { best = a; idx = i; }
    }
    if (best < 500) return false;

    rect = boundingRect(cs[idx]);

    int pad = 10;
    rect.x = max(0, rect.x - pad);
    rect.y = max(0, rect.y - pad);
    rect.width = min(bgr.cols - rect.x, rect.width + 2*pad);
    rect.height = min(bgr.rows - rect.y, rect.height + 2*pad);

    return true;
}

// ====================== MAIN =========================

int main()
{
    g_feat = KAZE::create();

    string left_tpl = "/root/autorace_kkk_ws/src/jiwon_mission/data/left.png";
    string right_tpl = "/root/autorace_kkk_ws/src/jiwon_mission/data/right.png";

    if (!loadTemplate(left_tpl, right_tpl)) return 0;

    VideoCapture cap(0);
    if (!cap.isOpened()) {
        cout << "[ERR] Cannot open webcam" << endl;
        return 0;
    }

    cout << "[INFO] Webcam running... (press q to quit)" << endl;

    while (true)
    {
        Mat frame;
        cap >> frame;
        if (frame.empty()) break;

        Rect roi_rect(0,0,frame.cols,frame.rows);
        Mat roi_gray;

        // 파란 ROI 찾기
        if (detectBlueROI(frame, roi_rect))
        {
            Mat gray;
            cvtColor(frame, gray, COLOR_BGR2GRAY);
            roi_gray = gray(roi_rect);
            rectangle(frame, roi_rect, Scalar(0,255,0), 2);
        }
        else
        {
            Mat gray;
            cvtColor(frame, gray, COLOR_BGR2GRAY);
            roi_gray = gray;
        }

        // 특징 추출
        vector<KeyPoint> kp;
        Mat desc;
        g_feat->detectAndCompute(roi_gray, noArray(), kp, desc);

        string label = "unknown";
        if (!desc.empty())
        {
            int sL = computeScore(desc, g_desc_left);
            int sR = computeScore(desc, g_desc_right);

            if (max(sL,sR) > g_min_good_matches)
                label = (sL > sR) ? "left" : "right";

            putText(frame, "Left="+to_string(sL)+" Right="+to_string(sR),
                    Point(10,30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0,0,255), 2);
        }

        putText(frame, label, Point(10,70),
                FONT_HERSHEY_SIMPLEX, 1.2, Scalar(0,255,0), 3);

        imshow("leftright_webcam", frame);

        char c = waitKey(1);
        if (c=='q') break;
    }

    return 0;
}
"""
