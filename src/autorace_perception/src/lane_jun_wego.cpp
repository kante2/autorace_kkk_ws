#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PointStamped.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

namespace lane_detection {

struct CenterPoint {
    double y{0.0};
    double x{0.0};
};

struct Mission1Result {
    int color_code{0};
    std::string color_str{"none"};
    cv::Rect roi;
};

struct SlidingWindowOutput {
    cv::Mat debug_image;
    std::vector<CenterPoint> left_centers;
    std::vector<CenterPoint> right_centers;
};

class LaneCurvatureNode {
public:
    LaneCurvatureNode()
        : nh_(),
          pnh_("~") {
        loadParameters();
        setupUi();
        setupRosIo();
        ROS_INFO("lane_curvature_node started");
    }

    void spin() const {
        ros::spin();
    }

private:
    void loadParameters() {
        pnh_.param("show_window", show_window_, true);
        pnh_.param("lane_width_px", lane_width_px_, 340.0);
        pnh_.param("num_windows", num_windows_, 12);
        pnh_.param("window_margin", window_margin_, 80);
        pnh_.param("minpix_recenter", minpix_recenter_, 50);
        pnh_.param("min_lane_sep", min_lane_sep_, 60);
        pnh_.param("center_ema_alpha", center_ema_alpha_, 0.8);

        pnh_.param("roi_top_y_ratio", roi_top_y_ratio_, 0.60);
        pnh_.param("roi_left_top_ratio", roi_left_top_ratio_, 0.22);
        pnh_.param("roi_right_top_ratio", roi_right_top_ratio_, 0.78);
        pnh_.param("roi_left_bot_ratio", roi_left_bot_ratio_, -0.40);
        pnh_.param("roi_right_bot_ratio", roi_right_bot_ratio_, 1.40);

        pnh_.param("mission1_min_pixel", mission1_min_pixel_, 500);
    }

    void setupUi() {
        if (!show_window_) {
            return;
        }
        win_src_ = "src_with_roi";
        win_bev_ = "bev_binary_and_windows";
        try {
            cv::startWindowThread();
        } catch (...) {
        }
        cv::namedWindow(win_src_, cv::WINDOW_NORMAL);
        cv::resizeWindow(win_src_, 960, 540);
        cv::namedWindow(win_bev_, cv::WINDOW_NORMAL);
        cv::resizeWindow(win_bev_, 960, 540);
    }

    void setupRosIo() {
        image_sub_ = nh_.subscribe("/usb_cam/image_rect_color",
                                   2,
                                   &LaneCurvatureNode::imageCallback,
                                   this);

        pub_k_left_ = nh_.advertise<std_msgs::Float32>("/perception/curvature_left", 1);
        pub_k_right_ = nh_.advertise<std_msgs::Float32>("/perception/curvature_right", 1);
        pub_k_center_ = nh_.advertise<std_msgs::Float32>("/perception/curvature_center", 1);
        pub_center_point_ = nh_.advertise<geometry_msgs::PointStamped>("/perception/center_point_px", 1);
        pub_center_color_ = nh_.advertise<geometry_msgs::PointStamped>("/perception/center_color_px", 1);
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        } catch (const cv_bridge::Exception& e) {
            ROS_WARN_STREAM_THROTTLE(1.0, "cv_bridge exception: " << e.what());
            return;
        }

        cv::Mat bgr = cv_ptr->image.clone();
        if (bgr.empty()) {
            return;
        }
        if (bgr.channels() == 1) {
            cv::cvtColor(bgr, bgr, cv::COLOR_GRAY2BGR);
        }

        try {
            processFrame(bgr, msg->header.stamp);
        } catch (const cv::Exception& e) {
            ROS_WARN_STREAM_THROTTLE(1.0, "[lane_curvature_node] OpenCV exception: " << e.what());
        } catch (const std::exception& e) {
            ROS_WARN_STREAM_THROTTLE(1.0, "[lane_curvature_node] exception: " << e.what());
        }
    }

    void processFrame(const cv::Mat& bgr, const ros::Time& stamp) {
        const int h = bgr.rows;
        const int w = bgr.cols;

        auto roi_poly = makeRoiPolygon(h, w);
        std::vector<cv::Point> roi_poly_int;
        roi_poly_int.reserve(roi_poly.size());
        for (const auto& pt : roi_poly) {
            roi_poly_int.emplace_back(static_cast<int>(std::round(pt.x)),
                                      static_cast<int>(std::round(pt.y)));
        }

        cv::Mat overlay = bgr.clone();
        std::vector<std::vector<cv::Point>> roi_container{roi_poly_int};
        cv::fillPoly(overlay, roi_container, cv::Scalar(0, 255, 0));

        cv::Mat src_vis;
        cv::addWeighted(overlay, 0.25, bgr, 0.75, 0.0, src_vis);
        cv::polylines(src_vis, roi_container, true, cv::Scalar(0, 0, 0), 2);

        cv::Mat bev_bgr = warpToBev(bgr, roi_poly);
        cv::Mat bev_binary = binarizeLanes(bev_bgr);

        auto sliding_output = runSlidingWindowCollectCenters(bev_binary);
        auto left_result = computeCurvatureFromCenters(sliding_output.left_centers, bev_binary.rows);
        auto right_result = computeCurvatureFromCenters(sliding_output.right_centers, bev_binary.rows);

        std::optional<cv::Vec3d> center_fit;
        std::optional<double> center_curvature;
        if (left_result.fit && right_result.fit) {
            center_fit = ((*left_result.fit + *right_result.fit) * 0.5);
            center_curvature = computeCurvature(*center_fit, static_cast<double>(bev_binary.rows - 1));
        }

        pub_k_left_.publish(makeCurvatureMsg(left_result.curvature));
        pub_k_right_.publish(makeCurvatureMsg(right_result.curvature));
        pub_k_center_.publish(makeCurvatureMsg(center_curvature));

        auto center_point = computeCenterPoint(sliding_output.left_centers,
                                               sliding_output.right_centers);
        if (center_point) {
            geometry_msgs::PointStamped center_msg;
            center_msg.header.stamp = stamp;
            center_msg.header.frame_id = "bev";
            center_msg.point.x = center_point->x;
            center_msg.point.y = center_point->y;
            center_msg.point.z = 0.0;
            pub_center_point_.publish(center_msg);

            cv::circle(sliding_output.debug_image,
                       cv::Point(static_cast<int>(std::round(center_point->x)),
                                 static_cast<int>(std::round(center_point->y))),
                       6,
                       cv::Scalar(255, 0, 255),
                       -1);
        }

        auto mission_result = mission1DetectCenterColor(bev_bgr);

        geometry_msgs::PointStamped color_msg;
        color_msg.header.stamp = stamp;
        color_msg.header.frame_id = "lane_color";
        color_msg.point.x = static_cast<double>(mission_result.color_code);
        color_msg.point.y = 0.0;
        color_msg.point.z = 0.0;
        pub_center_color_.publish(color_msg);

        cv::Scalar roi_color(0, 255, 255);
        if (mission_result.color_code == 1) {
            roi_color = cv::Scalar(0, 0, 255);
        } else if (mission_result.color_code == 2) {
            roi_color = cv::Scalar(255, 0, 0);
        }
        if (mission_result.roi.area() > 0) {
            cv::rectangle(sliding_output.debug_image, mission_result.roi, roi_color, 2);
        }

        if (left_result.fit) {
            drawPolynomial(sliding_output.debug_image, *left_result.fit, cv::Scalar(0, 0, 255));
        }
        if (right_result.fit) {
            drawPolynomial(sliding_output.debug_image, *right_result.fit, cv::Scalar(0, 255, 0));
        }
        if (center_fit) {
            drawPolynomial(sliding_output.debug_image, *center_fit, cv::Scalar(255, 0, 255));
        }

        annotateDebugImage(sliding_output.debug_image,
                           sliding_output.left_centers.size(),
                           sliding_output.right_centers.size(),
                           left_result.curvature,
                           right_result.curvature,
                           center_curvature,
                           mission_result.color_code,
                           mission_result.color_str);

        if (show_window_) {
            cv::Mat src_resized;
            cv::Mat dbg_resized;
            cv::resize(src_vis, src_resized, cv::Size(w, h));
            cv::resize(sliding_output.debug_image, dbg_resized, cv::Size(w, h));
            cv::Mat canvas;
            cv::hconcat(src_resized, dbg_resized, canvas);

            cv::imshow(win_src_, canvas);
            cv::imshow(win_bev_, bev_binary);
            cv::waitKey(1);
        }
    }

    std::vector<cv::Point2f> makeRoiPolygon(int h, int w) const {
        const int y_top = static_cast<int>(std::round(h * roi_top_y_ratio_));
        const int y_bot = h - 1;
        const int x_lt = static_cast<int>(std::round(w * roi_left_top_ratio_));
        const int x_rt = static_cast<int>(std::round(w * roi_right_top_ratio_));
        const int x_lb = static_cast<int>(std::round(w * roi_left_bot_ratio_));
        const int x_rb = static_cast<int>(std::round(w * roi_right_bot_ratio_));

        std::vector<cv::Point2f> poly(4);
        poly[0] = cv::Point2f(static_cast<float>(x_lb), static_cast<float>(y_bot));
        poly[1] = cv::Point2f(static_cast<float>(x_lt), static_cast<float>(y_top));
        poly[2] = cv::Point2f(static_cast<float>(x_rt), static_cast<float>(y_top));
        poly[3] = cv::Point2f(static_cast<float>(x_rb), static_cast<float>(y_bot));
        return poly;
    }

    cv::Mat warpToBev(const cv::Mat& bgr, const std::vector<cv::Point2f>& roi) const {
        CV_Assert(roi.size() == 4);
        const int h = bgr.rows;
        const int w = bgr.cols;

        std::array<cv::Point2f, 4> src = {
            cv::Point2f(roi[0].x, std::clamp(roi[0].y, 0.0f, static_cast<float>(h - 1))),
            cv::Point2f(roi[1].x, std::clamp(roi[1].y, 0.0f, static_cast<float>(h - 1))),
            cv::Point2f(roi[2].x, std::clamp(roi[2].y, 0.0f, static_cast<float>(h - 1))),
            cv::Point2f(roi[3].x, std::clamp(roi[3].y, 0.0f, static_cast<float>(h - 1)))};

        std::array<cv::Point2f, 4> dst = {
            cv::Point2f(0.0f, static_cast<float>(h - 1)),
            cv::Point2f(0.0f, 0.0f),
            cv::Point2f(static_cast<float>(w - 1), 0.0f),
            cv::Point2f(static_cast<float>(w - 1), static_cast<float>(h - 1))};

        cv::Mat M = cv::getPerspectiveTransform(src.data(), dst.data());
        cv::Mat bev;
        cv::warpPerspective(bgr,
                            bev,
                            M,
                            cv::Size(w, h),
                            cv::INTER_LINEAR,
                            cv::BORDER_CONSTANT,
                            cv::Scalar(0, 0, 0));
        return bev;
    }

    cv::Mat binarizeLanes(const cv::Mat& bgr) const {
        cv::Mat hsv;
        cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

        cv::Scalar yellow_lower(10, 80, 60);
        cv::Scalar yellow_upper(45, 255, 255);
        cv::Scalar white_lower(0, 0, 150);
        cv::Scalar white_upper(179, 60, 255);

        cv::Mat mask_yellow;
        cv::Mat mask_white;
        cv::inRange(hsv, yellow_lower, yellow_upper, mask_yellow);
        cv::inRange(hsv, white_lower, white_upper, mask_white);

        static const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::morphologyEx(mask_yellow, mask_yellow, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), 1);
        cv::morphologyEx(mask_white, mask_white, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), 1);

        cv::Mat binary;
        cv::bitwise_or(mask_yellow, mask_white, binary);
        return binary;
    }

    Mission1Result mission1DetectCenterColor(const cv::Mat& bev_bgr) const {
        Mission1Result result;
        if (bev_bgr.empty()) {
            return result;
        }
        cv::Mat hsv;
        cv::cvtColor(bev_bgr, hsv, cv::COLOR_BGR2HSV);

        const cv::Scalar red_lower1(0, 80, 80);
        const cv::Scalar red_upper1(10, 255, 255);
        const cv::Scalar red_lower2(160, 80, 80);
        const cv::Scalar red_upper2(179, 255, 255);
        const cv::Scalar blue_lower(100, 120, 80);
        const cv::Scalar blue_upper(130, 255, 255);

        cv::Mat mask_red1;
        cv::Mat mask_red2;
        cv::Mat mask_red;
        cv::Mat mask_blue;
        cv::inRange(hsv, red_lower1, red_upper1, mask_red1);
        cv::inRange(hsv, red_lower2, red_upper2, mask_red2);
        cv::bitwise_or(mask_red1, mask_red2, mask_red);
        cv::inRange(hsv, blue_lower, blue_upper, mask_blue);

        static const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::morphologyEx(mask_red, mask_red, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), 1);
        cv::morphologyEx(mask_blue, mask_blue, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), 1);

        const int w = bev_bgr.cols;
        const int h = bev_bgr.rows;
        const int x1 = static_cast<int>(0.25 * w);
        const int x2 = static_cast<int>(0.75 * w);
        const int y1 = static_cast<int>(0.5 * h);
        const int y2 = h;

        cv::Rect roi(cv::Point(x1, y1), cv::Point(x2, y2));
        roi &= cv::Rect(0, 0, w, h);
        result.roi = roi;

        int red_count = 0;
        int blue_count = 0;
        if (roi.area() > 0) {
            red_count = cv::countNonZero(mask_red(roi));
            blue_count = cv::countNonZero(mask_blue(roi));
        }

        if (red_count > mission1_min_pixel_ || blue_count > mission1_min_pixel_) {
            if (red_count > blue_count) {
                result.color_code = 1;
                result.color_str = "red";
            } else {
                result.color_code = 2;
                result.color_str = "blue";
            }
        }
        return result;
    }

    SlidingWindowOutput runSlidingWindowCollectCenters(const cv::Mat& binary_mask) const {
        SlidingWindowOutput output;
        if (binary_mask.empty()) {
            return output;
        }

        const int h = binary_mask.rows;
        const int w = binary_mask.cols;
        const int effective_windows = std::max(1, num_windows_);
        const int window_height = std::max(1, h / effective_windows);

        cv::cvtColor(binary_mask, output.debug_image, cv::COLOR_GRAY2BGR);

        cv::Mat lower_half = binary_mask(cv::Range(h / 2, h), cv::Range::all());
        cv::Mat histogram;
        cv::reduce(lower_half, histogram, 0, cv::REDUCE_SUM, CV_32S);

        auto findBase = [&](int start, int end) -> int {
            end = std::min(end, w);
            int idx = -1;
            int max_val = 0;
            for (int i = start; i < end; ++i) {
                int val = histogram.at<int>(0, i);
                if (val > max_val) {
                    max_val = val;
                    idx = i;
                }
            }
            return idx;
        };

        int left_current = findBase(0, w / 2);
        int right_current = findBase(w / 2, w);

        for (int win = 0; win < effective_windows; ++win) {
            int y_low = h - (win + 1) * window_height;
            int y_high = h - win * window_height;
            y_low = std::max(0, y_low);
            y_high = std::min(h, y_high);
            if (y_low >= y_high) {
                continue;
            }

            auto drawWindow = [&](int center_x) {
                if (center_x < 0) {
                    return;
                }
                int x_left = std::max(0, center_x - window_margin_);
                int x_right = std::min(w - 1, center_x + window_margin_);
                if (x_left >= x_right) {
                    return;
                }
                cv::rectangle(output.debug_image,
                              cv::Point(x_left, y_low),
                              cv::Point(x_right, y_high),
                              cv::Scalar(255, 0, 0),
                              2);
            };

            drawWindow(left_current);
            drawWindow(right_current);

            auto gatherPoints = [&](int center_x) {
                struct Gathered {
                    std::vector<cv::Point> pixels;
                    double mean_x{0.0};
                } gathered;
                if (center_x < 0) {
                    return gathered;
                }
                int x_left = std::max(0, center_x - window_margin_);
                int x_right = std::min(w, center_x + window_margin_);
                if (x_left >= x_right) {
                    return gathered;
                }
                cv::Rect roi(cv::Point(x_left, y_low), cv::Point(x_right, y_high));
                roi &= cv::Rect(0, 0, w, h);
                if (roi.width <= 0 || roi.height <= 0) {
                    return gathered;
                }
                cv::Mat window = binary_mask(roi);
                cv::Mat nonzero;
                cv::findNonZero(window, nonzero);
                if (nonzero.empty()) {
                    return gathered;
                }
                gathered.pixels.reserve(nonzero.rows);
                double sum_x = 0.0;
                for (int i = 0; i < nonzero.rows; ++i) {
                    const cv::Point& pt = nonzero.at<cv::Point>(i);
                    cv::Point global(pt.x + roi.x, pt.y + roi.y);
                    gathered.pixels.push_back(global);
                    sum_x += global.x;
                }
                gathered.mean_x = sum_x / static_cast<double>(gathered.pixels.size());
                return gathered;
            };

            auto left_points = gatherPoints(left_current);
            auto right_points = gatherPoints(right_current);

            if (left_current >= 0 && right_current >= 0 &&
                std::abs(left_current - right_current) < min_lane_sep_) {
                if (left_points.pixels.size() < right_points.pixels.size()) {
                    left_points.pixels.clear();
                } else {
                    right_points.pixels.clear();
                }
            }

            auto paintPoints = [&](const std::vector<cv::Point>& pixels, const cv::Scalar& color) {
                for (const auto& pt : pixels) {
                    if (pt.x >= 0 && pt.x < w && pt.y >= 0 && pt.y < h) {
                        output.debug_image.at<cv::Vec3b>(pt.y, pt.x) =
                            cv::Vec3b(static_cast<uchar>(color[0]),
                                      static_cast<uchar>(color[1]),
                                      static_cast<uchar>(color[2]));
                    }
                }
            };

            paintPoints(left_points.pixels, cv::Scalar(0, 0, 255));
            paintPoints(right_points.pixels, cv::Scalar(0, 255, 0));

            const int y_center = (y_low + y_high) / 2;

            if (!left_points.pixels.empty()) {
                output.left_centers.push_back(
                    CenterPoint{static_cast<double>(y_center), left_points.mean_x});
                cv::circle(output.debug_image,
                           cv::Point(static_cast<int>(std::round(left_points.mean_x)), y_center),
                           4,
                           cv::Scalar(0, 0, 255),
                           -1);
                if (static_cast<int>(left_points.pixels.size()) > minpix_recenter_ && left_current >= 0) {
                    left_current = static_cast<int>(center_ema_alpha_ * left_current +
                                                    (1.0 - center_ema_alpha_) * left_points.mean_x);
                }
            }

            if (!right_points.pixels.empty()) {
                output.right_centers.push_back(
                    CenterPoint{static_cast<double>(y_center), right_points.mean_x});
                cv::circle(output.debug_image,
                           cv::Point(static_cast<int>(std::round(right_points.mean_x)), y_center),
                           4,
                           cv::Scalar(0, 255, 255),
                           -1);
                if (static_cast<int>(right_points.pixels.size()) > minpix_recenter_ && right_current >= 0) {
                    right_current = static_cast<int>(center_ema_alpha_ * right_current +
                                                     (1.0 - center_ema_alpha_) * right_points.mean_x);
                }
            }
        }

        return output;
    }

    struct FitResult {
        std::optional<cv::Vec3d> fit;
        std::optional<double> curvature;
    };

    FitResult computeCurvatureFromCenters(const std::vector<CenterPoint>& centers, int image_height) const {
        FitResult result;
        if (centers.size() < 5) {
            return result;
        }

        cv::Mat A(static_cast<int>(centers.size()), 3, CV_64F);
        cv::Mat b(static_cast<int>(centers.size()), 1, CV_64F);
        for (size_t i = 0; i < centers.size(); ++i) {
            const double y = centers[i].y;
            A.at<double>(static_cast<int>(i), 0) = y * y;
            A.at<double>(static_cast<int>(i), 1) = y;
            A.at<double>(static_cast<int>(i), 2) = 1.0;
            b.at<double>(static_cast<int>(i), 0) = centers[i].x;
        }

        cv::Mat AT = A.t();
        cv::Mat ATA = AT * A;
        cv::Mat ATb = AT * b;
        cv::Mat coeffs;
        if (!cv::solve(ATA, ATb, coeffs, cv::DECOMP_SVD)) {
            return result;
        }

        result.fit = cv::Vec3d(coeffs.at<double>(0, 0),
                               coeffs.at<double>(1, 0),
                               coeffs.at<double>(2, 0));
        const double y_eval = static_cast<double>(image_height - 1);
        result.curvature = computeCurvature(*result.fit, y_eval);
        return result;
    }

    double computeCurvature(const cv::Vec3d& fit, double y_eval) const {
        const double a = fit[0];
        const double b = fit[1];
        const double dxdy = 2.0 * a * y_eval + b;
        const double d2xdy2 = 2.0 * a;
        return std::abs(d2xdy2) / std::pow(1.0 + dxdy * dxdy, 1.5);
    }

    std::optional<CenterPoint> computeCenterPoint(const std::vector<CenterPoint>& left_centers,
                                                  const std::vector<CenterPoint>& right_centers) const {
        auto meanPoint = [](const std::vector<CenterPoint>& pts) -> std::optional<CenterPoint> {
            if (pts.empty()) {
                return std::nullopt;
            }
            double sum_y = 0.0;
            double sum_x = 0.0;
            for (const auto& p : pts) {
                sum_y += p.y;
                sum_x += p.x;
            }
            const double denom = static_cast<double>(pts.size());
            return CenterPoint{sum_y / denom, sum_x / denom};
        };

        const auto left_mean = meanPoint(left_centers);
        const auto right_mean = meanPoint(right_centers);
        const double half_width = 0.5 * lane_width_px_;

        if (left_mean && right_mean) {
            return CenterPoint{0.5 * (left_mean->y + right_mean->y),
                               0.5 * (left_mean->x + right_mean->x)};
        }
        if (left_mean) {
            return CenterPoint{left_mean->y, left_mean->x + half_width};
        }
        if (right_mean) {
            return CenterPoint{right_mean->y, right_mean->x - half_width};
        }
        return std::nullopt;
    }

    void drawPolynomial(cv::Mat& canvas,
                        const cv::Vec3d& fit,
                        const cv::Scalar& color,
                        int step = 10) const {
        if (canvas.empty()) {
            return;
        }
        const int h = canvas.rows;
        for (int y = 0; y < h; y += step) {
            const double x = fit[0] * y * y + fit[1] * y + fit[2];
            const int xi = static_cast<int>(std::round(x));
            if (xi >= 0 && xi < canvas.cols) {
                cv::circle(canvas, cv::Point(xi, y), 2, color, -1);
            }
        }
    }

    void annotateDebugImage(cv::Mat& img,
                            size_t left_count,
                            size_t right_count,
                            const std::optional<double>& left_curv,
                            const std::optional<double>& right_curv,
                            const std::optional<double>& center_curv,
                            int color_code,
                            const std::string& color_str) const {
        auto put = [&](const std::string& text, int y) {
            cv::putText(img,
                        text,
                        cv::Point(10, y),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.6,
                        cv::Scalar(255, 255, 255),
                        2,
                        cv::LINE_AA);
        };

        auto curvToString = [](const std::optional<double>& v) {
            if (!v) {
                return std::string("nan");
            }
            char buf[64];
            std::snprintf(buf, sizeof(buf), "%.4e", *v);
            return std::string(buf);
        };

        put("Left centers: " + std::to_string(left_count), 24);
        put("Right centers: " + std::to_string(right_count), 48);
        put("Curvature Left: " + curvToString(left_curv) +
                " Curvature Right: " + curvToString(right_curv) +
                " Curvature Center: " + curvToString(center_curv),
            72);
        put("Mission1 CenterColor: " + color_str + " (code=" + std::to_string(color_code) + ")", 96);
    }

    std_msgs::Float32 makeCurvatureMsg(const std::optional<double>& curvature) const {
        std_msgs::Float32 msg;
        if (curvature) {
            msg.data = static_cast<float>(*curvature);
        } else {
            msg.data = std::numeric_limits<float>::quiet_NaN();
        }
        return msg;
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber image_sub_;
    ros::Publisher pub_k_left_;
    ros::Publisher pub_k_right_;
    ros::Publisher pub_k_center_;
    ros::Publisher pub_center_point_;
    ros::Publisher pub_center_color_;

    bool show_window_{true};
    std::string win_src_;
    std::string win_bev_;

    double lane_width_px_{340.0};
    int num_windows_{12};
    int window_margin_{80};
    int minpix_recenter_{50};
    int min_lane_sep_{60};
    double center_ema_alpha_{0.8};

    double roi_top_y_ratio_{0.60};
    double roi_left_top_ratio_{0.22};
    double roi_right_top_ratio_{0.78};
    double roi_left_bot_ratio_{-0.40};
    double roi_right_bot_ratio_{1.40};

    int mission1_min_pixel_{500};
};

}  // namespace lane_detection

int main(int argc, char** argv) {
    ros::init(argc, argv, "lane_curvature_node");
    lane_detection::LaneCurvatureNode node;
    node.spin();
    return 0;
}
