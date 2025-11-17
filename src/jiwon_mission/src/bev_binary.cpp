// bev_binary_node.py -> bev_binary_node.cpp
// C++ version of BevBinaryNode (ROI + BEV + white binary publishing)

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

class BevBinaryNode
{
public:
    BevBinaryNode()
        : nh_(), pnh_("~")
    {
        // === Parameters ===
        pnh_.param("show_window", show_window_, true);

        pnh_.param("roi_top_y_ratio",     roi_top_y_ratio_,     0.60);
        pnh_.param("roi_left_top_ratio",  roi_left_top_ratio_,  0.22);
        pnh_.param("roi_right_top_ratio", roi_right_top_ratio_, 0.78);
        pnh_.param("roi_left_bot_ratio",  roi_left_bot_ratio_,  -0.40);
        pnh_.param("roi_right_bot_ratio", roi_right_bot_ratio_, 1.40);

        // HSV white threshold (same as Python)
        //white_lower_ = cv::Scalar(  0,   0, 150);
        //white_upper_ = cv::Scalar(179,  60, 255);
        white_lower_ = cv::Scalar(10,   80, 60); //**테스트용 노란색으로 변경하여 사용중**
        white_upper_ = cv::Scalar(45,  255, 255);


        // 확인용 노란선 읽기
        //# self.yellow_lower = np.array([10,  80,  60], dtype=np.uint8)
        //# self.yellow_upper = np.array([45, 255, 255], dtype=np.uint8)
        //# self.white_lower  = np.array([ 0,   0, 150], dtype=np.uint8)
        //# self.white_upper  = np.array([179,  60, 255], dtype=np.uint8)
        // === ROS IO ===
        sub_image_ = nh_.subscribe(
            "/usb_cam/image_rect_color",
            2,
            &BevBinaryNode::imageCallback,
            this);

        pub_bev_binary_ = nh_.advertise<sensor_msgs::Image>(
            "/stopline/bev_binary",
            1);

        if (show_window_)
        {
            cv::namedWindow("bev_binary_and_windows", cv::WINDOW_NORMAL);
            cv::resizeWindow("bev_binary_and_windows", 960, 540);
            cv::namedWindow("src_with_roi", cv::WINDOW_NORMAL);
            cv::resizeWindow("src_with_roi", 960, 540);
        }

        ROS_INFO("BevBinaryNode started");
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber sub_image_;
    ros::Publisher pub_bev_binary_;

    bool show_window_;

    // ROI ratios
    double roi_top_y_ratio_;
    double roi_left_top_ratio_;
    double roi_right_top_ratio_;
    double roi_left_bot_ratio_;
    double roi_right_bot_ratio_;

    // HSV white
    cv::Scalar white_lower_;
    cv::Scalar white_upper_;

    // --------- Helpers ---------
    std::vector<cv::Point2f> makeRoiPolygon(int h, int w)
    {
        int y_top = static_cast<int>(h * roi_top_y_ratio_);
        int y_bot = h - 1;
        int x_lt  = static_cast<int>(w * roi_left_top_ratio_);
        int x_rt  = static_cast<int>(w * roi_right_top_ratio_);
        int x_lb  = static_cast<int>(w * roi_left_bot_ratio_);
        int x_rb  = static_cast<int>(w * roi_right_bot_ratio_);

        std::vector<cv::Point2f> poly(4);
        // BL, TL, TR, BR
        poly[0] = cv::Point2f(static_cast<float>(x_lb), static_cast<float>(y_bot));
        poly[1] = cv::Point2f(static_cast<float>(x_lt), static_cast<float>(y_top));
        poly[2] = cv::Point2f(static_cast<float>(x_rt), static_cast<float>(y_top));
        poly[3] = cv::Point2f(static_cast<float>(x_rb), static_cast<float>(y_bot));
        return poly;
    }

    cv::Mat warpToBev(const cv::Mat& bgr, const std::vector<cv::Point2f>& roi_poly)
    {
        int h = bgr.rows;
        int w = bgr.cols;

        cv::Point2f BL = roi_poly[0];
        cv::Point2f TL = roi_poly[1];
        cv::Point2f TR = roi_poly[2];
        cv::Point2f BR = roi_poly[3];

        // y는 [0, h-1]로 클리핑
        BL.y = std::max(0.f, std::min(static_cast<float>(h - 1), BL.y));
        TL.y = std::max(0.f, std::min(static_cast<float>(h - 1), TL.y));
        TR.y = std::max(0.f, std::min(static_cast<float>(h - 1), TR.y));
        BR.y = std::max(0.f, std::min(static_cast<float>(h - 1), BR.y));

        std::vector<cv::Point2f> srcPts{BL, TL, TR, BR};
        std::vector<cv::Point2f> dstPts{
            cv::Point2f(0.0f,        static_cast<float>(h - 1)),
            cv::Point2f(0.0f,        0.0f),
            cv::Point2f(static_cast<float>(w - 1), 0.0f),
            cv::Point2f(static_cast<float>(w - 1), static_cast<float>(h - 1))
        };

        cv::Mat M = cv::getPerspectiveTransform(srcPts, dstPts);

        cv::Mat bev;
        cv::warpPerspective(
            bgr, bev, M, cv::Size(w, h),
            cv::INTER_LINEAR,
            cv::BORDER_CONSTANT,
            cv::Scalar(0, 0, 0));
        return bev;
    }

    cv::Mat binarizeWhite(const cv::Mat& bgr)
    {
        cv::Mat hsv;
        cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

        cv::Mat mask_w;
        cv::inRange(hsv, white_lower_, white_upper_, mask_w);

        cv::Mat kernel = cv::getStructuringElement(
            cv::MORPH_RECT,
            cv::Size(3, 3));

        cv::morphologyEx(mask_w, mask_w, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), 1);
        return mask_w;  // uint8(0,255)
    }

    // --------- Callback ---------
    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat bgr = cv_ptr->image.clone();
        if (bgr.empty())
            return;

        int h = bgr.rows;
        int w = bgr.cols;

        // 1) ROI polygon & visualize
        auto roi_poly = makeRoiPolygon(h, w);

        cv::Mat src_vis = bgr.clone();
        cv::Mat overlay = bgr.clone();

        std::vector<cv::Point> roi_poly_int;
        for (const auto& p : roi_poly)
            roi_poly_int.emplace_back(static_cast<int>(p.x), static_cast<int>(p.y));

        cv::fillPoly(overlay, std::vector<std::vector<cv::Point>>{roi_poly_int},
                     cv::Scalar(0, 255, 0));
        cv::addWeighted(overlay, 0.25, bgr, 0.75, 0.0, src_vis);
        cv::polylines(src_vis, std::vector<std::vector<cv::Point>>{roi_poly_int},
                      true, cv::Scalar(0, 0, 0), 2);

        // 2) BEV
        cv::Mat bev_bgr = warpToBev(bgr, roi_poly);

        // 3) Binary (white only)
        cv::Mat bev_binary = binarizeWhite(bev_bgr);

        // 4) Publish as mono8 Image
        cv_bridge::CvImage out_msg;
        out_msg.header = msg->header;
        out_msg.encoding = "mono8";
        out_msg.image = bev_binary;
        pub_bev_binary_.publish(out_msg.toImageMsg());

        // 5) Debug show
        if (show_window_)
        {
            cv::imshow("src_with_roi", src_vis);
            cv::imshow("bev_binary_and_windows", bev_binary);
            cv::waitKey(1);
        }
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "bev_binary");
    BevBinaryNode node;
    ros::spin();
    return 0;
}
