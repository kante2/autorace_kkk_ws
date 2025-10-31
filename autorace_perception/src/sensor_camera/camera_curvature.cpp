#include <opencv2/opencv.hpp>
#include <iostream>

// =========== 1) 버드뷰 변환 (원근변환) ===========
cv::Mat warpBirdview(const cv::Mat& img_binary,
                     const std::vector<cv::Point2f>& src,
                     const std::vector<cv::Point2f>& dst,
                     cv::Mat& M, cv::Mat& Minv,
                     const cv::Size& bev_size)
{
    M = cv::getPerspectiveTransform(src, dst);
    Minv = cv::getPerspectiveTransform(dst, src);
    cv::Mat bev;
    cv::warpPerspective(img_binary, bev, M, bev_size, cv::INTER_NEAREST);
    return bev;
}

// =========== 2) 히스토그램 + 슬라이딩 윈도우로 차선 픽셀 추출 ===========
struct LanePixels {
    std::vector<cv::Point> left_pts;
    std::vector<cv::Point> right_pts;
};

LanePixels slidingWindowSearch(const cv::Mat& bev_binary,
                               int nwindows=9, int margin=100, int minpix=50)
{
    CV_Assert(bev_binary.type()==CV_8UC1);
    int H = bev_binary.rows, W = bev_binary.cols;

    // 하단 절반 히스토그램 (열별 픽셀 합)
    cv::Mat lower = bev_binary(cv::Rect(0, H/2, W, H - H/2));
    cv::Mat hist;
    cv::reduce(lower, hist, 0, cv::REDUCE_SUM, CV_32S); // 1xW

    // 좌/우 시작점: 히스토그램 왼쪽/오른쪽 최대값
    cv::Point left_max, right_max;
    cv::minMaxLoc(hist.colRange(0, W/2), nullptr, nullptr, nullptr, &left_max);
    cv::minMaxLoc(hist.colRange(W/2, W), nullptr, nullptr, nullptr, &right_max);
    int leftx_current  = left_max.x;
    int rightx_current = right_max.x + W/2;

    int window_height = H / nwindows;

    std::vector<cv::Point> left_pts, right_pts;

    for(int w=0; w<nwindows; ++w){
        int win_y_low  = H - (w+1)*window_height;
        int win_y_high = H - w*window_height;

        int left_x_low  = std::max(0, leftx_current - margin);
        int left_x_high = std::min(W-1, leftx_current + margin);
        int right_x_low  = std::max(0, rightx_current - margin);
        int right_x_high = std::min(W-1, rightx_current + margin);

        // ROI 잘라서 nonzero 픽셀 좌표 수집
        cv::Mat left_win  = bev_binary(cv::Rect(left_x_low,  win_y_low,  left_x_high-left_x_low+1,  win_y_high-win_y_low));
        cv::Mat right_win = bev_binary(cv::Rect(right_x_low, win_y_low, right_x_high-right_x_low+1, win_y_high-win_y_low));

        std::vector<cv::Point> nz_left, nz_right;
        cv::findNonZero(left_win, nz_left);
        cv::findNonZero(right_win, nz_right);

        // 원래 좌표계로 복원
        for(auto &p : nz_left){
            left_pts.emplace_back(p.x + left_x_low, p.y + win_y_low);
        }
        for(auto &p : nz_right){
            right_pts.emplace_back(p.x + right_x_low, p.y + win_y_low);
        }

        // 픽셀이 충분하면 중심 재설정
        if((int)nz_left.size() > minpix){
            int sumx=0;
            for(auto &p:nz_left) sumx += p.x + left_x_low;
            leftx_current = sumx / (int)nz_left.size();
        }
        if((int)nz_right.size() > minpix){
            int sumx=0;
            for(auto &p:nz_right) sumx += p.x + right_x_low;
            rightx_current = sumx / (int)nz_right.size();
        }
    }

    return {left_pts, right_pts};
}

// =========== 3) 2차 다항식 피팅 (x = a*y^2 + b*y + c) ===========
bool polyfit2(const std::vector<cv::Point>& pts, cv::Vec3d& coeff_aybc)
{
    if(pts.size() < 6) return false; // 최소 샘플 보장

    // A*[a,b,c]^T = x  (A: [y^2 y 1], x: x)
    cv::Mat A((int)pts.size(), 3, CV_64F);
    cv::Mat X((int)pts.size(), 1, CV_64F);

    for(size_t i=0;i<pts.size();++i){
        double y = (double)pts[i].y;
        A.at<double>(i,0) = y*y;
        A.at<double>(i,1) = y;
        A.at<double>(i,2) = 1.0;
        X.at<double>(i,0) = (double)pts[i].x;
    }

    cv::Mat sol; // 3x1
    bool ok = cv::solve(A, X, sol, cv::DECOMP_NORMAL|cv::DECOMP_SVD);
    if(!ok) return false;

    coeff_aybc = cv::Vec3d(sol.at<double>(0), sol.at<double>(1), sol.at<double>(2));
    return true;
}

// =========== 4) 곡률반경 계산 (미터 좌표에서 피팅) ===========
double curvatureRadiusMeters(const std::vector<cv::Point>& pts,
                             double xm_per_pix, double ym_per_pix,
                             double y_eval_pix)
{
    if(pts.size() < 6) return std::numeric_limits<double>::infinity();

    // 픽셀점을 미터점으로 변환해서 "다시" 피팅: X_m = a_m*Y_m^2 + b_m*Y_m + c_m
    std::vector<cv::Point2d> meter_pts;
    meter_pts.reserve(pts.size());
    for(const auto& p : pts){
        meter_pts.push_back( cv::Point2d(p.x*xm_per_pix, p.y*ym_per_pix) );
    }

    cv::Mat A((int)meter_pts.size(), 3, CV_64F);
    cv::Mat X((int)meter_pts.size(), 1, CV_64F);
    for(size_t i=0;i<meter_pts.size();++i){
        double Ym = meter_pts[i].y;
        A.at<double>(i,0) = Ym*Ym;
        A.at<double>(i,1) = Ym;
        A.at<double>(i,2) = 1.0;
        X.at<double>(i,0) = meter_pts[i].x;
    }

    cv::Mat sol;
    bool ok = cv::solve(A, X, sol, cv::DECOMP_NORMAL|cv::DECOMP_SVD);
    if(!ok) return std::numeric_limits<double>::infinity();

    double a = sol.at<double>(0);
    double b = sol.at<double>(1);

    double y_eval_m = y_eval_pix * ym_per_pix;
    double dy = 2.0*a*y_eval_m + b;
    double R = std::pow(1.0 + dy*dy, 1.5) / std::fabs(2.0*a);
    return R; // meters
}

// =========== 5) 차량 중심 오프셋 (미터) ===========
double lateralOffsetFromCenter(const cv::Vec3d& poly_pix,
                               int img_width, double xm_per_pix)
{
    // 하단(y = H-1)에서 차선 x좌표
    double y = img_width; // 잘못! → 아래에서 고침
    return 0.0;
}

// 수정: 오프셋은 “이미지 하단 y=H-1”에서 차선 중심과 이미지 중심 차이
double laneCenterOffsetMeters(const cv::Vec3d& left_poly_pix,
                              const cv::Vec3d& right_poly_pix,
                              int img_h, int img_w,
                              double xm_per_pix)
{
    double y = img_h - 1;
    auto x_at = [&](const cv::Vec3d& p, double ypix){
        return p[0]*ypix*ypix + p[1]*ypix + p[2];
    };
    double x_left  = x_at(left_poly_pix,  y);
    double x_right = x_at(right_poly_pix, y);
    double lane_center_x = 0.5*(x_left + x_right);
    double image_center_x = img_w * 0.5;
    double offset_pix = (image_center_x - lane_center_x);
    return offset_pix * xm_per_pix; // +면 차량이 오른쪽으로 치우침(정의는 필요에 맞게)
}

// ===================== 예시 사용법 =====================
int main()
{
    // 0) 이진화된 입력 이미지(차선=255, 배경=0). 예시로 임의 생성:
    cv::Mat bin = cv::imread("binary_lane.png", cv::IMREAD_GRAYSCALE);
    if(bin.empty()){
        std::cerr << "Load a binary lane image (white lane on black)" << std::endl;
        return 0;
    }
    cv::threshold(bin, bin, 128, 255, cv::THRESH_BINARY);

    // 1) 버드뷰: src/dst 코너는 카메라/트랙에 맞게 교정
    std::vector<cv::Point2f> src = {
        {560, 460}, {720, 460}, {1120, 720}, {200, 720}
    };
    std::vector<cv::Point2f> dst = {
        {300, 0}, {980, 0}, {980, 720}, {300, 720}
    };
    cv::Mat M, Minv;
    cv::Mat bev = warpBirdview(bin, src, dst, M, Minv, bin.size());

    // 2) 슬라이딩 윈도우로 좌/우 차선 픽셀 탐색
    LanePixels px = slidingWindowSearch(bev, 9, 100, 50);

    // 3) 픽셀좌표에서 2차식 피팅 (x = a*y^2 + b*y + c)
    cv::Vec3d left_poly, right_poly;
    bool okL = polyfit2(px.left_pts, left_poly);
    bool okR = polyfit2(px.right_pts, right_poly);
    if(!okL || !okR){
        std::cerr << "Polynomial fit failed." << std::endl;
        return 0;
    }

    // 4) 곡률반경 (미터 스케일로 재피팅 후 계산)
    double xm_per_pix = 3.7/700.0;   // 가정: 차선폭 3.7m가 BEV에서 700px
    double ym_per_pix = 30.0/720.0;  // 가정: 세로 30m가 720px
    int H = bev.rows, W = bev.cols;

    double y_eval_pix = H - 1; // 차량 바로 앞(화면 하단)에서 곡률평가
    double R_left  = curvatureRadiusMeters(px.left_pts,  xm_per_pix, ym_per_pix, y_eval_pix);
    double R_right = curvatureRadiusMeters(px.right_pts, xm_per_pix, ym_per_pix, y_eval_pix);
    double R_mean  = 0.5*(R_left + R_right);

    // 5) 차량 중심 오프셋(미터)
    double offset_m = laneCenterOffsetMeters(left_poly, right_poly, H, W, xm_per_pix);

    std::cout << "R_left(m): "  << R_left
              << ", R_right(m): " << R_right
              << ", R_mean(m): "  << R_mean << std::endl;
    std::cout << "Center offset (m, + = right): " << offset_m << std::endl;

    // (옵션) 가시화
    cv::Mat vis; cv::cvtColor(bev, vis, cv::COLOR_GRAY2BGR);
    auto drawPoly = [&](const cv::Vec3d& p, const cv::Scalar& color){
        for(int y=0; y<H; y+=5){
            double x = p[0]*y*y + p[1]*y + p[2];
            if(0<=x && x<W) cv::circle(vis, cv::Point((int)x, y), 2, color, -1);
        }
    };
    drawPoly(left_poly,  {0,255,0});
    drawPoly(right_poly, {255,0,0});
    cv::putText(vis, "R_mean(m): "+std::to_string((int)R_mean), {30,50}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {0,255,255}, 2);
    cv::putText(vis, "offset(m): "+cv::format("%.2f", offset_m), {30,90}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {0,255,255}, 2);
    cv::imshow("BEV+fit", vis);
    cv::waitKey(0);
    return 0;
}

/*

버드뷰 -> 슬라이딩 윈도우 -> 2차 다항식으로 피팅 -> 곡률 계산
*/