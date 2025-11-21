// 통합한 코드 교체해서 확인해보는 목적이라
// computeContours, computeCrosswalk만 넣을게요
// 원래 computewhiteratio(), compute_StopState()




// =======정지선 / 횡단보도 관련 전역 변수 변경 =========
"""
Origin:
// 정지선 관련 전역 변수
double    g_white_ratio_threshold = 0.25;  // 흰색 비율 threshold
bool      g_stop_active           = false; // 현재 정지 중인지 여부
ros::Time g_stop_start_time;              // 정지 시작 시간
double    g_stop_duration         = 7.0;   // 정지 유지 시간(초)
"""


// 변경 내용
double    g_crosswalk_stripe_threshold = 3.5;  // 예: stripe 3~4개 이상이면 정지
bool      g_stop_active                = false; // 현재 정지 중인지 여부
ros::Time g_stop_start_time;                   // 정지 시작 시간
double    g_stop_duration              = 7.0;   // 정지 유지 시간(초)

// 컨투어 필터링 파라미터
double    g_min_contour_area   = 50.0;  // 너무 작은 노이즈 컨투어 제거
double    g_min_aspect_ratio   = 3.0;   // w/h >= 3.0 이면 가로로 긴 stripe
double    g_min_width_ratio    = 0.4;   // 이미지 폭의 40% 이상 길이


// ============= main() parameter loading ======================

"""
Origin:
pnh.param<double>("white_ratio_threshold", g_white_ratio_threshold, 0.25);
pnh.param<double>("stop_duration",        g_stop_duration,        7.0);
"""


// 변경내용
pnh.param<double>("crosswalk_stripe_threshold", g_crosswalk_stripe_threshold, 3.5);
pnh.param<double>("stop_duration",              g_stop_duration,              7.0);

pnh.param<double>("min_contour_area", g_min_contour_area, 50.0);
pnh.param<double>("min_aspect_ratio", g_min_aspect_ratio, 3.0);
pnh.param<double>("min_width_ratio",  g_min_width_ratio,  0.4);




// ======================== 컨투어 계산 함수 =================
// origin: computeWhiteRatio()
// 컨투어 기반으로 "횡단보도 stripe 개수"를 score로 계산
// 컨투어 기반으로 "횡단보도 stripe 개수"를 score로 계산
double compute_Contours(const cv::Mat& binary)
{
  int h = binary.rows;
  int w = binary.cols;

  if (w <= 0 || h <= 0) {
    return 0.0;
  }

  // findContours가 입력 이미지를 바꾸므로 복사본 사용
  cv::Mat bin_clone = binary.clone();

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(bin_clone, contours,
                   cv::RETR_EXTERNAL,       // 외곽만
                   cv::CHAIN_APPROX_SIMPLE  // 포인트 수 줄이기
  );

  struct Stripe
  {
    double cy;   // stripe 중심 y (세로 방향)
    cv::Rect rect;
  };

  std::vector<Stripe> stripes;
  stripes.reserve(contours.size());

  for (const auto& c : contours)
  {
    cv::Rect r = cv::boundingRect(c);
    double area = static_cast<double>(r.area());
    if (area < g_min_contour_area) {
      continue; // 너무 작은 노이즈 컨투어는 무시
    }

    // 가로로 충분히 긴 직사각형만 사용 (횡단보도 stripe 특성)
    double aspect = static_cast<double>(r.width) /
                    static_cast<double>(std::max(1, r.height));
    if (aspect < g_min_aspect_ratio) {
      continue;
    }

    // 이미지 폭의 일정 비율 이상이어야 함
    if (r.width < static_cast<int>(g_min_width_ratio * w)) {
      continue;
    }

    double cy = r.y + 0.5 * r.height;
    stripes.push_back({cy, r});
  }

  if (stripes.empty())
  {
    ROS_INFO_THROTTLE(1.0,
      "[stopline_node] stripes=0 (no crosswalk-like rectangles)");
    return 0.0;
  }

  // y(세로 방향) 기준으로 정렬
  std::sort(stripes.begin(), stripes.end(),
            [](const Stripe& a, const Stripe& b){
              return a.cy < b.cy;
            });

  // 같은 횡단보도 내 stripe 간 최대 간격 (이미지 높이 비율)
  const double max_gap = h * 0.12;  // 필요 시 0.1~0.2에서 튜닝

  int best_group = 1;
  int current    = 1;

  for (size_t i = 1; i < stripes.size(); ++i)
  {
    double dy = stripes[i].cy - stripes[i - 1].cy;
    if (dy <= max_gap)
    {
      // 이전 stripe와 충분히 가까우면 같은 그룹
      current++;
    }
    else
    {
      // 그룹 끊기 → 최대값 갱신
      best_group = std::max(best_group, current);
      current = 1;
    }
  }
  best_group = std::max(best_group, current);

  double score = static_cast<double>(best_group);

  ROS_INFO_THROTTLE(1.0,
      "[stopline_node] stripe_group_count=%.1f thr=%.1f",
      score, g_crosswalk_stripe_threshold);

  return score;
}



// ============ 정지 상태 관리 함수 =========
// origin: compute_StopState()

void compute_Crosswalk(double score, const ros::Time& now)
{
  const bool can_start  = (!g_stop_active) &&
                          (score > g_crosswalk_stripe_threshold);
  const bool can_finish = g_stop_active &&
                          !g_stop_start_time.isZero() &&
                          (now - g_stop_start_time).toSec() >= g_stop_duration;

  if (can_start)
  {
    g_stop_active     = true;
    g_stop_start_time = now;
    ROS_INFO("[stopline_node] STOP TRIGGERED (score=%.3f thr=%.3f)",
             score, g_crosswalk_stripe_threshold);
  }

  if (can_finish)
  {
    g_stop_active     = false;
    g_stop_start_time = ros::Time(0);
    ROS_INFO("[stopline_node] STOP RELEASED (duration done)");
  }
}




// ================ image 호츌 변경 ================
"""
Origin:
// 3-1) 횡단보도(정지선) 인식: 흰색 비율 계산
double white_ratio = computeWhiteRatio(bev_binary);

// 3-2) 정지 상태 업데이트 (7초 정지 등)
ros::Time now = ros::Time::now();
compute_StopState(white_ratio, now);
"""

//변경 

// 3-1) 횡단보도(정지선) 인식: 컨투어 기반 score 계산
double score = compute_Contours(bev_binary);

// 3-2) 정지 상태 업데이트 (7초 정지 등)
ros::Time now = ros::Time::now();
compute_Crosswalk(score, now);
