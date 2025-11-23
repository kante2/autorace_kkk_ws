#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>

// 현재 케이스 학습한 yolo에 대해 class + bb나오게 해놓음, 

// -------------------- 유틸 --------------------
inline float clampf(float x, float lo, float hi)
{
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

// -------------------- 전역 상태 --------------------

// 파라미터 / 토픽 이름 기본값
std::string g_model_path_onnx = "/root/autorace_kkk_ws/src/yolo/best.onnx";
std::string g_classes_file    = "/root/autorace_kkk_ws/src/yolo/classes.txt";
std::string g_image_topic     = "/usb_cam/image_rect_color";

// YOLO 추론 관련 파라미터
bool  g_show_window = true;
float g_conf_thr    = 0.5f;
float g_iou_thr     = 0.5f;
int   g_imgsz       = 640;

// OpenCV 윈도우 이름
std::string g_win_name = "YOLO Camera";

// DNN 네트워크 + 클래스 이름
cv::dnn::Net           g_net;
std::vector<std::string> g_class_names;

// 이미지 크기 (콜백에서 기록)
int g_last_img_w = 0;
int g_last_img_h = 0;

// -------------------- 클래스 이름 파일 로드 --------------------
bool loadClassNames(const std::string& path,
                    std::vector<std::string>& class_names)
{
  class_names.clear();
  std::ifstream ifs(path.c_str());
  if (!ifs.is_open()) {
    ROS_ERROR("[yolo_cam_node] failed to open classes file: %s", path.c_str());
    return false;
  }

  std::string line;
  while (std::getline(ifs, line)) {
    if (!line.empty()) {
      // 앞뒤 공백 제거
      line.erase(0, line.find_first_not_of(" \t\r\n"));
      line.erase(line.find_last_not_of(" \t\r\n") + 1);
      if (!line.empty()) {
        class_names.push_back(line);
      }
    }
  }

  if (class_names.empty()) {
    ROS_WARN("[yolo_cam_node] classes file is empty: %s", path.c_str());
    return false;
  }

  ROS_INFO("[yolo_cam_node] loaded %zu classes from '%s'",
           class_names.size(), path.c_str());
  return true;
}

// -------------------- 헬퍼: 모델 클래스 목록 로그 --------------------
void logModelClasses()
{
  ROS_INFO("[yolo_cam_node] ===== YOLO Model Classes =====");
  for (size_t i = 0; i < g_class_names.size(); ++i) {
    ROS_INFO("  id=%zu  name='%s'", i, g_class_names[i].c_str());
  }
  ROS_INFO("[yolo_cam_node] ==================================");
}

// -------------------- 헬퍼: NMS 적용 --------------------
struct Detection {
  cv::Rect box;
  int      class_id;
  float    score;
};

void nmsDetections(const std::vector<Detection>& dets,
                   float iou_thr,
                   std::vector<Detection>& out)
{
  out.clear();
  if (dets.empty()) return;

  // score 기준 내림차순 정렬
  std::vector<int> idxs(dets.size());
  for (size_t i = 0; i < dets.size(); ++i) idxs[i] = (int)i;
  std::sort(idxs.begin(), idxs.end(),
            [&](int a, int b) {
              return dets[a].score > dets[b].score;
            });

  std::vector<bool> suppressed(dets.size(), false);

  for (size_t i = 0; i < idxs.size(); ++i) {
    int idx_i = idxs[i];
    if (suppressed[idx_i]) continue;

    const Detection& di = dets[idx_i];
    out.push_back(di);

    for (size_t j = i + 1; j < idxs.size(); ++j) {
      int idx_j = idxs[j];
      if (suppressed[idx_j]) continue;

      const Detection& dj = dets[idx_j];
      // 같은 클래스만 NMS 처리 (원하면 제거 가능)
      if (di.class_id != dj.class_id) continue;

      float iou = (float) (di.box & dj.box).area() /
                  (float) (di.box | dj.box).area();
      if (iou > iou_thr) {
        suppressed[idx_j] = true;
      }
    }
  }
}

// -------------------- 콜백 --------------------
void imageCB(const sensor_msgs::ImageConstPtr& msg)
{
  // ROS Image -> OpenCV BGR
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
  } catch (cv_bridge::Exception& e) {
    ROS_WARN("[yolo_cam_node] cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat frame = cv_ptr->image;
  if (frame.empty()) return;

  g_last_img_w = frame.cols;
  g_last_img_h = frame.rows;

  // ----- DNN 입력 준비 (blob) -----
  cv::Mat blob = cv::dnn::blobFromImage(
      frame,
      1.0f / 255.0f,
      cv::Size(g_imgsz, g_imgsz),
      cv::Scalar(0, 0, 0),
      true,   // swapRB
      false   // crop
  );

  g_net.setInput(blob);

  // Ultralytics YOLOv8 ONNX: (1, 84, N) 형태로 나오는 경우가 많음
  cv::Mat out = g_net.forward();

  // out: shape [1, C, N] 또는 [1, N, C]
  const int dims = out.dims;  // 보통 3
  const int* sizes = out.size; // e.g. [1, 84, 8400]
  if (dims != 3) {
    ROS_WARN_THROTTLE(1.0, "[yolo_cam_node] unexpected output dims=%d", dims);
    return;
  }

  int out0 = sizes[0]; // 1
  int out1 = sizes[1]; // C or N
  int out2 = sizes[2]; // N or C

  int num_preds = 0;
  int num_ch    = 0;
  bool channels_first = false;

  // 가정: [1, C, N] 형태
  if (out1 > out2) {
    num_ch = out1;
    num_preds = out2;
    channels_first = true;
  } else {
    // [1, N, C] 형태인 경우
    num_ch = out2;
    num_preds = out1;
    channels_first = false;
  }

  if (num_ch < 5) {
    ROS_WARN_THROTTLE(1.0,
      "[yolo_cam_node] num_ch=%d < 5, unsupported format", num_ch);
    return;
  }

  int num_classes = num_ch - 4;
  if (!g_class_names.empty() && (int)g_class_names.size() != num_classes) {
    ROS_WARN_THROTTLE(1.0,
      "[yolo_cam_node] class_names.size()=%zu != num_classes=%d (from ONNX)",
      g_class_names.size(), num_classes);
  }

  // 예측 결과 파싱
  std::vector<Detection> detections;
  detections.reserve(num_preds);

  const float* data = (float*)out.data;

  for (int i = 0; i < num_preds; ++i) {
    float cx, cy, w, h;
    if (channels_first) {
      // 인덱싱: data[c * num_preds + i]
      cx = data[0 * num_preds + i];
      cy = data[1 * num_preds + i];
      w  = data[2 * num_preds + i];
      h  = data[3 * num_preds + i];
    } else {
      // 인덱싱: data[i * num_ch + c]
      cx = data[i * num_ch + 0];
      cy = data[i * num_ch + 1];
      w  = data[i * num_ch + 2];
      h  = data[i * num_ch + 3];
    }

    // 클래스 점수 중 최대값 찾기
    float max_score = -1.0f;
    int   max_cls   = -1;
    if (channels_first) {
      for (int c = 0; c < num_classes; ++c) {
        float s = data[(4 + c) * num_preds + i];
        if (s > max_score) {
          max_score = s;
          max_cls = c;
        }
      }
    } else {
      for (int c = 0; c < num_classes; ++c) {
        float s = data[i * num_ch + (4 + c)];
        if (s > max_score) {
          max_score = s;
          max_cls = c;
        }
      }
    }

    if (max_score < g_conf_thr) continue;

    // 좌표 복원: (cx,cy,w,h)는 모델 입력 스케일 기준
    float x = cx - w * 0.5f;
    float y = cy - h * 0.5f;

    // 입력 이미지(g_imgsz, g_imgsz) -> 원본 이미지 크기
    float sx = (float)g_last_img_w  / (float)g_imgsz;
    float sy = (float)g_last_img_h / (float)g_imgsz;

    int x1 = (int)(x * sx);
    int y1 = (int)(y * sy);
    int x2 = (int)((x + w) * sx);
    int y2 = (int)((y + h) * sy);

    x1 = (int)clampf((float)x1, 0.0f, (float)(g_last_img_w  - 1));
    x2 = (int)clampf((float)x2, 0.0f, (float)(g_last_img_w  - 1));
    y1 = (int)clampf((float)y1, 0.0f, (float)(g_last_img_h - 1));
    y2 = (int)clampf((float)y2, 0.0f, (float)(g_last_img_h - 1));

    cv::Rect box(cv::Point(x1, y1), cv::Point(x2, y2));
    if (box.width <= 0 || box.height <= 0) continue;

    Detection det;
    det.box      = box;
    det.class_id = max_cls;
    det.score    = max_score;
    detections.push_back(det);
  }

  // NMS 적용
  std::vector<Detection> nms_out;
  nmsDetections(detections, g_iou_thr, nms_out);

  // 클래스별 카운트 (프레임 단위 통계)
  std::map<std::string, int> det_counts;

  cv::Mat vis = frame.clone();

  for (const auto& det : nms_out) {
    int cls_id = det.class_id;
    float score = det.score;
    const cv::Rect& box = det.box;

    std::string cls_name;
    if (cls_id >= 0 && cls_id < (int)g_class_names.size()) {
      cls_name = g_class_names[cls_id];
    } else {
      std::ostringstream oss;
      oss << "id" << cls_id;
      cls_name = oss.str();
    }

    det_counts[cls_name]++;

    std::ostringstream label_ss;
    label_ss << cls_name << " " << std::fixed << std::setprecision(2) << score;
    std::string label = label_ss.str();

    // 바운딩 박스
    cv::rectangle(vis, box, cv::Scalar(0, 255, 0), 2);

    // 텍스트 박스
    int base = 0;
    cv::Size tsize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX,
                                     0.5, 1, &base);
    int tx = box.x;
    int ty = box.y - 5;
    if (ty < tsize.height + 5) {
      ty = box.y + tsize.height + 5;
    }

    cv::Rect bg_rect(tx, ty - tsize.height - base,
                     tsize.width, tsize.height + base);
    bg_rect &= cv::Rect(0, 0, vis.cols, vis.rows); // 이미지 안으로 클리핑

    cv::rectangle(vis, bg_rect, cv::Scalar(0, 255, 0), cv::FILLED);
    cv::putText(vis, label,
                cv::Point(tx, ty),
                cv::FONT_HERSHEY_SIMPLEX,
                0.5,
                cv::Scalar(0, 0, 0),
                1,
                cv::LINE_AA);
  }

  // 프레임 단위 검출 통계 (1초에 한 번만 출력)
  if (!det_counts.empty()) {
    std::ostringstream oss;
    bool first = true;
    for (const auto& kv : det_counts) {
      if (!first) oss << ", ";
      oss << kv.first << " x" << kv.second;
      first = false;
    }
    ROS_INFO_THROTTLE(1.0, "[yolo_cam_node] detections: %s",
                      oss.str().c_str());
  }

  // 화면 표시
  if (g_show_window) {
    cv::imshow(g_win_name, vis);
    cv::waitKey(1);
  }
}

// -------------------- main --------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "yolo_cam_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ROS_INFO("[yolo_cam_node] YOLO (ONNX) camera visualizer");

  // --- 파라미터 로드 ---
  pnh.param<std::string>("model_path",  g_model_path_onnx, g_model_path_onnx);
  pnh.param<std::string>("classes_file", g_classes_file,    g_classes_file);
  pnh.param<std::string>("image_topic", g_image_topic,     g_image_topic);

  pnh.param<bool>("show_window", g_show_window, g_show_window);
  pnh.param<float>("conf_thr",   g_conf_thr,   g_conf_thr);
  pnh.param<float>("iou_thr",    g_iou_thr,    g_iou_thr);
  pnh.param<int>("imgsz",        g_imgsz,      g_imgsz);

  ROS_INFO("[yolo_cam_node] model_path='%s'", g_model_path_onnx.c_str());
  ROS_INFO("[yolo_cam_node] classes_file='%s'", g_classes_file.c_str());
  ROS_INFO("[yolo_cam_node] subscribe image='%s'",
           ros::names::resolve(g_image_topic).c_str());
  ROS_INFO("[yolo_cam_node] conf_thr=%.2f, iou_thr=%.2f, imgsz=%d",
           g_conf_thr, g_iou_thr, g_imgsz);

  // --- 클래스 이름 로드 ---
  if (!loadClassNames(g_classes_file, g_class_names)) {
    ROS_WARN("[yolo_cam_node] failed to load class names. continue anyway.");
  } else {
    logModelClasses();
  }

  // --- DNN 모델 로드 ---
  try {
    ROS_INFO("[yolo_cam_node] loading ONNX model...");
    g_net = cv::dnn::readNetFromONNX(g_model_path_onnx);

    // 필요하면 GPU 사용 설정 (OpenCV가 CUDA 빌드된 경우)
    // g_net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    // g_net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA_FP16);

    g_net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    g_net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

    ROS_INFO("[yolo_cam_node] model loaded.");
  } catch (const cv::Exception& e) {
    ROS_ERROR("[yolo_cam_node] FAILED to load ONNX model: %s", e.what());
    return 1;
  }

  // --- OpenCV 윈도우 ---
  if (g_show_window) {
    cv::namedWindow(g_win_name, cv::WINDOW_NORMAL);
    cv::resizeWindow(g_win_name, 960, 540);
  }

  // --- 구독 ---
  ros::Subscriber img_sub = nh.subscribe(
      g_image_topic, 1, imageCB);

  ROS_INFO("[yolo_cam_node] spinning...");
  ros::spin();

  if (g_show_window) {
    cv::destroyAllWindows();
  }

  return 0;
}
