#!/usr/bin/env python3
#-*- coding:utf-8 -*- 
import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '../..'))  # drive_controller 상위 폴더
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2 
import json
from std_msgs.msg import String
from drive_perception.camera.preprocessing import CameraPreprocessor
from drive_perception.camera.feature_extraction import LaneFeatureExtractor
from drive_perception.camera.sliding_window import SlidingWindow
from utills import check_timer

class PerCamera:
    # 차선 정보를 추출해 ROS 토픽으로 발행하는 메인 카메라 인지 노드
    def __init__(self):
        # ROS 노드 초기화
        rospy.init_node("per_camera_node")
        print(f"PerCamera start")
        self.init_pubSub()
        self.init_msg()
        self.init_processing()
        self.init_timer()

    def init_pubSub(self):
        # 카메라 센서에서 JPEG 압축 영상을 구독하고 차선 메시지를 발행
        rospy.Subscriber("/usb_cam/image_raw", CompressedImage, self.CB_cam_raw, queue_size=1)
        self.pub = rospy.Publisher('/perception/camera', String, queue_size=1)
    def init_msg(self):
        # OpenCV 이미지 객체 및 브리지 초기화
        self.img = None
        self.bridge = CvBridge()
    def init_processing(self):
        # 차선 검출 파이프라인에서 사용하는 전처리, 특징 추출, 창 탐색 객체 준비
        self.CameraPreprocessor = CameraPreprocessor()
        self.LaneFeatureExtractor = LaneFeatureExtractor()
        self.SlidingWindow = SlidingWindow()
    def init_timer(self):
        # 처리 주기 모니터링용 타이머
        self.check_timer = check_timer.CheckTimer("per_camera_node")

    def CB_cam_raw(self,msg):
        # 센서 이미지 콜백: 압축 이미지를 OpenCV 형식으로 변환 후 처리 파이프라인 실행
        self.img = self.bridge.compressed_imgmsg_to_cv2(msg)
        self.processing()

    def pub_cam_info(self,dataset):
        # 추출한 차선 정보를 JSON 문자열로 직렬화하여 발행
        json_str = json.dumps(dataset)
        self.pub.publish(json_str)
        
    def view_cam(self):
        # 디버깅용 시각화: 검출된 차선과 정지선 정보를 한 화면에 표시
        combined_img = cv2.bitwise_or(self.white_lane_img,self.yellow_lane_img)
        if self.stop_line != []:
            cross_threshold = 35
            min_y, max_y = self.stop_line
            cross_diff = (max_y - min_y)
            if cross_threshold < cross_diff:
                cv2.rectangle(combined_img,[0,min_y],[self.img_x, max_y],[0,0,255],3)
        #cv2.imshow("white_lane_img",white_lane_img)
        cv2.imshow("lane_img",combined_img)
        # cv2.imshow("self.img",self.img)
        cv2.waitKey(1)
        # end = time()
        # print(f"time1 {end - start1} ")
        
    def processing(self):
        try:
            # 1) 원본 이미지 크기를 구하고 BEV(Bird's Eye View) 변환으로 차선 평면화
            self.img_y, self.img_x = self.img.shape[0:2]
            warped_img = self.CameraPreprocessor.BEV_img_warp(self.img,self.img_y,self.img_x)
            # 2) HSV 공간에서 노란색/흰색 차선 후보를 추출할 수 있도록 색 필터링 준비
            warped_img_hsv = cv2.cvtColor(warped_img,cv2.COLOR_BGR2HSV)
            yellow_filtered_img, white_filtered_img = self.CameraPreprocessor.detect_color_yAndw(warped_img,warped_img_hsv)
            yellow_bin_img,white_bin_img = self.CameraPreprocessor.img_binary_yAndw(yellow_filtered_img, white_filtered_img)

            # 3) 슬라이딩 윈도우 파라미터 설정 및 차선 종류/정지선 추정
            self.SlidingWindow.set_img_y(self.img_y)
            self.lane_mode = self.LaneFeatureExtractor.estimate_lane_mode(warped_img)
            self.stop_line, white_bin_img= self.LaneFeatureExtractor.estimate_stop_line(white_bin_img,self.img_y)
            # 4) 좌우 차선을 각각 슬라이딩 윈도우로 추적해 궤적 점을 수집
            self.yellow_lane_img, yellow_left_lane, yellow_right_lane = self.SlidingWindow.sliding_window_adaptive(yellow_bin_img)
            self.white_lane_img, white_left_lane, white_right_lane = self.SlidingWindow.sliding_window_adaptive(white_bin_img)
            
            # 5) 정지선, 좌우 차선, 차선 모드 정보를 패키징 후 발행
            dataset = [self.stop_line, yellow_left_lane, yellow_right_lane,white_left_lane, white_right_lane,self.lane_mode]           
            self.pub_cam_info(dataset)
            
            # 6) 결과 시각화 (필요 시)
            self.view_cam()
        except Exception as e:
            pass
     
        
if __name__ == '__main__':
    node = PerCamera()
    rospy.spin()
    
# ---------------------------------------------------------------------------
# 전체 알고리즘 개요
# 1) 노드 초기화: ROS 노드/퍼블리셔/구독자를 구성하고 카메라 전처리, 특징 추출, 슬라이딩 윈도우 객체를 준비한다.
# 2) 이미지 수신: 콤프레스트 이미지를 수신하면 CvBridge로 OpenCV 이미지로 변환한 뒤 처리 파이프라인을 호출한다.
# 3) BEV 변환: 원근을 제거한 탑뷰(BEV) 이미지를 생성하여 차선 검출이 안정적으로 이루어지도록 한다.
# 4) 색상 분리: HSV 색공간으로 변환한 뒤 노란/흰 차선을 각각 강조하는 필터를 적용한다.
# 5) 이진화: 필터링된 결과를 바이너리 이미지로 변환하여 명확한 차선 픽셀 후보를 만든다.
# 6) 특징 추출: 차선 종류(차로수/패턴)와 정지선 위치를 추정하고, 이미지 높이를 슬라이딩 윈도우에 전달한다.
# 7) 슬라이딩 윈도우 추적: 노란색·흰색 이진 영상에서 좌우 차선을 각각 추적하며 궤적 좌표와 시각화 이미지를 만든다.
# 8) 메시지 발행: 정지선, 좌우 차선 궤적, 차선 모드 정보를 JSON 배열로 묶어 ROS 토픽으로 발행한다.
# 9) 시각화: 디버깅을 위해 추적된 차선과 정지선을 하나의 화면으로 띄워 실시간 검증이 가능하도록 한다.
# 예외 상황 발생 시 try/except로 전체 노드가 중단되지 않도록 안전하게 무시한다.
