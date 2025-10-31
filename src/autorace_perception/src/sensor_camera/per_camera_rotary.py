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
from utills import check_timer

class PerCameraRotary:
    # 회전 구간(로터리) 탈출 여부를 판단하는 카메라 인지 노드
    def __init__(self):
        # ROS 노드 초기화
        rospy.init_node("per_camera_rotary_node")
        print(f"PerCameraRotary start")
        self.init_pubSub()
        self.init_msg()
        self.init_processing()
        self.init_timer()

    def init_pubSub(self):
        # 공통 카메라 토픽에서 이미지를 수신하고 로터리 전용 결과를 발행
        rospy.Subscriber("/usb_cam/image_raw", CompressedImage, self.CB_cam_raw, queue_size=1)
        self.pub = rospy.Publisher('/perception/camera/rotary', String, queue_size=1)
    def init_msg(self):
        # 카메라 이미지 버퍼 및 브리지 초기화
        self.img = None
        self.bridge = CvBridge()
    def init_processing(self):
        # 로터리 판단에 필요한 전처리 및 특징 추출기 생성
        self.CameraPreprocessor = CameraPreprocessor()
        self.LaneFeatureExtractor = LaneFeatureExtractor()
    def init_timer(self):
        # 처리 시간 모니터링 (공용 타이머 사용)
        self.check_timer = check_timer.CheckTimer("per_camera_node")

    def CB_cam_raw(self,msg):
        # 압축 이미지를 OpenCV 형식으로 변환 후 로터리 판단 파이프라인 실행
        self.img = self.bridge.compressed_imgmsg_to_cv2(msg)
        self.processing()

    def pub_cam_info(self,dataset):
        # 탈출 여부 결과를 JSON 문자열로 포장하여 발행
        json_str = json.dumps(dataset)
        self.pub.publish(json_str)
        
    def view_cam(self):
        # 디버깅용 영상: 색상별 바이너리 결과를 확인
        combined_img = cv2.bitwise_or(self.white_bin_img,self.yellow_bin_img)

        cv2.imshow("self.yellow_bin_img",self.yellow_bin_img)
        # cv2.imshow("lane_img",combined_img)
        # cv2.imshow("self.img",self.img)
        cv2.waitKey(1)
        # end = time()
        # print(f"time1 {end - start1} ")
        
    def processing(self):
        try:
            # print(f"hi")
            # 1) 입력 이미지를 BEV로 변환하여 로터리 구간 하단부를 직관적으로 관찰
            self.img_y, self.img_x = self.img.shape[0:2]
            warped_img = self.CameraPreprocessor.BEV_img_warp_rotary(self.img,self.img_y,self.img_x)
            # 2) HSV 변환 후 노란/흰 차선 후보를 추출해 로터리 패턴에 사용
            warped_img_hsv = cv2.cvtColor(warped_img,cv2.COLOR_BGR2HSV)
            yellow_filtered_img, white_filtered_img = self.CameraPreprocessor.detect_color_yAndw(warped_img,warped_img_hsv)
            self.yellow_bin_img, self.white_bin_img = self.CameraPreprocessor.img_binary_yAndw(yellow_filtered_img, white_filtered_img)

            # 3) 노란색 차선의 형태를 분석해 로터리 탈출 여부를 판단
            is_out_rotary = self.LaneFeatureExtractor.detect_out_rotary(self.yellow_bin_img)
            
            # 4) 판단 결과를 패키징 후 발행/시각화
            dataset = [is_out_rotary]           
            self.pub_cam_info(dataset)
            
            self.view_cam()
        except Exception as e:
            pass
     
        
if __name__ == '__main__':
    node = PerCameraRotary()
    rospy.spin()
    
# ---------------------------------------------------------------------------
# 전체 알고리즘 개요
# 1) 노드 초기화: 회전 교차로 인지를 위한 ROS 노드를 띄우고 카메라 토픽 구독자와 결과 퍼블리셔를 구성한다.
# 2) 이미지 수신: 압축 이미지를 콜백에서 받아 CvBridge로 OpenCV 이미지로 변환한다.
# 3) BEV 변환: 로터리 구간 전용 투시 변환으로 차량 전방 하단부를 탑뷰 형태로 정규화한다.
# 4) 색상 분리: HSV 색공간에서 노란/흰 차선 후보를 필터링하여 도로 표시선을 강조한다.
# 5) 이진화: 필터링된 이미지를 바이너리 형태로 변환해 명확한 차선 픽셀 마스크를 생성한다.
# 6) 로터리 판단: 노란색 차선 마스크 패턴을 분석하여 회전 구간을 빠져나왔는지 여부를 판정한다.
# 7) 메시지 발행: 판단 결과를 JSON 배열로 직렬화해 `/perception/camera/rotary` 토픽으로 전송한다.
# 8) 시각화: 디버깅 시 노란/흰 색상 마스크를 별도로 표시하여 인식 품질을 확인한다.
