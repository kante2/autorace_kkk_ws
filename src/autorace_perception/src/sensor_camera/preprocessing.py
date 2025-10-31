#!/usr/bin/env python3
#-*- coding:utf-8 -*- 
import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))  # drive_controller 상위 폴더
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

import cv2 
import numpy as np

class CameraPreprocessor:
    """카메라 영상에서 차선을 추출하기 위한 전처리 단계(BEV, 색 분리, 이진화)를 담당."""
    def __init__(self):
        self.init_color()
    def init_color(self):
        """노란/흰 차선 검출에 사용할 HSV 임계값 설정."""
        self.yellow_lower = np.array([15,128,0])
        self.yellow_upper = np.array([40,255,255])
        self.white_lower = np.array([0,0,192])
        self.white_upper = np.array([179,64,255])
    def BEV_img_warp(self,filtered_img,y,x):
        """일반 주행용 BEV(Bird's-Eye View) 투시 변환을 적용."""
        delta = 25
        # 원근 보정을 위해 차량 전방 사다리꼴 영역을 정의
        src_point1 = [-delta,400]
        src_point2 = [180,300] # 180
        src_point3 = [x-180,300] # 180
        src_point4 = [x+delta,400]
        src_points = np.float32([src_point1,src_point2,src_point3,src_point4])
        
        dst_point1 = [x//8,480]
        dst_point2 = [x//8,0]
        dst_point3 = [x//8*7,0]
        dst_point4 = [x//8*7,480]
        dst_points = np.float32([dst_point1,dst_point2,dst_point3,dst_point4])
        
        # 사다리꼴을 직사각형 영역으로 펴서 도로 차선이 수직하게 보이도록 맞춤
        matrix = cv2.getPerspectiveTransform(src_points,dst_points)
        warped_img = cv2.warpPerspective(filtered_img,matrix,[x,y])
        # cv2.imwrite("warped_img.png", warped_img) # 저장하기 위한 코드
        return warped_img        

    def detect_color_yAndw(self,img,img_hsv):
        """HSV 범위 기반으로 노란색과 흰색 영역을 각각 마스크링."""
        yellow_range = cv2.inRange(img_hsv,self.yellow_lower,self.yellow_upper)
        white_range = cv2.inRange(img_hsv,self.white_lower,self.white_upper)
        yellow_filtered_img = cv2.bitwise_and(img,img,mask=yellow_range)
        white_filtered_img = cv2.bitwise_and(img,img,mask=white_range)
        # cv2.imwrite("filtered_img.png", filtered_img) # 저장하기 위한 코드
        return yellow_filtered_img, white_filtered_img   

    def img_binary_yAndw(self,yellow_filtered_img, white_filtered_img):
        """색상 필터링 결과를 이진 이미지로 변환하여 차선 픽셀만 남긴다."""
        yellow_grayed_img = cv2.cvtColor(yellow_filtered_img,cv2.COLOR_BGR2GRAY)
        yellow_bin_img = np.zeros_like(yellow_grayed_img)
        yellow_bin_img[yellow_grayed_img>50] = 255
        white_grayed_img = cv2.cvtColor(white_filtered_img,cv2.COLOR_BGR2GRAY)
        white_bin_img = np.zeros_like(white_grayed_img)
        white_bin_img[white_grayed_img>50] = 255
        # cv2.imwrite("bin_img.png", bin_img) # 저장하기 위한 코드 
        return yellow_bin_img,white_bin_img

    def BEV_img_warp_rotary(self,filtered_img,y,x):
        """로터리 구간 전용 BEV 투시 변환을 적용."""
        delta = 25
        src_point1 = [-delta,480]
        # src_point2 = [285,260] # 180
        # src_point3 = [x-285,260] # 180
        src_point2 = [291,260] # 180
        src_point3 = [x-291,260] # 180
        src_point4 = [x+delta,480]
        src_points = np.float32([src_point1,src_point2,src_point3,src_point4])
        
        dst_point1 = [x//8,480]
        dst_point2 = [x//8,0]
        dst_point3 = [x//8*7,0]
        dst_point4 = [x//8*7,480]
        dst_points = np.float32([dst_point1,dst_point2,dst_point3,dst_point4])
        
        matrix = cv2.getPerspectiveTransform(src_points,dst_points)
        warped_img = cv2.warpPerspective(filtered_img,matrix,[x,y])
        # cv2.imwrite("warped_img.png", warped_img) # 저장하기 위한 코드
        return warped_img     

# ---------------------------------------------------------------------------
# 전체 알고리즘과 역할
# - 카메라 영상을 BEV로 변환해 차선이 평행하게 보이도록 정규화하고, 일반 주행/로터리 환경별 투시 행렬을 제공한다.
# - 미리 정의한 HSV 임계값으로 노란/흰 차선 영역을 분리하고, 이진화하여 후행 알고리즘이 안정적인 차선 픽셀을 활용하게 한다.
