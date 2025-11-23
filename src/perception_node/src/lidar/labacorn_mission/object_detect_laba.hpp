#pragma once

// 최소 정의: Point2D와 전처리에서 참조하는 전역 파라미터 extern 선언
struct Point2D
{
  double x;
  double y;
};

// lidar_preprocessing_labacorn.cpp 에서 extern 으로 참조
extern double g_range_min;
extern double g_range_max;
extern double g_front_min_deg;
extern double g_front_max_deg;
