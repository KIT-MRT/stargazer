//
// Created by bandera on 28.03.16.
//

#pragma once

#include "opencv/cv.h"
#include "opencv/highgui.h"

class DebugVisualizer {
public:
  DebugVisualizer();

  ~DebugVisualizer(){};

  void ShowImage(cv::Mat &img, std::string name = "Image");

  // Setters
  void SetWaitTime(int milliseconds) { m_wait_time = milliseconds; };

  void SetWindowMode(int mode) { m_window_mode = mode; }

  void DrawPoints(cv::Mat &img, std::vector<cv::Point> points);
  void DrawPoints(cv::Mat &img, std::vector<std::vector<cv::Point>> points);

private:
  int m_wait_time;
  int m_window_mode;
  cv::Mat baseImg;
};
