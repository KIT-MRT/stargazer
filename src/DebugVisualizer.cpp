//
// Created by bandera on 28.03.16.
//

#include "DebugVisualizer.h"

using namespace stargazer;

DebugVisualizer::DebugVisualizer()
    : m_window_mode(CV_WINDOW_NORMAL), m_wait_time(0) {}

void DebugVisualizer::ShowImage(cv::Mat &img, std::string name) {
  cv::namedWindow(name, m_window_mode);
  cv::imshow(name, img);
  cv::waitKey(m_wait_time);
}

void DebugVisualizer::DrawPoints(cv::Mat &img, std::vector<cv::Point> points) {
  cv::Mat temp = img.clone();
  for (auto &point : points)
    circle(temp, point, 3, cv::Scalar(0, 0, 255), 2); // Red
  ShowImage(temp, "Points");
}

void DebugVisualizer::DrawPoints(cv::Mat &img,
                                 std::vector<std::vector<cv::Point>> points) {
  cv::Mat temp = img.clone();
  for (auto &group : points)
    for (auto &point : group)
      circle(temp, point, 3, cv::Scalar(0, 255, 0), 2); // Red
  ShowImage(temp, "PointGroups");
}