//
// Created by bandera on 28.03.16.
//

#pragma once

#include "StargazerImgTypes.h"
#include "StargazerTypes.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

namespace stargazer {

class DebugVisualizer {
public:
    DebugVisualizer();

    ~DebugVisualizer(){};

    void ShowImage(cv::Mat& img, std::string name = "Image");

    // Setters
    void SetWaitTime(int milliseconds) {
        m_wait_time = milliseconds;
    };

    void SetWindowMode(int mode) {
        m_window_mode = mode;
    }

    cv::Mat ShowPoints(const cv::Mat& img, const std::vector<cv::Point> points);
    cv::Mat ShowClusters(const cv::Mat& img, const std::vector<std::vector<cv::Point>> points);
    void DrawLandmarks(cv::Mat& img, const std::vector<ImgLandmark>& landmarks);
    void DrawLandmarks(cv::Mat& img, const landmark_map_t& landmarks, const camera_params_t& camera_intrinsics,
                       const pose_t& ego_pose);

private:
    int m_wait_time;
    int m_window_mode;
    cv::Mat baseImg;
    void prepareImg(cv::Mat& img);
};

} // namespace stargazer
