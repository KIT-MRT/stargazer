//
// This file is part of the stargazer library.
//
// Copyright 2016 Claudio Bandera <claudio.bandera@kit.edu (Karlsruhe Institute of Technology)
//
// The stargazer library is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// The stargazer library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.

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
