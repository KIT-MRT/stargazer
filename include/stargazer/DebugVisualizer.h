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
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace stargazer {

/**
 * @brief This class is used for debugging. It contains several methods for visualizing points and landmarks.
 *
 */
class DebugVisualizer {
public:
    /**
     * @brief Constructor
     *
     */
    DebugVisualizer();

    /**
     * @brief Destructor
     *
     */
    ~DebugVisualizer(){};

    /**
     * @brief Open a cv::namedWindow and display the img
     *
     * @param img Image to be shown.
     * @param name  (optional) Name of the window to be opened.
     */
    void ShowImage(cv::Mat& img, std::string name = "Image");

    // Setters
    /**
     * @brief Value passed to cv::waitKey. Defines how long the image should be displayed. If 0 is specified, the window
     * will stay open until a key is pressed.
     *
     * @param milliseconds
     */
    void SetWaitTime(int milliseconds) {
        m_wait_time = milliseconds;
    };

    /**
     * @brief Setter for the window mode to use.
     *
     * @param mode Can be any of CV_WINDOW_NORMAL (set by default), CV_WINDOW_AUTOSIZE, CV_WINDOW_OPENGL
     */
    void SetWindowMode(int mode) {
        m_window_mode = mode;
    }

    /**
     * @brief Draws and shows the vector of points given.
     *
     * @param img Input image is copied
     * @param points    Points to be drawn
     * @return cv::Mat A copy of the input image with the drawn points
     */
    cv::Mat ShowPoints(const cv::Mat& img, const std::vector<cv::Point> points);
    /**
     * @brief Draws and shows the vector of clusters given.
     *
     * @param img Input image is copied
     * @param points    Clusters to be drawn
     * @return cv::Mat A copy of the input image with the drawn points
     */
    cv::Mat ShowClusters(const cv::Mat& img, const std::vector<std::vector<cv::Point>> points);
    /**
     * @brief Draws the observed image landmarks into the input image
     *
     * @param img   Input image, gets modified!
     * @param landmarks Landmarks to be drawn
     */
    void DrawLandmarks(cv::Mat& img, const std::vector<ImgLandmark>& landmarks);

    /**
     * @brief Draws the landmarks of a map into the img based on the given camera pose
     *
     * @param img   Input image, gets modified!
     * @param landmarks Map of Landmarks in world coordinates!
     * @param camera_intrinsics Camera parameters
     * @param ego_pose  Camera pose
     */
    void DrawLandmarks(cv::Mat& img, const landmark_map_t& landmarks, const camera_params_t& camera_intrinsics,
                       const pose_t& ego_pose);

private:
    int m_wait_time;   /**< Time to wait when displaying image */
    int m_window_mode; /**< cvWindowProperty */
    cv::Mat baseImg;   /**< dummy image */
                       /**
                        * @brief Converts image to color image
                        *
                        * @param img
                        */
    void prepareImg(cv::Mat& img);
};

} // namespace stargazer
