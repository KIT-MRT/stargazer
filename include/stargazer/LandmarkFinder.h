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

#include "StargazerConfig.h"
#include "StargazerImgTypes.h"
#include "StargazerTypes.h"

#include <fstream>
#include <iostream>
#include <boost/lexical_cast.hpp>
#include "iostream"
#include "math.h"
#include "vector"
#include "opencv/cv.h"

namespace stargazer {

class LandmarkFinder {
public:
    /// constructors and destructors
    LandmarkFinder(std::string cfgfile);
    ~LandmarkFinder();

    /// methods
    int DetectLandmarks(const cv::Mat& img, std::vector<ImgLandmark>& detected_landmarks);

    cv::Mat_<cv::Vec3b> rawImage_;
    cv::Mat grayImage_;
    cv::Mat filteredImage_;
    std::vector<cv::Point> clusteredPixels_;
    std::vector<Cluster> clusteredPoints_;

    uint8_t threshold;
    float maxRadiusForPixelCluster;
    uint16_t minPixelForCluster;
    uint16_t maxPixelForCluster;
    float maxRadiusForCluster;
    uint16_t minPointsPerLandmark;
    uint16_t maxPointsPerLandmark;
    std::vector<uint16_t> valid_ids_;

private:
    void FilterImage(const cv::Mat& img_in, cv::Mat& img_out);
    std::vector<cv::Point> FindPoints(cv::Mat& img_in);
    void FindClusters(const std::vector<cv::Point>& points_in, std::vector<Cluster>& clusters,
                      const float radiusThreshold, const unsigned int minPointsThreshold,
                      const unsigned int maxPointsThreshold);
    bool FindCorners(std::vector<cv::Point>& point_list, std::vector<cv::Point>& corner_points);
    std::vector<ImgLandmark> FindLandmarks(const std::vector<Cluster>& clusteredPoints);
    int GetIDs(std::vector<ImgLandmark>& landmarks);

    bool CalculateIdForward(ImgLandmark& landmark, std::vector<uint16_t>& valid_ids);
    bool CalculateIdBackward(ImgLandmark& landmark, std::vector<uint16_t>& valid_ids);

    void parallel_vector_sort(std::vector<uint16_t>& ids, std::vector<cv::Point>& points);
    cv::Point getVector(const cv::Point& from, const cv::Point& to);
    float getDistance(const cv::Point& p1, const cv::Point& p2);
    float getNorm(const cv::Point& p);
    float getCrossProduct(const cv::Point& v1, const cv::Point& v2);
};

} // namespace stargazer
