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
#include "opencv/highgui.h"

namespace stargazer {

class LandmarkFinder {
public:
    /// constructors and destructors
    LandmarkFinder(std::string cfgfile);
    ~LandmarkFinder();

    /// methods
    int FindLandmarks(const cv::Mat& i_oImage, std::vector<ImgLandmark>& o_vLandmarks);

    cv::Mat_<cv::Vec3b> m_oImage;
    cv::Mat m_oGrayImage;

    void setDebug_mode(bool value);

    char m_cThreshold;
    float m_fMaxRadiusForPixelCluster;
    unsigned int m_nMinPixelForCluster;
    unsigned int m_nMaxPixelForCluster;
    float m_fMaxRadiusForCluster;
    unsigned int m_nMinPointsPerLandmark;
    unsigned int m_nMaxPointsPerLandmark;
    std::vector<int> m_vnIDs;
    bool debug_mode;

private:
    std::vector<ImgLandmark> FindCorners(std::vector<Cluster>& Clusters);
    std::vector<cv::Point> FindPoints(cv::Mat& i_oGrayImage);
    void FindClusters(std::vector<cv::Point>& i_voPoints, std::vector<Cluster>& o_voCluster,
                      const float i_fRadiusThreshold, const unsigned int i_nMinPointsThreshold,
                      const unsigned int i_nMaxPointsThreshold);
    int GetIDs(std::vector<ImgLandmark>& io_voLandmarks);
    void Check(cv::Mat& Filtered, int XPos, int YPos, int Threshold, int& Pixelcount, int& SummedX, int& SummedY);
    void vec_sort(std::vector<int>& ids, std::vector<cv::Point>& points);
};

} // namespace stargazer
