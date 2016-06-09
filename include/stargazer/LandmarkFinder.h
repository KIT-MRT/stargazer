#ifndef LANDMARKFINDER_H
#define LANDMARKFINDER_H

#include "StargazerConfig.h"
#include "StargazerImgTypes.h"
#include "StargazerTypes.h"

#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "math.h"
#include "iostream"
#include "vector"
#include <iostream>
#include <fstream>
#include <boost/lexical_cast.hpp>

class LandmarkFinder {
 public:
  /// constructors and destructors
  LandmarkFinder(std::string cfgfile);
  ~LandmarkFinder();

  /// accessors
  void SetImage(cv::Mat &i_oImage);

  /// methods
  int FindLandmarks(std::vector<ImgLandmark> &o_vLandmarks);

  cv::Mat_<cv::Vec3b> m_oImage;
  cv::Mat m_oGrayImage;

  void setDebug_mode(bool value);

 private:
  char m_cThreshold;
  float m_fMaxRadiusForPixelCluster;
  unsigned int m_nMinPixelForCluster;
  unsigned int m_nMaxPixelForCluster;
  float m_fMaxRadiusForCluster;
  unsigned int m_nMinPointsPerLandmark;
  unsigned int m_nMaxPointsPerLandmark;
  float m_fMaxCosForRightAngle;
  std::vector<int> m_vnIDs;
  std::vector<cv::Point> m_vStuckPixels;
  bool debug_mode;

  std::vector<ImgLandmark> FindCorners(std::vector<Cluster> &Clusters);
  std::vector<cv::Point> FindPoints(cv::Mat &i_oGrayImage);
  void FindClusters(std::vector<cv::Point> &i_voPoints,
                    std::vector<Cluster> &o_voCluster,
                    const float i_fRadiusThreshold,
                    const unsigned int i_nMinPointsThreshold,
                    const unsigned int i_nMaxPointsThreshold);
  int GetIDs(std::vector<ImgLandmark> &io_voLandmarks);
  void Check(cv::Mat &Filtered, int XPos, int YPos, int Threshold, int &Pixelcount, int &SummedX, int &SummedY);
  void vec_sort(std::vector<int> &ids, std::vector<cv::Point> &points);
};

#endif /// ifndef LANDMARKFINDER_H
