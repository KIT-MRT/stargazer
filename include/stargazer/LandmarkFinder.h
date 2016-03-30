//
// Created by bandera on 28.03.16.
//

#pragma once

#include "StargazerTypes.h"
#include "StargazerImgTypes.h"
#include "StargazerConfig.h"
#include "DebugVisualizer.h"

#include "opencv/cv.h"
#include <vector>
#include <string>


class LandmarkFinder {
 public:
  /// constructors and destructors
  LandmarkFinder(std::string cfgfile);

  ~LandmarkFinder() { };


  /// methods
  std::vector<Landmark> FindLandmarks(cv::Mat &i_oImage);

  /// accessors
  void SetImage(cv::Mat &i_oImage);
  void SetMaxRadiusForPixelCluster(float i_fMaxRadiusForPixelCluster);
  void SetMinPixelForCluster(unsigned int i_nMinPixelForCluster);
  void SetMaxPixelForCluster(unsigned int i_nMaxPixelForCluster);
  void SetMaxRadiusForCluster(float i_fMaxRadiusForCluster);
  void SetMinPointsPerLandmark(unsigned int i_nMinPointsPerLandmark);
  void SetMaxPointsPerLandmark(unsigned int i_nMaxPointsPerLandmark);

  cv::Mat_<cv::Vec3b> m_oImage;
  cv::Mat m_oOutputImage;
  cv::Mat m_oGrayImage;
  bool perform_undistort;

  void setDebug_mode(bool value) { debug_mode = value; };

  DebugVisualizer debugVisualizer;
 private:

  char m_cThreshold;
  float m_fMaxRadiusForPixelCluster;
  unsigned int m_nMinPixelForCluster;
  unsigned int m_nMaxPixelForCluster;
  float m_fMaxRadiusForCluster;
  unsigned int m_nMinPointsPerLandmark;
  unsigned int m_nMaxPointsPerLandmark;
  float m_fMaxCosForRightAngle;
  camera_params_t m_camera_intrinsics;
  landmark_map_t m_landmarks;
  std::vector<int> m_vnIDs;
  std::vector<cv::Point> m_vStuckPixels;
  bool debug_mode;

  std::vector<cv::Point> FindPoints(cv::Mat &i_oGrayImage);

  std::vector<Cluster> FindClusters(std::vector<cv::Point> &i_voPoints,
                                    const float i_fRadiusThreshold,
                                    const unsigned int i_nMinPointsThreshold,
                                    const unsigned int i_nMaxPointsThreshold);

  std::vector<Landmark> IdentifyLandmarks(std::vector<Cluster> &Clusters);

  std::vector<Landmark> FindCorners(std::vector<Cluster> &Clusters);

  int ThisLandmarkSucks(Landmark &io_oLandmark);
  int GetIDs(std::vector<Landmark> &io_voLandmarks);
  void Check(cv::Mat &Filtered, int XPos, int YPos, int Threshold, int &Pixelcount, int &SummedX, int &SummedY);
  void vec_sort(std::vector<int> &ids, std::vector<cv::Point> &points);
};

/// inlined accessors
inline void LandmarkFinder::SetMaxRadiusForPixelCluster(float i_fMaxRadiusForPixelCluster) {
  m_fMaxRadiusForPixelCluster = i_fMaxRadiusForPixelCluster;
}

inline void LandmarkFinder::SetMinPixelForCluster(unsigned int i_nMinPixelForCluster) {
  m_nMinPixelForCluster = i_nMinPixelForCluster;
}

inline void LandmarkFinder::SetMaxPixelForCluster(unsigned int i_nMaxPixelForCluster) {
  m_nMaxPixelForCluster = i_nMaxPixelForCluster;
}

inline void LandmarkFinder::SetMaxRadiusForCluster(float i_fMaxRadiusForCluster) {
  m_fMaxRadiusForCluster = i_fMaxRadiusForCluster;
}

inline void LandmarkFinder::SetMinPointsPerLandmark(unsigned int i_nMinPointsPerLandmark) {
  m_nMinPointsPerLandmark = i_nMinPointsPerLandmark;
}

inline void LandmarkFinder::SetMaxPointsPerLandmark(unsigned int i_nMaxPointsPerLandmark) {
  m_nMaxPointsPerLandmark = i_nMaxPointsPerLandmark;
}