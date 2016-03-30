//
// Created by bandera on 28.03.16.
//

#pragma once

#include "opencv/cv.h"
#include <vector>

typedef std::vector <cv::Point> Cluster;

struct ImgLandmark {
  int nID;
  int nPointCount;
  int nErrors;
  std::vector <cv::Point> voCorners;
  std::vector <cv::Point> voIDPoints;
  cv::Point oPosition;
};

