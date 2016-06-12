//
// Created by bandera on 28.03.16.
//

#pragma once

#include "StargazerTypes.h"
#include "opencv/cv.h"
#include <vector>

namespace stargazer {

typedef std::vector<cv::Point> Cluster;

struct ImgLandmark {
  int nID;
  int nPointCount;
  int nErrors;
  std::vector<cv::Point> voCorners;
  std::vector<cv::Point> voIDPoints;
  cv::Point oPosition;
};

inline Landmark convert2Landmark(ImgLandmark &lm_in) {
  Landmark lm_out(lm_in.nID);
  lm_out.points.clear();

  for (auto &el : lm_in.voCorners) {
    Point pt = {(double)el.x, (double)el.y, 0};
    lm_out.points.push_back(pt);
  }
  for (auto &el : lm_in.voIDPoints) {
    Point pt = {(double)el.x, (double)el.y, 0};
    lm_out.points.push_back(pt);
  }

  return lm_out;
};

} // namespace stargazer
