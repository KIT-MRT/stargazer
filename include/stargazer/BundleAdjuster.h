//
// Created by bandera on 19.03.16.
//

#pragma once

#include "ros/ros.h"
#include "internal/CostFunction.h"
#include "StargazerTypes.h"
#include "CoordinateTransformations.h"

#include <ceres/ceres.h>
#include <vector>
#include <map>


class BundleAdjuster {
 public:
  BundleAdjuster();

  void AddReprojectionResidualBlocks(std::vector<std::vector<Landmark>> measurements);

  ceres::Problem problem;
  camera_params_t camera_intrinsics;
  landmark_map_t landmarks;
  std::vector<pose_t> camera_poses;

  void Optimize();
  void SetParametersConstant();
  void AddCameraPoses(std::vector<std::array<double, 3>> measurements);
  void ClearProblem();
};


