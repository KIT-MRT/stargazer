//
// Created by bandera on 19.03.16.
//

#pragma once

#include "ros/ros.h"
#include "internal/CostFunction.h"
#include "StargazerTypes.h"

#include "util_print/prettyprint.h"

#include <ceres/ceres.h>
#include <vector>
#include <map>


class BundleAdjuster {
 public:
  BundleAdjuster();

  static constexpr double w_dist = 1000;
  static constexpr double w_orient = 1000;

  void AddReprojectionResidualBlocks(std::vector<std::vector<Landmark>> measurements);
  void AddDistanceResidualBlocks();
  void AddOrientationResidualBlocks();

  landmark_map_t landmark_poses;
  std::vector<pose_t> camera_poses;
  camera_params_t camera_intrinsics;
  ceres::Problem problem;

  void showSetup() {
    std::cout << std::endl;
    std::cout << "Bundle Adjuster Setup" << std::endl;
    std::cout << "=====================" << std::endl;
    std::cout << std::endl;
    std::cout << "Intrinsics (" << camera_intrinsics.data() << "): " << camera_intrinsics << std::endl;
    std::cout << std::endl;
    std::cout << "Landmarks:" << std::endl;
    for (auto &lm :landmark_poses)
      std::cout << "ID: " << lm.first << " Pose (" << lm.second.data() << "): " << lm.second << std::endl;
    std::cout << std::endl;
    std::cout << "Poses:" << std::endl;
    for (auto &lm :camera_poses)
      std::cout << "Pose (" << lm.data() << "): " << lm << std::endl;
  }

  void Optimize();
  void SetParametersConstant();
  void AddLandmarkPose(int id);
  void AddCameraPoses(std::vector<std::array<double, 3>> measurements);
  void ClearProblem();
};


