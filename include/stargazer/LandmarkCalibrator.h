//
// Created by bandera on 19.03.16.
//

#pragma once

#include "CoordinateTransformations.h"
#include "StargazerImgTypes.h"
#include "internal/CostFunction.h"
#include "ros/ros.h"

#include <map>
#include <vector>
#include <ceres/ceres.h>
namespace stargazer {

class LandmarkCalibrator {
public:
    LandmarkCalibrator(std::string cfgfile);

    void AddReprojectionResidualBlocks(const std::vector<pose_t>& observed_poses,
                                       const std::vector<std::vector<ImgLandmark>>& observed_landmarks);

    void Optimize();
    void SetParametersConstant();
    void SetLandmarkConstant(landmark_map_t::key_type id);
    void ClearProblem();

    const camera_params_t& getIntrinsics() const {
        return camera_intrinsics_;
    }

    const landmark_map_t& getLandmarks() const {
        return landmarks_;
    }

    const std::vector<pose_t>& getPoses() const {
        return camera_poses_;
    }

private:
    ceres::Problem problem;
    camera_params_t camera_intrinsics_;
    landmark_map_t landmarks_; // Points need to be in landmark coordinates!
    std::vector<pose_t> camera_poses_;
};

} // namespace stargazer
