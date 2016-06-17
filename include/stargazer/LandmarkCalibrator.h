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
    void SetPoseConstant(size_t id);
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
