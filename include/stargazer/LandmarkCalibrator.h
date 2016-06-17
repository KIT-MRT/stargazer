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

/**
 * @brief This is the class responsible for map generation. It computes a full bundle adjustment SLAM optimizing all
 * observations of a full calibration sequenz in one optimization problem.
 *
 */
class LandmarkCalibrator {
public:
    /**
     * @brief Constructor.
     *
     * @param cfgfile Path to map file with camera intrinsics and landmark poses.
     * @remark The config file has to be generated with ::writeConfig!
     */
    LandmarkCalibrator(std::string cfgfile);

    /**
     * @brief Adds all residual blocks to the problem. For every marker of every seen landmark at every pose a residual
     * block is added to the problem.
     *
     * @param observed_poses    Initial guess of the cameras poses
     * @param observed_landmarks    Vector of all observed Image landmarks
     */
    void AddReprojectionResidualBlocks(const std::vector<pose_t>& observed_poses,
                                       const std::vector<std::vector<ImgLandmark>>& observed_landmarks);

    /**
     * @brief Main worker function. It calls the solver of the underlying ceres library.
     *
     */
    void Optimize();
    /**
     * @brief Sets some of the parameters constant. (Unused, debugging only)
     *
     */
    void SetParametersConstant();
    /**
     * @brief Sets an individual landmarks pose constant. This is usefull, for fixing the maps coordinate system to one
     * landmark.
     *
     * @param id    Id of the landmark to hold constant.
     */
    void SetLandmarkConstant(landmark_map_t::key_type id);
    /**
     * @brief Sets an individual camera pose constant. This is usefull, for fixing the maps coordinate system to one
     * pose (normally the first).
     *
     * @param id
     */
    void SetPoseConstant(size_t id);
    /**
     * @brief Removes all parameter and residual blocks from the problem. So that one can start from scratch.
     *
     */
    void ClearProblem();

    /**
     * @brief Getter for the cameras' optimized intrinsic parameters
     *
     * @return const camera_params_t
     */
    const camera_params_t& getIntrinsics() const {
        return camera_intrinsics_;
    }

    /**
     * @brief Getter for map of optimized landmarks
     *
     * @return const landmark_map_t
     */
    const landmark_map_t& getLandmarks() const {
        return landmarks_;
    }

    /**
     * @brief Getter for the optimized camera poses.
     *
     * @return const std::vector<pose_t>
     */
    const std::vector<pose_t>& getPoses() const {
        return camera_poses_;
    }

private:
    ceres::Problem problem;             /**< Ceres problem */
    camera_params_t camera_intrinsics_; /**< Camera parameters */
    landmark_map_t landmarks_;          /**< Map of landmarks. Points have to be defined in landmark coordinates!*/
    std::vector<pose_t> camera_poses_;  /**< Camera poses */
};

} // namespace stargazer
