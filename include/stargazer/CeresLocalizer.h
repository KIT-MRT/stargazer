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


#include <string>
#include "CoordinateTransformations.h"
#include "Localizer.h"
#include "internal/CostFunction.h"

namespace stargazer {

/**
 * @brief Derived Localizer class, that uses numeric optimization with ceres library, to compute the current pose.
 * For this, the reprojection error is minimized, meaning the difference between the observed landmarks and their equivalent from the map projected into the camera image.
 * The parameters to be optimized are the cameras pose parameters.
 *
 */
class CeresLocalizer : public Localizer {

public:
    /**
     * @brief Constructor.
     *
     * @param cfgfile Path to map file with camera intrinsics and landmark poses.
     * @remark The config file has to be generated with ::writeConfig!
     * @remark The CeresLocalizer converts all landmark points into world coordinates after readin!
     */
    CeresLocalizer(std::string cfgfile);

    /**
     * @brief Main update method. Computes pose from landmark observations and stores it in Localizer::ego_pose
     *
     * @param img_landmarks Vector of all observed landmarks in image coordinates
     * @param dt Time since last update (unused in this implementation)
     */
    virtual void UpdatePose(std::vector<ImgLandmark>& img_landmarks, float dt) override;

    /**
     * @brief Returns the full summary of the ceres optimization process. It contains all relevant information for debugging.
     *
     * @return const ceres::Solver::Summary
     */
    const ceres::Solver::Summary& getSummary() const {
        return summary;
    }

private:
    ceres::Problem problem; /**< Ceres Prolem */
    ceres::Solver::Summary summary; /**< Summary of last optimization run */

    bool is_initialized; /**< Flag indicating whether the pose is initialized */

    /**
     * @brief Will remove the residuals from last run.
     *
     */
    void ClearResidualBlocks();
    /**
     * @brief   Will add a new residual block for every marker of every landmark given in img_landmarks
     *
     * @param img_landmarks Vector of observerved landmarks.
     */
    void AddResidualBlocks(std::vector<ImgLandmark> img_landmarks);
    /**
     * @brief Will set the camera parameters constant, so that they do not get changed during optimization.
     *
     */
    void SetCameraParamsConstant();
    /**
     * @brief This is the actual working method, that sets the ceres configuration and runs to solver.
     *
     */
    void Optimize();
};

} /// namespace stargazer
