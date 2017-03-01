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

#include "CeresLocalizer.h"
#include <ceres/ceres.h>
#include <opencv/highgui.h>

#include "internal/CostFunction.h"

using namespace stargazer;

CeresLocalizer::CeresLocalizer(const std::string& cfgfile) : CeresLocalizer(cfgfile, false) {}

CeresLocalizer::CeresLocalizer(const std::string& cfgfile, bool estimate_2d_pose)
  : Localizer(cfgfile), estimate_2d_pose(estimate_2d_pose) {

    // Convert landmark points to worldcoordinates once.
    for (auto& el : landmarks) {
        for (auto& pt : el.second.points) {
            double x, y, z;
            transformLandMarkToWorld(pt[(int)POINT::X], pt[(int)POINT::Y], el.second.pose.data(), &x, &y, &z);
            pt = {x, y, z};
        }
    }

    is_initialized = false;
}

void CeresLocalizer::UpdatePose(std::vector<ImgLandmark>& img_landmarks, float dt) {
    if (img_landmarks.empty()) {
        std::cout << "Localizer received empty landmarks vector" << std::endl;
        return;
    }

    if (!is_initialized) {
        for (auto& el : img_landmarks) {
            ego_pose[(int)POSE::X] += landmarks[el.nID].pose[(int)POSE::X];
            ego_pose[(int)POSE::Y] += landmarks[el.nID].pose[(int)POSE::Y];
        }
        ego_pose[(int)POSE::X] /= img_landmarks.size();
        ego_pose[(int)POSE::Y] /= img_landmarks.size();
        //is_initialized = true;
    }

    // Delete old data
    ClearResidualBlocks();

    // Add new data
    AddResidualBlocks(img_landmarks);

    // Optimize
    Optimize();
}

void CeresLocalizer::ClearResidualBlocks() {
    std::vector<ceres::ResidualBlockId> residual_blocks;
    problem.GetResidualBlocks(&residual_blocks);
    for (auto& block : residual_blocks) {
        problem.RemoveResidualBlock(block);
    }
}

void CeresLocalizer::AddResidualBlocks(std::vector<ImgLandmark> img_landmarks) {
    for (auto& img_lm : img_landmarks) {

        if (img_lm.voIDPoints.size() + img_lm.voCorners.size() != landmarks[img_lm.nID].points.size()) {
            std::cerr << "point count does not match! " << img_lm.voIDPoints.size() + img_lm.voCorners.size()
                      << "(observed) vs. " << landmarks[img_lm.nID].points.size() << "(map)\t ID: " << img_lm.nID
                      << std::endl;
            return;
        };

        // Add residual block, for every one of the seen points.
        for (size_t k = 0; k < landmarks[img_lm.nID].points.size(); k++) {
            ceres::CostFunction* cost_function;
            if (k < 3) {
                cost_function = WorldToImageReprojectionFunctor::Create(
                    img_lm.voCorners[k].x, img_lm.voCorners[k].y, landmarks[img_lm.nID].points[k][(int)POINT::X],
                    landmarks[img_lm.nID].points[k][(int)POINT::Y], landmarks[img_lm.nID].points[k][(int)POINT::Z]);
            } else {
                cost_function = WorldToImageReprojectionFunctor::Create(
                    img_lm.voIDPoints[k - 3].x, img_lm.voIDPoints[k - 3].y,
                    landmarks[img_lm.nID].points[k][(int)POINT::X], landmarks[img_lm.nID].points[k][(int)POINT::Y],
                    landmarks[img_lm.nID].points[k][(int)POINT::Z]);
            }
            // CauchyLoss(9): a pixel-error of 3 is still considered as inlayer
            problem.AddResidualBlock(cost_function,
                                     new ceres::CauchyLoss(9),
                                     ego_pose.data(), camera_intrinsics.data());
        }
    }
    if (!is_initialized) {
        std::vector<int> constant_parameters = {{(int)POSE::Z}};
        if (estimate_2d_pose) {
          constant_parameters.push_back((int)POSE::Rx);
          constant_parameters.push_back((int)POSE::Ry);
        }
        problem.SetParameterization(ego_pose.data(), new ceres::SubsetParameterization((int)POSE::N_PARAMS, constant_parameters));
        ego_pose[(int)POSE::Z] = 0.0;
        is_initialized = true;
    }
    SetCameraParamsConstant();
}

void CeresLocalizer::SetCameraParamsConstant() {
    // Set Camera Parameters Constant
    if (problem.HasParameterBlock(camera_intrinsics.data()))
        problem.SetParameterBlockConstant(camera_intrinsics.data());
}

void CeresLocalizer::Optimize() {
    ceres::Solver::Options options;
    // set optimization settings
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = 20;
    //    options.function_tolerance = 0.0000000000000001;
    //    options.gradient_tolerance = 0.0000000000000001;
    //    options.parameter_tolerance = 0.0000000000000001;
    //    options.min_relative_decrease = 0.0000000000000001;

    ceres::Solve(options, &problem, &summary);
}
