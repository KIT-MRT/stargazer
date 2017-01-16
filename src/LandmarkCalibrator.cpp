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

#include "LandmarkCalibrator.h"
#include "StargazerConfig.h"
#include "internal/CostFunction.h"

using namespace stargazer;

LandmarkCalibrator::LandmarkCalibrator(std::string cfgfile) {
    readConfig(cfgfile, camera_intrinsics_, landmarks_);
};

void LandmarkCalibrator::AddReprojectionResidualBlocks(
    const std::vector<pose_t>& observed_poses, const std::vector<std::vector<ImgLandmark>>& observed_landmarks) {

    if (observed_landmarks.size() != observed_poses.size())
        throw std::runtime_error("Got different amount of observations for landmarks and poses");

    // Copy poses, as we keep and modify them. Landmark observations get only copied into costfunctor.
    camera_poses_ = observed_poses;

    for (size_t i = 0; i < observed_landmarks.size(); i++) {
        for (size_t j = 0; j < observed_landmarks[i].size(); j++) {
            auto& observation = observed_landmarks[i][j];

            if (!landmarks_.count(observation.nID) > 0)
                throw std::runtime_error("Observed Landmark id not found in cfg!");

            Landmark& real_lm = landmarks_.at(observation.nID);

            if (observation.voCorners.size() + observation.voIDPoints.size() != real_lm.points.size())
                throw std::runtime_error("Observed Landmark has different ammount of points then real landmark!");

            // Add residual block, for every one of the seen points.
            for (size_t k = 0; k < real_lm.points.size(); k++) {
                const cv::Point* point_under_test;
                if (k < 3)
                    point_under_test = &observation.voCorners[k];
                else
                    point_under_test = &observation.voIDPoints[k - 3];

                // Test reprojection error
                double u, v;
                transformLandMarkToImage<double>(real_lm.points[k][(int)POINT::X], real_lm.points[k][(int)POINT::Y],
                                                 real_lm.pose.data(), camera_poses_[i].data(),
                                                 camera_intrinsics_.data(), &u, &v);

                ceres::CostFunction* cost_function = LandMarkToImageReprojectionFunctor::Create(
                    point_under_test->x, point_under_test->y, real_lm.points[k][(int)POINT::X],
                    real_lm.points[k][(int)POINT::Y]);

                problem.AddResidualBlock(cost_function,
                                         new ceres::CauchyLoss(50), // NULL /* squared loss */, //
                                                                    // Alternatively: new
                                                                    // ceres::ScaledLoss(NULL, w_v_des,
                                                                    // ceres::TAKE_OWNERSHIP),
                                         real_lm.pose.data(), camera_poses_[i].data(), camera_intrinsics_.data());
            }
        }
    }
}

void LandmarkCalibrator::Optimize() {
    std::cout << "Starting Optimization..." << std::endl;
    // Make Ceres automatically detect the bundle structure. Note that the
    // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
    // for standard bundle adjustment problems.
    ceres::Solver::Options options;
    //  options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.num_threads = 8;
    options.num_linear_solver_threads = 8;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 200;
    options.function_tolerance = 0.000001;
    options.gradient_tolerance = 0.000001;
    options.parameter_tolerance = 0.000001;
    options.min_relative_decrease = 0.000001;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << std::endl;
}

void LandmarkCalibrator::SetLandmarkConstant(landmark_map_t::key_type id) {
    if (problem.HasParameterBlock(landmarks_[id].pose.data()))
        problem.SetParameterBlockConstant(landmarks_[id].pose.data());
    else
        throw std::runtime_error("No parameter used of landmark that should get fixed");
}

void LandmarkCalibrator::SetPoseConstant(size_t id) {
    if (problem.HasParameterBlock(camera_poses_[id].data()))
        problem.SetParameterBlockConstant(camera_poses_[id].data());
    else
        throw std::runtime_error("No parameter used of landmark that should get fixed");
}

void LandmarkCalibrator::SetParametersConstant() {
    std::vector<double*> parameter_blocks;
    problem.GetParameterBlocks(&parameter_blocks);

    // Set Landmark rotation parameters constant
    std::vector<int> constant_landmark_params = {(int)POSE::Rx, (int)POSE::Ry, (int)POSE::Rz};
    ceres::SubsetParameterization* constant_landmark_params_mask =
        new ceres::SubsetParameterization((int)POSE::N_PARAMS, constant_landmark_params);
    for (auto& lm : landmarks_) {
        if (problem.HasParameterBlock(lm.second.pose.data()))
            //            problem.SetParameterization(lm.second.pose.data(), constant_landmark_params_mask);
            problem.SetParameterBlockConstant(lm.second.pose.data());
    }

    // Set some pose parameters constant
    std::vector<int> constant_vehicle_params = {(int)POSE::Z, (int)POSE::Rx, (int)POSE::Ry};
    ceres::SubsetParameterization* constant_vehicle_params_mask =
        new ceres::SubsetParameterization((int)POSE::N_PARAMS, constant_vehicle_params);
    for (auto& pose : camera_poses_) {
        if (problem.HasParameterBlock(pose.data()))
            //            problem.SetParameterization(pose.data(), constant_vehicle_params_mask);
            problem.SetParameterBlockConstant(pose.data());
    }

    //    // Set Camera Parameters Constant
    // Set Landmark rotation parameters constant
    //    if (problem.HasParameterBlock(camera_intrinsics_.data()))
    //        problem.SetParameterBlockConstant(camera_intrinsics_.data());
}

void LandmarkCalibrator::ClearProblem() {
    std::vector<ceres::ResidualBlockId> residual_blocks;
    problem.GetResidualBlocks(&residual_blocks);
    for (auto& block : residual_blocks) {
        problem.RemoveResidualBlock(block);
    }
    std::vector<double*> parameter_blocks;
    problem.GetParameterBlocks(&parameter_blocks);
    for (auto& block : parameter_blocks) {
        problem.RemoveParameterBlock(block);
    }
}
