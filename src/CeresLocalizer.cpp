//
// Created by bandera on 28.03.16.
//

#include "CeresLocalizer.h"
#include <ceres/ceres.h>
#include <opencv/highgui.h>

using namespace stargazer;

CeresLocalizer::CeresLocalizer(std::string cfgfile) {
    readConfig(cfgfile, camera_intrinsics, landmarks);

    // Convert landmark points to worldcoordinates once.
    for (auto& el : landmarks) {
        for (auto& pt : el.second.points) {
            double x, y, z;
            transformLM2World(&pt[(int)POINT::X], &pt[(int)POINT::Y], el.second.pose.data(), &x, &y, &z);
            pt = {x, y, z};
        }
    }

    is_initialized = false;
}

void CeresLocalizer::UpdatePose(std::vector<Landmark>& img_landmarks) {
    if (img_landmarks.empty()) {
        std::cout << "Localizer received empty landmarks vector" << std::endl;
        return;
    }
    for (auto& img_lm : img_landmarks) {
        img_lm.pose = landmarks[img_lm.id].pose;
    }

    if (!is_initialized) {
        for (auto& el : img_landmarks) {
            ego_pose[(int)POSE::X] += el.pose[(int)POSE::X];
            ego_pose[(int)POSE::Y] += el.pose[(int)POSE::Y];
        }
        ego_pose[(int)POSE::X] /= img_landmarks.size();
        ego_pose[(int)POSE::Y] /= img_landmarks.size();
        is_initialized = true;
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

void CeresLocalizer::AddResidualBlocks(std::vector<Landmark> img_landmarks) {
    for (auto& lm_obs : img_landmarks) {

        if (lm_obs.points.size() != landmarks[lm_obs.id].points.size()) {
            std::cerr << "point count does not match! " << lm_obs.points.size() << "(observed) vs. "
                      << landmarks[lm_obs.id].points.size() << "(map)" << std::endl;
            return;
        };

        // Add residual block, for every one of the seen points.
        for (size_t k = 0; k < lm_obs.points.size(); k++) {

            ceres::CostFunction* cost_function = World2ImgReprojectionFunctor::Create(
                lm_obs.points[k][(int)POINT::X], lm_obs.points[k][(int)POINT::Y],
                landmarks[lm_obs.id].points[k][(int)POINT::X], landmarks[lm_obs.id].points[k][(int)POINT::Y],
                landmarks[lm_obs.id].points[k][(int)POINT::Z]);

            problem.AddResidualBlock(cost_function,
                                     new ceres::CauchyLoss(50), // NULL /* squared loss */, //
                                                                // Alternatively: new
                                                                // ceres::ScaledLoss(NULL, w_v_des,
                                                                // ceres::TAKE_OWNERSHIP),
                                     ego_pose.data(), camera_intrinsics.data());
        }
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
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    //  std::cout << summary.FullReport() << std::endl;
}