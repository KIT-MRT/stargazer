//
// Created by bandera on 28.03.16.
//

#include <ceres/ceres.h>
#include "Localizer.h"


Localizer::Localizer(std::string cfgfile) {
  readConfig(cfgfile, camera_intrinsics, landmarks);

  // Convert landmark points to worldcoordinates once.
  for (auto &el:landmarks) {
    for (auto &pt : el.second.points) {
      double x, y, z;
      transformLM2World(&pt[(int) POINT::X],
                        &pt[(int) POINT::Y],
                        el.second.pose.data(),
                        &x,
                        &y,
                        &z);
      pt = {x, y, z};
    }
  }
}

void Localizer::UpdatePose(std::vector<Landmark> img_landmarks) {
  // Delete old data
  ClearResidualBlocks();

  // Add new data
  AddResidualBlocks(img_landmarks);

  // Optimize
  Optimize();
}

void Localizer::ClearResidualBlocks() {
  std::vector<ceres::ResidualBlockId> residual_blocks;
  problem.GetResidualBlocks(&residual_blocks);
  for (auto &block:residual_blocks) {
    problem.RemoveResidualBlock(block);
  }
}

void Localizer::AddResidualBlocks(std::vector<Landmark> img_landmarks) {
  for (auto &lm_obs : img_landmarks) {

    assert(lm_obs.points.size() == landmarks[lm_obs.id].points.size());

    // Add residual block, for every one of the seen points.
    for (int k = 0; k < lm_obs.points.size(); k++) {

      ceres::CostFunction *cost_function = World2ImgReprojectionFunctor::Create(
          lm_obs.points[k][(int) POINT::X],
          lm_obs.points[k][(int) POINT::Y],
          landmarks[lm_obs.id].points[k][(int) POINT::X],
          landmarks[lm_obs.id].points[k][(int) POINT::Y],
          landmarks[lm_obs.id].points[k][(int) POINT::Z]);

      problem.AddResidualBlock(cost_function,
                               new ceres::CauchyLoss(50), // NULL /* squared loss */, // Alternatively: new ceres::ScaledLoss(NULL, w_v_des, ceres::TAKE_OWNERSHIP),
                               ego_pose.data(),
                               camera_intrinsics.data());
    }
  }

  SetCameraParamsConstant();

}


void Localizer::SetCameraParamsConstant() {
  // Set Camera Parameters Constant
  if (problem.HasParameterBlock(camera_intrinsics.data()))
    problem.SetParameterBlockConstant(camera_intrinsics.data());
}


void Localizer::Optimize() {
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << std::endl;
}

