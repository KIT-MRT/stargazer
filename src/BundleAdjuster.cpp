//
// Created by bandera on 19.03.16.
//

#include "BundleAdjuster.h"


BundleAdjuster::BundleAdjuster() { };

void BundleAdjuster::AddCameraPoses(std::vector <std::array<double, 3>> measurements) {
  camera_poses.reserve(measurements.size());

  for (auto &measurement : measurements) {
    std::array<double, (int) POSE::N_PARAMS> camera_pose;
    camera_pose[(int) POSE::X] = measurement[0];
    camera_pose[(int) POSE::Y] = measurement[1];
    camera_pose[(int) POSE::Z] = 0.2;
    camera_pose[(int) POSE::Rx] = 0;
    camera_pose[(int) POSE::Ry] = 0;
    camera_pose[(int) POSE::Rz] = measurement[2] + M_PI / 2;
    camera_poses.push_back(camera_pose);
  }
}

void BundleAdjuster::AddReprojectionResidualBlocks(std::vector <std::vector<Landmark>> measurements) {
  assert(measurements.size() == camera_poses.size());

  for (int i = 0; i < measurements.size(); i++) {
    for (int j = 0; j < measurements[i].size(); j++) {
      auto &observation = measurements[i][j];

      Landmark landmark(observation.id);
      assert(landmark_poses.count(observation.id) > 0);
      assert(observation.points.size() == landmark.points.size());

      // Add residual block, for every one of the seen points.
      for (int k = 0; k < landmark.points.size(); k++) {

        // Test reprojection error
        double u, v;
        transformPoint<double>(&std::get<(int) POINT::X>(landmark.points[k]),
                               &std::get<(int) POINT::Y>(landmark.points[k]),
                               landmark_poses[observation.id].data(),
                               camera_poses[i].data(),
                               camera_intrinsics.data(),
                               &u,
                               &v);
        double residuals[2];
        residuals[0] = u - std::get<(int) POINT::X>(observation.points[k]);
        residuals[1] = v - std::get<(int) POINT::Y>(observation.points[k]);
        double error = std::hypot(residuals[0], residuals[1]);
//        std::cout << "Residual: " << error << std::endl;
//        if (error > 100) {
//          std::cout << "Skipping point " << k << " of landmark " << landmark.id << " in frame " << i << ". Residual: "
//              << error << std::endl;
//          continue;
//        }

        ceres::CostFunction *cost_function = ReprojectionFunctor::Create(
            std::get<(int) POINT::X>(observation.points[k]),
            std::get<(int) POINT::Y>(observation.points[k]),
            std::get<(int) POINT::X>(landmark.points[k]),
            std::get<(int) POINT::Y>(landmark.points[k])
        );

        problem.AddResidualBlock(cost_function,
                                 new ceres::CauchyLoss(50), // NULL /* squared loss */, // Alternatively: new ceres::ScaledLoss(NULL, w_v_des, ceres::TAKE_OWNERSHIP),
                                 landmark_poses[observation.id].data(),
                                 camera_poses[i].data(),
                                 camera_intrinsics.data());
      }
    }
  }
}

void BundleAdjuster::AddDistanceResidualBlocks() {
  for (int i = 1; i < camera_poses.size(); i++) {
    ceres::CostFunction *cost_function = DistanceFunctor::Create();
    problem.AddResidualBlock(cost_function,
                             new ceres::ScaledLoss(NULL, w_dist, ceres::TAKE_OWNERSHIP),
                             camera_poses[i - 1].data(),
                             camera_poses[i].data());
  }
}

void BundleAdjuster::AddOrientationResidualBlocks() {
  for (auto &pose : landmark_poses) {
    ceres::CostFunction *cost_function = OrientationFunctor::Create();
    problem.AddResidualBlock(cost_function,
                             new ceres::ScaledLoss(NULL, w_orient, ceres::TAKE_OWNERSHIP),
                             pose.second.data());
  }
  for (auto &pose : camera_poses) {
    ceres::CostFunction *cost_function = OrientationFunctor::Create();
    problem.AddResidualBlock(cost_function,
                             new ceres::ScaledLoss(NULL, w_orient, ceres::TAKE_OWNERSHIP),
                             pose.data());
  }
}


void BundleAdjuster::Optimize() {
  std::cout << "Starting Optimization..." << std::endl;
  // Make Ceres automatically detect the bundle structure. Note that the
  // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
  // for standard bundle adjustment problems.
  ceres::Solver::Options options;
//  options.linear_solver_type = ceres::SPARSE_SCHUR;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << std::endl;
}


void BundleAdjuster::SetParametersConstant() {
  std::vector<double *> parameter_blocks;
  problem.GetParameterBlocks(&parameter_blocks);

  // Set Landmark rotation parameters constant
  std::vector<int> constant_landmark_params = {(int) POSE::Rx, (int) POSE::Ry, (int) POSE::Rz};
  ceres::SubsetParameterization *constant_landmark_params_mask =
      new ceres::SubsetParameterization((int) POSE::N_PARAMS, constant_landmark_params);
  for (auto &lm : landmark_poses) {
    if (problem.HasParameterBlock(lm.second.data()))
      problem.SetParameterization(lm.second.data(), constant_landmark_params_mask);
//          problem.SetParameterBlockConstant(lm.second.data());
  }

  // Set some pose parameters constant
  std::vector<int> constant_vehicle_params = {(int) POSE::Z, (int) POSE::Rx, (int) POSE::Ry};
  ceres::SubsetParameterization *constant_vehicle_params_mask =
      new ceres::SubsetParameterization((int) POSE::N_PARAMS, constant_vehicle_params);
  for (auto &pose : camera_poses) {
    if (problem.HasParameterBlock(pose.data()))
      problem.SetParameterization(pose.data(), constant_vehicle_params_mask);
//          problem.SetParameterBlockConstant(pose.data());
  }

  // Set Camera Parameters Constant
  if (problem.HasParameterBlock(camera_intrinsics.data()))
    problem.SetParameterBlockConstant(camera_intrinsics.data());
}

void BundleAdjuster::ClearProblem() {
  std::vector <ceres::ResidualBlockId> residual_blocks;
  problem.GetResidualBlocks(&residual_blocks);
  for (auto &block:residual_blocks) {
    problem.RemoveResidualBlock(block);
  }
  std::vector<double *> parameter_blocks;
  problem.GetParameterBlocks(&parameter_blocks);
  for (auto &block:parameter_blocks) {
    problem.RemoveParameterBlock(block);
  }

}