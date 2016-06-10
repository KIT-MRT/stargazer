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

void CeresLocalizer::visualizeLandmarks(std::vector<Landmark>& img_landmarks) {
    std::stringstream out1;
    std::string txt;
    cv::Mat img = cv::Mat::zeros(1024, 1360, CV_8UC3); ///@todo read those in from somewhere!
    cv::Point reprojectedPoint;
    cv::Point seenPoint;
    double max_error = 0;
    double error = 0;
    for (auto& lm : img_landmarks) {
        /// Step 1: Get individual points of landmark in world coordinates

        for (size_t i = 0; i < landmarks[lm.id].points.size(); i++) {
            /// Step 3: Convert every point into camera frame
            double x_out = 0.;
            double y_out = 0.;
            transformWorld2Img(&landmarks[lm.id].points[i][(int)POINT::X], &landmarks[lm.id].points[i][(int)POINT::Y],
                               landmarks[lm.id].pose.data(), ego_pose.data(), camera_intrinsics.data(), &x_out, &y_out);
            reprojectedPoint = cvPoint(x_out, y_out);
            seenPoint = cvPoint(lm.points[i][(int)POINT::X], lm.points[i][(int)POINT::Y]);
            error += pow(x_out - lm.points[i][(int)POINT::X], 2) + pow(y_out - lm.points[i][(int)POINT::X], 2);
//            std::cout << x_out << "/" << lm.points[i][(int)POINT::X] << " - " << y_out << "/"
//                      << lm.points[i][(int)POINT::X] << std::endl;

            /// Step 4: Add up error
            if (i < 3) {
                /// Corner Points
                circle(img, reprojectedPoint, 3, cv::Scalar(255, 0, 255), 2); // Orange
                circle(img, seenPoint, 2, cv::Scalar(255, 128, 0), 2);        // Magenta
            } else {
                /// ID Points
                circle(img, reprojectedPoint, 3, cv::Scalar(0, 0, 255), 2); // Red
                circle(img, seenPoint, 2, cv::Scalar(255, 0, 0), 2);        // Blue
            }
        }

        reprojectedPoint = cvPoint(reprojectedPoint.x + 25, reprojectedPoint.y + 25);
        out1 << "Error: " << error;
        txt = out1.str();
        putText(img, txt, reprojectedPoint, 2, 0.4, cvScalar(0, 255, 0));
        out1.str(std::string());
        reprojectedPoint = cvPoint(reprojectedPoint.x, reprojectedPoint.y - 50);
        out1 << "ID: " << lm.id;
        txt = out1.str();
        putText(img, txt, reprojectedPoint, 2, 0.4, cvScalar(255, 255, 0));
        out1.str(std::string());
        if (error > max_error)
            max_error = error;
    }

    cv::namedWindow("Reprojection Image", CV_WINDOW_NORMAL);
    cv::imshow("Reprojection Image", img);
    cv::waitKey(10);
}