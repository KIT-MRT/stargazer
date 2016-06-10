//
// Created by bandera on 28.03.16.
//

#pragma once

#include <fstream>
#include <iostream>
#include <iostream>
#include <math.h>
#include <sstream>
#include <stdarg.h>
#include <stdexcept>
#include <string>
#include <vector>
#include <vector>
#include <Eigen/Core>
#include <opencv/highgui.h>
#include "CoordinateTransformations.h"
#include "LandmarkFinder.h"
#include "Localizer.h"
#include "internal/CostFunction.h"
#include "libCalibIO/CalibIO.hpp"
#include "libmv/numeric/levenberg_marquardt.h"

namespace stargazer {

typedef struct {
    int ID;
    float H;
    float X;
    float Y;
    float Theta;
} LandmarkPosition;

/* State Vector is defined as:
 * X:           X-Coordinate
 * Y:           Y-Coordinate
 * Theta        Orientation
 * v            Speed
 * theta_dot    Yawrate
 * */

class TriangulationLocalizer : public Localizer {

public:
    TriangulationLocalizer(std::string cfgfile);

    void UpdatePose(std::vector<ImgLandmark>& img_landmarks, float dt) override;

    /// Kalman Filter processing
    int InitKF(cv::Mat& i_oInitialState, cv::Mat& i_oInitialCovariance);
    int PredictKF(float fDeltaT);
    int UpdateKF(cv::Mat& i_oMeasurement, cv::Mat& i_oMeasurementNoise);

    /// Public Helper Functions
    cv::Mat calcReprojectionError(ImgLandmark& lm, pose_t position);

private:
    /// Known landmarks
    std::vector<LandmarkPosition> m_asLandmarkLUT;

    /// Kalman Filter Definitions
    cv::Mat m_oStateVector;
    cv::Mat m_oCovarianceMatrix;
    cv::Mat m_oStateTransitionMatrix;
    cv::Mat m_oProcessNoise;
    cv::Mat m_oMeasModel;

    /// Camera intrinsics
    cv::Mat m_oCameraIntrinsics;
    bool m_bTriangulationLocalizerInit;

    /// private helper functions
    pose_t TriangulateTwoPoints(const Point& p_world_1, const cv::Point& p_img_1, const Point& p_world_2,
                                const cv::Point& p_img_2);
    int int_pow(int x, int p);
};

/////////////////////////////////////
/// OTHER
/////////////////////////////////////
inline int TriangulationLocalizer::int_pow(int x, int p) {
    if (p == 0)
        return 1;
    if (p == 1)
        return x;
    return x * int_pow(x, p - 1);
}

// LibMv minimization
struct PoseFunctor {
    typedef Eigen::VectorXd FMatrixType;
    typedef Eigen::Vector3d XMatrixType;
    Eigen::VectorXd operator()(Eigen::Vector3d Input) const;
    std::vector<ImgLandmark>* pLandmarks;
    std::vector<int>* pSupporters;
    TriangulationLocalizer* triangulationLocalizer;
};

} // namespace stargazer
