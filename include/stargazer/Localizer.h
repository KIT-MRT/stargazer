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
#include "StargazerConfig.h"
#include "StargazerTypes.h"
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

class Localizer {

public:
    Localizer(std::string cfgfile);

    ~Localizer(){};

    void UpdatePose(std::vector<ImgLandmark>& img_landmarks, float dt);

    const pose_t getPose() const {
        pose_t pose;
        pose[(int)POSE::X] = m_oStateVector.at<float>(0, 0);
        pose[(int)POSE::Y] = m_oStateVector.at<float>(1, 0);
        pose[(int)POSE::Z] = 0;
        pose[(int)POSE::Rx] = 0.;
        pose[(int)POSE::Ry] = 0.;
        // TODO is this correct (angle axis)?
        pose[(int)POSE::Rz] = m_oStateVector.at<float>(2, 0);
        return pose;
    };

    /// Kalman Filter processing
    int InitKF(cv::Mat& i_oInitialState, cv::Mat& i_oInitialCovariance);
    int PredictKF(float fDeltaT);
    int UpdateKF(cv::Mat& i_oMeasurement, cv::Mat& i_oMeasurementNoise);

    /// Public Helper Functions
    cv::Mat calcReprojectionError(ImgLandmark& lm, pose_t position);

    /// Getter
    const std::map<int, Landmark>& getLandmarks() const {
        return landmarks;
    }

    const camera_params_t& getIntrinsics() const {
        return camera_intrinsics;
    }

private:
    std::map<int, Landmark> landmarks;
    camera_params_t camera_intrinsics;

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
    bool m_bLocalizerInit;

    /// private helper functions
    pose_t TriangulateTwoPoints(const Point& p_world_1, const cv::Point& p_img_1, const Point& p_world_2,
                                const cv::Point& p_img_2);
    int int_pow(int x, int p);
};

/////////////////////////////////////
/// OTHER
/////////////////////////////////////
inline int Localizer::int_pow(int x, int p) {
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
    Localizer* localizer;
};

} // namespace stargazer
