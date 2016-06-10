//
// Created by bandera on 10.06.16.
//

#include "Localizer.h"

using namespace stargazer;

Localizer::Localizer(std::string cfgfile)
        : m_oStateVector(5, 1, CV_32FC1), m_oCovarianceMatrix(5, 5, CV_32FC1), m_oStateTransitionMatrix(5, 5, CV_32FC1),
          m_oProcessNoise(5, 5, CV_32FC1), m_oCameraIntrinsics(3, 3, CV_32FC1), m_oMeasModel(3, 5, CV_32FC1) {
    m_bLocalizerInit = false;

    /// Read in Landmark ids
    readConfig(cfgfile, camera_intrinsics, landmarks);
}

void Localizer::UpdatePose(std::vector<ImgLandmark>& img_landmarks, float fDeltaT) {
    std::vector<pose_t> assumptions;
    std::vector<std::vector<int>> supporters;
    std::vector<double> errors;
    int i_winnerIndex = 0;
    double error_threshold = 1000; ///@todo Wert finden/Parametrisieren

    if (img_landmarks.size() > 1) {
        /// Step 1: For every combination of seen landmarks, calculate triangulation and
        ///         test this assumption on other landmarks by calculating reprojection error
        ///         those, that have little error, are kept as supporters
        for (size_t i = 0; i < img_landmarks.size(); i++) {
            for (size_t j = i + 1; j < img_landmarks.size(); j++) {
                /// use triangulation to generate measurement
                Point p_world_1, p_world_2;
                transformLM2World(&landmarks[img_landmarks[i].nID].points[0][(int)POINT::X],
                                  &landmarks[img_landmarks[i].nID].points[0][(int)POINT::Y],
                                  landmarks[img_landmarks[i].nID].pose.data(), &p_world_1[(int)POINT::X],
                                  &p_world_1[(int)POINT::Y], &p_world_1[(int)POINT::Z]);
                transformLM2World(&landmarks[img_landmarks[j].nID].points[0][(int)POINT::X],
                                  &landmarks[img_landmarks[j].nID].points[0][(int)POINT::Y],
                                  landmarks[img_landmarks[j].nID].pose.data(), &p_world_2[(int)POINT::X],
                                  &p_world_2[(int)POINT::Y], &p_world_2[(int)POINT::Z]);
                cv::Point p_img_1 = img_landmarks[i].voCorners[0];
                cv::Point p_img_2 = img_landmarks[j].voCorners[0];
                assumptions.push_back(TriangulateTwoPoints(p_world_1, p_img_1, p_world_2, p_img_2));

                std::vector<int> support;
                for (size_t k = 0; k < img_landmarks.size(); k++) {
                    cv::Mat errorMatrix = calcReprojectionError(img_landmarks[k], assumptions.back());
                    double error = cv::norm(errorMatrix);

                    if (error < error_threshold) {
                        support.push_back(k);
                        errors.push_back(error);
                    }
                }
                supporters.push_back(support);
            }
        }

        /// Step 2: Choose the assumption with the most supporters
        size_t maxSupporters = 0;
        for (size_t i = 0; i < supporters.size(); i++) {
            if (maxSupporters < supporters[i].size()) {
                maxSupporters = supporters[i].size();
                i_winnerIndex = i;
            }
        }
        if (maxSupporters == 0) {
            std::cerr << "No supporters found!" << std::endl;
            return;
        }
    } else if (img_landmarks.size() == 1) // Only one landmark
    {
        // std::cout << "Using only one landmark!" << std::endl;
        /// Step 1:
        for (size_t i = 0; i < 3; i++) {
            for (size_t j = i + 1; j < 3; j++) {
                /// use triangulation to generate measurement
                Point p_world_1, p_world_2;
                transformLM2World(&landmarks[img_landmarks[0].nID].points[i][(int)POINT::X],
                                  &landmarks[img_landmarks[0].nID].points[i][(int)POINT::Y],
                                  landmarks[img_landmarks[0].nID].pose.data(), &p_world_1[(int)POINT::X],
                                  &p_world_1[(int)POINT::Y], &p_world_1[(int)POINT::Z]);
                transformLM2World(&landmarks[img_landmarks[0].nID].points[j][(int)POINT::X],
                                  &landmarks[img_landmarks[0].nID].points[j][(int)POINT::Y],
                                  landmarks[img_landmarks[0].nID].pose.data(), &p_world_2[(int)POINT::X],
                                  &p_world_2[(int)POINT::Y], &p_world_2[(int)POINT::Z]);
                cv::Point p_img_1 = img_landmarks[0].voCorners[i];
                cv::Point p_img_2 = img_landmarks[0].voCorners[j];
                assumptions.push_back(TriangulateTwoPoints(p_world_1, p_img_1, p_world_2, p_img_2));

                cv::Mat errorMatrix = calcReprojectionError(img_landmarks[0], assumptions.back());
                double error = cv::norm(errorMatrix);

                if (error < error_threshold) {
                    std::vector<int> support;
                    support.push_back(0);
                    errors.push_back(error);
                    supporters.push_back(support);
                } else {
                    assumptions.pop_back(); // Remove the assumption again
                }
            }
        }
        /// Step 2: Choose the assumption with the smallest error
        double minError = DBL_MAX;
        for (size_t i = 0; i < errors.size(); i++) {
            if (minError > errors[i]) {
                minError = errors[i];
                i_winnerIndex = i;
            }
        }
        if (minError == DBL_MAX) {
            return;
        }
    } else // No landmarks received
    {
        PredictKF(fDeltaT); // Predict only and then return
        return;
    }

    /// Step 3: Optimize this assumption locally, using the supporters!
    // std::cout << img_landmarks.size() << " Landmarks, " << assumptions.size() << " Assumptions, "<< std::endl;
    // double error = calcReprojectionError(img_landmarks.at(supporters[i_winnerIndex].at(0)),m_oStateVector);
    // std::cout << "Error before: " << error << std::endl;
    //    visualizeLandmarks(img_landmarks,m_oStateVector);
    //    cv::waitKey(100);
    Eigen::Vector3d pose_guess; // Take the winning assumption
    pose_guess(0) = assumptions[i_winnerIndex][(int)POSE::X];
    pose_guess(1) = assumptions[i_winnerIndex][(int)POSE::Y];
    pose_guess(2) = assumptions[i_winnerIndex][(int)POSE::Rz];

    // LibMV initialization and minimization
    PoseFunctor a;
    a.pLandmarks = &img_landmarks;
    a.pSupporters = &supporters[i_winnerIndex];
    a.localizer = this;
    libmv::LevenbergMarquardt<PoseFunctor> optimizer(a);
    libmv::LevenbergMarquardt<PoseFunctor>::SolverParameters Param;
    optimizer.minimize(Param, &pose_guess);
    cv::Mat pose_measurement(3, 1, CV_32FC1);
    pose_measurement.at<float>(0, 0) = pose_guess(0);
    pose_measurement.at<float>(1, 0) = pose_guess(1);
    pose_measurement.at<float>(2, 0) = pose_guess(2);
    // Correct angle
    while (M_PI < pose_measurement.at<float>(2, 0)) {
        pose_measurement.at<float>(2, 0) -= 2 * M_PI;
    }
    while (-M_PI > pose_measurement.at<float>(2, 0)) {
        pose_measurement.at<float>(2, 0) += 2 * M_PI;
    }

    /// Step 4: Update Kalmanfilter
    if (!m_bLocalizerInit) {
        cv::Mat oCovariance = 1000 * cv::Mat::eye(5, 5, CV_32FC1);
        oCovariance.at<float>(2, 2) = 100;
        oCovariance.at<float>(4, 4) = 100;
        InitKF(pose_measurement, oCovariance);
        return;
    }
    PredictKF(fDeltaT);

    //    cv::Mat oMeasNoise = cv::Mat::zeros(3,3,CV_32FC1);
    //    for(int k = 0; k<supporters[i_winnerIndex].size(); k++)
    //    {
    //        oMeasNoise += calcReprojectionError(img_landmarks[supporters[i_winnerIndex].at(k)], assumptions.back());
    //    }
    //    oMeasNoise /= supporters[i_winnerIndex].size();
    /// noise is just a guess
    cv::Mat oMeasNoise = 1000 * cv::Mat::eye(3, 3, CV_32FC1);
    oMeasNoise.at<float>(2, 2) = (1 * M_PI / 180) * (1 * M_PI / 180);
    UpdateKF(pose_measurement, oMeasNoise);

    /// DEBUG: Use measurement without filter
    //    m_oStateVector.at<float>(0,0) = pose_measurement.at<float>(0,0);
    //    m_oStateVector.at<float>(1,0) = pose_measurement.at<float>(1,0);
    //    m_oStateVector.at<float>(2,0) = pose_measurement.at<float>(2,0);
}

/////////////////////////////////////
/// KALMAN FILTER
/////////////////////////////////////
int Localizer::InitKF(cv::Mat& i_oInitialState, cv::Mat& i_oInitialCovariance) {
    /// set initial state vector
    m_oStateVector = cv::Mat::zeros(5, 1, CV_32FC1);
    m_oStateVector.at<float>(0, 0) = i_oInitialState.at<float>(0, 0);
    m_oStateVector.at<float>(1, 0) = i_oInitialState.at<float>(1, 0);
    m_oStateVector.at<float>(2, 0) = i_oInitialState.at<float>(2, 0);
    /// set initial covariance
    m_oCovarianceMatrix = i_oInitialCovariance;
    /// set process noise matrix
    m_oProcessNoise = cv::Mat::eye(5, 5, CV_32FC1);
    m_oProcessNoise.at<float>(0, 0) = 10;                                    // ca. 3mm
    m_oProcessNoise.at<float>(1, 1) = 10;                                    // ca. 3mm
    m_oProcessNoise.at<float>(2, 2) = (5 * M_PI / 180) * (5 * M_PI / 180);   // 5°
    m_oProcessNoise.at<float>(3, 3) = 1000;                                  // ca. 31 mm/s
    m_oProcessNoise.at<float>(4, 4) = (15 * M_PI / 180) * (15 * M_PI / 180); // 15°/s
    /// set measurement model matrix
    m_oMeasModel = cv::Mat::zeros(3, 5, CV_32FC1);
    m_oMeasModel.at<float>(0, 0) = 1;
    m_oMeasModel.at<float>(1, 1) = 1;
    m_oMeasModel.at<float>(2, 2) = 1;

    m_bLocalizerInit = true;
    return 0;
}

int Localizer::PredictKF(float fDeltaT) {
    float fTheta = m_oStateVector.at<float>(2, 0);
    float fV = m_oStateVector.at<float>(3, 0);
    float fThetaDot = m_oStateVector.at<float>(4, 0);

    /// predict state
    /// Kalman Update: x^  = A*x + B*u (here without input u)
    /* Transistion matrix:
     *      X   Y   Theta   v               Theta_dot
     * X    1   0   0       dt*cos(Theta)   0
     * Y    0   1   0       dt*sin(Theta)   0
     * Th   0   0   1       0               dt
     * v    0   0   0       1               0
     * Th_d 0   0   0       0               1
     */
    m_oStateTransitionMatrix = cv::Mat::eye(5, 5, CV_32F); // Constant velocity, constant yawrate
    m_oStateTransitionMatrix.at<float>(0, 3) = fDeltaT * cos(fTheta);
    m_oStateTransitionMatrix.at<float>(1, 3) = fDeltaT * sin(fTheta);
    m_oStateTransitionMatrix.at<float>(2, 4) = fDeltaT;
    m_oStateVector = m_oStateTransitionMatrix * m_oStateVector;

    //    /// predict state with constant circular velocity model
    //    if (fThetaDot == 0){
    //        m_oStateVector.at<float>(0,0) += fV*fDeltaT*cos(fTheta);
    //        m_oStateVector.at<float>(1,0) += fV*fDeltaT*sin(fTheta);
    //    } else {
    //        m_oStateVector.at<float>(0,0) += fV/fThetaDot*(-sin(fTheta)+sin(fTheta+fThetaDot*fDeltaT));
    //        m_oStateVector.at<float>(1,0) += fV/fThetaDot*(+cos(fTheta)-sin(fTheta+fThetaDot*fDeltaT));
    //    }
    //    m_oStateVector.at<float>(2,0) += fThetaDot*fDeltaT;
    //    m_oStateVector.at<float>(3,0) = m_oStateVector.at<float>(3,0);
    //    m_oStateVector.at<float>(4,0) = m_oStateVector.at<float>(4,0);

    /// Ensure orientation is within +-pi
    while (M_PI < m_oStateVector.at<float>(2, 0)) {
        m_oStateVector.at<float>(2, 0) -= 2 * M_PI;
    }
    while (-M_PI > m_oStateVector.at<float>(2, 0)) {
        m_oStateVector.at<float>(2, 0) += 2 * M_PI;
    }

    //! Compute noise process noise covariance Q
    cv::Mat oW = cv::Mat::eye(5, 5, CV_32F);
    oW.at<float>(0, 2) = -fDeltaT * fV * sin(fTheta);
    oW.at<float>(0, 3) = fDeltaT * cos(fTheta);
    oW.at<float>(1, 2) = fDeltaT * fV * cos(fTheta);
    oW.at<float>(1, 3) = fDeltaT * sin(fTheta);
    oW.at<float>(2, 4) = fDeltaT;

    //    /// set process noise matrix
    //    cv::Mat temp = cv::Mat::eye(5,5,CV_32FC1);
    //    temp.at<float>(0,0) = fDeltaT*fV*(0.15*cos(fTheta)+0.05*sin(fTheta));
    //    temp.at<float>(1,1) = fDeltaT*fV*(0.15*sin(fTheta)+0.05*cos(fTheta));
    //    temp.at<float>(2,2) = (5*M_PI/180)*(5*M_PI/180);
    //    temp.at<float>(3,3) = fDeltaT*100;
    //    temp.at<float>(4,4) = (5*M_PI/180)*(5*M_PI/180);

    /// predict covariance
    /// Kalman Update: P^  = A*P*A' + Q
    m_oCovarianceMatrix = m_oStateTransitionMatrix * m_oCovarianceMatrix * m_oStateTransitionMatrix.t() +
                          oW * m_oProcessNoise * oW.t(); // temp; //

    return 0;
}

int Localizer::UpdateKF(cv::Mat& i_oMeasurement, cv::Mat& i_oMeasurementNoise) {
    /// Discrete Kalman filter measurement update
    /* Measurement is of the form
     *      (X)
     * z =  (Y)
     *      (Z)
     * The measurement Model H (m_oMeasModel) is of the form
     *      (   1   0   0   0   0)
     * H =  (   0   1   0   0   0)
     *      (   0   0   1   0   0)
     * since the measurement and the first three elements of state vector are the same.
     * The measurement noise is of the form:
     *      ( sigma_x   0       0           )
     * R =  ( 0         sigma_y 0           )
     *      ( 0         0       sigma_theta )
     * */
    /// compute kalman matrix
    /// K = P^*H'*(H*P^*H' + R)^-1
    cv::Mat oTemp = m_oCovarianceMatrix * m_oMeasModel.t();
    cv::Mat oS = m_oMeasModel * oTemp + i_oMeasurementNoise;
    cv::Mat oK = oTemp * oS.inv();

    /// update state vector
    /// x = x^ + K*(z-H*x^)
    cv::Mat oInnovation = i_oMeasurement - m_oMeasModel * m_oStateVector;
    /// Ensure orientation is within +-pi
    while (M_PI < oInnovation.at<float>(2, 0)) {
        oInnovation.at<float>(2, 0) -= 2 * M_PI;
    }
    while (-M_PI > oInnovation.at<float>(2, 0)) {
        oInnovation.at<float>(2, 0) += 2 * M_PI;
    }
    m_oStateVector = m_oStateVector + oK * oInnovation;

    /// Ensure orientation is within +-pi
    while (M_PI < m_oStateVector.at<float>(2, 0)) {
        m_oStateVector.at<float>(2, 0) -= 2 * M_PI;
    }
    while (-M_PI > m_oStateVector.at<float>(2, 0)) {
        m_oStateVector.at<float>(2, 0) += 2 * M_PI;
    }

    /// update covariance
    /// P = (I - K*H)*P^
    /// todo: get size and type of eye matrix from inputs
    m_oCovarianceMatrix = (cv::Mat::eye(5, 5, CV_32FC1) - oK * m_oMeasModel) * m_oCovarianceMatrix;

    return 0;
}

/////////////////////////////////////
/// HELPER FUNCTIONS
/////////////////////////////////////

void Localizer::GetPointsFromID(int ID, std::vector<cv::Mat>& corner_points, std::vector<cv::Mat>& id_points) {
    for (size_t i = 0; i < landmarks[ID].points.size(); i++) {
        cv::Mat pt(3, 1, CV_32FC1);
        pt.at<float>(0, 0) = landmarks[ID].points[i][(int)POINT::X];
        pt.at<float>(1, 0) = landmarks[ID].points[i][(int)POINT::Y];
        pt.at<float>(2, 0) = landmarks[ID].pose[(int)POSE::Z];
        if (i < 3) {
            corner_points.push_back(pt);
        } else {
            id_points.push_back(pt);
        }
    }
}

cv::Mat Localizer::calcReprojectionError(ImgLandmark& lm, pose_t position) {
    cv::Mat errorMatrix = cv::Mat::zeros(3, 3, CV_32FC1);

    for (size_t i = 0; i < landmarks[lm.nID].points.size(); i++) {
        double residuals[2];
        if (i < 3) {
            auto functor = LM2ImgReprojectionFunctor(lm.voCorners[i].x, lm.voCorners[i].y,
                                                     landmarks[lm.nID].points[i][(int)POINT::X],
                                                     landmarks[lm.nID].points[i][(int)POINT::Y]);
            functor(landmarks[lm.nID].pose.data(), position.data(), camera_intrinsics.data(), residuals);
        } else {
            auto functor = LM2ImgReprojectionFunctor(lm.voIDPoints[i - 3].x, lm.voIDPoints[i - 3].y,
                                                     landmarks[lm.nID].points[i][(int)POINT::X],
                                                     landmarks[lm.nID].points[i][(int)POINT::Y]);
            functor(landmarks[lm.nID].pose.data(), position.data(), camera_intrinsics.data(), residuals);
        }
        errorMatrix.at<float>(0, 0) += fabs(residuals[0]);
        errorMatrix.at<float>(1, 1) += fabs(residuals[1]);
    }
    /// Mean the errors
    errorMatrix /= landmarks[lm.nID].points.size();

    /// Calc Angle error
    double ang1 = atan2(landmarks[lm.nID].points[2][(int)POINT::Y] - landmarks[lm.nID].points[0][(int)POINT::Y],
                        landmarks[lm.nID].points[2][(int)POINT::X] - landmarks[lm.nID].points[0][(int)POINT::X]);
    double ang2 = atan2(lm.voCorners[2].y - lm.voCorners[0].y, lm.voCorners[2].x - lm.voCorners[0].x);
    ang1 = std::fmod(ang1, 2 * M_PI);
    ang2 = std::fmod(ang2, 2 * M_PI);
    double angErr = fabs(ang1 - ang2);
    if (angErr > M_PI)
        angErr = 2 * M_PI - angErr;
    errorMatrix.at<float>(2, 2) = angErr;
    return errorMatrix;
}

pose_t Localizer::TriangulateTwoPoints(const Point& p_world_1, const cv::Point& p_img_1, const Point& p_world_2,
                                       const cv::Point& p_img_2) {
    ///    Pos = Vehicle Position
    ///    a1  = angle from landmark 1 to vehicle
    ///    b1  = viewing angle of landmark 1
    ///    R1  = radius around landmark 1 determined by elevation angle
    ///    L1  = Landmark position in global coordinate frame
    ///
    ///     [PosX]        [L1X]       [cos(a1)    -sin(a1)]   [R1]
    /// (1) [    ]    =   [   ]   +   [                   ]*  [  ]
    ///     [PosY]        [L1Y]       [sin(a1)     cos(a1)]   [ 0]
    ///
    ///     the same holds for landmark 2
    ///
    ///     [PosX]        [L2X]       [cos(a2)    -sin(a2)]   [R2]
    /// (2) [    ]    =   [   ]   +   [                   ]*  [  ]
    ///     [PosY]        [L2Y]       [sin(a2)     cos(a2)]   [ 0]
    ///
    ///     furthermore:
    ///
    ///
    /// (3) Theta = a1 - 180 - b1 = a2 - 180 - b2
    ///
    ///     This leads to: a1 = a2 - b2 + b1
    ///
    ///     Setting (1) equal to (2) and substituting a1 by (3):
    ///     [L1X - L2X]     (   [1  0]      [ cos(b2-b1)  sin(b2-b1)]) [cos(a2)]
    /// (4) [         ] =   (R2*[    ] - R1*[                       ])*[       ]
    ///     [L1X - L2X]     (   [0  1]      [-sin(b2-b1)  cos(b2-b1)]) [sin(a2)]

    /// Landmark positions in global coordinate frame
    cv::Mat oL1(2, 1, CV_32FC1);
    cv::Mat oL2(2, 1, CV_32FC1);
    double fH1 = 1, fH2 = 1;

    /// match landmarks from LUT with observations and get their positions
    /// Landmark 1
    oL1.at<float>(0, 0) = p_world_1[(int)POINT::X];
    oL1.at<float>(1, 0) = p_world_1[(int)POINT::Y];
    fH1 = p_world_1[(int)POINT::Z];
    ;
    /// Landmark 2
    oL2.at<float>(0, 0) = p_world_2[(int)POINT::X];
    oL2.at<float>(1, 0) = p_world_2[(int)POINT::Y];
    fH2 = p_world_2[(int)POINT::Z];

    /// intrinsics and stuff
    float fF = camera_intrinsics[(int)INTRINSICS::f]; /// focal length in pixels
    float fCx = camera_intrinsics[(int)INTRINSICS::u0];
    float fCy = camera_intrinsics[(int)INTRINSICS::v0];

    /// this is to find the observation angles
    float fX = float(p_img_1.x - fCx);
    float fY = float(p_img_1.y - fCy);
    float fLength = sqrt(fX * fX + fY * fY);

    /// elevation angle of landmark 1
    float fTanElev = fLength / fF;
    /// radius of the search circle for landmark 1
    float fR1 = fH1 * fTanElev;
    /// viewing angle for landmark 1
    float fBeta1 = atan2f(fY, fX);

    /// same for landmark 2
    fX = float(p_img_2.x - fCx);
    fY = float(p_img_2.y - fCy);
    fLength = sqrt(fX * fX + fY * fY);

    /// elevation angle of landmark 2
    fTanElev = fLength / fF;
    /// radius of the search circle for landmark 2
    float fR2 = fH2 * fTanElev;
    /// viewing angle for landmark 2
    float fBeta2 = atan2f(fY, fX);

    /// matrices
    cv::Mat oRot1(2, 2, CV_32FC1);
    cv::Mat oRot2(2, 2, CV_32FC1);
    cv::Mat oTot(2, 2, CV_32FC1);

    /// see derivation above for further information
    oRot1.at<float>(0, 0) = cos(fBeta2 - fBeta1);
    oRot1.at<float>(0, 1) = sin(fBeta2 - fBeta1);
    oRot1.at<float>(1, 0) = -sin(fBeta2 - fBeta1);
    oRot1.at<float>(1, 1) = cos(fBeta2 - fBeta1);

    oRot2 = cv::Mat::eye(2, 2, CV_32FC1);

    oTot = fR2 * oRot2 - fR1 * oRot1;

    /// solution to (4)
    cv::Mat oDir2 = oTot.inv() * (oL1 - oL2);

    /// position according to (2)
    cv::Mat oPos = oL2 + fR2 * oDir2;

    /// orientation according to (3)
    float fOrientation = atan2f(oDir2.at<float>(1, 0), oDir2.at<float>(0, 0)) - M_PI - fBeta2;

    /// limit orientation angle between [-pi, pi]
    while (M_PI < fOrientation) {
        fOrientation -= 2 * M_PI;
    }
    while (-M_PI > fOrientation) {
        fOrientation += 2 * M_PI;
    }

    /// generate output state vector
    pose_t pose;
    pose[(int)POSE::X] = oPos.at<float>(0, 0);
    pose[(int)POSE::Y] = oPos.at<float>(1, 0);
    pose[(int)POSE::Rz] = fOrientation - M_PI / 2;

    return pose;
}

void Localizer::visualizeLandmarks(std::vector<ImgLandmark>& img_landmarks, pose_t position) {
    std::stringstream out1;
    std::string txt;
    cv::Mat img = cv::Mat::zeros(1048, 1363, CV_8UC3); ///@todo read those in from somewhere!
    cv::Point reprojectedPoint;
    double max_error = 0;
    double error = 0;
    for (auto& lm : img_landmarks) {
        /// Step 1: Get individual points of landmark in world coordinates

        for (size_t i = 0; i < landmarks[lm.nID].points.size(); i++) {
            /// Step 3: Convert every point into camera frame
            double x_out = 0.;
            double y_out = 0.;
            transformLM2Img(&landmarks[lm.nID].points[i][(int)POINT::X], &landmarks[lm.nID].points[i][(int)POINT::Y],
                            landmarks[lm.nID].pose.data(), position.data(), camera_intrinsics.data(), &x_out, &y_out);
            reprojectedPoint = cvPoint(x_out, y_out);

            /// Step 4: Add up error
            if (i < 3) {
                /// Corner Points
                error += pow(x_out - lm.voCorners[i].x, 2) + pow(y_out - lm.voCorners[i].y, 2);
                circle(img, reprojectedPoint, 3, cv::Scalar(255, 0, 255), 2); // Orange
                circle(img, lm.voCorners[i], 2, cv::Scalar(255, 128, 0), 2);  // Magenta
                std::cout << x_out << "/" << lm.voCorners[i].x << " - " << y_out << "/" << lm.voCorners[i].y
                          << std::endl;
            } else {
                /// ID Points
                error += pow(x_out - lm.voIDPoints[i - 3].x, 2) + pow(y_out - lm.voIDPoints[i - 3].y, 2);
                std::cout << x_out << "/" << lm.voIDPoints[i - 3].x << " - " << y_out << "/" << lm.voIDPoints[i - 3].y
                          << std::endl;
                circle(img, reprojectedPoint, 3, cv::Scalar(0, 0, 255), 2);     // Red
                circle(img, lm.voIDPoints[i - 3], 2, cv::Scalar(255, 0, 0), 2); // Blue
            }
        }

        reprojectedPoint = cvPoint(lm.oPosition.x + 25, lm.oPosition.y + 25);
        out1 << "Error: " << error;
        txt = out1.str();
        putText(img, txt, reprojectedPoint, 2, 0.4, cvScalar(0, 255, 0));
        out1.str(std::string());
        reprojectedPoint = cvPoint(lm.oPosition.x + 25, lm.oPosition.y - 25);
        out1 << "ID: " << lm.nID;
        txt = out1.str();
        putText(img, txt, reprojectedPoint, 2, 0.4, cvScalar(255, 255, 0));
        out1.str(std::string());
        if (error > max_error)
            max_error = error;
    }

    cv::namedWindow("Reprojection Image", CV_WINDOW_NORMAL);
    cv::imshow("Reprojection Image", img);
    cv::waitKey(1);
}

Eigen::VectorXd PoseFunctor::operator()(Eigen::Vector3d Input) const {
    Eigen::VectorXd Output(pSupporters->size());

    while (M_PI < Input(2)) {
        Input(2) -= 2 * M_PI;
    }
    while (-M_PI > Input(2)) {
        Input(2) += 2 * M_PI;
    }

    pose_t pose;
    pose[(int)POSE::X] = Input(0);
    pose[(int)POSE::Y] = Input(1);
    pose[(int)POSE::Rz] = Input(2);

    for (size_t nI = 0; nI < pSupporters->size(); nI++) {
        cv::Mat errorMatrix = localizer->calcReprojectionError(pLandmarks->at(nI), pose);
        Output(nI) = cv::norm(errorMatrix);
    }

    return Output;
}
