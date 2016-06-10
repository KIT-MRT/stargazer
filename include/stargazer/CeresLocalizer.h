//
// Created by bandera on 28.03.16.
//

#pragma once

#include <string>
#include "CoordinateTransformations.h"
#include "Localizer.h"
#include "internal/CostFunction.h"

namespace stargazer {

class CeresLocalizer : public Localizer {

public:
    CeresLocalizer(std::string cfgfile);

    virtual void UpdatePose(std::vector<ImgLandmark>& img_landmarks, float dt) override;

private:
    ceres::Problem problem;

    bool is_initialized;

    void ClearResidualBlocks();
    void AddResidualBlocks(std::vector<ImgLandmark> img_landmarks);
    void SetCameraParamsConstant();
    void Optimize();
};

} // namespace stargazer
