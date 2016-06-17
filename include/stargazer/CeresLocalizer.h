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

    const ceres::Solver::Summary& getSummary() const {
        return summary;
    }

private:
    ceres::Problem problem;
    ceres::Solver::Summary summary;

    bool is_initialized;

    void ClearResidualBlocks();
    void AddResidualBlocks(std::vector<ImgLandmark> img_landmarks);
    void SetCameraParamsConstant();
    void Optimize();
};

} // namespace stargazer
