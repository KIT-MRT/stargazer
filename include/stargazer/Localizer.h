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


#include "StargazerConfig.h"
#include "StargazerImgTypes.h"
#include "StargazerTypes.h"

namespace stargazer {

class Localizer {

public:
    Localizer(){};

    Localizer(std::string cfgfile) {
        readConfig(cfgfile, camera_intrinsics, landmarks);
    };

    ~Localizer(){};

    virtual void UpdatePose(std::vector<ImgLandmark>& img_landmarks, float dt) = 0;

    /// Getter
    const pose_t getPose() const {
        return ego_pose;
    }

    const std::map<int, Landmark>& getLandmarks() const {
        return landmarks;
    }

    const camera_params_t& getIntrinsics() const {
        return camera_intrinsics;
    }

protected:
    std::map<int, Landmark> landmarks;
    camera_params_t camera_intrinsics = {{0., 0., 0., 0., 0., 0.}};
    pose_t ego_pose = {{0., 0., 0., 0., 0., 0.}};
};

} // namespace stargazer
