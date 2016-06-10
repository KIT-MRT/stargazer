//
// Created by bandera on 28.03.16.
//

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
    camera_params_t camera_intrinsics = {};
    pose_t ego_pose = {};
};

} // namespace stargazer
