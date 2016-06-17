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

/**
 * @brief This is the abstract localizer class. Given a set of image landmarks, it computes the cameras pose based on
 * information taken from the map file.
 *
 */
class Localizer {

public:
    /**
     * @brief Default stub constructor
     *
     */
    Localizer(){};

    /**
     * @brief Constructor.
     *
     * @param cfgfile Path to map file with camera intrinsics and landmark poses.
     * @remark The config file has to be generated with ::writeConfig!
     */
    Localizer(std::string cfgfile) {
        readConfig(cfgfile, camera_intrinsics, landmarks);
    };

    /**
     * @brief Default destructor
     *
     */
    ~Localizer(){};

    /**
     * @brief Main update method. Computes pose from landmark observations and stores it in Localizer::ego_pose
     *
     * @param img_landmarks Vector of all observed landmarks in image coordinates
     * @param dt Time since last update
     */
    virtual void UpdatePose(std::vector<ImgLandmark>& img_landmarks, float dt) = 0;

    /**
     * @brief Getter for computed pose from last call to Localizer::UpdatePose
     *
     * @return const pose_t 6D Pose of the camera
     */
    const pose_t getPose() const {
        return ego_pose;
    }

    /**
     * @brief Getter for map of landmarks
     *
     * @return const std::map<int, Landmark>
     */
    const std::map<int, Landmark>& getLandmarks() const {
        return landmarks;
    }

    /**
     * @brief Getter for the cameras' intrinsic parameters
     *
     * @return const camera_params_t
     */
    const camera_params_t& getIntrinsics() const {
        return camera_intrinsics;
    }

protected:
    std::map<int, Landmark> landmarks;                              /**< Map of landmarks, read from config */
    camera_params_t camera_intrinsics = {{0., 0., 0., 0., 0., 0.}}; /**< Parameter of camera, read from config*/
    pose_t ego_pose = {{0., 0., 0., 0., 0., 0.}}; /**< Ego pose as computed by last call to Localizer::UpdatePose */
};

} // namespace stargazer
