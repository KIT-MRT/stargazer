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

#include <iostream>
#include <ceres/rotation.h>
#include "StargazerTypes.h"

namespace stargazer {

template <typename T>
/**
 * @brief This function will transform a point, given in landmark coordinates into world coordinates
 *
 * @param x_landmark  x value of input point in landmark coordinates
 * @param y_landmark  y value of input point in landmark coordinates
 * @param landmark_pose the six dimensional pose of the landmark (rotation in rodriguez angles)
 * @param x_world x value of ouput point in world coordinates
 * @param y_world y value of ouput point in world coordinates
 * @param z_world z value of ouput point in world coordinates
 */
void transformLandMarkToWorld(const T& x_landmark, const T& y_landmark, const T* const landmark_pose, T* const x_world,
                              T* const y_world, T* const z_world) {
    // Create point in Landmark coordinates
    const T point_landmark[3] = {x_landmark, y_landmark, T(0.0)};

    // Transform point from Landmark to world coordinates
    T point_world[3];
    T angleAxisLM[3];
    angleAxisLM[0] = landmark_pose[(int)POSE::Rx];
    angleAxisLM[1] = landmark_pose[(int)POSE::Ry];
    angleAxisLM[2] = landmark_pose[(int)POSE::Rz];

    ceres::AngleAxisRotatePoint(&angleAxisLM[0], &point_landmark[0], &point_world[0]);

    // lm_pose[0,1,2] are the translation.
    point_world[0] += landmark_pose[(int)POSE::X];
    point_world[1] += landmark_pose[(int)POSE::Y];
    point_world[2] += landmark_pose[(int)POSE::Z];

    *x_world = point_world[0];
    *y_world = point_world[1];
    *z_world = point_world[2];
}

template <typename T>
/**

 * @brief This function will transform a point, given in world coordinates into image coordinates
 *
 * @param x_world  x value of input point in world coordinates
 * @param y_world  y value of input point in world coordinates
 * @param z_world  z value of input point in world coordinates
 * @param camera_pose the six dimensional pose of the camera (rotation in rodriguez angles)
 * @param camera_intrinsics the cameras intrinsic parameters
 * @param x_image x value of ouput point in image coordinates
 * @param y_image y value of ouput point in image coordinates
 */
void transformWorldToImg(const T& x_world, const T& y_world, const T& z_world, const T* const camera_pose,
                         const T* const camera_intrinsics, T* const x_image, T* const y_image) {
    // Create point in wolrd coordinates
    const T p_world[3] = {x_world, y_world, z_world};

    // Transform point to camera coordinates
    T p_camera[3];

    // This time we go from world -> cam
    // camera_pose[3,4,5] are the translation.
    p_camera[0] = p_world[0] - camera_pose[(int)POSE::X];
    p_camera[1] = p_world[1] - camera_pose[(int)POSE::Y];
    p_camera[2] = p_world[2] - camera_pose[(int)POSE::Z];
    T angleAxisCam[3];
    angleAxisCam[0] = -camera_pose[(int)POSE::Rx];
    angleAxisCam[1] = -camera_pose[(int)POSE::Ry];
    angleAxisCam[2] = -camera_pose[(int)POSE::Rz];
    ceres::AngleAxisRotatePoint(&angleAxisCam[0], &p_camera[0], &p_camera[0]);

    // Transform point to image coordinates
    T p_image[3];
    p_image[0] =
        camera_intrinsics[(int)INTRINSICS::fu] * p_camera[0] + camera_intrinsics[(int)INTRINSICS::u0] * p_camera[2];
    p_image[1] =
        camera_intrinsics[(int)INTRINSICS::fv] * p_camera[1] + camera_intrinsics[(int)INTRINSICS::v0] * p_camera[2];
    p_image[2] = p_camera[2];
    if (p_image[2] == T(0)) {
        std::cout << "WARNING; Attempt to divide by 0!" << std::endl;
        return;
    }
    p_image[0] = p_image[0] / p_image[2];
    p_image[1] = p_image[1] / p_image[2];
    p_image[2] = p_image[2] / p_image[2];

    *x_image = p_image[0];
    *y_image = p_image[1];
}

template <typename T>
/**
 * @brief This function will transform a point, given in landmark coordinates into image coordinates
 *
 * @param x_landmark  x value of input point in landmark coordinates
 * @param y_landmark  y value of input point in landmark coordinates
 * @param landmark_pose the six dimensional pose of the landmark (rotation in rodriguez angles)
 * @param camera_pose the six dimensional pose of the camera (rotation in rodriguez angles)
 * @param camera_intrinsics the cameras intrinsic parameters
 * @param x_image x value of ouput point in image coordinates
 * @param y_image y value of ouput point in image coordinates
 */
void transformLandMarkToImage(const T& x_landmark, const T& y_landmark, const T* const landmark_pose,
                              const T* const camera_pose, const T* const camera_intrinsics, T* const x_image,
                              T* const y_image) {
    T x_world, y_world, z_world;
    transformLandMarkToWorld(x_landmark, y_landmark, landmark_pose, &x_world, &y_world, &z_world);
    transformWorldToImg(x_world, y_world, z_world, camera_pose, camera_intrinsics, x_image, y_image);
}

} // namespace stargazer
