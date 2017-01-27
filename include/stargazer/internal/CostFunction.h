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

// Ceres includes
#include "../CoordinateTransformations.h"
#include "../StargazerTypes.h"
#include "ceres/ceres.h"

namespace stargazer {

/**
 * @brief Cost functor for ceres optimization. Computes the error of by transforming a landmark point into image
 * coordinates
 *
 */
struct LandMarkToImageReprojectionFunctor {

    double u_observed, v_observed; /**< Image coordinates of observed point */
    double x_marker, y_marker;     /**< Landmark coordinates of map point */

    /**
     * @brief Constructor
     *
     * @param u_observed    u-coordinate of observed point
     * @param v_observed    v-coordinate of observed point
     * @param x_marker      x-coordinate of map point
     * @param y_marker      y-coordinate of map point
     */
    LandMarkToImageReprojectionFunctor(double u_observed, double v_observed, double x_marker, double y_marker)
            : u_observed(u_observed), v_observed(v_observed), x_marker(x_marker), y_marker(y_marker) {
    }

    template <typename T>
    /**
     * @brief   Computes the error based on input parameters
     *
     * @param landmark_pose   Pose of landmark
     * @param camera_pose   Pose of camera
     * @param camera_intrinsics Intrinsic camera parameters
     * @param residuals Residual array
     * @return bool Flag indicating success or failure
     */
    bool operator()(const T* const landmark_pose, const T* const camera_pose, const T* const camera_intrinsics,
                    T* residuals) const {

        // Transform landmark point to camera
        T u_marker = T(0.0);
        T v_marker = T(0.0);

        transformLandMarkToImage<T>(T(x_marker), T(y_marker), landmark_pose, camera_pose, camera_intrinsics, &u_marker,
                                    &v_marker);

        // Compute residual
        residuals[0] = u_marker - T(u_observed);
        residuals[1] = v_marker - T(v_observed);

        return true;
    }

    /**
     * @brief Factory to hide the construction of the CostFunction object from the client code.
     *
     * @param u_observed    u-coordinate of observed point
     * @param v_observed    v-coordinate of observed point
     * @param x_marker      x-coordinate of map point
     * @param y_marker      y-coordinate of map point
     * @return ceres::CostFunction Cost Function to be applied
     */
    static ceres::CostFunction* Create(const double u_observed, const double v_observed, const double x_marker,
                                       const double y_marker) {
        return (new ceres::AutoDiffCostFunction<LandMarkToImageReprojectionFunctor, 2, (int)POSE::N_PARAMS, (int)POSE::N_PARAMS, (int)INTRINSICS::N_PARAMS>(
            new LandMarkToImageReprojectionFunctor(u_observed, v_observed, x_marker, y_marker)));
    }
};

/**
 * @brief Cost functor for ceres optimization. Computes the error by transforming a world point into image coordinates
 *
 */
struct WorldToImageReprojectionFunctor {

    double u_observed, v_observed;       /**< Image coordinates of observed point */
    double x_marker, y_marker, z_marker; /**< World coordinates of map point */

    /**
     * @brief
     *
     * @param u_observed    u-coordinate of observed point
     * @param v_observed    v-coordinate of observed point
     * @param x_marker      x-coordinate of map point
     * @param y_marker      y-coordinate of map point
     * @param z_marker      z-coordinate of map point
     */
    WorldToImageReprojectionFunctor(double u_observed, double v_observed, double x_marker, double y_marker,
                                    double z_marker)
            : u_observed(u_observed), v_observed(v_observed), x_marker(x_marker), y_marker(y_marker),
              z_marker(z_marker) {
    }

    template <typename T>
    /**
     * @brief   Computes the error based on input parameters
     *
     * @param camera_pose   Pose of camera
     * @param camera_intrinsics Intrinsic camera parameters
     * @param residuals Residual array
     * @return bool Flag indicating success or failure
     */
    bool operator()(const T* const camera_pose, const T* const camera_intrinsics, T* residuals) const {

        // Transform landmark point to camera
        T u_marker = T(0.0);
        T v_marker = T(0.0);

        transformWorldToImg<T>(T(x_marker), T(y_marker), T(z_marker), camera_pose, camera_intrinsics, &u_marker,
                               &v_marker);

        // Compute residual
        residuals[0] = u_marker - T(u_observed);
        residuals[1] = v_marker - T(v_observed);

        return true;
    }

    /**
     * @brief Factory to hide the construction of the CostFunction object from the client code.
     *
     * @param u_observed    u-coordinate of observed point
     * @param v_observed    v-coordinate of observed point
     * @param x_marker      x-coordinate of map point
     * @param y_marker      y-coordinate of map point
     * @param z_marker      z-coordinate of map point
     * @return ceres::CostFunction Cost Function to be applied
     */
    static ceres::CostFunction* Create(const double u_observed, const double v_observed, const double x_marker,
                                       const double y_marker, const double z_marker) {
        return (new ceres::AutoDiffCostFunction<WorldToImageReprojectionFunctor, 2, (int)POSE::N_PARAMS,
                                                (int)INTRINSICS::N_PARAMS>(
            new WorldToImageReprojectionFunctor(u_observed, v_observed, x_marker, y_marker, z_marker)));
    }
};

} // namespace stargazer
