#pragma once

// Ceres includes
#include "../CoordinateTransformations.h"
#include "../StargazerTypes.h"
#include "ceres/ceres.h"

namespace stargazer {

struct LM2ImgReprojectionFunctor {

    double u_observed, v_observed;
    double x_marker, y_marker;

    LM2ImgReprojectionFunctor(double u_observed, double v_observed, double x_marker, double y_marker)
            : u_observed(u_observed), v_observed(v_observed), x_marker(x_marker), y_marker(y_marker) {
    }

    template <typename T>
    bool operator()(const T* const lm_pose, const T* const camera_pose, const T* const camera_intrinsics,
                    T* residuals) const {

        // Transform landmark point to camera
        T u_marker, v_marker;
        T x_marker_tmp = T(x_marker);
        T y_marker_tmp = T(y_marker);

        transformLM2Img<T>(&x_marker_tmp, &y_marker_tmp, lm_pose, camera_pose, camera_intrinsics, &u_marker, &v_marker);

        // Compute residual
        residuals[0] = u_marker - T(u_observed);
        residuals[1] = v_marker - T(v_observed);

        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(const double u_observed, const double v_observed, const double x_marker,
                                       const double y_marker) {
        return (new ceres::AutoDiffCostFunction<LM2ImgReprojectionFunctor, 2, 6, 6, 6>(
            new LM2ImgReprojectionFunctor(u_observed, v_observed, x_marker, y_marker)));
    }
};

struct World2ImgReprojectionFunctor {

    double u_observed, v_observed;
    double x_marker, y_marker, z_marker;

    World2ImgReprojectionFunctor(double u_observed, double v_observed, double x_marker, double y_marker,
                                 double z_marker)
            : u_observed(u_observed), v_observed(v_observed), x_marker(x_marker), y_marker(y_marker),
              z_marker(z_marker) {
    }

    template <typename T>
    bool operator()(const T* const camera_pose, const T* const camera_intrinsics, T* residuals) const {

        // Transform landmark point to camera
        T u_marker, v_marker;
        T x_marker_tmp = T(x_marker);
        T y_marker_tmp = T(y_marker);
        T z_marker_tmp = T(z_marker);
        transformWorld2Img<T>(&x_marker_tmp, &y_marker_tmp, &z_marker_tmp, camera_pose, camera_intrinsics, &u_marker,
                              &v_marker);

        // Compute residual
        residuals[0] = u_marker - T(u_observed);
        residuals[1] = v_marker - T(v_observed);

        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(const double u_observed, const double v_observed, const double x_marker,
                                       const double y_marker, const double z_marker) {
        return (new ceres::AutoDiffCostFunction<World2ImgReprojectionFunctor, 2, 6, 6>(
            new World2ImgReprojectionFunctor(u_observed, v_observed, x_marker, y_marker, z_marker)));
    }
};

} // namespace stargazer