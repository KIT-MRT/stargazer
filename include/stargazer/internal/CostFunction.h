#pragma once

// Ceres includes
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "../StargazerTypes.h"


template<typename T>
void transformPoint(const T *const x_in,
                    const T *const y_in,
                    const T *const lm_pose,
                    const T *const camera_pose,
                    const T *const camera_intrinsics,
                    T *const x_out,
                    T *const y_out) {
  // Create point in Landmark coordinates
  T p_lm[3];
  p_lm[0] = *x_in;
  p_lm[1] = *y_in;
  p_lm[2] = T(0.);

  // Transform point from Landmark to world coordinates
  T p_w[3];
  T angleAxisLM[3];
  angleAxisLM[0] = lm_pose[(int) POSE::Rx];
  angleAxisLM[1] = lm_pose[(int) POSE::Ry];
  angleAxisLM[2] = lm_pose[(int) POSE::Rz];

  ceres::AngleAxisRotatePoint(&angleAxisLM[0], &p_lm[0], &p_w[0]);

  // lm_pose[0,1,2] are the translation.
  p_w[0] += lm_pose[(int) POSE::X];
  p_w[1] += lm_pose[(int) POSE::Y];
  p_w[2] += lm_pose[(int) POSE::Z];


  // Transform point to camera coordinates
  T p_c[3];

  // This time we go from world -> cam
  // camera_pose[3,4,5] are the translation.
  p_c[0] = p_w[0] - camera_pose[(int) POSE::X];
  p_c[1] = p_w[1] - camera_pose[(int) POSE::Y];
  p_c[2] = p_w[2] - camera_pose[(int) POSE::Z];
  T angleAxisCam[3];
  angleAxisCam[0] = -camera_pose[(int) POSE::Rx];
  angleAxisCam[1] = -camera_pose[(int) POSE::Ry];
  angleAxisCam[2] = -camera_pose[(int) POSE::Rz];
  ceres::AngleAxisRotatePoint(&angleAxisCam[0], &p_c[0], &p_c[0]);

  /* Transform point to image coordinates
  intrinsics[0] is focal length
  intrinsics[1] is x0
  intrinsics[2] is y0
  intrinsics[3] is alpha
  intrinsics[4] is beta
  intrinsics[5] is theta --> assumed 90° */
  T p_i[3];
  p_i[0] = camera_intrinsics[(int) INTRINSICS::f] * camera_intrinsics[(int) INTRINSICS::alpha] * p_c[0] +
      T(0) * p_c[1] +
      camera_intrinsics[(int) INTRINSICS::u0] * p_c[2];
  p_i[1] = T(0) * p_c[0] +
      camera_intrinsics[(int) INTRINSICS::f] * camera_intrinsics[(int) INTRINSICS::beta] * p_c[1] / T(1) +
      camera_intrinsics[(int) INTRINSICS::v0] * p_c[2];
  p_i[2] = p_c[2];
  if (p_i[2] == T(0)) {
    std::cout << "WARNING; Attempt to divide by 0!" << std::endl;
    return;
  }
  p_i[0] = p_i[0] / p_i[2];
  p_i[1] = p_i[1] / p_i[2];
  p_i[2] = p_i[2] / p_i[2];

  *x_out = p_i[0];
  *y_out = p_i[1];
}

struct ReprojectionFunctor {

  double u_observed, v_observed;
  double x_marker, y_marker;

  ReprojectionFunctor(double u_observed, double v_observed, double x_marker, double y_marker)
      : u_observed(u_observed), v_observed(v_observed),
        x_marker(x_marker), y_marker(y_marker) {

  }

  template<typename T>
  bool operator()(const T *const lm_pose,
                  const T *const camera_pose,
                  const T *const camera_intrinsics,
                  T *residuals) const {

    // Transform landmark point to camera
    T u_marker, v_marker;
    T x_marker_tmp = T(x_marker);
    T y_marker_tmp = T(y_marker);
    transformPoint<T>(&x_marker_tmp, &y_marker_tmp, lm_pose, camera_pose, camera_intrinsics, &u_marker, &v_marker);

    // Compute residual
    residuals[0] = u_marker - T(u_observed);
    residuals[1] = v_marker - T(v_observed);

    return true;
  }


  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction *Create(const double u_observed,
                                     const double v_observed,
                                     const double x_marker,
                                     const double y_marker) {
    return (new ceres::AutoDiffCostFunction<ReprojectionFunctor, 2, 6, 6, 6>(
        new ReprojectionFunctor(u_observed, v_observed, x_marker, y_marker)));
  }
};

struct DistanceFunctor {

  DistanceFunctor() { };

  template<typename T>
  bool operator()(const T *const pose1,
                  const T *const pose2,
                  T *residuals) const {
    T dx = pose2[(int) POSE::X] - pose1[(int) POSE::X];
    T dy = pose2[(int) POSE::Y] - pose1[(int) POSE::Y];
    T dist = dx * dx + dy * dy;

    // Compute residual
    residuals[0] = dist;

    return true;
  }

  static ceres::CostFunction *Create() {
    return (new ceres::AutoDiffCostFunction<DistanceFunctor, 1, 6, 6>(new DistanceFunctor()));
  }
};

struct OrientationFunctor {

  OrientationFunctor() { };

  template<typename T>
  bool operator()(const T *const pose,
                  T *residuals) const {

    // Weight, by how much it differs from perfect z-Orientation
    // We do this by norming the pose's orientation vector (to get invariant to the amount of rotation
    // and projecting it onto [0,0,1], which is effectively taking the z component of the vector.
    T length = ceres::sqrt(
        pose[(int) POSE::Rx] * pose[(int) POSE::Rx] +
            pose[(int) POSE::Ry] * pose[(int) POSE::Ry] +
            pose[(int) POSE::Rz] * pose[(int) POSE::Rz]
    );
    if (length == T(0)) {
      // TODO maybe thats not perfect, as this will favour unrotated poses...
      residuals[0] = T(0);
    } else {
      residuals[0] = T(1) - pose[(int) POSE::Rz] / length;
    }
    return true;
  }

  static ceres::CostFunction *Create() {
    return (new ceres::AutoDiffCostFunction<OrientationFunctor, 1, 6>(new OrientationFunctor()));
  }
};
