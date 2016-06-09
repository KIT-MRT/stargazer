// utility function for reading a calibration file
// Author: Andreas Geiger, 2011

#ifndef __CALIB_IO_H__
#define __CALIB_IO_H__

#include <iomanip>
#include <iostream>
#include <stdint.h>
#include <stdio.h>
#include <string>
#include <vector>

class CvMat; // forward declaration to avoid OpenCV compile time dependency
             // (jz)
namespace cv {
class Mat; // forward declaration to avoid OpenCV compile time dependency (jz)
}
#include <Eigen/Dense>

#include "../cereal/types/Eigen.h" //! This file originaly comes from MRT/libPersistantMap !!!
#include <boost/filesystem.hpp>
#include <cereal/archives/portable_binary.hpp>
#include <cereal/cereal.hpp>
#include <cereal/types/vector.hpp>
#include <fstream>

template <typename Func>
bool cerealRead(const boost::filesystem::path &path, Func &&func) {
  std::ifstream file(path.generic_string(), std::ios::binary);
  if (file.is_open() == false) {
    return false;
  }

  cereal::PortableBinaryInputArchive archive(file);
  func(archive);

  return true;
}

struct UndistortedCamera {
  Eigen::Matrix4d A_rect;
  Eigen::Matrix3d R_rect;
  Eigen::Vector3d T_rect;
  Eigen::Vector2d S_rect;
  Eigen::Matrix3d K_rect;
  Eigen::Matrix<double, 3, 4> P_rect;
  Eigen::MatrixXd LUT_U;
  Eigen::MatrixXd LUT_V;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

namespace cereal {

template <typename Archive>
void serialize(Archive &archive, UndistortedCamera &c) {
  archive(c.A_rect, c.R_rect, c.T_rect, c.S_rect, c.K_rect, c.P_rect, c.LUT_U,
          c.LUT_V);
}
}

namespace cereal {

//  template <class Archive, class _Scalar, int _Rows, int _Cols, int _Options,
//  int _MaxRows, int _MaxCols> inline
//    typename
//    std::enable_if<traits::is_output_serializable<BinaryData<_Scalar>,
//    Archive>::value, void>::type
//    save(Archive & ar, Eigen::Matrix<_Scalar, _Rows, _Cols, _Options,
//    _MaxRows, _MaxCols> const & m) {
//    int64_t rows = (int64_t)m.rows();
//    int64_t cols = (int64_t)m.cols();
//    ar(rows);
//    ar(cols);
//    ar(binary_data(m.data(), rows * cols * sizeof(_Scalar)));
//  }

//  template <class Archive, class _Scalar, int _Rows, int _Cols, int _Options,
//  int _MaxRows, int _MaxCols> inline
//    typename std::enable_if<traits::is_input_serializable<BinaryData<_Scalar>,
//    Archive>::value, void>::type
//    load(Archive & ar, Eigen::Matrix<_Scalar, _Rows, _Cols, _Options,
//    _MaxRows, _MaxCols> & m) {
//    int64_t rows = 0;
//    int64_t cols = 0;
//    ar(rows);
//    ar(cols);

//    m.resize(rows, cols);

//    ar(binary_data(m.data(), static_cast<std::size_t>(rows * cols *
//    sizeof(_Scalar))));
//  }

template <typename Archive>
inline void save(Archive &archive, const Eigen::Affine3d &c) {
  Eigen::Matrix4d A = c.matrix();
  archive(A);
}

template <typename Archive>
inline void load(Archive &archive, Eigen::Affine3d &c) {
  Eigen::Matrix4d A;
  archive(A);
  c.matrix() = A;
}
}

class CalibIO {
public:
  // calibration parameters of a single camera
  struct Camera {
    CvMat *S;      // image size before rectification
    CvMat *K;      // calibration matrix before rectification
    CvMat *D;      // distortion parameters before rectification
    CvMat *R;      // extrinsic rotation before rectification
    CvMat *T;      // extrinsic translation before rectification
    CvMat *S_rect; // image size after rectification
    CvMat *R_rect; // rectifying rotation
    CvMat *K_rect; // projection matrix after rectification
    CvMat *P_rect; // projection matrix after rectification
    Camera();
    Camera(const Camera &that);
    Camera &operator=(const Camera &that);
    ~Camera();
  };

  CalibIO();
  ~CalibIO();
  bool readCalibFromFile(std::string calib_file_name);
  void showCalibrationParameters(bool compact = true);
  // Warning, returns pointers to internal data. If you want the data to
  // life longer than this object, you should copy it.
  void computeLUT(int camera_index, bool unwarp_only, cv::Mat &cu_out,
                  cv::Mat &cv_out);

  std::string calib_time;      // calibration time
  float corner_dist;           // corner dist
  std::vector<Camera> cameras; // camera calibration

private:
  void showCvMat(CvMat *m, std::string matrix_name, uint32_t cam);
  std::vector<std::string> splitLine(std::string line);
  std::string readString(FILE *calib_file, const char *string_name,
                         bool &success);
  float readValue(FILE *calib_file, const char *value_name, bool &success);
  CvMat *readMatrix(FILE *calib_file, const char *matrix_name, uint32_t cam,
                    uint32_t m, uint32_t n, bool &success);

  // LUTs read directly from a TS style file
  std::vector<CvMat *> cus_TS;
  std::vector<CvMat *> cvs_TS;
};

#endif
