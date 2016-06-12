#pragma once

#include <Eigen/Dense>
#include <cereal/archives/binary.hpp>
#include <cereal/cereal.hpp>

namespace cereal {

template <class Archive, class _Scalar, int _Rows, int _Cols, int _Options,
          int _MaxRows, int _MaxCols>
inline typename std::enable_if<
    traits::is_output_serializable<BinaryData<_Scalar>, Archive>::value,
    void>::type
save(Archive &ar, Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows,
                                _MaxCols> const &m) {
  int64_t rows = (int64_t)m.rows();
  int64_t cols = (int64_t)m.cols();
  ar(rows);
  ar(cols);
  ar(binary_data(m.data(), rows * cols * sizeof(_Scalar)));
}

template <class Archive, class _Scalar, int _Rows, int _Cols, int _Options,
          int _MaxRows, int _MaxCols>
inline typename std::enable_if<
    traits::is_input_serializable<BinaryData<_Scalar>, Archive>::value,
    void>::type
load(Archive &ar,
     Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> &m) {
  int64_t rows = 0;
  int64_t cols = 0;
  ar(rows);
  ar(cols);

  m.resize(rows, cols);

  ar(binary_data(m.data(),
                 static_cast<std::size_t>(rows * cols * sizeof(_Scalar))));
}

template <typename Archive, typename T, int Dim, int Mode>
inline void serialize(Archive &archive, Eigen::Transform<T, Dim, Mode> &c) {
  archive(c.matrix());
}

template <typename Archive, typename T, int _Options>
inline void serialize(Archive &archive, Eigen::Quaternion<T, _Options> &q) {
  archive(q);
}
}
