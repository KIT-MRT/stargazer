#include "CalibIO.hpp"

#include <sstream>
#include <fstream>

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/algorithm/string/classification.hpp>

#include <opencv/cv.h>

using namespace std;

CalibIO::Camera::Camera() {
  S = 0;
  K = 0;
  D = 0;
  R = 0;
  T = 0;
  S_rect = 0;
  R_rect = 0;
  P_rect = 0;
}

CalibIO::Camera::Camera(const CalibIO::Camera& that) {
  S = 0;
  K = 0;
  D = 0;
  R = 0;
  T = 0;
  S_rect = 0;
  R_rect = 0;
  P_rect = 0;

  printf("that.S @ %x\n", that.S);
  if (that.S) {
    this->S = cvCreateMat(that.S->rows, that.S->cols, that.S->type);
    cvCopy(that.S, this->S);
  } else {
    this->S = 0;
  };
  if (that.K) {
    this->K = cvCreateMat(that.K->rows, that.K->cols, that.K->type);
    cvCopy(that.K, this->K);
  } else {
    this->K = 0;
  };
  if (that.D) {
    this->D = cvCreateMat(that.D->rows, that.D->cols, that.D->type);
    cvCopy(that.D, this->D);
  } else {
    this->D = 0;
  };
  if (that.R) {
    this->R = cvCreateMat(that.R->rows, that.R->cols, that.R->type);
    cvCopy(that.R, this->R);
  } else {
    this->R = 0;
  };
  if (that.T) {
    this->T = cvCreateMat(that.T->rows, that.T->cols, that.T->type);
    cvCopy(that.T, this->T);
  } else {
    this->T = 0;
  };
  if (that.S_rect) {
    this->S_rect =
        cvCreateMat(that.S_rect->rows, that.S_rect->cols, that.S_rect->type);
    cvCopy(that.S_rect, this->S_rect);
  } else {
    this->S_rect = 0;
  };
  if (that.R_rect) {
    this->R_rect =
        cvCreateMat(that.R_rect->rows, that.R_rect->cols, that.R_rect->type);
    cvCopy(that.R_rect, this->R_rect);
  } else {
    this->R_rect = 0;
  };
  if (that.P_rect) {
    this->P_rect =
        cvCreateMat(that.P_rect->rows, that.P_rect->cols, that.P_rect->type);
    cvCopy(that.P_rect, this->P_rect);
  } else {
    this->P_rect = 0;
  };
}

CalibIO::Camera& CalibIO::Camera::operator=(const CalibIO::Camera& that) {
  printf("assign that.S @ %x\n", that.S);
  if (S) {
    cvReleaseMat(&S);
    S = 0;
  }
  if (that.S) {
    this->S = cvCreateMat(that.S->rows, that.S->cols, that.S->type);
    cvCopy(that.S, this->S);
  }
  if (K) {
    cvReleaseMat(&K);
    K = 0;
  }
  if (that.K) {
    this->K = cvCreateMat(that.K->rows, that.K->cols, that.K->type);
    cvCopy(that.K, this->K);
  }
  if (D) {
    cvReleaseMat(&D);
    D = 0;
  }
  if (that.D) {
    this->D = cvCreateMat(that.D->rows, that.D->cols, that.D->type);
    cvCopy(that.D, this->D);
  }
  if (R) {
    cvReleaseMat(&R);
    R = 0;
  }
  if (that.R) {
    this->R = cvCreateMat(that.R->rows, that.R->cols, that.R->type);
    cvCopy(that.R, this->R);
  }
  if (T) {
    cvReleaseMat(&T);
    T = 0;
  }
  if (that.T) {
    this->T = cvCreateMat(that.T->rows, that.T->cols, that.T->type);
    cvCopy(that.T, this->T);
  }
  if (S_rect) {
    cvReleaseMat(&S_rect);
    S_rect = 0;
  }
  if (that.S_rect) {
    this->S_rect =
        cvCreateMat(that.S_rect->rows, that.S_rect->cols, that.S_rect->type);
    cvCopy(that.S_rect, this->S_rect);
  }
  if (R_rect) {
    cvReleaseMat(&R_rect);
    R_rect = 0;
  }
  if (that.R_rect) {
    this->R_rect =
        cvCreateMat(that.R_rect->rows, that.R_rect->cols, that.R_rect->type);
    cvCopy(that.R_rect, this->R_rect);
  }
  if (P_rect) {
    cvReleaseMat(&P_rect);
    P_rect = 0;
  }
  if (that.P_rect) {
    this->P_rect =
        cvCreateMat(that.P_rect->rows, that.P_rect->cols, that.P_rect->type);
    cvCopy(that.P_rect, this->P_rect);
  }

  return *this;
}

CalibIO::Camera::~Camera() {
  if (S) {
    std::cout << "release " << S->rows << " x " << S->cols << "\n";
    cvReleaseMat(&S);
    S = 0;
  }
  if (K) {
    cvReleaseMat(&K);
    K = 0;
  }
  if (D) {
    cvReleaseMat(&D);
    D = 0;
  }
  if (R) {
    cvReleaseMat(&R);
    R = 0;
  }
  if (T) {
    cvReleaseMat(&T);
    T = 0;
  }
  if (S_rect) {
    cvReleaseMat(&S_rect);
    S_rect = 0;
  }
  if (R_rect) {
    cvReleaseMat(&R_rect);
    R_rect = 0;
  }
  if (P_rect) {
    cvReleaseMat(&P_rect);
    P_rect = 0;
  }
}

CalibIO::CalibIO() { corner_dist = 0; }

CalibIO::~CalibIO() {}

template <typename _Tp, int _rows, int _cols, int _options, int _maxRows,
          int _maxCols>
void eigen2cvMat(
    const Eigen::Matrix<_Tp, _rows, _cols, _options, _maxRows, _maxCols>& src,
    CvMat*& dstCvMat) {
  dstCvMat = cvCreateMat(src.rows(), src.cols(), CV_32FC1);

  for (int v = 0; v < src.rows(); ++v) {
    for (int u = 0; u < src.cols(); ++u) {
      cvmSet(dstCvMat, v, u, src(v, u));
    }
  }
}

// read calibration text file
bool CalibIO::readCalibFromFile(string calib_file_name) {
  std::cout << __PRETTY_FUNCTION__ << " calib_file_name: " << calib_file_name
            << std::endl;

  // binary file optimization
  if (calib_file_name.substr(calib_file_name.size() - 3, 3) == "bin") {
    std::cout << "Reading binary calibration file... " << std::endl;
    std::vector<UndistortedCamera> undistortedCamera;
    auto success = cerealRead(
        calib_file_name,
        [&undistortedCamera](cereal::PortableBinaryInputArchive& archive) {
          archive(undistortedCamera);
        });
    if (!success) return false;

    cameras.resize(undistortedCamera.size());
    cus_TS.resize(undistortedCamera.size());
    cvs_TS.resize(undistortedCamera.size());

    for (int i = 0; i < undistortedCamera.size(); ++i) {
      // convert data
      const auto& cam = undistortedCamera[i];

      std::cout << "Loading config of camera " << i << "..." << std::endl;
      eigen2cvMat(cam.R_rect, cameras[i].R_rect);
      eigen2cvMat(cam.T_rect, cameras[i].T);
      Eigen::RowVector2d S_rect_T = cam.S_rect.transpose();
      eigen2cvMat(S_rect_T, cameras[i].S_rect);
      eigen2cvMat(cam.P_rect, cameras[i].P_rect);
      eigen2cvMat(cam.K_rect, cameras[i].K);
      eigen2cvMat(cam.LUT_U, cus_TS[i]);
      eigen2cvMat(cam.LUT_V, cvs_TS[i]);
    }

    std::cout << "done." << std::endl;

    // showCalibrationParameters(false);
    return true;
  }

  bool is_TS_style = false;

  {  // peek into file to see if it is a Tobi Strauss file

    std::ifstream file(calib_file_name);
    std::string first_line;
    getline(file, first_line);

    std::cout << "first_line == " << first_line << "\n";

    if (first_line.substr(0, 2) == "TS") {
      is_TS_style = true;
      std::cout << "is TS style.\n";
    }
    file.close();
  }

  {
    // open calibration file
    FILE* calib_file = fopen(calib_file_name.c_str(), "r");
    if (calib_file == NULL) return false;

    // read time and dist
    bool success = true;

    if (is_TS_style) {
      std::string dummy = readString(calib_file, "TS", success);
      assert(success);
    }

    calib_time = readString(calib_file, "calib_time:", success);
    corner_dist = readValue(calib_file, "corner_dist:", success);

    // error reading time and dist
    if (!success) {
      fclose(calib_file);
      return success;
    }

    // read all cameras

    // cameras.reserve( 10 );
    for (uint32_t i = 0; i < 100; i++) {
      Camera camera;
      if (i > 0) printf("pre: cameras[0].S @ %x\n", cameras[0].S);

      printf("pre: camera.S @ %x\n", camera.S);

      if (!is_TS_style) {
        camera.S = readMatrix(calib_file, "S", i, 1, 2, success);
        if (!success) break;
        camera.K = readMatrix(calib_file, "K", i, 3, 3, success);
        if (!success) break;
        camera.D = readMatrix(calib_file, "D", i, 1, 5, success);
        if (!success) break;
        camera.R = readMatrix(calib_file, "R", i, 3, 3, success);
        if (!success) break;
        camera.T = readMatrix(calib_file, "T", i, 3, 1, success);
        if (!success) break;
        camera.K_rect = readMatrix(calib_file, "K_rect", i, 3, 3, success);
        if (!success) break;
        camera.S_rect = readMatrix(calib_file, "S_rect", i, 1, 2, success);
        if (!success) break;
        camera.R_rect = readMatrix(calib_file, "R_rect", i, 3, 3, success);
        if (!success) break;
        camera.P_rect = readMatrix(calib_file, "P_rect", i, 3, 4, success);
        if (!success) break;
      } else {
        // camera.S      = readMatrix(calib_file,"S_input",     i,1,2,success);
        camera.R_rect = readMatrix(calib_file, "R", i, 3, 3, success);
        if (!success) break;
        camera.T = readMatrix(calib_file, "T", i, 3, 1, success);
        if (!success) break;
        camera.S_rect = readMatrix(calib_file, "S_rect", i, 1, 2, success);
        if (!success) break;
        camera.P_rect = readMatrix(calib_file, "P_rect", i, 3, 4, success);
        if (!success) break;
        camera.K = readMatrix(calib_file, "K_rect", i, 3, 3, success);
        if (!success) break;
        if (!success) break;

        std::cout << camera.S_rect->rows << " " << camera.S_rect->cols << "\n";

        int w = cvmGet(camera.S_rect, 0, 0) + 0.5;
        int h = cvmGet(camera.S_rect, 0, 1) + 0.5;

        std::cout << "w h: " << w << " " << h << "\n";

        cus_TS.push_back(readMatrix(calib_file, "CU", i, h, w, success));
        if (!success) break;
        cvs_TS.push_back(readMatrix(calib_file, "CV", i, h, w, success));
        if (!success) break;
      }

      if (!success)
        break;
      else {
        printf("post: camera.S@%x\n", camera.S);
        cameras.push_back(camera);
      }
    }

    // close file
    fclose(calib_file);

    // success, if at least one camera could be read from this file
    if (cameras.size() > 0)
      return true;
    else
      return false;
  }
}

// displays all calibration matrices
void CalibIO::showCalibrationParameters(bool compact) {
  if (cameras.size() == 0) {
    cout << "No calibration parameters loaded." << endl;
    return;
  }
  if (compact) {
    cout << endl;
    cout << "Calibration parameters:" << endl;
    cout << "Calibration time: " << calib_time << endl;
    cout << "Corner distance:  " << corner_dist << " meters" << endl;
    for (uint32_t i = 0; i < cameras.size(); i++) {
      cout << "Camera " << i << ": ";
      cout << cvmGet(cameras[i].S, 0, 0) << "x" << cvmGet(cameras[i].S, 0, 1);
      cout << " => " << cvmGet(cameras[i].S_rect, 0, 0) << "x"
           << cvmGet(cameras[i].S_rect, 0, 1);
      cout << ", f: " << cvmGet(cameras[i].P_rect, 0, 0);
      cout << ", x: " << fabs(cvmGet(cameras[i].P_rect, 0, 3) /
                              cvmGet(cameras[i].P_rect, 0, 0));
      cout << endl;
    }
    cout << endl;
  } else {
    cout << endl
         << "========================" << endl;
    cout << "Calibration parameters:";
    cout << endl
         << "========================" << endl
         << endl;
    cout << "Calibration time: " << calib_time << endl;
    cout << "Corner distance: " << corner_dist << endl;
    for (uint32_t i = 0; i < cameras.size(); i++) {
      showCvMat(cameras[i].S, "S", i);
      showCvMat(cameras[i].K, "K", i);
      showCvMat(cameras[i].D, "D", i);
      showCvMat(cameras[i].R, "R", i);
      showCvMat(cameras[i].T, "T", i);
      showCvMat(cameras[i].S_rect, "S_rect", i);
      showCvMat(cameras[i].R_rect, "R_rect", i);
      showCvMat(cameras[i].P_rect, "P_rect", i);
    }
    cout << endl;
  }
}

void CalibIO::computeLUT(int camera_index, bool unwarp_only, cv::Mat& cu_out,
                         cv::Mat& cv_out) {
  CvMat* cu;
  CvMat* cv;
  if (cus_TS.size() > 0)  // this is a TS style calibration
  {
    cu = cus_TS.at(camera_index);
    cv = cvs_TS.at(camera_index);
  } else {
    // rectification maps
    Camera _cam = cameras.at(camera_index);

    cu = cvCreateMat(cvmGet(_cam.S_rect, 0, 1), cvmGet(_cam.S_rect, 0, 0),
                     CV_32F);
    cv = cvCreateMat(cvmGet(_cam.S_rect, 0, 1), cvmGet(_cam.S_rect, 0, 0),
                     CV_32F);

    if (unwarp_only)
      cvInitUndistortRectifyMap(_cam.K, _cam.D, 0, _cam.P_rect, cu, cv);
    else
      cvInitUndistortRectifyMap(_cam.K, _cam.D, _cam.R_rect, _cam.P_rect, cu,
                                cv);
  }
  cu_out = cv::Mat(cu, true);
  cv_out = cv::Mat(cv, true);
}

/////////////////////////////////////////////////////////////////////////////////
///////////////////////////// PRIVATE FUNCTIONS
////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

// displays an opencv matrix
void CalibIO::showCvMat(CvMat* m, string matrix_name, uint32_t cam) {
  // show full matrix name
  char matrix_name_full[1024];
  sprintf(matrix_name_full, "%s_%02d:", matrix_name.c_str(), cam);
  cout << "Matrix \"" << matrix_name_full << "\":" << endl;

  // show matrix
  if (m != 0) {
    CvSize s = cvGetSize(m);
    for (int32_t i = 0; i < s.height; i++) {
      for (int32_t j = 0; j < s.width; j++) {
        cout << setw(10) << cvmGet(m, i, j) << " ";
      }
      cout << endl;
    }
  } else {
    cout << " --- undefined --- " << endl;
  }
  cout << endl;
}

// splits a string into its elements
vector<string> CalibIO::splitLine(string line) {
  vector<string> result;
  boost::trim(line);
  boost::split(result, line, boost::algorithm::is_any_of(" \n\r\0\t,;"),
               boost::token_compress_on);
  return result;
}

// read a string from 'calib_file'
string CalibIO::readString(FILE* calib_file, const char* string_name,
                           bool& success) {
  // init result
  string str = "";

  // go to beginning of file
  rewind(calib_file);

  std::vector<char> linebuffer(500000000);

  // buffer for lines of file
  char* line = linebuffer.data();

  // for all lines of calib file do
  while (calib_file != NULL &&
         fgets(line, linebuffer.size(), calib_file) != NULL) {
    // split current line into elements
    vector<string> line_vector = splitLine(line);

    // if matrix_name is first passage of this line => return subsequent words
    if (!line_vector[0].compare(string_name)) {
      for (uint32_t i = 1; i < line_vector.size(); i++)
        str = str + line_vector[i] + " ";
      return str;
    }
  }

  // failure
  success = false;
  return str;
}

// read a single value from 'calib_file'
float CalibIO::readValue(FILE* calib_file, const char* value_name,
                         bool& success) {
  // go to beginning of file
  rewind(calib_file);

  // buffer for lines of file
  char line[20000];

  // for all lines of calib file do
  while (calib_file != NULL && fgets(line, sizeof(line), calib_file) != NULL) {
    // split current line into elements
    vector<string> line_vector = splitLine(line);

    // if matrix_name is first passage of this line
    if (!line_vector[0].compare(value_name)) {
      // check, that we have only one value
      if (line_vector.size() - 1 != 1) {
        cout << "ERROR Number of elements in " << value_name << ": "
             << line_vector.size() - 1 << "!=1" << endl;
        success = false;
        return 0;
      }

      // set value
      float val;
      stringstream sst;
      sst << line_vector[1];
      sst >> val;
      return val;
    }
  }

  // failure
  success = false;
  return 0.0;
}

// read an opencv matrix from 'calib_file'
CvMat* CalibIO::readMatrix(FILE* calib_file, const char* matrix_name,
                           uint32_t cam, uint32_t m, uint32_t n,
                           bool& success) {
  // compose full matrix name
  char matrix_name_full[1024];
  sprintf(matrix_name_full, "%s_%02d:", matrix_name, cam);

  //    std::cout << "want " << matrix_name_full << "\n";

  // go to beginning of file
  rewind(calib_file);

  // buffer for lines of file
  std::vector<char> linebuffer(500000000);
  char* line = linebuffer.data();

  // for all lines of calib file do
  while (calib_file != NULL &&
         fgets(line, linebuffer.size(), calib_file) != NULL) {
    // split current line into elements
    vector<string> line_vector = splitLine(line);

    // if matrix_name_full is first passage of this line
    if (!line_vector[0].compare(matrix_name_full)) {
      //            std::cout << "name MATCH, wanted " << matrix_name_full << "
      //            got " << line_vector[0] << "\n";

      // check for right numer of matrix elements
      if (line_vector.size() - 1 != m * n) {
        cout << "ERROR Number of elements in " << matrix_name_full << ": "
             << line_vector.size() - 1 << "!=" << m * n << endl;
        success = false;
        return NULL;
      }

      // create opencv matrix
      CvMat* M = cvCreateMat(m, n, CV_32FC1);
      float val;

      // set matrix elements
      uint32_t k = 1;
      for (uint32_t i = 0; i < m; i++) {
        for (uint32_t j = 0; j < n; j++) {
          stringstream sst;
          sst << line_vector[k++];
          sst >> val;
          cvmSet(M, i, j, val);
        }
      }
      return M;
    }
    //        else
    //            std::cout << "name mismatch, want " << matrix_name_full << "
    //            got " << line_vector[0] << "\n";
  }

  // failure
  success = false;
  return NULL;
}
