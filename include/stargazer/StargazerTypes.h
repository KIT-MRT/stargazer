//
// Created by bandera on 19.03.16.
//

#pragma once

#include <cmath>
#include <map>
#include <tuple>
#include <vector>

using std::pow;

enum struct POSE { X, Y, Z, Rx, Ry, Rz, N_PARAMS };

enum struct INTRINSICS { f, u0, v0, alpha, beta, theta, N_PARAMS };

enum struct POINT { X, Y, Z, N_PARAMS };

typedef std::array<double, (int)POINT::N_PARAMS> Point;
typedef std::array<double, (int)INTRINSICS::N_PARAMS> camera_params_t;
typedef std::array<double, (int)POSE::N_PARAMS> pose_t;

std::vector<Point> getLandmarkPoints(int ID); // Forward declaration

struct Landmark {
  ///--------------------------------------------------------------------------------------///
  /// The Landmarks are made similar to those from Hagisonic.
  /// ID of a landmark is coded see http://hagisonic.com/ for information on
  /// pattern
  ///--------------------------------------------------------------------------------------///

  /*  Numbering of corners and coordinate frame
   *  The origin of the landmark lies within Corner 1.
   *       ---> y'
   *  |   o   .   .   .
   *  |   .   .   .   .
   *  V   .   .   .   .
   *  x'  o   .   .   o
   *
   * The id of points
   *      0   4   8   12
   *      1   5   9   13
   *      2   6   10  14
   *      3   7   11  15
   */

  Landmark(){};

  Landmark(int ID) : id(ID), points(getLandmarkPoints(ID)){};

  int id;
  std::array<double, (int)POSE::N_PARAMS> pose;
  std::vector<Point> points;                    // TODO make this a map?
  static constexpr int kGridCount = 4;          // 4x4 Grid
  static constexpr double kGridDistance = 0.08; // 80mm = 8cm = 0.08 m
};

// Computes x^p assuring, to return an int
inline int pow(int x, int p) {
  if (p == 0)
    return 1;
  if (p == 1)
    return x;
  return x * pow(x, p - 1);
}

inline std::vector<Point> getLandmarkPoints(int ID) {
  std::vector<Point> points;

  // Add corner points
  Point pt1 = {0 * Landmark::kGridDistance, 0 * Landmark::kGridDistance, 0};
  Point pt3 = {3 * Landmark::kGridDistance, 0 * Landmark::kGridDistance, 0};
  Point pt15 = {3 * Landmark::kGridDistance, 3 * Landmark::kGridDistance, 0};
  points.push_back(pt1);
  points.push_back(pt3);
  points.push_back(pt15);

  /// Add ID points
  int col = 0;
  for (int y = 0; y < Landmark::kGridCount; y++) // For every column
  {
    /* StarLandmark IDs are coded:
    * the binary values ar coded:  x steps are binary shifts within 4 bit blocks
    *                              y steps are binary shifts of 4 bit blocks
    */

    // Modulo 16^(i+1) tells us how much this row contributed to the ID
    col = (ID % pow(pow(Landmark::kGridCount, 2), y + 1));
    col /= pow(pow(Landmark::kGridCount, 2), y);
    ID -= col;
    // Convert to binary
    for (int x = 0; x < Landmark::kGridCount; x++) { // For every row
      if (col % 2 != 0) { // Modulo 2 effectively converts the number to binary.
                          // If this returns 1, we have a point
        // Point found
        int id = y * Landmark::kGridCount + x;
        Point pt = {x * Landmark::kGridDistance, y * Landmark::kGridDistance,
                    0};
        points.push_back(pt);
      }
      col /= 2;
    }
  }
  return points;
}

typedef std::map<int, Landmark> landmark_map_t;
