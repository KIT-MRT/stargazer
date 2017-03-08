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

#include <cmath>
#include <map>
#include <tuple>
#include <vector>

namespace stargazer {

using std::pow;

/**
 * @brief Definition of the six pose parameters. The rotation angles are given as rodriguez angles
 *
 */
enum struct POSE { X, Y, Z, Rx, Ry, Rz, N_PARAMS };

/**
 * @brief Definition of the intrinsic camera parameters
 *
 */
enum struct INTRINSICS { fu, fv, u0, v0, N_PARAMS };

/**
 * @brief   Definition of the three position parmaters of a point
 *
 */
enum struct POINT { X, Y, Z, N_PARAMS };

/**
 * @brief   A point is a 3D translation-only position. See ::POINT for the indexing scheme.
 *
 */
typedef std::array<double, (int)POINT::N_PARAMS> Point;
/**
 * @brief   This object hold the camera parameters. See ::INTRINSICS for the indexing scheme.
 */
typedef std::array<double, (int)INTRINSICS::N_PARAMS> camera_params_t;
/**
 * @brief   This object hold the parameters of a translation and orientation pose. See ::POSE for the indexing scheme.
 *
 */
typedef std::array<double, (int)POSE::N_PARAMS> pose_t;

/**
 * @brief Point generator function for a given ID.
 *
 * @param ID    Landmark ID
 * @return std::vector<Point> List of points in landmark coordinates. The first three are the three corner points.
 */
std::vector<Point> getLandmarkPoints(int ID); // Forward declaration

/**
 * @brief This class resembles a map landmark. After construction with the id, the landmark holds its marker points in
 * landmark coordinates.
 *
 */
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

    /**
     * @brief Constructor
     *
     * @param ID
     */
    Landmark(int ID) : id(ID), points(getLandmarkPoints(ID)){};

    int id;                                                                    /**< The landmarks id */
    std::array<double, (int)POSE::N_PARAMS> pose = {{0., 0., 0., 0., 0., 0.}}; /**< The landmarks pose */
    std::vector<Point> points;           /**< Vector of landmark points. The first three are the corners */
    static constexpr int kGridCount = 4; /**< Defines how many rows and columns the landmark has */
    static constexpr double kGridDistance =
        0.08; /**< Defines the distance between two landmark LEDs in meters. This is important for
                 esimating the scale. */
};

/**
 * @brief Computes x^p assuring, to return an int
 *
 * @param x Base
 * @param p Exponent
 * @return int Result
 */
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
            if (col % 2 != 0) {                          // Modulo 2 effectively converts the number to binary.
                // If this returns 1, we have a point
                // Point found
                int id = y * Landmark::kGridCount + x;
                Point pt = {x * Landmark::kGridDistance, y * Landmark::kGridDistance, 0};
                points.push_back(pt);
            }
            col /= 2;
        }
    }
    return points;
}

/**
 * @brief This class resembles the map representation. It hold a map of all known landmarks.
 *
 */
typedef std::map<int, Landmark> landmark_map_t;

} // namespace stargazer
