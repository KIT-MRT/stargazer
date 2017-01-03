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

#include "LandmarkFinder.h"

using namespace std;
using namespace stargazer;

///--------------------------------------------------------------------------------------///
/// Default constructor
///--------------------------------------------------------------------------------------///
LandmarkFinder::LandmarkFinder(std::string cfgfile) {

    /// set parameters
    threshold = 20;
    tight_filter_size = 3;
    wide_filter_size = 11;

    maxRadiusForPixelCluster = 3;
    minPixelForCluster = 1;
    maxPixelForCluster = 1000;
    maxRadiusForCluster = 40;
    minPointsPerLandmark = 5;
    maxPointsPerLandmark = 9;

    /// Read in Landmark ids
    camera_params_t dummy;
    landmark_map_t landmarks;
    readConfig(cfgfile, dummy, landmarks);
    for (auto& el : landmarks)
        valid_ids_.push_back(el.first);
}

///--------------------------------------------------------------------------------------///
/// default destructor
///
///--------------------------------------------------------------------------------------///
LandmarkFinder::~LandmarkFinder() {
}

///--------------------------------------------------------------------------------------///
/// FindMarker processing method
/// Handles the complete processing
///--------------------------------------------------------------------------------------///
int LandmarkFinder::DetectLandmarks(const cv::Mat& img, std::vector<ImgLandmark>& detected_landmarks) {
    clusteredPixels_.clear();
    clusteredPoints_.clear();

    /// check if input is valid
    // Explanation for CV_ Codes :
    // CV_[The number of bits per item][Signed or Unsigned][Type Prefix]C[The channel number]
    img.assignTo(grayImage_, CV_8UC1); // 8bit unsigned with 3 channels
    if (!grayImage_.data) {            /// otherwise: return with error
        std::cerr << "Input data is invalid" << std::endl;
        return -1;
    }
    detected_landmarks.clear();

    /// convert color to gray for further processing
    cvtColor(grayImage_, rawImage_, CV_GRAY2BGR);

    /// smooth image
    FilterImage(grayImage_, filteredImage_);
    /// This method finds bright points in image
    /// returns vector of center points of pixel groups
    clusteredPixels_ = FindPoints(filteredImage_);

    /// cluster points to groups which could be landmarks
    /// returns a vector of clusters which themselves are vectors of points
    FindClusters(clusteredPixels_, clusteredPoints_, maxRadiusForCluster, minPointsPerLandmark, maxPointsPerLandmark);

    /// on the clustered points, extract corners
    /// output is of type landmark, because now you can almost be certain that
    /// what you have is a landmark
    detected_landmarks = FindLandmarks(clusteredPoints_);
    //  std::cout << "Number of preliminary landmarks found: "<<
    //  detected_landmarks.size() << std::endl;

    return 0;
}

///--------------------------------------------------------------------------------------///
/// FilterImage for pixel groups
/// disk filter image to find round shapes
///--------------------------------------------------------------------------------------///
void LandmarkFinder::FilterImage(const cv::Mat& img_in, cv::Mat& img_out) {

    cv::Mat tight_filtered, wide_filtered;
    if (tight_filter_size == 0) {
        tight_filtered = img_in;
    } else {
        cv::boxFilter(img_in, tight_filtered, -1, cv::Size(tight_filter_size, tight_filter_size), cv::Point(-1, -1), true, cv::BORDER_DEFAULT);
    }
    cv::boxFilter(img_in, wide_filtered, -1, cv::Size(wide_filter_size, wide_filter_size), cv::Point(-1, -1), true, cv::BORDER_DEFAULT);
    img_out = tight_filtered - wide_filtered;
}

///--------------------------------------------------------------------------------------///
/// FindPoints for pixel groups
/// threshold pixels and group them
///--------------------------------------------------------------------------------------///
std::vector<cv::Point> LandmarkFinder::FindPoints(cv::Mat& img_in) {

    /// thresholding for pixels: put all pixels over a threshold in vector
    cv::Mat binary;
    cv::threshold(img_in, binary, threshold, 255, cv::THRESH_BINARY);

    std::vector<cv::Point> pixels;
    cv::findNonZero(binary, pixels);

    /// use this vector to group all pixels
    /// todo: this can be done more efficiently, e.g. region growing
    std::vector<Cluster> clusteredPixels;
    FindClusters(pixels, clusteredPixels, maxRadiusForPixelCluster, minPixelForCluster, maxPixelForCluster);

    /// compute mean of each pixel cluster and put it into output vector
    /// todo: this can be done more efficiently
    std::vector<cv::Point> points;
    for (auto& cluster : clusteredPixels) {
        cv::Point thisPoint = cv::Point(0, 0);
        for (auto& pixel : cluster) { /// go thru all points in this cluster
            thisPoint += pixel;
        }
        thisPoint *= 1.0 / cluster.size();
        points.push_back(thisPoint);
    }

    return points;
}

///--------------------------------------------------------------------------------------///
/// FindClusters groups points from input vector into groups
///
///--------------------------------------------------------------------------------------///
void LandmarkFinder::FindClusters(const std::vector<cv::Point>& points_in, std::vector<Cluster>& clusters,
                                  const float radiusThreshold, const unsigned int minPointsThreshold,
                                  const unsigned int maxPointsThreshold) {

    for (auto& thisPoint : points_in) /// go thru all points
    {
        bool clusterFound = 0; /// set flag that not used yet

        for (auto& cluster : clusters) {         /// go thru all clusters
            for (auto& clusterPoint : cluster) { /// go thru all points in this cluster
                /// if distance is smaller than threshold, add point to cluster
                if (getDistance(clusterPoint, thisPoint) <= radiusThreshold) {
                    cluster.push_back(thisPoint);
                    clusterFound = true;
                    break; /// because point has been added to cluster, no further search is neccessary
                }
            }

            if (clusterFound) /// because point has been added to cluster, no further search is neccessary
                break;
        }

        if (!clusterFound) /// not assigned to any cluster
        {
            Cluster newCluster;              /// create new cluster
            newCluster.push_back(thisPoint); /// put this point in this new cluster
            clusters.push_back(newCluster);  /// add this cluster to the list
        }
    }

    /// second rule: check for minimum and maximum of points per cluster
    clusters.erase(std::remove_if(clusters.begin(), clusters.end(),
                                  [&](Cluster& cluster) {
                                      return (minPointsThreshold > cluster.size() ||
                                              maxPointsThreshold < cluster.size());
                                  }),
                   clusters.end());
}

///--------------------------------------------------------------------------------------///
/// FindCorners identifies the three corner points and sorts them into output vector
/// -> find three points whos sum of length is maximum and two edges are perpendicular
///--------------------------------------------------------------------------------------///
bool LandmarkFinder::FindCorners(std::vector<cv::Point>& point_list, std::vector<cv::Point>& corner_points) {

    float fw1 = 0.6, fw2 = 30.0, fw3 = 3.0; // Weight factors for score function
    float fp = 1.05;                        // safety_factor_for_length_comparison
    float best_score = -10000;              // Score for best combination of points

    /*  Numbering of corners and coordinate frame FOR THIS FUNCTION ONLY // TODO use normal numbering
     *       ---> y
     *  |   1   .   .   .
     *  |   .   .   .   .
     *  V   .   .   .   .
     *  x   3   .   .   2
     */

    /// Try all combinations of three points
    bool corners_found = false;
    cv::Point *cornerOne, *cornerTwo, *cornerThree;
    for (size_t i = 0; i < point_list.size(); i++) {
        cv::Point& firstPoint = point_list[i];
        for (size_t j = i + 1; j < point_list.size(); j++) {
            cv::Point& secondPoint = point_list[j];
            cv::Point v12 = getVector(secondPoint, firstPoint);
            for (size_t k = j + 1; k < point_list.size(); k++) {
                cv::Point& thirdPoint = point_list[k];
                cv::Point v31 = getVector(thirdPoint, firstPoint);
                cv::Point v32 = getVector(thirdPoint, secondPoint);

                /// Since we test every combination only once, make sure the lengths are correct:
                // norm(v12) > norm(v31) >= (v32)
                if ((getNorm(v31) > fp * getNorm(v32)) && (getNorm(v31) > fp * getNorm(v12))) {
                    v12 = v31; // v12 should be longest -> hypotenuse
                    v31 = getVector(secondPoint, firstPoint);
                    v32 = v32;
                } else if ((getNorm(v32) > fp * getNorm(v31)) && (getNorm(v32) > fp * getNorm(v12))) {
                    v12 = v32;
                    v32 = v31;
                    v31 = getVector(secondPoint, firstPoint);
                }

                float dist12 = getNorm(v12);
                float dist31 = getNorm(v31);
                float dist32 = getNorm(v32);
                float sumOfLength = dist12 + dist31 + dist32;

                // Project v32 onto v31 -> resulting length should be close to zero
                // TODO use cross product?
                float projectedLength = std::abs((v32.x * v31.x + v32.y * v31.y)) / dist31 / dist32;
                float diffInLength = fabs(dist31 - dist32);

                // We are trying to maximise sumOfLength while minimizing projectedLength and diffInLength
                if (best_score < (fw1 * sumOfLength - (fw2 * projectedLength + fw3 * diffInLength))) {
                    if (100.0 > dist12 && 80.0 > dist31 && 80.0 > dist32) {
                        if (fabs(dist31 - dist32) < 0.5 * dist12) {
                            /// remember their addresses and distance
                            corners_found = true;
                            cornerOne = &firstPoint;
                            cornerTwo = &secondPoint;
                            cornerThree = &thirdPoint;
                            best_score = fw1 * sumOfLength - fw2 * projectedLength - fw3 * diffInLength;
                        }
                    }
                }
            }
        }
    }
    if (!corners_found) {
        return false;
    }

    /// The three distances have to be updated for calculations in the next steps
    cv::Point v12 = getVector(*cornerOne, *cornerTwo);
    cv::Point v31 = getVector(*cornerThree, *cornerOne);
    cv::Point v32 = getVector(*cornerThree, *cornerTwo);

    /// Compare the distances and get the diagonal of the landmark
    /// note the reversed order: it's 1-3-2, because the corner 3 is the one
    /// between 1 and 2 this helps for post-processing
    if ((getNorm(v12) > fp * getNorm(v31)) && (getNorm(v12) > fp * getNorm(v32))) {
        ;
    } else if ((getNorm(v31) > fp * getNorm(v32)) && (getNorm(v31) > fp * getNorm(v12))) {
        std::swap(cornerTwo, cornerThree);
    } else {
        std::swap(cornerOne, cornerThree);
    }

    /// Store in output container
    corner_points.push_back(*cornerOne);
    corner_points.push_back(*cornerThree);
    corner_points.push_back(*cornerTwo);

    /// Remove from input list
    point_list.erase(std::remove(point_list.begin(), point_list.end(), *cornerOne), point_list.end());
    point_list.erase(std::remove(point_list.begin(), point_list.end(), *cornerTwo), point_list.end());
    point_list.erase(std::remove(point_list.begin(), point_list.end(), *cornerThree), point_list.end());

    return true;
}

///--------------------------------------------------------------------------------------///
/// FindLandmarks identifies landmark inside a point cluster
///
///--------------------------------------------------------------------------------------///
std::vector<ImgLandmark> LandmarkFinder::FindLandmarks(const std::vector<Cluster>& clusteredPoints) {

    std::vector<ImgLandmark> OutputLandmarks;

    for (auto& cluster : clusteredPoints) { /// go thru all clusters

        /// since most probably each cluster represents a landmark, create one
        ImgLandmark newLandmark;
        newLandmark.nID = 0; /// we have not identified anything, so default ID is zero
        newLandmark.voIDPoints =
            cluster; /// all points in this cluster are copied to the ID point vector for further examination

        /// FindCorners will move the three corner points into the corners vector
        if (!FindCorners(newLandmark.voIDPoints, newLandmark.voCorners))
            continue;

        /// add this landmark to the landmark vector
        OutputLandmarks.push_back(newLandmark);
    }

    GetIDs(OutputLandmarks);

    /// done and return landmarks
    return OutputLandmarks;
}

///--------------------------------------------------------------------------------------///
/// CalculateIdForward sorts the given idPoints and calculates the id
///
///--------------------------------------------------------------------------------------///
bool LandmarkFinder::CalculateIdForward(ImgLandmark& landmark, std::vector<uint16_t>& valid_ids) {
    // TOD clean up this function
    /// first of all: get the three corner points
    const cv::Point* oCornerOne = &landmark.voCorners.at(0);
    const cv::Point* oCornerTwo = &landmark.voCorners.at(1);
    const cv::Point* oCornerThree = &landmark.voCorners.at(2);

    // TODO move these checks into FindCorners function
    /// second: get the x- and y-axis of the landmark
    cv::Point oTwoOne = getVector(*oCornerTwo, *oCornerOne);
    cv::Point oTwoThree = getVector(*oCornerTwo, *oCornerThree);

    /// third: make sure, they are in the right order.
    /// we do this by checking if the cross product is positive
    float fCrossProduct = getCrossProduct(oTwoOne, oTwoThree);

    if (0 > fCrossProduct) {
        std::swap(oCornerOne, oCornerThree);
        oTwoOne = getVector(*oCornerTwo, *oCornerOne);
        oTwoThree = getVector(*oCornerTwo, *oCornerThree);
        std::swap(landmark.voCorners.at(0), landmark.voCorners.at(2));
    }

    /// at this point we have a right hand system in image coordinates

    /// now, we find the affine transformation which maps the landmark from
    /// image coordinate in a landmark-related coordinate frame
    /// with the corners defined as (0,0), (1,0) and (0,1)

    /// for this, we just compute the inverse of the two side vectors
    cv::Mat Transform(2, 2, CV_32FC1);
    Transform.at<float>(0, 0) = float(oTwoOne.x);
    Transform.at<float>(1, 0) = float(oTwoOne.y);
    Transform.at<float>(0, 1) = float(oTwoThree.x);
    Transform.at<float>(1, 1) = float(oTwoThree.y);

    Transform = Transform.inv();

    /// now we have a transform which maps [0,1028]x[0,1280] -> [0,1]x[0,1],
    /// i.e. our landmark is in the latter do
    ///

    /// next, the ID points are transformed accordingly and then matched to
    /// their binary values

    /// the point under examination
    cv::Mat ThisPoint(2, 1, CV_32FC1);

    /// the x and y value of the point
    float x = 0;
    float y = 0;

    /// the total ID
    uint16_t ID = 0;

    /// the value a certain point contributes to the ID
    uint16_t ThisPointID = 0;

    /// go thru all ID points in this landmark structure
    std::vector<cv::Point>::iterator pPointsIt;
    std::vector<uint16_t> pPointsIDs;
    for (pPointsIt = landmark.voIDPoints.begin(); pPointsIt != landmark.voIDPoints.end(); pPointsIt++) {
        /// first step: bring the ID point in relation to the origin of the
        /// landmark
        ThisPoint.at<float>(0, 0) = float(pPointsIt->x) - float(oCornerTwo->x);
        ThisPoint.at<float>(1, 0) = float(pPointsIt->y) - float(oCornerTwo->y);

        /// apply transfrom
        ThisPoint = Transform * ThisPoint;

        /// next step is the quantization in values between 0 and 3
        x = ThisPoint.at<float>(0, 0);
        y = ThisPoint.at<float>(1, 0);

        /// it's 1-y because in the definition of the landmark ID the x axis runs
        /// down
        int nY = floor((y) / 0.25);
        int nX = floor((1 - x) / 0.25);

        nX = nX < 0 ? 0 : nX;
        nX = nX > 3 ? 3 : nX;
        nY = nY < 0 ? 0 : nY;
        nY = nY > 3 ? 3 : nY;

        /// the binary values ar coded: x steps are binary shifts within 4 bit
        /// blocks
        ///                             y steps are binary shifts of 4 bit blocks
        ///                             see http://hagisonic.com/ for more
        ///                             information on this
        ThisPointID = static_cast<uint16_t>((1 << nX) << 4 * nY);
        pPointsIDs.push_back(ThisPointID);

        /// add this point's contribution to the landmark ID
        ID += ThisPointID;
    }

    /// Sort points
    /* The order of id points
    *      x   3   7   .
    *      1   4   8   12
    *      2   5   9   13
    *      x   6   10  x
    */
    parallel_vector_sort(pPointsIDs, landmark.voIDPoints);

    /// assign ID to landmark
    landmark.nID = ID;

    /// validate with the vector of available IDs
    std::vector<uint16_t>::iterator idIterator = std::find(valid_ids.begin(), valid_ids.end(), landmark.nID);
    if (idIterator != valid_ids.end()) { /// ID matches one which is available:
        valid_ids.erase(idIterator);     /// remove this ID
        return true;
    } else { /// no ID match
        return false;
    }
}

///--------------------------------------------------------------------------------------///
/// CalculateIdBackward searches in the filtered image for id points, given the corners.
///
///--------------------------------------------------------------------------------------///
bool LandmarkFinder::CalculateIdBackward(ImgLandmark& landmark, std::vector<uint16_t>& valid_ids) {
    // TOD clean up this function
    uint16_t nThisID = 0;

    /// same as before: finde affine transformation, but this time from landmark
    /// coordinates to image coordinates
    const cv::Point* oCornerOne = &landmark.voCorners.at(0);
    const cv::Point* oCornerTwo = &landmark.voCorners.at(1);
    const cv::Point* oCornerThree = &landmark.voCorners.at(2);

    cv::Point oTwoOne = getVector(*oCornerTwo, *oCornerOne);
    cv::Point oTwoThree = getVector(*oCornerTwo, *oCornerThree);

    /// make it a right hand system
    float fCrossProduct = getCrossProduct(oTwoOne, oTwoThree);

    if (0 > fCrossProduct) {
        std::swap(oCornerOne, oCornerThree);
        oTwoOne = getVector(*oCornerTwo, *oCornerOne);
        oTwoThree = getVector(*oCornerTwo, *oCornerThree);
        std::swap(landmark.voCorners.at(0), landmark.voCorners.at(2));
    }

    /// now we delete the previously detected points and go the other way around
    landmark.voIDPoints.clear();

    cv::Mat Transform(2, 2, CV_32FC1);
    Transform.at<float>(0, 0) = float(oTwoOne.x);
    Transform.at<float>(0, 1) = float(oTwoThree.x);
    Transform.at<float>(1, 0) = float(oTwoOne.y);
    Transform.at<float>(1, 1) = float(oTwoThree.y);

    cv::Mat ThisPoint(2, 1, CV_32FC1);
    std::vector<uint16_t> pPointsIDs;

    /// go thru all possible ID points and see if the image has a high gray
    /// value there, i.e. there's light
    for (int nX = 0; nX < 4; nX++) {
        for (int nY = 0; nY < 4; nY++) {
            /// this must not be done for the three corner points of course
            if ((nX != 0 || nY != 0) && (nX != 0 || nY != 3) && (nX != 3 || nY != 0)) {
                uint16_t ThisPointID = 0;
                /// since we know the corners, we can go in thirds between them to see
                /// if theres a light
                ThisPoint.at<float>(0, 0) = float(nX) * 0.333;
                ThisPoint.at<float>(1, 0) = float(nY) * 0.333;

                ThisPoint = Transform * ThisPoint;

                ThisPoint.at<float>(0, 0) += float(oCornerTwo->x);
                ThisPoint.at<float>(1, 0) += float(oCornerTwo->y);

                cv::Point Index(int(ThisPoint.at<float>(0, 0)), int(ThisPoint.at<float>(1, 0)));

                /// same as for the pixel detection: see if the gray value at the
                /// point where the light should be exceeds a threshold and thus
                /// supports the light hypothesis
                if (0 > Index.x || 0 > Index.y || grayImage_.cols <= Index.x || grayImage_.rows <= Index.y) {
                    continue;
                }

                if (threshold < grayImage_.at<uint8_t>(Index.y,
                                                       Index.x)) { /// todo: this might be extended to some area
                    ThisPointID = static_cast<uint16_t>((1 << (3 - nX)) << 4 * nY);
                    landmark.voIDPoints.push_back(Index);
                    pPointsIDs.push_back(ThisPointID);
                }

                /// add the contribution to the total ID
                nThisID += ThisPointID;
            }
        }
    }

    /// Sort points
    /* The order of id points
    *      x   3   7   .
    *      1   4   8   12
    *      2   5   9   13
    *      x   6   10  x
    */
    parallel_vector_sort(pPointsIDs, landmark.voIDPoints);

    landmark.nID = nThisID;

    /// now, same as before, validate with available IDs
    std::vector<uint16_t>::iterator pIDLUTIt = std::find(valid_ids.begin(), valid_ids.end(), nThisID);
    /// if the new ID is valid, enqueue the landmark again
    if (pIDLUTIt != valid_ids.end()) {
        valid_ids.erase(pIDLUTIt);
        return true;
    } else {
        return false;
    }
}

///--------------------------------------------------------------------------------------///
/// GetIDs is to identify the ID of a landmark according to the point pattern
/// see http://hagisonic.com/ for information on pattern
///--------------------------------------------------------------------------------------///
int LandmarkFinder::GetIDs(std::vector<ImgLandmark>& landmarks) {
    /*  Numbering of corners and coordinate frame
     *       ---> y
     *  |   1   .   .   .
     *  |   .   .   .   .
     *  V   .   .   .   .
     *  x   2   .   .   3
     */
    /// get vector of possible IDs
    std::vector<uint16_t> validIDs = valid_ids_;

    /// vector of iterators to Landmarks which where not identified correctly
    /// once we've been through all landmarks, we can look up available IDs in the
    /// vector define above.
    /// this is why we remember errors but don't correct them right away.
    std::vector<ImgLandmark> landmarksInQueue;

    /// First, try to use the id points given to determine landmark id.
    std::vector<ImgLandmark>::iterator pLandmarkIt = landmarks.begin();
    while (pLandmarkIt != landmarks.end()) {

        if (CalculateIdForward(*pLandmarkIt, validIDs)) {
            ++pLandmarkIt; /// go to next landmark
        } else {
            landmarksInQueue.push_back(*pLandmarkIt); /// put this landmark in queue for second processing run
            landmarks.erase(pLandmarkIt); /// delete it from valid landmark list. This also is a step to next landmark
        }
    }

    /// now, go thru all landmarks which did not match a valid ID and try to match them to one of the remaining
    for (auto& landmark : landmarksInQueue) {
        if (CalculateIdBackward(landmark, validIDs)) {
            landmarks.push_back(landmark);
        } else {
            ; /// go to next landmark
        }
    }

    return 0;
}

void LandmarkFinder::parallel_vector_sort(std::vector<uint16_t>& ids, std::vector<cv::Point>& points) {
    size_t len = ids.size();
    size_t stepsize = len / 2; // Zu Beginn ist die Lücke über den halben Array.
    bool b = true;
    while (b) {
        b = false; // b bleibt auf false, wenn kein einziges Mal etwas falsch ist.
        for (size_t i = 0; i < len; i++) {
            if (stepsize + i >= len) // Schutz vor Speicherfehlern
            {
                break;
            }
            if (ids[i] > ids[i + stepsize]) // überprüft ob die zwei Elemente falsch herum sind
            {
                std::swap(ids[i], ids[i + stepsize]); // wenn ja -> vertauschen
                std::swap(points[i], points[i + stepsize]);
                b = true;
            }
        }
        stepsize = stepsize / 1.3; // Lücke verkleinern für nächsten Durchlauf
        if (stepsize < 1) {
            stepsize = 1;
        }
    }
}

cv::Point LandmarkFinder::getVector(const cv::Point& from, const cv::Point& to) {
    return cv::Point(to.x - from.x, to.y - from.y);
}

float LandmarkFinder::getDistance(const cv::Point& p1, const cv::Point& p2) {
    return std::hypot(p2.x - p1.x, p2.y - p1.y);
}

float LandmarkFinder::getNorm(const cv::Point& p) {
    return std::hypot(p.x, p.y);
}

float LandmarkFinder::getCrossProduct(const cv::Point& v1, const cv::Point& v2) {
    return float(v1.x) * float(v2.y) - float(v1.y) * float(v2.x);
}
