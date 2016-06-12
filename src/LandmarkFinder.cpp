
#include "LandmarkFinder.h"

using namespace std;
using namespace stargazer;

///--------------------------------------------------------------------------------------///
/// Default constructor
///--------------------------------------------------------------------------------------///
LandmarkFinder::LandmarkFinder(std::string cfgfile) : debug_mode(false) {

    /// set parameters
    m_cThreshold = 20;
    m_fMaxRadiusForPixelCluster = 3;
    m_nMinPixelForCluster = 1;
    m_nMaxPixelForCluster = 1000;
    m_fMaxRadiusForCluster = 40;
    m_nMinPointsPerLandmark = 5;
    m_nMaxPointsPerLandmark = 9;

    /// Read in Landmark ids
    camera_params_t dummy;
    landmark_map_t landmarks;
    readConfig(cfgfile, dummy, landmarks);
    for (auto& el : landmarks)
        m_vnIDs.push_back(el.first);
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
int LandmarkFinder::FindLandmarks(const cv::Mat& i_oImage, std::vector<ImgLandmark>& o_vLandmarks) {
    std::vector<cv::Point> ClusteredPixels;
    std::vector<Cluster> ClusteredPoints;
    i_oImage.assignTo(m_oImage, CV_8UC3);

    /// check if input is valid
    if (!m_oImage.data) { /// otherwise: return with error
        std::cerr << "Input data is invalid" << std::endl;
        return -1;
    }
    o_vLandmarks.clear();

    /// convert color to gray for further processing
    cvtColor(m_oImage, m_oGrayImage, CV_BGR2GRAY);

    /// This method finds bright points in image
    /// returns vector of center points of pixel groups
    ClusteredPixels = FindPoints(m_oGrayImage);
    //  std::cout << "Number of bright points found: " <<  ClusteredPixels.size()
    //  << std::endl;

    /// cluster points to groups which could be landmarks
    /// returns a vector of clusters which themselves are vectors of points
    FindClusters(ClusteredPixels, ClusteredPoints, m_fMaxRadiusForCluster, m_nMinPointsPerLandmark,
                 m_nMaxPointsPerLandmark);
    //  std::cout << "Number of clusters found: " <<  ClusteredPoints.size() <<
    //  std::endl;

    /// on the clustered points, extract corners
    /// output is of type landmark, because now you can almost be certain that
    /// what you have is a landmark
    o_vLandmarks = FindCorners(ClusteredPoints);
    // LOUT ("FINDLAND: " << ClusteredPixels.size() << ' ' << o_vLandmarks.size()
    // << ' ' << std::endl);
    //  std::cout << "Number of preliminary landmarks found: "<<
    //  o_vLandmarks.size() << std::endl;

    GetIDs(o_vLandmarks);

    return 0;
}

///--------------------------------------------------------------------------------------///
/// Region Growing for Clustering Points
///
///--------------------------------------------------------------------------------------///
void LandmarkFinder::Check(cv::Mat& Filtered, int XPos, int YPos, int Threshold, int& Pixelcount, int& SummedX,
                           int& SummedY) {
    int x, y;
    if ((XPos > 0) && (XPos < 1280) && (YPos > 0) && (YPos < 1024)) {
        if (Filtered.at<unsigned char>(YPos, XPos) > Threshold) {
            Pixelcount += 1;
            SummedX += XPos;
            SummedY += YPos;
            Filtered.at<unsigned char>(YPos, XPos) = 0;
            for (x = -2; x <= 2; x++) {
                for (y = -2; y <= 2; y++) {
                    if (x == 0 && y == 0) {
                        continue;
                    }
                    Check(Filtered, XPos + x, YPos + y, Threshold, Pixelcount, SummedX, SummedY);
                }
            }
        }
    }
}

///--------------------------------------------------------------------------------------///
/// FindPoints for pixel groups
/// disk filter image to find round shapes and group them
///--------------------------------------------------------------------------------------///
std::vector<cv::Point> LandmarkFinder::FindPoints(cv::Mat& i_oGrayImage) {
    int i, j;
    /// declare output image of disk filter
    cv::Mat filtered;

    /// declare filter kernel
    float Kernel_pill[7][7];

    for (i = 0; i < 7; i++) {
        for (j = 0; j < 7; j++) {
            if (i == 0 || i == 6 || j == 0 || j == 6) {
                Kernel_pill[i][j] = -1;
                continue;
            }

            if (i == 1 || i == 5 || j == 1 || j == 5) {
                Kernel_pill[i][j] = 0;
                continue;
            }

            if (i == 2 || i == 4 || j == 2 || j == 4) {
                Kernel_pill[i][j] = 1.0;
                continue;
            }

            Kernel_pill[i][j] = 2;
        }
    }

    //      Kernel_pill= {     { -1 ,      -1   ,  -1   ,  -1   ,  -1   ,  -1   ,
    //      -1},
    //                         { -1 ,       0   ,  0    ,  0    ,  0    ,  0    ,
    //                         -1},
    //                         { -1 ,       0   ,  0.5  ,  0.5  ,  0.5  ,  0    ,
    //                         -1},
    //                         { -1 ,       0   ,  0.5  ,  1    ,  0.5  ,  0    ,
    //                         -1},
    //                         { -1 ,       0   ,  0.5  ,  0.5  ,  0.5  ,  0    ,
    //                         -1},
    //                         { -1 ,       0   ,  0    ,  0    ,  0    ,  0    ,
    //                         -1},
    //                         { -1 ,      -1   ,  -1   ,  -1   ,  -1   ,  -1   ,
    //                         -1}};

    //      Kernel_pill= {     { -1  ,  -1   ,  -1   ,  -1   ,  -1},
    //                         { -1  ,  0    ,  2.5  ,  0    ,  -1},
    //                         { -1  ,  2.5  ,  5    ,  2.5  ,  -1},
    //                         { -1  ,  0    ,  2.5  ,  0    ,  -1},
    //                         { -1  ,  -1   ,  -1   ,  -1   ,  -1}};

    /// kernel definition
    cv::Mat MyKernel = cv::Mat(7, 7, CV_32F, Kernel_pill).clone();

    /// apply diskfilter to image
    cv::filter2D(m_oGrayImage, filtered, m_oGrayImage.depth(), MyKernel, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);

    // alternative 1:  without region growing
    /// thresholding for pixels: put all pixels over a threshold in vector
    std::vector<cv::Point> Pixels;
    for (int x = 0; x < m_oGrayImage.cols; x++) {
        for (int y = 0; y < m_oGrayImage.rows; y++) {
            if (m_cThreshold < filtered.at<unsigned char>(y, x)) {
                Pixels.push_back(cv::Point(x, y));
            }
        }
    }

    /// use this vector to group all pixels
    /// todo: this can be done more efficiently, e.g. region growing
    std::vector<Cluster> ClusteredPixels;
    FindClusters(Pixels, ClusteredPixels, m_fMaxRadiusForPixelCluster, m_nMinPixelForCluster, m_nMaxPixelForCluster);

    /// compute mean of each pixel cluster and put it ouput vector
    /// todo: this can be done more efficiently
    std::vector<Cluster>::iterator pAllClustersIt;
    Cluster::iterator pThisClusterIt;
    std::vector<cv::Point> Points;
    cv::Point ThisPoint;
    float fPixelsForPoint = 0;
    for (pAllClustersIt = ClusteredPixels.begin(); pAllClustersIt != ClusteredPixels.end(); pAllClustersIt++) {
        fPixelsForPoint = 0;
        ThisPoint = cv::Point(0, 0);
        for (pThisClusterIt = pAllClustersIt->begin(); pThisClusterIt != pAllClustersIt->end();
             pThisClusterIt++) { /// go thru all points in this cluster
            ThisPoint += *pThisClusterIt;
            ++fPixelsForPoint;
        }
        ThisPoint *= 1 / fPixelsForPoint;
        Points.push_back(ThisPoint);
    }

    return Points;
    //    alternative 1:  without region growing

    /*  //alternative 2:  with region growing

    /// thresholding for pixels: put all pixels over a threshold in vector
    std::vector<cv::Point> Pixels;
    cv::Point TPoint;
    int XPos, YPos, Pixelcount, SummedX, SummedY;

    for(int x = 0; x < m_oGrayImage.cols; x++)
    {
        for(int y = 0; y < m_oGrayImage.rows; y++)
        {
            XPos=0; YPos=0; Pixelcount=0; SummedX=0; SummedY=0;
            Check(filtered, x, y, threshold, Pixelcount, SummedX,SummedY);
            if((Pixelcount>minPixelForCluster) && (Pixelcount<maxPixelForCluster))
            {
              TPoint= cv::Point(int(SummedX/Pixelcount),int(SummedY/Pixelcount));
              Pixels.push_back(TPoint);
            }


        }
    }

    return Pixels;

*/ // alternative 2:  with region
                                               // growing
}

///--------------------------------------------------------------------------------------///
/// FindClusters groups points from input vector into groups
///
///--------------------------------------------------------------------------------------///
void LandmarkFinder::FindClusters(std::vector<cv::Point>& i_voPoints, std::vector<Cluster>& o_voCluster,
                                  const float i_fRadiusThreshold, const unsigned int i_nMinPointsThreshold,
                                  const unsigned int i_nMaxPointsThreshold) {
    std::vector<Cluster>::iterator pAllClustersIt; /// iterator for clusters

    Cluster::iterator pThisClusterIt; /// iterator within a cluster

    cv::Point ThisPoint;

    char cUsed = 0;

    cv::Point Dist;

    std::vector<cv::Point>::iterator pPointsIt;
    for (pPointsIt = i_voPoints.begin(); pPointsIt != i_voPoints.end(); pPointsIt++) /// go thru all points
    {
        ThisPoint = *pPointsIt; /// take this point
        cUsed = 0;              /// set flag that not used yet

        for (pAllClustersIt = o_voCluster.begin(); pAllClustersIt != o_voCluster.end();
             pAllClustersIt++) { /// go thru all clusters
            for (pThisClusterIt = pAllClustersIt->begin(); pThisClusterIt != pAllClustersIt->end();
                 pThisClusterIt++) { /// go thru all points in this cluster
                /// calculate distance to a point
                Dist = *pThisClusterIt - ThisPoint;
                if (i_fRadiusThreshold >= sqrt((float)(Dist.x * Dist.x + Dist.y * Dist.y))) /// if distance
                                                                                            /// is smaller
                                                                                            /// than
                                                                                            /// threshold,
                                                                                            /// add point to
                                                                                            /// cluster
                {
                    pAllClustersIt->push_back(ThisPoint);
                    cUsed = 1;
                    break; /// because point has been added to cluster, no further search
                           /// is neccessary
                }
            }

            if (cUsed) { /// because point has been added to cluster, no further
                         /// search is neccessary
                break;
            }
        }

        if (!cUsed) /// not assigned to any cluster
        {
            Cluster ThisCluster;                /// create new cluster
            ThisCluster.push_back(ThisPoint);   /// put this point in this new cluster
            o_voCluster.push_back(ThisCluster); /// add this cluster to the list
        }
    }

    /// second rule: check fo minimum and maximum of points per cluster
    pAllClustersIt = o_voCluster.begin();
    while (pAllClustersIt != o_voCluster.end()) {
        if (i_nMinPointsThreshold > pAllClustersIt->size() ||
            i_nMaxPointsThreshold < pAllClustersIt->size()) { /// if there are too few or too many
                                                              /// points within this cluster, delete it
            o_voCluster.erase(pAllClustersIt);
        } else { /// otherwise, keep going
            ++pAllClustersIt;
        }
    }
}

///--------------------------------------------------------------------------------------///
/// FindCorners identifies the landmark corners and puts them into the landmark
/// structure
///
///--------------------------------------------------------------------------------------///
std::vector<ImgLandmark> LandmarkFinder::FindCorners(std::vector<Cluster>& ClusteredPoints) {
    float fDist, fDist2, fDist3, fSumOfLength, fProjection, fDiffOfLength, fMaxfunc;
    float fw1 = 0.6, fw2 = 30.0, fw3 = 3.0;
    float fp = 1.05;

    std::vector<Cluster>::iterator pAllClustersIt;
    std::vector<cv::Point>::iterator pPointIterator, pPointIterator2;
    std::vector<cv::Point>::iterator pAdjecentPointIt, pAdjecentPointIt2, pCornerOne, pCornerTwo, pCornerThree;
    cv::Point ThisPoint, Dist, Dist2, Dist3, Middle, CornerOne, CornerTwo, CornerThree;

    std::vector<ImgLandmark> OutputLandmarks;

    bool bCornersSet;

    for (pAllClustersIt = ClusteredPoints.begin(); pAllClustersIt != ClusteredPoints.end();
         pAllClustersIt++) { /// go thru all clusters
        bCornersSet = false;

        /// in each cluster, find three points that sum of length is maximum and two
        /// edges are perpendicular
        fMaxfunc = -10000;

        fSumOfLength = 0;
        fProjection = 0;
        fDist = 0;
        fDist2 = 0;
        fDist3 = 0;

        /// since most probably each cluster represents a landmark, create one
        ImgLandmark ThisLandmark;
        ThisLandmark.nErrors = 0; /// we have no detection so far, so error count
                                  /// in detections is zero
        ThisLandmark.nID = 0;     /// we have not identified anything, so default ID is zero

        ThisLandmark.voIDPoints = *pAllClustersIt; /// all points in this cluster are copied to the ID
                                                   /// point vector for further examination

        ThisLandmark.nPointCount = ThisLandmark.voIDPoints.size(); /// the total number of available
                                                                   /// points in this landmark, this might
                                                                   /// be of interest if misdetections
                                                                   /// happen

        /// now, go thru all points and compare them with all the other points
        for (pPointIterator = ThisLandmark.voIDPoints.begin(); pPointIterator != ThisLandmark.voIDPoints.end();
             pPointIterator++) {
            ThisPoint = *pPointIterator;

            /// since a point has been compared to all points befor it, we start
            /// further comparison here
            for (pAdjecentPointIt = pPointIterator; pAdjecentPointIt != ThisLandmark.voIDPoints.end();
                 pAdjecentPointIt++) {
                /// if the two points under examination are equal, dont do anything
                if (pAdjecentPointIt != pPointIterator) {
                    Dist = ThisPoint - *pAdjecentPointIt;

                    for (pAdjecentPointIt2 = pAdjecentPointIt; pAdjecentPointIt2 != ThisLandmark.voIDPoints.end();
                         pAdjecentPointIt2++) {
                        /// if the third point under examination is equal to the first or
                        /// second, dont do anything
                        if ((pAdjecentPointIt2 != pPointIterator) && (pAdjecentPointIt2 != pAdjecentPointIt)) {
                            Dist2 = *pPointIterator - *pAdjecentPointIt2;
                            Dist3 = *pAdjecentPointIt - *pAdjecentPointIt2;

                            /// norm(Dist) > norm(Dist2) >(Dist3)
                            if (((Dist2.x * Dist2.x + Dist2.y * Dist2.y) >
                                 fp * (Dist3.x * Dist3.x + Dist3.y * Dist3.y)) &&
                                ((Dist2.x * Dist2.x + Dist2.y * Dist2.y) > fp * (Dist.x * Dist.x + Dist.y * Dist.y))) {
                                Dist = *pPointIterator - *pAdjecentPointIt2;
                                Dist2 = *pPointIterator - *pAdjecentPointIt;
                                Dist3 = *pAdjecentPointIt - *pAdjecentPointIt2;
                            } else if (((Dist3.x * Dist3.x + Dist3.y * Dist3.y) >
                                        fp * (Dist2.x * Dist2.x + Dist2.y * Dist2.y)) &&
                                       ((Dist3.x * Dist3.x + Dist3.y * Dist3.y) >
                                        fp * (Dist.x * Dist.x + Dist.y * Dist.y))) {
                                Dist = *pAdjecentPointIt - *pAdjecentPointIt2;
                                Dist2 = *pPointIterator - *pAdjecentPointIt;
                                Dist3 = *pPointIterator - *pAdjecentPointIt2;
                            }

                            fDist = sqrt((float)(Dist.x * Dist.x + Dist.y * Dist.y));
                            fDist2 = sqrt((float)(Dist2.x * Dist2.x + Dist2.y * Dist2.y));
                            fDist3 = sqrt((float)(Dist3.x * Dist3.x + Dist3.y * Dist3.y));

                            fSumOfLength = fDist + fDist2 + fDist3;
                            // 4   fProjection =
                            // -exp(-fabs(asin((Dist3.x*Dist2.x+Dist3.y*Dist2.y)/fDist2/fDist3)))*50;
                            // 3   fProjection =
                            // fabs((Dist3.x*Dist2.x+Dist3.y*Dist2.y))/fDist2/fDist3*50;
                            // 2   fProjection =
                            // fabs(asin((Dist3.x*Dist2.x+Dist3.y*Dist2.y)/fDist2/fDist3))*50;
                            fProjection = fabs((Dist3.x * Dist2.x + Dist3.y * Dist2.y)) / fDist2 /
                                          fDist3; //+fabs((Dist2.x*Dist3.x+Dist2.y*Dist3.y))/fDist3/fDist3;
                            fDiffOfLength = fabs(fDist2 - fDist3);

                            // LOUT("Sum of length " << fSumOfLength << " Projection " <<
                            // fProjection << " Diff " << fDiffOfLength << std::endl);

                            if (fMaxfunc < (fw1 * fSumOfLength - (fw2 * fProjection + fw3 * fDiffOfLength))) {
                                // LOUT("1 ");
                                if (100.0 > fDist && 80.0 > fDist2 && 80.0 > fDist3) {
                                    // LOUT("2 ");
                                    if (fabs(fDist2 - fDist3) < 0.5 * fDist) {
                                        // LOUT("3 " << std::endl);
                                        /// remember their addresses and distance
                                        bCornersSet = true;
                                        pCornerOne = pPointIterator;
                                        pCornerTwo = pAdjecentPointIt;
                                        pCornerThree = pAdjecentPointIt2;
                                        fMaxfunc = fw1 * fSumOfLength - fw2 * fProjection - fw3 * fDiffOfLength;
                                        // LOUT("NEW MAX FUNC: " << fMaxfunc << std::endl);
                                        // ThisLandmark.nID = int(fMaxfunc);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        if (!bCornersSet) {
            ClusteredPoints.erase(pAllClustersIt);
            pAllClustersIt--;
            continue;
        }
        /// The three distances have to be updated for calculations in the next
        /// steps
        Dist = *pCornerTwo - *pCornerOne;
        Dist2 = *pCornerThree - *pCornerOne;
        Dist3 = *pCornerThree - *pCornerTwo;

        /// the middle point is evaluated
        /*  Numbering of corners and coordinate frame
         *       ---> y
         *  |   1   .   .   .
         *  |   .   .   .   .
         *  V   .   .   .   .
         *  x   3   .   .   2
         */
        /// Compare the distances and get the diagonal of the landmark
        /// note the reversed order: it's 1-3-2, because the corner 3 is the one
        /// between 1 and 2
        /// this helps for post-processing
        if (((Dist.x * Dist.x + Dist.y * Dist.y) > fp * (Dist2.x * Dist2.x + Dist2.y * Dist2.y)) &&
            ((Dist.x * Dist.x + Dist.y * Dist.y) > fp * (Dist3.x * Dist3.x + Dist3.y * Dist3.y))) {
            Middle = 0.5 * (*pCornerOne + *pCornerTwo);
            ThisLandmark.voCorners.push_back(*pCornerOne);
            ThisLandmark.voCorners.push_back(*pCornerThree);
            ThisLandmark.voCorners.push_back(*pCornerTwo);
        } else if (((Dist2.x * Dist2.x + Dist2.y * Dist2.y) > fp * (Dist3.x * Dist3.x + Dist3.y * Dist3.y)) &&
                   ((Dist2.x * Dist2.x + Dist2.y * Dist2.y) > fp * (Dist.x * Dist.x + Dist.y * Dist.y))) {
            Middle = 0.5 * (*pCornerOne + *pCornerThree);
            ThisLandmark.voCorners.push_back(*pCornerOne);
            ThisLandmark.voCorners.push_back(*pCornerTwo);
            ThisLandmark.voCorners.push_back(*pCornerThree);
        } else {
            Middle = 0.5 * (*pCornerTwo + *pCornerThree);
            ThisLandmark.voCorners.push_back(*pCornerThree);
            ThisLandmark.voCorners.push_back(*pCornerOne);
            ThisLandmark.voCorners.push_back(*pCornerTwo);
        }

        // LOUT("CORNERS: " << ThisLandmark.voCorners.size() << std::endl);
        CornerOne = *pCornerOne;
        CornerTwo = *pCornerTwo;
        CornerThree = *pCornerThree;

        ThisLandmark.voIDPoints.erase(pCornerThree);
        ThisLandmark.voIDPoints.erase(pCornerTwo);
        ThisLandmark.voIDPoints.erase(pCornerOne);

        /// the marker is defined as the middel of the connection between the two
        /// outer corners
        ThisLandmark.oPosition = Middle;

        /// add this landmark to the landmark vector
        OutputLandmarks.push_back(ThisLandmark);
    }

    /// done and return landmarks
    return OutputLandmarks;
}

///--------------------------------------------------------------------------------------///
/// GetIDs is to identify the ID of a landmark according to the point pattern
/// see http://hagisonic.com/ for information on pattern
///--------------------------------------------------------------------------------------///
int LandmarkFinder::GetIDs(std::vector<ImgLandmark>& io_voLandmarks) {
    /*  Numbering of corners and coordinate frame
     *       ---> y
     *  |   1   .   .   .
     *  |   .   .   .   .
     *  V   .   .   .   .
     *  x   2   .   .   3
     */
    /// get vector of possible IDs
    std::vector<int> vnIDs = m_vnIDs;

    /// vector of iterators to Landmarks which where not identified correctly
    /// once we've been through all landmarks, we can look up available IDs in the
    /// vector define above.
    /// this is why we remember errors but don't correct them right away.
    std::vector<ImgLandmark> voLandmarksInQueue;

    /// go thru all landmarks
    std::vector<ImgLandmark>::iterator pLandmarkIt = io_voLandmarks.begin();
    while (pLandmarkIt != io_voLandmarks.end()) {
        /// first of all: get the three corner points
        cv::Point oCornerOne = pLandmarkIt->voCorners.at(0);
        cv::Point oCornerTwo = pLandmarkIt->voCorners.at(1);
        cv::Point oCornerThree = pLandmarkIt->voCorners.at(2);

        /// second: get the x- and y-axis of the landmark
        cv::Point oTwoOne = oCornerOne - oCornerTwo;
        cv::Point oTwoThree = oCornerThree - oCornerTwo;

        /// third: make sure, they are in the right order.
        /// we do this by checking if the cross product is positive
        float fCrossProduct = float(oTwoOne.x) * float(oTwoThree.y) - float(oTwoOne.y) * float(oTwoThree.x);

        if (fCrossProduct < 0) {
            cv::Point Temp = oCornerOne;
            oCornerOne = oCornerThree;
            oCornerThree = Temp;

            oTwoOne = oCornerOne - oCornerTwo;
            oTwoThree = oCornerThree - oCornerTwo;

            std::swap(pLandmarkIt->voCorners.at(0), pLandmarkIt->voCorners.at(2));
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
        int ID = 0;

        /// the value a certain point contributes to the ID
        int ThisPointID = 0;

        /// go thru all ID points in this landmark structure
        std::vector<cv::Point>::iterator pPointsIt;
        std::vector<int> pPointsIDs;
        for (pPointsIt = pLandmarkIt->voIDPoints.begin(); pPointsIt != pLandmarkIt->voIDPoints.end(); pPointsIt++) {
            /// first step: bring the ID point in relation to the origin of the
            /// landmark
            ThisPoint.at<float>(0, 0) = float(pPointsIt->x) - float(oCornerTwo.x);
            ThisPoint.at<float>(1, 0) = float(pPointsIt->y) - float(oCornerTwo.y);

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
            ThisPointID = ((1 << nX) << 4 * nY);
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
        vec_sort(pPointsIDs, (*pLandmarkIt).voIDPoints);

        /// assign ID to landmark
        pLandmarkIt->nID = ID;
        // LOUT("GET IDS: " << ID << std::endl);
        /// validate with the vector of available IDs
        std::vector<int>::iterator pIDLUTIt;
        for (pIDLUTIt = vnIDs.begin(); pIDLUTIt != vnIDs.end(); pIDLUTIt++) {
            if (*pIDLUTIt == ID) { /// ID matches one which is available: stop search
                break;
            }
        }

        if (pIDLUTIt != vnIDs.end()) {                  /// ID matches one which is available:
            vnIDs.erase(pIDLUTIt);                      /// remove this ID
            ++pLandmarkIt;                              /// go to next landmark
        } else {                                        /// no ID match
            voLandmarksInQueue.push_back(*pLandmarkIt); /// put this landmark in
                                                        /// queue for second
                                                        /// processing run
            io_voLandmarks.erase(pLandmarkIt);          /// delete it from valid landmark
                                                        /// list. This also is a step to next
                                                        /// landmark
        }
    }

    /// now, go thru all landmarks which did not match a valid ID and try to match
    /// them to one of the remaining
    int nThisID = 0;

    std::vector<ImgLandmark>::iterator pvoLandmarksInQueue;
    for (pvoLandmarksInQueue = voLandmarksInQueue.begin(); pvoLandmarksInQueue != voLandmarksInQueue.end();
         pvoLandmarksInQueue++) {
        nThisID = 0;

        /// same as before: finde affine transformation, but this time from landmark
        /// coordinates to image coordinates
        cv::Point oCornerOne = pvoLandmarksInQueue->voCorners.at(0);
        cv::Point oCornerTwo = pvoLandmarksInQueue->voCorners.at(1);
        cv::Point oCornerThree = pvoLandmarksInQueue->voCorners.at(2);

        cv::Point oTwoOne = oCornerOne - oCornerTwo;
        cv::Point oTwoThree = oCornerThree - oCornerTwo;

        /// make it a right hand system
        float fCrossProduct = float(oTwoOne.x) * float(oTwoThree.y) - float(oTwoOne.y) * float(oTwoThree.x);

        if (0 > fCrossProduct) {
            cv::Point oTemp = oCornerOne;
            oCornerOne = oCornerThree;
            oCornerThree = oTemp;

            oTwoOne = oCornerOne - oCornerTwo;
            oTwoThree = oCornerThree - oCornerTwo;
        }

        /// now we delete the previously detected points and go the other way around
        pvoLandmarksInQueue->voIDPoints.clear();

        cv::Mat Transform(2, 2, CV_32FC1);
        Transform.at<float>(0, 0) = float(oTwoOne.x);
        Transform.at<float>(0, 1) = float(oTwoThree.x);
        Transform.at<float>(1, 0) = float(oTwoOne.y);
        Transform.at<float>(1, 1) = float(oTwoThree.y);

        cv::Mat ThisPoint(2, 1, CV_32FC1);
        std::vector<int> pPointsIDs;

        /// go thru all possible ID points and see if the image has a high gray
        /// value there, i.e. there's light
        for (int nX = 0; nX < 4; nX++) {
            for (int nY = 0; nY < 4; nY++) {
                /// this must not be done for the three corner points of course
                if ((nX != 0 || nY != 0) && (nX != 0 || nY != 3) && (nX != 3 || nY != 0)) {
                    int ThisPointID = 0;
                    /// since we know the corners, we can go in thirds between them to see
                    /// if theres a light
                    ThisPoint.at<float>(0, 0) = float(nX) * 0.333;
                    ThisPoint.at<float>(1, 0) = float(nY) * 0.333;

                    ThisPoint = Transform * ThisPoint;

                    ThisPoint.at<float>(0, 0) += float(oCornerTwo.x);
                    ThisPoint.at<float>(1, 0) += float(oCornerTwo.y);

                    cv::Point Index(int(ThisPoint.at<float>(0, 0)), int(ThisPoint.at<float>(1, 0)));

                    /// same as for the pixel detection: see if the gray value at the
                    /// point where the light should be exceeds a threshold and thus
                    /// supports the light hypothesis
                    if (0 > Index.x || 0 > Index.y || m_oGrayImage.cols <= Index.x || m_oGrayImage.rows <= Index.y) {
                        continue;
                    }

                    if (m_cThreshold <
                        m_oGrayImage.at<unsigned char>(Index.y,
                                                       Index.x)) { /// todo: this might be extended to some area
                        ThisPointID = ((1 << (3 - nX)) << 4 * nY);
                        pvoLandmarksInQueue->voIDPoints.push_back(Index);
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
        vec_sort(pPointsIDs, (*pvoLandmarksInQueue).voIDPoints);
        // LOUT("GET IDS SECOND TRY: " << nThisID << std::endl);

        /// now, same as before, validate with available IDS
        std::vector<int>::iterator pIDLUTIt;
        for (pIDLUTIt = vnIDs.begin(); pIDLUTIt != vnIDs.end(); pIDLUTIt++) {
            if (*pIDLUTIt == nThisID) {
                break;
            }
        }

        /// if the new ID is valid, enqueue the landmark again
        if (pIDLUTIt != vnIDs.end()) {
            vnIDs.erase(pIDLUTIt);
            pvoLandmarksInQueue->nID = nThisID;
            io_voLandmarks.push_back(*pvoLandmarksInQueue);
        }
    }
    return 0;
}

void LandmarkFinder::vec_sort(std::vector<int>& ids, std::vector<cv::Point>& points) {
    int len = ids.size();
    int luecke = len / 2; // Zu Beginn ist die Lücke über den halben Array.
    bool b = true;
    while (b) {
        b = false; // b bleibt auf false, wenn kein einziges Mal etwas falsch ist.
        for (int i = 0; i < len; i++) {
            if (luecke + i >= len) // Schutz vor Speicherfehlern
            {
                break;
            }
            if (ids[i] > ids[i + luecke]) // überprüft ob die zwei Elemente falsch herum sind
            {
                std::swap(ids[i], ids[i + luecke]); // wenn ja -> vertauschen
                std::swap(points[i], points[i + luecke]);
                b = true;
            }
        }
        luecke = luecke / 1.3; // Lücke verkleinern für nächsten Durchlauf
        if (luecke < 1) {
            luecke = 1;
        }
    }
}
