//
// Created by bandera on 28.03.16.
//

#include "DebugVisualizer.h"
#include "CoordinateTransformations.h"

using namespace stargazer;

DebugVisualizer::DebugVisualizer() : m_window_mode(CV_WINDOW_NORMAL), m_wait_time(1) {
}
// TODO make this a singleton
// TODO make waitKey unique

void DebugVisualizer::prepareImg(cv::Mat& img) {
    if (img.type() == CV_8UC1) {
        // input image is grayscale
        cvtColor(img, img, CV_GRAY2RGB);
    } else {
    }
}
void DebugVisualizer::ShowImage(cv::Mat& img, std::string name) {
    cv::namedWindow(name, m_window_mode);
    cv::imshow(name, img);
    cv::waitKey(m_wait_time);
}

cv::Mat DebugVisualizer::ShowPoints(const cv::Mat& img, const std::vector<cv::Point> points) {
    cv::Mat temp = img.clone();
    prepareImg(temp);
    for (auto& point : points)
        circle(temp, point, 1, cv::Scalar(73, 119, 0), 2); // FZI Green
    ShowImage(temp, "Points");
    return temp;
}

cv::Mat DebugVisualizer::ShowClusters(const cv::Mat& img, const std::vector<std::vector<cv::Point>> points) {
    cv::Mat temp = img.clone();
    prepareImg(temp);
    for (auto& group : points) {
        cv::Point median(0, 0);
        for (auto& point : group) {
            median += point;
            circle(temp, point, 1, cv::Scalar(73, 119, 0), 2); // FZI Green
        }
        median *= 1.0 / group.size();
        double variance = 0.0;
        for (auto& point : group) {
            variance += std::pow(median.x - point.x, 2) + std::pow(median.y - point.y, 2);
        }
        variance /= (group.size());
        int radius = static_cast<int>(2 * sqrt(variance));

        circle(temp, median, radius, cv::Scalar(163, 101, 0), 2); // FZI Blue
    }
    ShowImage(temp, "Clusters");
    return temp;
}

void DebugVisualizer::DrawLandmarks(cv::Mat& img, const std::vector<ImgLandmark>& landmarks) {

    for (auto& lm : landmarks) {
        for (auto& imgPoint : lm.voCorners) {
            circle(img, imgPoint, 1, cv::Scalar(73, 119, 0), 2); // FZI Green
        }
        for (auto& imgPoint : lm.voIDPoints) {
            circle(img, imgPoint, 1, cv::Scalar(73, 119, 0), 2); // FZI Green
        }
        cv::Point median{(lm.voCorners[2].x + lm.voCorners[0].x) / 2, (lm.voCorners[2].y + lm.voCorners[0].y) / 2};
        double radius =
            sqrt(pow(lm.voCorners[2].x - lm.voCorners[0].x, 2) + pow(lm.voCorners[2].y - lm.voCorners[0].y, 2));
        circle(img, median, radius, cv::Scalar(163, 101, 0), 2); // FZI Blue

        std::string text = "ID: ";
        text += std::to_string(lm.nID);
        cv::Point imgPoint = lm.voCorners.front();
        imgPoint.x += 25;
        imgPoint.y += 25;
        putText(img, text, imgPoint, 2, 0.4, cvScalar(0, 0, 0));
    }
}

void DebugVisualizer::DrawLandmarks(cv::Mat& img, const landmark_map_t& landmarks,
                                    const camera_params_t& camera_intrinsics, const pose_t& ego_pose) {
    cv::Point imgPoint;

    for (auto& lm : landmarks) {
        for (size_t i = 0; i < lm.second.points.size(); i++) {
            auto& pt = lm.second.points[i];

            // Convert point into camera frame
            double x = 0.0;
            double y = 0.0;

            transformWorld2Img(&pt[(int)POINT::X], &pt[(int)POINT::Y], &pt[(int)POINT::Z], ego_pose.data(),
                               camera_intrinsics.data(), &x, &y);
            imgPoint.x = static_cast<int>(x);
            imgPoint.y = static_cast<int>(y);

            /// Corner Points
            circle(img, imgPoint, 4, cv::Scalar(39, 157, 236), 2); // FZI Red
        }

        std::string text = "ID: ";
        text += std::to_string(lm.second.id);
        imgPoint.x += 25;
        imgPoint.y += 25;
        putText(img, text, imgPoint, 2, 0.4, cvScalar(0, 0, 0));
    }
}
