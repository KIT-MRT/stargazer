#include <CeresLocalizer.h>
#include <DebugVisualizer.h>
#include "LandmarkFinder.h"

using namespace stargazer;
using namespace std;

int main(int argc, char** argv) {
    if (argc != 3) {
        cout << " Usage: " << argv[0] << " <image_file> <config_file>" << endl;
        return -1;
    }
    cv::Mat input_image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR); // Read the filea

    DebugVisualizer debugVisualizer;
    debugVisualizer.SetWaitTime(-1); // Wait until user has pressed key
    debugVisualizer.SetWindowMode(CV_WINDOW_NORMAL);

    LandmarkFinder landmarkFinder(argv[2]);
    std::vector<ImgLandmark> detected_landmarks;
    // DetectLandmarks expects gray-value-images (CV_8UC1)
    cv::Mat input_image_grey;
    cv::cvtColor(input_image, input_image_grey, CV_BGR2GRAY);
    landmarkFinder.DetectLandmarks(input_image_grey, detected_landmarks);

    cout << "Displaying images, press any key to continue.... " << endl;

    // Invert images for better visibilty
    cv::bitwise_not(landmarkFinder.grayImage_, landmarkFinder.grayImage_);
    cv::bitwise_not(landmarkFinder.filteredImage_, landmarkFinder.filteredImage_);

    // Show images
    debugVisualizer.ShowImage(landmarkFinder.grayImage_, "Gray Image");
    debugVisualizer.ShowImage(landmarkFinder.filteredImage_, "Filtered Image");

    // Show detections
    auto point_img = debugVisualizer.ShowPoints(landmarkFinder.filteredImage_, landmarkFinder.clusteredPixels_);
    auto cluster_img = debugVisualizer.ShowClusters(landmarkFinder.filteredImage_, landmarkFinder.clusteredPoints_);

    // Show landmarks
    cv::Mat temp;
    cvtColor(landmarkFinder.grayImage_, temp, CV_GRAY2BGR);
    debugVisualizer.DrawLandmarks(temp, detected_landmarks);
    debugVisualizer.ShowImage(temp, "Detected Landmarks");

    // Localize
    const std::string args(argv[2]);
    CeresLocalizer localizer(args);
    localizer.UpdatePose(detected_landmarks, 0.0);
    cout << localizer.getSummary().FullReport() << endl << endl;

    pose_t pose = localizer.getPose();
    cout << "Pose is x=" << pose[(int)POSE::X] << " y=" << pose[(int)POSE::Y] << " z=" << pose[(int)POSE::Z]
         << " rx=" << pose[(int)POSE::Rx] << " ry=" << pose[(int)POSE::Ry] << " rz=" << pose[(int)POSE::Rz] << endl;

    return EXIT_SUCCESS;
}
