#include "CharucoSync.hh"

CharucoSync::CharucoSync()
    : getPose(false)
{
}

CharucoSync::~CharucoSync()
{
}

bool CharucoSync::SetParameters(string camParm, string detParam)
{
    cv::FileStorage fs(camParm, cv::FileStorage::READ);
    if (!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;

    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    board = cv::aruco::CharucoBoard::create(5, 7, 0.0765f, 0.0535f, dictionary);
    params = cv::aruco::DetectorParameters::create();

    FileStorage fs2(detParam, FileStorage::READ);
    if (!fs2.isOpened())
        return false;
    fs2["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
    fs2["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
    fs2["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
    fs2["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
    fs2["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
    fs2["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
    fs2["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
    fs2["minCornerDistanceRate"] >> params->minCornerDistanceRate;
    fs2["minDistanceToBorder"] >> params->minDistanceToBorder;
    fs2["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
    fs2["cornerRefinementMethod"] >> params->cornerRefinementMethod;
    fs2["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
    fs2["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
    fs2["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
    fs2["markerBorderBits"] >> params->markerBorderBits;
    fs2["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
    fs2["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
    fs2["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
    fs2["minOtsuStdDev"] >> params->minOtsuStdDev;
    fs2["errorCorrectionRate"] >> params->errorCorrectionRate;

    return true;
}

extern Rect cropRect;
void CharucoSync::EstimatePose(Mat color, Vec3d &rvec, Vec3d &tvec)
{
    color.copyTo(display);

    Mat cropImg;
    if (cropRect.width > 0 && cropRect.height > 0)
        cropImg = color(cropRect).clone();
    else
        color.copyTo(cropImg);

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    cv::aruco::detectMarkers(cropImg, board->dictionary, markerCorners, markerIds, params);
    // if at least one marker detected
    if (markerIds.size() > 0)
    {
        std::for_each(markerCorners.begin(), markerCorners.end(), [](vector<Point2f> &vec){for(Point2f &point:vec) point +=Point2f(cropRect.tl());});
        cv::aruco::drawDetectedMarkers(display, markerCorners, markerIds);
        std::vector<cv::Point2f> charucoCorners;
        std::vector<int> charucoIds;
        cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, color, board, charucoCorners, charucoIds, camMatrix, distCoeffs);
        // if at least one charuco corner detected
        if (charucoIds.size() > 0)
        {
            cv::Scalar color = cv::Scalar(255, 0, 0);
            cv::aruco::drawDetectedCornersCharuco(display, charucoCorners, charucoIds, color);
            bool valid = cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board, camMatrix, distCoeffs, rvec, tvec);
            // if charuco pose is valid
            if (valid)
                cv::aruco::drawAxis(display, camMatrix, distCoeffs, rvec, tvec, 0.1f);
        }
    }
    imshow("crop", cropImg);
}

extern bool clicked;
extern Point2i P1, P2;
extern float sfInv;
extern void onMouseCropImage(int event, int x, int y, int f, void *param);
void CharucoSync::Render()
{
    setMouseCallback("Synchronization", onMouseCropImage);
    if (clicked)
        cv::rectangle(display, P1, P2, CV_RGB(255, 255, 0), 3);
    else if (cropRect.width > 0)
        cv::rectangle(display, cropRect, CV_RGB(255, 255, 0), 3);
    resize(display, display, Size(display.cols * sf, display.rows * sf));
    imshow("Synchronization", display);
    waitKey(1);
}

void CharucoSync::SetScalingFactor(float s)
{
    sf = s;
    sfInv = 1 / s;
}
