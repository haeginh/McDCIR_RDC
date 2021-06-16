#include "CharucoSync.hh"

CharucoSync::CharucoSync()
:getPose(false)
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

    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    board = cv::aruco::CharucoBoard::create(5, 7, 0.04f, 0.02f, dictionary);
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

void CharucoSync::EstimatePose(Mat color, Mat &display, Vec3d &rvec, Vec3d &tvec)
{
    color.copyTo(display);
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    cv::aruco::detectMarkers(color, board->dictionary, markerCorners, markerIds, params);
    // if at least one marker detected
    if (markerIds.size() > 0)
    {
        cv::aruco::drawDetectedMarkers(display, markerCorners, markerIds);
        std::vector<cv::Point2f> charucoCorners;
        std::vector<int> charucoIds;
        cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, color, board, charucoCorners, charucoIds, camMatrix, distCoeffs);
        // if at least one charuco corner detected
        if (charucoIds.size() > 0)
        {
            cv::Scalar color = cv::Scalar(255, 0, 0);
            cv::aruco::drawDetectedCornersCharuco(display, charucoCorners, charucoIds, color);
            //cv::Vec3d rvec, tvec;
            // cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board, cameraMatrix, distCoeffs, rvec, tvec);
            bool valid = cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board, camMatrix, distCoeffs, rvec, tvec);
            // if charuco pose is valid
            if (valid)
                cv::aruco::drawAxis(display, camMatrix, distCoeffs, rvec, tvec, 0.1f);
        }
    }
}