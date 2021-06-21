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
void CharucoSync::EstimatePose(const Mat &color, Vec3d &rvec, Vec3d &tvec)
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
        std::for_each(markerCorners.begin(), markerCorners.end(), [](vector<Point2f> &vec)
                      {
                          for (Point2f &point : vec)
                              point += Point2f(cropRect.tl());
                      });
        cv::aruco::drawDetectedMarkers(display, markerCorners, markerIds);
        std::vector<cv::Point2f> charucoCorners;
        std::vector<int> charucoIds;
        cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, color, board, charucoCorners, charucoIds, camMatrix, distCoeffs);
        // if at least one charuco corner detected
        if (charucoIds.size() > 0)
        {
            cv::aruco::drawDetectedCornersCharuco(display, charucoCorners, charucoIds);
            bool valid = cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board, camMatrix, distCoeffs, rvec, tvec);
            // if charuco pose is valid
            if (valid)
                cv::aruco::drawAxis(display, camMatrix, distCoeffs, rvec, tvec, 0.1f);
            if (getPose)
            {
                double angle = norm(rvec);
                Vector3d axis(rvec(0) / angle, rvec(1) / angle, rvec(2) / angle);
                Quaterniond q(AngleAxisd(angle, axis)); q.normalize();
                quaternions.push_back(Vector4d(q.x(), q.y(), q.z(), q.w()));
                tvec_sum += tvec;
            }
        }
    }
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
    else if (cropRect.width > 0){
        imshow("crop img.", display(cropRect));
        cv::rectangle(display, cropRect, CV_RGB(255, 255, 0), 3);
    }
    resize(display, display, Size(display.cols * sf, display.rows * sf));
    putText(display, "number of data: "+to_string(quaternions.size()), Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255.f,0.f,0.f), 1.2);
    if(getPose) 
        putText(display, "obtaining pose data..", Point(10, 40), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255.f,0.f,0.f), 1);
    imshow("Synchronization", display);
}

#include "functions.hh"
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
void CharucoSync::ShowAvgValue(const Mat &color)
{
    AngleAxisd avg(Quaterniond((quaternionAverage(quaternions)))); 

    Vec3d rvec;
    eigen2cv(avg.axis(), rvec);
    rvec *= avg.angle();
    Vec3d tvec_avg = tvec_sum / (double)quaternions.size();

    color.copyTo(display);
    cv::aruco::drawAxis(display, camMatrix, distCoeffs, rvec, tvec_avg, 0.1f);
    resize(display, display, Size(display.cols * sf, display.rows * sf));
    imshow("Synchronization", display);
    waitKey(0);
}

void CharucoSync::SetScalingFactor(float s)
{
    sf = s;
    sfInv = 1 / s;
}
