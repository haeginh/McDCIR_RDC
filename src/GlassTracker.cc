#include "GlassTracker.hh"
#include <opencv2/core/eigen.hpp>
#include <igl/slice.h>

bool clicked;
Point2i P1, P2;
Rect cropRect;
float sfInv;
void onMouseCropImage(int event, int x, int y, int f, void *param);

GlassTracker::GlassTracker()
    : cumulCount(0)
{
    dictionary = aruco::generateCustomDictionary(6, 4, 2);
    params = aruco::DetectorParameters::create();
    //marker position (relative position-based methodology)
    coeffX = {15, -1, -15, 15, -1, -15};
    coeffY = {-10, -10, -10, 10, 10, 10};
}

GlassTracker::~GlassTracker()
{
}

bool GlassTracker::ReadDetectorParameters(string filename)
{
    FileStorage fs(filename, FileStorage::READ);
    if (!fs.isOpened())
        return false;
    fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
    fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
    fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params->markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params->minOtsuStdDev;
    fs["errorCorrectionRate"] >> params->errorCorrectionRate;
    fs["markerLength"] >> markerLength;
    return true;
}

bool GlassTracker::ReadCameraParameters(string filename)
{
    FileStorage fs(filename, FileStorage::READ);
    if (!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}

bool GlassTracker::ProcessCurrentFrame(Eigen::Quaterniond &q_current, Eigen::Vector3d &t_current)
{
    Mat cropImg;
    if (cropRect.width > 0)
        cropImg = color(cropRect).clone();
    else
        color.copyTo(cropImg);

    // detect markers
    vector<int> ids;
    vector<vector<Point2f>> cornersCrop, cornersWhole, rejected;
    aruco::detectMarkers(cropImg, dictionary, cornersCrop, ids, params, rejected);

    // compare with the previous result
    if (ids.size() > 0)
    {
        bool isNewPose(false);
        for (int i = 0; i < ids.size(); i++)
        {
            vector<Point2f> points;
            for (auto p : cornersCrop[i])
                points.push_back(Point2f(p.x + P1.x, p.y + P1.y));
            cornersWhole.push_back(points);
            if (isNewPose)
                continue;
            if (corner_cumul.find(ids[i]) != corner_cumul.end())
            {
                Point2f oldCen(0, 0), newCen(0, 0);
                for (int n = 0; n < 4; n++)
                {
                    newCen += points[n];
                    oldCen += corner_cumul[ids[i]][n];
                }
                Point2f vec = oldCen - newCen;
                if (vec.dot(vec) > 40)
                {
                    corner_cumul.clear();
                    //pose_cumul.clear();
                    isNewPose = true;
                    cumulCount = 0;
                    //                       cout<<endl<<"new Pose!------------------------"<<endl;
                }
                else
                    cumulCount++;
            }
        }
        for (int i = 0; i < ids.size(); i++)
            corner_cumul[ids[i]] = cornersWhole[i];

        vector<Vec3d> rvecs, tvecs;
        aruco::estimatePoseSingleMarkers(cornersWhole, markerLength, camMatrix, distCoeffs, rvecs,
                                         tvecs);

        //rvec
        std::vector<Eigen::Vector4d> q_vec;
        for (Vec3d v : rvecs)
        {
            double angle = norm(v);
            Eigen::Vector3d axis(v(0) / angle, v(1) / angle, v(2) / angle);
            Eigen::Quaterniond q(Eigen::AngleAxisd(angle, axis));
            q.normalize();
            q_vec.push_back(Eigen::Vector4d(q.x(), q.y(), q.z(), q.w()));
        }
        Eigen::Quaterniond q_avg(quaternionAverage(q_vec));
        if (cumulCount > 1)
            q_cumul = q_cumul.slerp(1.f / (cumulCount + 1.f), q_avg);
        else
            q_cumul = q_avg;

        Eigen::AngleAxisd avg(q_cumul);
        Vec3d rvec;
        eigen2cv(avg.axis(), rvec);
        rvec *= avg.angle();
        Vec3d axisX, axisY, axisZ;
        eigen2cv(q_cumul * Eigen::Vector3d(1.f, 0.f, 0.f), axisX);
        eigen2cv(q_cumul * Eigen::Vector3d(0.f, 1.f, 0.f), axisY);
        eigen2cv(q_cumul * Eigen::Vector3d(0.f, 0.f, 1.f), axisZ);

        //tvec
        Vec3d tvec(0, 0, 0);
        for (int i = 0; i < ids.size(); i++)
        {
            int id = ids[i];
            Vec3d xTrans = coeffX[id] * axisX;
            Vec3d yTrans = coeffY[id] * axisY;
            tvec += (tvecs[i] + xTrans + yTrans);
        }
        tvec *= 1.f / ids.size();
        if (cumulCount > 1)
        {
            tvec += tvec_cumul * (cumulCount - 1);
            tvec /= (double)cumulCount;
        }
        tvec_cumul = tvec;
        aruco::drawAxis(display, camMatrix, distCoeffs, rvec, tvec,
                        markerLength * 3.f);

        q_current = q_cumul;
        cv2eigen(tvec_cumul, t_current);
    }
    else
    {
        corner_cumul.clear();
        cumulCount = 0;
        return false;
    }

    //draw result
    if (corner_cumul.size() > 0)
    {
        vector<vector<Point2f>> cornersTemp;
        vector<int> idsTemp;
        for (auto iter : corner_cumul)
        {
            cornersTemp.push_back(iter.second);
            idsTemp.push_back(iter.first);
        }
        aruco::drawDetectedMarkers(display, cornersTemp, idsTemp);
    }
    return true;
}

void GlassTracker::Render(bool showResult)
{
    setMouseCallback("Glass Tracker", onMouseCropImage, &display);
    if (clicked)
        cv::rectangle(display, P1, P2, CV_RGB(255, 255, 0), 3);
    else if (cropRect.width > 0)
        cv::rectangle(display, cropRect, CV_RGB(255, 255, 0), 3);
    resize(display, display, Size(display.cols * sf, display.rows * sf));
    if (showResult)
    {
        putText(display, "cummulated data #: " + to_string(cumulCount), Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255.f, 0.f, 0.f), 1.2);
        putText(display, "q: " + to_string(q_cumul.w()) + ", " + to_string(q_cumul.x()) + ", " + to_string(q_cumul.y()) + ", " + to_string(q_cumul.z()), Point(10, 40), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255.f, 0.f, 0.f), 1.2);
        putText(display, "tvec: " + to_string(tvec_cumul(0)) + ", " + to_string(tvec_cumul(1)) + ", " + to_string(tvec_cumul(2)), Point(10, 60), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255.f, 0.f, 0.f), 1.2);
    }
    imshow("Glass Tracker", display);
    waitKey(1);
}

void GlassTracker::SetScalingFactor(float s)
{
    sf = s;
    sfInv = 1 / s;
}

void onMouseCropImage(int event, int x, int y, int f, void *param)
{
    switch (event)
    {
    case EVENT_LBUTTONDOWN:
        clicked = true;
        P1.x = x * sfInv;
        P1.y = y * sfInv;
        P2.x = x * sfInv;
        P2.y = y * sfInv;
        break;
    case EVENT_LBUTTONUP:
        P2.x = x * sfInv;
        P2.y = y * sfInv;
        clicked = false;
        break;
    case EVENT_MOUSEMOVE:
        if (clicked)
        {
            P2.x = x * sfInv;
            P2.y = y * sfInv;
        }
        break;
    case EVENT_RBUTTONUP:
        clicked = false;
        P1.x = 0;
        P1.y = 0;
        P2.x = 0;
        P2.y = 0;
        break;
    default:
        break;
    }

    if (clicked)
    {
        if (P1.x > P2.x)
        {
            cropRect.x = P2.x;
            cropRect.width = P1.x - P2.x;
        }
        else
        {
            cropRect.x = P1.x;
            cropRect.width = P2.x - P1.x;
        }

        if (P1.y > P2.y)
        {
            cropRect.y = P2.y;
            cropRect.height = P1.y = P2.y;
        }
        else
        {
            cropRect.y = P1.y;
            cropRect.height = P2.y - P1.y;
        }
    }
}