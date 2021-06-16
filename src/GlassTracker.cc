#include "GlassTracker.hh"

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
    coeffX = {15,-1,-15,15,-1,-15};
    coeffY = {-10,-10,-10,10,10,10};
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

bool GlassTracker::ProcessCurrentFrame()
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

        Vec3d tvec(0, 0, 0), axisX(0, 0, 0), axisY(0, 0, 0), axisZ(0, 0, 0);

        // six variations (x -> z -> y)
        for (int i = 0; i < ids.size(); i++)
        {
            Affine3d rot(rvecs[i]);
            axisX += rot * Vec3d(1, 0, 0);
            axisY += rot * Vec3d(0, 1, 0);
            //                axisZ += rot*Vec3d(0,0,1);
        }

        //            ofs<<endl;
        if (cumulCount > 1)
        {
            axisX += pose_cumul[0] * (cumulCount - 1);
            axisY += pose_cumul[1] * (cumulCount - 1);
        }
        axisX = normalize(axisX);
        axisY = normalize(axisY);
        axisZ = axisX.cross(axisY);
        axisY = axisZ.cross(axisX);
        double rotData[9] = {axisX(0), axisY(0), axisZ(0), axisX(1), axisY(1), axisZ(1), axisX(2), axisY(2), axisZ(2)};
        Affine3d rot;
        rot.rotation(cv::Mat(3, 3, CV_64F, rotData));

        for (int i = 0; i < ids.size(); i++)
        {
            int id = ids[i];
            Vec3d xTrans = coeffX[id] * axisX;
            Vec3d yTrans = coeffY[id] * axisY;
            //              cout<<id<<tvecs[i]<<endl;
            tvec += (tvecs[i] + xTrans + yTrans);
        }
        tvec *= 1.f / ids.size();
        if (cumulCount > 1)
        {
            //               cout<<tvec<<endl;
            tvec += pose_cumul[3] * (cumulCount - 1);
            tvec /= (double)cumulCount;
        }
        // cout<<tvec<<endl;
        aruco::drawAxis(display, camMatrix, distCoeffs, rot.rvec(), tvec,
                        markerLength * 3.f);

        pose_cumul = {axisX, axisY, axisZ, tvec};
    }
    else
    {
        corner_cumul.clear();
        cumulCount = 0;
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
}

void GlassTracker::Render()
{
    setMouseCallback("Glass Tracker", onMouseCropImage, &display);
    if (clicked)
        cv::rectangle(display, P1, P2, CV_RGB(255, 255, 0), 3);
    else if (cropRect.width > 0)
        cv::rectangle(display, cropRect, CV_RGB(255, 255, 0), 3);
    resize(display, display, Size(display.cols*sf, display.rows*sf));
    imshow("Glass Tracker", display);
    waitKey(1);
}

void GlassTracker::SetScalingFactor(float s){sf = s; sfInv = 1/s;}

void onMouseCropImage(int event, int x, int y, int f, void *param)
{
    switch (event)
    {
    case EVENT_LBUTTONDOWN:
        clicked = true;
        P1.x = x*sfInv;
        P1.y = y*sfInv;
        P2.x = x*sfInv;
        P2.y = y*sfInv;
        break;
    case EVENT_LBUTTONUP:
        P2.x = x*sfInv;
        P2.y = y*sfInv;
        clicked = false;
        break;
    case EVENT_MOUSEMOVE:
        if (clicked)
        {
            P2.x = x*sfInv;
            P2.y = y*sfInv;
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