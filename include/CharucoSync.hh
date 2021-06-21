#ifndef CharucoSync_hh
#define CharucoSync_hh

#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <string>

using namespace std;
using namespace cv;
class CharucoSync
{
    public:
    CharucoSync();
    virtual ~CharucoSync();
    bool SetParameters(string camParam, string detParam);
    void EstimatePose(Mat color, Vec3d &rvec, Vec3d &tvec);
    void Render();
    void SetScalingFactor(float s);
    
    private:
    cv::Ptr<cv::aruco::CharucoBoard> board;
    cv::Ptr<cv::aruco::DetectorParameters> params;
    Mat camMatrix;
    Mat distCoeffs;
    bool getPose;
    Mat display;
    float sf;
};

#endif
