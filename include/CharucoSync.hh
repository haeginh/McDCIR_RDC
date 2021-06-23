#ifndef CharucoSync_hh
#define CharucoSync_hh

#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <Eigen/Geometry>

#include <string>

using namespace std;
using namespace cv;
using namespace Eigen;

class CharucoSync
{
    public:
    CharucoSync();
    virtual ~CharucoSync();
    bool SetParameters(string camParam, string detParam);
    void EstimatePose(const Mat &color, Vec3d &rvec, Vec3d &tvec);
    void Render();
    void ShowAvgValue(const Mat &color);
    void WriteTransformationData(string file);

    void ClearData(){quaternions.clear(); tvec_sum = Vec3d(0,0,0);}
    void TickSwitch(){
        getPose = !getPose;
    }
    void SetScalingFactor(float s);
    
    private:
    cv::Ptr<cv::aruco::CharucoBoard> board;
    cv::Ptr<cv::aruco::DetectorParameters> params;
    Mat camMatrix;
    Mat distCoeffs;
    bool getPose;
    Mat display;
    float sf;

    //avg values
    vector<Vector4d> quaternions;
    Vec3d tvec_sum;
};

#endif
