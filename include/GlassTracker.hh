#ifndef GlassTracker_hh
#define GlassTracker_hh

#include <map>
#include <opencv2/aruco.hpp>
#include "Kinect2OpenCV.hh"

using namespace cv;
using namespace std;


class GlassTracker
{
    public:
    GlassTracker();
    virtual ~GlassTracker();

    bool ReadDetectorParameters(string filename);
    bool ReadCameraParameters(string filename);
    void SetNewFrame(Mat &_color)
    {
        _color.copyTo(color);
        _color.copyTo(display);
    }
    bool ProcessCurrentFrame();
    void Render();
    void SetScalingFactor(float s);

    private:
    Ptr<aruco::Dictionary> dictionary;
    Ptr<aruco::DetectorParameters> params;
    Mat camMatrix, distCoeffs;
    Mat color, display;

    map<int, vector<Point2f>> corner_cumul;
    vector<Vec3d> pose_cumul; //rvec XYZ, tvec
    int cumulCount;
    vector<double> coeffX;
    vector<double> coeffY;
    float markerLength; 

    float sf;
};
#endif