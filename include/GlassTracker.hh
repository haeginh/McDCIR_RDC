#ifndef GlassTracker_hh
#define GlassTracker_hh

#include <map>
#include <opencv2/aruco.hpp>
#include "Kinect2OpenCV.hh"
#include "functions.hh"

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
    bool ProcessCurrentFrame(Eigen::Quaterniond &q_current, Eigen::Vector3d &t_current);
    void Render(bool showResult = true);

    void SetScalingFactor(float s);

    private:
    Ptr<aruco::Dictionary> dictionary;
    Ptr<aruco::DetectorParameters> params;
    Mat camMatrix, distCoeffs;
    Mat color, display;

    map<int, vector<Point2f>> corner_cumul;
    Vec3d tvec_cumul;
    Eigen::Quaterniond q_cumul;
    int cumulCount;
    vector<double> coeffX;
    vector<double> coeffY;
    float markerLength; 

    float sf;
};
#endif