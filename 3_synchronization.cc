#include <iostream>
#include "CharucoSync.hh"
#include "Kinect2OpenCV.hh"
#include <Utilities.h>

extern Rect cropRect;
void PrintUsage()
{
    cout << "<Usage>" << endl;
    cout << "./3_synchronization [outputfile]" << endl;
    exit(1);
}

using namespace Eigen;
using namespace std;

int main(int argc, char **argv)
{
    //trackin option configuration3
    string detParm("detector_params.yml");
    string camParm("camera2160.yml");
    if (argc != 2)
        PrintUsage();

    // Start camera
    k4a_device_t device = nullptr;
    VERIFY(k4a_device_open(0, &device), "Open K4A Device failed!");

    // Start camera. Make sure depth camera is enabled.
    k4a_device_configuration_t deviceConfig = get_default_config();
    VERIFY(k4a_device_start_cameras(device, &deviceConfig), "Start K4A cameras failed!");

    // Get calibration information
    k4a_calibration_t sensorCalibration;
    VERIFY(k4a_device_get_calibration(device, deviceConfig.depth_mode, deviceConfig.color_resolution, &sensorCalibration),
           "Get depth camera calibration failed!");
    int depthWidth = sensorCalibration.depth_camera_calibration.resolution_width;
    int depthHeight = sensorCalibration.depth_camera_calibration.resolution_height;

    // Synchronization
    CharucoSync sync;
    sync.SetParameters(camParm, detParm);
    sync.SetScalingFactor(0.3f);
    bool getData(false);
    waitKey(1000);
    while (1)
    {
        k4a_capture_t sensorCapture = nullptr;
        k4a_wait_result_t getCaptureResult = k4a_device_get_capture(device, &sensorCapture, 0);

        if (getCaptureResult == K4A_WAIT_RESULT_FAILED)
        {
            std::cout << "Get img capture returned error: " << getCaptureResult << std::endl;
            break;
        }
        else if (getCaptureResult == K4A_WAIT_RESULT_TIMEOUT)
            continue;

        Mat color;
        Vec3d rvec, tvec;
        k4a_image_t color_img = k4a_capture_get_color_image(sensorCapture);
        color = color_to_opencv(color_img);
        k4a_image_release(color_img);
        k4a_capture_release(sensorCapture);
        sync.EstimatePose(color, rvec, tvec);

        sync.Render();
        char key = waitKey(1);
        if (key == 'q')
            break;
        else if (key == 'a')
        {
            sync.ShowAvgValue(color);
            char key2 = waitKey(0);
            if(key2=='q') break;
        }
        else if (key == 'c')
            sync.ClearData();
        else if (key == 'g')
            sync.TickSwitch();
    }
    k4a_device_close(device);
    sync.WriteTransformationData(string(argv[1]));

    return EXIT_SUCCESS;
}


