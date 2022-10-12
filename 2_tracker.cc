#include <iostream>
#include <ctime>
#include <functions.hh>
#include <functional>

#include "OCR_main.hh"

#include "bodytracking.hh"
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/SparseCore>
#define PI 3.14159265358979323846

#include "ClientSocket.hh"
#include "GlassTracker.hh"
#define BE_NUM 22
#define C_NUM 24

// #define TEST
// #define OCR_SETTINGS
#define ABT_DISPLAY
#define GLASS_DISPLAY
extern Rect cropRect;
void PrintUsage()
{
    cout << "<Usage>" << endl;
    cout << "./2_tracker [ip] [port] [worker#] [tracking options]" << endl;
    cout << "<tracking options>" << endl;
    cout << "-m motion, -g glass, -b bed" << endl;
    exit(1);
}

using namespace Eigen;
using namespace std;

typedef Triplet<double> T;

int main(int argc, char **argv)
{
    int id = atoi(argv[3]);
    int opt = atoi(argv[4]);
    if (opt != 1 && opt & 1)
    {
        cout << "WORKER: OCR and KINECT based tracking options cannot selected at the same time" << endl;
        return 1;
    }
    // environment variable check
    string dir, envV;
    if (opt & 1)
        envV = "DCIR_OCR_DIR_PATH";
    else
        envV = "DCIR_TRACKER_DIR_PATH";
    dir = getenv(envV.c_str());
    if (dir.empty())
    {
        cout << "WORKER: please check environment variable " + envV << endl;
        return 1;
    }

    // connection to server
    int client_socket;
    struct sockaddr_in serverAddress;
    socklen_t server_addr_size;
    float sendBuff[1500];
    sendBuff[0] = id;
    sendBuff[1] = opt;
    char recvBuff[4];

    double kinectRecordF = 0.3;

    // --OCR
    if (opt & 1)
    {
        OcrMain ocr_main(dir);
        if (!getenv("DCIR_OCR_VIDEO"))
            ocr_main.SetSource(SOURCE::CAPTUREBOARD);
        else
            ocr_main.SetSource(SOURCE::VIDEO, string(getenv("DCIR_OCR_VIDEO")));
        // ocr_main.Initialize("config_video2.yml");
        ocr_main.Initialize("config_20220914_ocr.yml");

        // for settings
#ifdef OCR_SETTINGS
        ocr_main.RenderForSetting();
        ocr_main.RenderForLearning();
        return 0;
#endif
        // ssize_t sentBytes;

        memset(&serverAddress, 0, sizeof(serverAddress));
        serverAddress.sin_family = AF_INET;
        serverAddress.sin_addr.s_addr = inet_addr(argv[1]);
        serverAddress.sin_port = htons(atoi(argv[2]));

        if ((client_socket = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
        {
            cout << "WORKER: socket generation failed" << endl;
            return false;
        }
        else
            cout << "WORKER: socket generation -> success!" << endl;
        struct timeval optVal = {0, 50};
        int optLen = sizeof(optVal);
        setsockopt(client_socket, SOL_SOCKET, SO_RCVTIMEO, &optVal, optLen);
        // first msg.
        ocr_main.SetRecord(true);
        while (1)
        {
            sendto(client_socket, sendBuff, 1500 * 4, 0, (struct sockaddr *)&serverAddress, sizeof(serverAddress));
            if (!ocr_main.Render(&sendBuff[2]))
                break;
        }
        return 0;
    }

    // --KINECT connected functions
    // exceptions

    // read sync data
    string dump;
    ifstream ifs(dir + "/sync.txt");
    ifs >> dump >> sendBuff[2] >> sendBuff[3] >> sendBuff[4] >> sendBuff[5];
    ifs >> dump >> sendBuff[6] >> sendBuff[7] >> sendBuff[8];

    // initialize devices
    // KINECT variables
    k4a_device_t device;
    k4a_calibration_t sensorCalibration;
    int depthWidth, depthHeight;
    k4abt_tracker_t tracker;
    // glass tracker variable
    GlassTracker glassTracker;
    // body tracker variable
    vector<int> poseJoints = {0, 1, 2, 4, 5, 6, 7, 26, 11, 12, 13, 14, 18, 19, 20, 22, 23, 24, 8, 15, 21, 25, 28, 30};
    // map<int, map<int, int>> colorIdx;
    map<int, pair<int, bool>> colorIdx; // blue is true
    int colorMargin(10);

    string colorComment;
    if (argc > 5)
    {
        for (int i = 0; i < atoi(argv[5]); i++)
        {
            colorIdx[atoi(argv[6 + i * 3])].first = atoi(argv[7 + i * 3]);
            colorIdx[atoi(argv[6 + i * 3])].second = bool(atoi(argv[8 + i * 3]));
            string maskColor("red");
            if (colorIdx[atoi(argv[6 + i * 3])].second)
                maskColor = "blue";
            colorComment += string(argv[6 + i * 3]) + ":" + string(argv[7 + i * 3]) + "/" + maskColor + " | ";
        }
    }

    // Start camera
    VERIFY(k4a_device_open(0, &device), "Open K4A Device failed!");

    // Start camera. Make sure depth camera is enabled.
    k4a_device_configuration_t deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    deviceConfig.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_2160P;
    deviceConfig.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED; // No need for depth during calibration
    deviceConfig.camera_fps = K4A_FRAMES_PER_SECOND_15;     // Don't use all USB bandwidth
    deviceConfig.synchronized_images_only = true;
    VERIFY(k4a_device_start_cameras(device, &deviceConfig), "Start K4A cameras failed!");
    cout << "WORKER" << id << ": KINECT turned on" << endl;
    // Get calibration information
    VERIFY(k4a_device_get_calibration(device, deviceConfig.depth_mode, deviceConfig.color_resolution, &sensorCalibration),
           "Get depth camera calibration failed!");
    depthWidth = sensorCalibration.depth_camera_calibration.resolution_width;
    depthHeight = sensorCalibration.depth_camera_calibration.resolution_height;

    // tracker
    k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
    tracker_config.processing_mode = K4ABT_TRACKER_PROCESSING_MODE_GPU_CUDA;
    VERIFY(k4abt_tracker_create(&sensorCalibration, tracker_config, &tracker), "Body tracker initialization failed!");

    if (opt & 4)
    {
        glassTracker.SetScalingFactor(0.3);
        if (!glassTracker.ReadDetectorParameters(dir + "/detector_params.yml"))
        {
            cerr << "Check detector parmeter file! (" << dir + "/kinect2160.yml)" << endl;
            return 1;
        }
        if (!glassTracker.ReadCameraParameters(dir + "/kinect2160.yml"))
        {
            cerr << "Check camera parmeter file! (" << dir + "/kinect2160.yml)" << endl;
            return 1;
        }
        glassTracker.SetRecord(true);
    }

    // ssize_t sentBytes;
    memset(&serverAddress, 0, sizeof(serverAddress));
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_addr.s_addr = inet_addr(argv[1]);
    serverAddress.sin_port = htons(atoi(argv[2]));

    if ((client_socket = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
    {
        cout << "WORKER: socket generation failed" << endl;
        return false;
    }
    else
        cout << "WORKER: socket generation -> success!" << endl;
    struct timeval optVal = {0, 50};
    int optLen = sizeof(optVal);
    setsockopt(client_socket, SOL_SOCKET, SO_RCVTIMEO, &optVal, optLen);
    // first msg.
    sendto(client_socket, sendBuff, 1500 * 4, 0, (struct sockaddr *)&serverAddress, sizeof(serverAddress));

#ifdef ABT_DISPLAY
    Window3dWrapper window3d;
    VideoWriter recorder;
    if (opt & 8)
    {
        window3d.Create("Motion Capture", sensorCalibration);
        window3d.SetCloseCallback(CloseCallback);
        window3d.SetKeyCallback(ProcessKey);
        time_t now = chrono::system_clock::to_time_t(chrono::system_clock::now());
        VideoWriter recorder;
        cv::Size recordSize(sensorCalibration.color_camera_calibration.resolution_width * kinectRecordF,
                            sensorCalibration.color_camera_calibration.resolution_height * kinectRecordF);
        recorder.open(string(ctime(&now)) + "_kinect.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 15, recordSize, true);
        if (!recorder.isOpened())
            cout << "error in recorder!" << endl;
    }
#endif

    // main loop
    Mat color;
    vector<int> kinectData = {0, 1, 2, 4, 5, 6, 8, 26, 11, 12, 13, 15, 18, 19, 20, 22, 23, 24, 9, 16, 21, 25, 28, 30}; // first 18 belongs to bone data
    map<int, pair<int, int>> givenIDs;                                                                                 // pair<num, count>
    while (1)
    {
        int capture_opt(0);

        k4a_capture_t sensorCapture = nullptr;
        k4a_wait_result_t getCaptureResult = k4a_device_get_capture(device, &sensorCapture, 0);

        if (getCaptureResult == K4A_WAIT_RESULT_FAILED)
        {
            std::cout << "Get img capture returned error: " << getCaptureResult << std::endl;
            break;
        }
        else if (getCaptureResult == K4A_WAIT_RESULT_TIMEOUT)
            continue;

        k4a_image_t color_img = k4a_capture_get_color_image(sensorCapture);
        color = color_to_opencv(color_img);
        k4a_image_release(color_img);
        int pos(2);

        if (opt & 2) // bed tracking
        {
        }
        if (opt & 4) // glass tracking
        {
            glassTracker.SetNewFrame(color);
            Quaterniond q;
            Eigen::Vector3d t;
            bool detected = glassTracker.ProcessCurrentFrame(q, t);
            if (!glassTracker.Render(detected))
                break;
            if (detected)
            {
                capture_opt |= 4;
                sendBuff[pos++] = (float)q.w();
                sendBuff[pos++] = (float)q.x();
                sendBuff[pos++] = (float)q.y();
                sendBuff[pos++] = (float)q.z();
                sendBuff[pos++] = (float)t(0);
                sendBuff[pos++] = (float)t(1);
                sendBuff[pos++] = (float)t(2);
            }
        }
        if (opt & 8) // body tracking
        {
            k4a_wait_result_t queueCaptureResult = k4abt_tracker_enqueue_capture(tracker, sensorCapture, 0);
            if (queueCaptureResult == K4A_WAIT_RESULT_FAILED)
            {
                std::cout << "Error! Add capture to tracker process queue failed!" << std::endl;
                break;
            }

            k4abt_frame_t bodyFrame = nullptr;
            k4a_wait_result_t popFrameResult = k4abt_tracker_pop_result(tracker, &bodyFrame, 0); // timeout_in_ms is set to 0
            map<int, Point2i> idPos;
            map<int, int> currentID;
            if (popFrameResult == K4A_WAIT_RESULT_SUCCEEDED)
            {

#ifdef ABT_DISPLAY
                VisualizeResult(bodyFrame, window3d, depthWidth, depthHeight);
                if (!s_isRunning)
                    break;
#endif

                int num = k4abt_frame_get_num_bodies(bodyFrame);
                int numPos = pos;
                sendBuff[pos++] = 0;
                for (int i = 0; i < num; i++)
                {
                    int id0 = k4abt_frame_get_body_id(bodyFrame, i);
                    if(id0>100) id0 = id0%100;
                    id0 = id*100+id0;
                    // givenIDs[id0] = make_pair(-id0, 0);////////////////

                    // sendBuff[pos++] = id0;
                    k4abt_body_t body;
                    k4abt_frame_get_body_skeleton(bodyFrame, i, &body.skeleton);
 
/////// WORKER IDENTIFICATION
                    transform_joint_from_depth_3d_to_color_2d(&sensorCalibration, body.skeleton.joints[3].position, idPos[id0]);
                    // imshow("neck", color(cv::Rect(p0, p1)));
                    int givenID(-id0);
                    if (givenIDs.find(id0)==givenIDs.end() || givenIDs[id0].first < 0 || givenIDs[id0].second < 10) // if ID was not assigned
                    {
                        k4a_quaternion_t q = body.skeleton.joints[2].orientation;
                        Quaterniond quat2(q.wxyz.w, q.wxyz.x, q.wxyz.y, q.wxyz.z);

                        if (Vector3d(0, 0, -1).dot(quat2 * Vector3d(0, 1, 0)) > 0)
                            for (auto iter : colorIdx)
                            {
                                bool match(true);
                                k4a_float3_t p; //stomach
                                p.v[0] = body.skeleton.joints[11].position.v[0];
                                p.v[1] = (body.skeleton.joints[1].position.v[1] + body.skeleton.joints[2].position.v[1]) * 0.5;
                                p.v[2] = body.skeleton.joints[1].position.v[2];
                                Point2i p0, p1;
                                transform_joint_from_depth_3d_to_color_2d(&sensorCalibration, p, p0);
                                p.v[0] = body.skeleton.joints[4].position.v[0];
                                p.v[1] = (body.skeleton.joints[1].position.v[1] + body.skeleton.joints[0].position.v[1]) * 0.5;
                                transform_joint_from_depth_3d_to_color_2d(&sensorCalibration, p, p1);
                                Scalar low(105-5, 0, 0);
                                Scalar high(105+5, 255, 255);
                                Mat output;
                                if(p0==p1 || p0.x>=p1.x || p0.y >= p1.y) continue;
                                cvtColor(color(Rect(p0, p1)), output, COLOR_BGR2HSV);
                                inRange(output,low, high, output);
                                // imshow("test", output); waitKey(1);
                                // imshow("test1", color(Rect(p0, p1))); waitKey(1);
                                if((countNonZero(output) > output.cols*output.rows*0.7)==iter.second.second) //blue clothing
                                {
                                    p = body.skeleton.joints[31].position;
                                    p.v[1] = body.skeleton.joints[30].position.v[1];
                                    transform_joint_from_depth_3d_to_color_2d(&sensorCalibration, body.skeleton.joints[31].position, p0);
                                    p = body.skeleton.joints[29].position;
                                    p.v[1] = (body.skeleton.joints[3].position.v[1] + body.skeleton.joints[26].position.v[1]) * 0.5;
                                    transform_joint_from_depth_3d_to_color_2d(&sensorCalibration, p, p1);
                                }
                                else
                                {
                                    p.v[0] = (body.skeleton.joints[11].position.v[0] + body.skeleton.joints[12].position.v[0]) * 0.5;
                                    p.v[1] = (body.skeleton.joints[3].position.v[1] + body.skeleton.joints[26].position.v[1]) * 0.5;
                                    p.v[2] = body.skeleton.joints[11].position.v[2];
                                    transform_joint_from_depth_3d_to_color_2d(&sensorCalibration, p, p0);
                                    p = body.skeleton.joints[2].position;
                                    p.v[0] = (body.skeleton.joints[4].position.v[0] + body.skeleton.joints[5].position.v[0]) * 0.5;
                                    transform_joint_from_depth_3d_to_color_2d(&sensorCalibration, p, p1);
                                }
                                low = Scalar(iter.second.first-5, 0, 0);
                                high= Scalar(iter.second.first+5, 255, 255);
                                Mat output1;
                                if(p0==p1 || p0.x>=p1.x || p0.y >= p1.y) continue;
                                cvtColor(color(Rect(p0, p1)), output1, COLOR_BGR2HSV);
                                inRange(output1,low, high, output);
                                if(high(0)>180){
                                    low = Scalar(0, 0, 0);
                                    high= Scalar(high(0) -180, 255, 255);
                                    inRange(output1,low, high, output1);
                                    output += output1;
                                }
                                else if(low(0)<0)
                                {
                                    low = Scalar(low(0)+180, 0, 0);
                                    high= Scalar(255, 255, 255);
                                    inRange(output1,low, high, output1);
                                    output += output1;
                                }
                                //  imshow("test2", output); waitKey(1);
                                // imshow("test3", color(Rect(p0, p1))); waitKey(1);
                               if((countNonZero(output) >  output.cols*output.rows*0.4)) 
                                {
                                    if (givenID < 0)
                                        givenID = iter.first;
                                    else
                                    {
                                        cout << "two possible ID: " << givenID << ", " << iter.first;
                                        givenID = -id0;
                                    }
                                }
                            }
                        if (givenID < 0) {
                            givenIDs[id0].second = max(0, givenIDs[id0].second-1);
                            continue;
                        }
                        
                        else if (givenID != givenIDs[id0].first)
                            givenIDs[id0] = make_pair(givenID, 0);
                        givenIDs[id0].second++;
                    }

                    // if(givenIDs[id0].second >= 10) //if assigned
                    // {
                    //     sendBuff[pos++] = givenIDs[id0].first;
                    //     sendBuff[numPos]++;
                    // }
                    // else continue;

                    if(givenIDs.find(id0) != givenIDs.end() && givenIDs[id0].second>=10) sendBuff[pos++] = givenIDs[id0].first;
                    else sendBuff[pos++] = givenID;
                    currentID[id0]=sendBuff[pos-1];
                    sendBuff[numPos]++;

                    for (int i = 0; i < 18; i++)
                    {
                        k4a_quaternion_t q = body.skeleton.joints[kinectData[i]].orientation;
                        Quaternionf q1(q.wxyz.w,q.wxyz.x,q.wxyz.y,q.wxyz.z);
                        q1 = AngleAxisf(-6./180*PI,Vector3f(1,0,0))*q1;
                        sendBuff[pos++] = q1.w();
                        sendBuff[pos++] = q1.x();
                        sendBuff[pos++] = q1.y();
                        sendBuff[pos++] = q1.z();
                    }
                    int score(0);
                    for (int i : kinectData)
                    {
                        if(body.skeleton.joints[i].confidence_level ==K4ABT_JOINT_CONFIDENCE_MEDIUM) score++;
                        k4a_float3_t p;// =body.skeleton.joints[i].position;
                        k4a_calibration_3d_to_3d(&sensorCalibration, &body.skeleton.joints[i].position, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &p);
                        sendBuff[pos++] = p.xyz.x * 0.1;
                        sendBuff[pos++] = p.xyz.y * 0.1;
                        sendBuff[pos++] = p.xyz.z * 0.1;
                    }
                    sendBuff[pos++] = score;
                }
                k4abt_frame_release(bodyFrame);
                if (sendBuff[numPos] > 0)
                    capture_opt |= 8;
            }
#ifdef ABT_DISPLAY
            // VisualizeResult(bodyFrame, window3d, depthWidth, depthHeight);
            // VisualizeResult(bodyFrame, window3d, depthWidth, depthHeight, givenIDs);
            window3d.SetLayout3d(s_layoutMode);
            window3d.SetJointFrameVisualization(s_visualizeJointFrame);
            window3d.Render();

            cv::resize(color, color, Size(color.cols * kinectRecordF, color.rows * kinectRecordF));
            cv::putText(color, colorComment, Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255.f, 0.f, 0.f), 1.2);
            for (auto iter : idPos)
            {
                if (givenIDs.find(iter.first) == givenIDs.end())
                {
                    cv::putText(color, to_string(iter.first), iter.second * kinectRecordF, FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255, 0, 0), 2);
                }
                else 
                {
                string tag = to_string(currentID[iter.first]);
                if (givenIDs[iter.first].second < 10)
                    tag += "(X)";
                cv::putText(color, tag, iter.second * kinectRecordF, FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 0, 255), 2);
                }
            }
            cv::imshow("color_body", color);
            cv::waitKey(1);
            recorder << color;
#endif
        }
        if (sensorCapture)
            k4a_capture_release(sensorCapture);

        sendBuff[1] = capture_opt;
        server_addr_size = sizeof(serverAddress);
        sendto(client_socket, sendBuff, 1500 * 4, 0, (struct sockaddr *)&serverAddress, sizeof(serverAddress));
        if (recvfrom(client_socket, recvBuff, 4, 0, (struct sockaddr *)&serverAddress, &server_addr_size) > 0)
        {
            cout << "WORKER: server says - " << recvBuff << endl;
            break;
        }
        // sleep(0.02);
    }
    if (opt != 1)
        k4a_device_close(device);
    if (opt & 8)
    {
        k4abt_tracker_destroy(tracker);
#ifdef ABT_DISPLAY
        window3d.Delete();
#endif
    }
}
//     string detParm(dir + "/detector_params.yml");
//     string camParm(dir + "/kinect2160.yml");
//     GlassTracker glassTracker;
//     glassTracker.SetScalingFactor(0.3);
//     if (argc < 3)
//         PrintUsage();
//     int option = atoi(argv[3]);
//     if (option & 4)
//     {
//         if (!glassTracker.ReadDetectorParameters(detParm))
//         {
//             cerr << "Check detector parmeter file! (" << detParm << ")" << endl;
//             return 1;
//         }
//         if (!glassTracker.ReadCameraParameters(camParm))
//         {
//             cerr << "Check camera parmeter file! (" << camParm << ")" << endl;
//             return 1;
//         }
//     }

// #ifndef TEST
//     ClientSocket socket(string(argv[1]), atoi(argv[2]));
//     string msg;
//     array<double, 187> pack;
//     socket >> msg;
//     cout << msg << endl;

//     Eigen::Affine3d aff = Eigen::Affine3d::Identity();
//     aff.rotate(Quaterniond(pack[4], pack[1], pack[2], pack[3]).normalized().toRotationMatrix().transpose());
//     aff.translate(-Vector3d(pack[5], pack[6], pack[7]));

//     ifs.close();
//     pack[0] = option;
//     socket.SendDoubleBuffer(pack.data(), 8);
//     // socket.SendIntBuffer(&option, 1);

//     // recieve alignRot/BE
//     RotationList alignRot;
//     MatrixXi BE(BE_NUM, 2);
//     if (option & 8)
//     {
//         socket.RecvDoubleBuffer(pack.data(), BE_NUM * 6);
//         for (int i = 0; i < BE_NUM; i++)
//         {
//             alignRot.push_back(Quaterniond(pack[i * 6], pack[i * 6 + 1], pack[i * 6 + 2], pack[i * 6 + 3]));
//             BE.row(i) << (int)pack[i * 6 + 4], (int)pack[i * 6 + 5];
//         }
//     }
//     socket << "success!";

// #endif
//     // joint number conv.
//     vector<int> i2k = {0, 1, 2, 4, 5, 6, 7, 26, 11, 12, 13, 14, 18, 19, 20, 22, 23, 24};
//     map<int, int> k2i;
//     for (size_t i = 0; i < i2k.size(); i++)
//         k2i[i2k[i]] = i;

//     int signal(-1);
//     Window3dWrapper window3d;
// #ifndef TEST
//     // calibration part
//     while (signal < 0)
//     {
//         signal = -1;
//         socket << "ready";
//         socket.RecvIntBuffer(&signal, 1);
//         if (signal > 0)
//             break;
//         cout << "Start cliabration" << endl;
//         // Start calibration
//         window3d.Create("Calibration", sensorCalibration);
//         window3d.SetCloseCallback(CloseCallback);
//         window3d.SetKeyCallback(ProcessKey);

//         map<int, double> calibLengths;
//         Vector3d eyeL_pos(0, 0, 0), eyeR_pos(0, 0, 0);
//         int calibFrame(0);
//         s_isRunning = true;
//         while (s_isRunning)
//         {
//             k4a_capture_t sensorCapture = nullptr;
//             k4a_wait_result_t getCaptureResult = k4a_device_get_capture(device, &sensorCapture, 0);
//             if (getCaptureResult == K4A_WAIT_RESULT_SUCCEEDED)
//             {
//                 k4a_wait_result_t queueCaptureResult = k4abt_tracker_enqueue_capture(tracker, sensorCapture, 0);
//                 k4a_capture_release(sensorCapture);
//                 if (queueCaptureResult == K4A_WAIT_RESULT_FAILED)
//                 {
//                     std::cout << "Error! Add capture to tracker process queue failed!" << std::endl;
//                     break;
//                 }
//             }
//             else if (getCaptureResult != K4A_WAIT_RESULT_TIMEOUT)
//             {
//                 std::cout << "Get depth capture returned error: " << getCaptureResult << std::endl;
//                 break;
//             }

//             // Pop Result from Body Tracker
//             k4abt_frame_t bodyFrame = nullptr;
//             k4a_wait_result_t popFrameResult = k4abt_tracker_pop_result(tracker, &bodyFrame, 0); // timeout_in_ms is set to 0

//             i2k[7] = 3;
//             if (popFrameResult == K4A_WAIT_RESULT_SUCCEEDED)
//             {
//                 VisualizeResult(bodyFrame, window3d, depthWidth, depthHeight);
//                 if (calibSwitch && k4abt_frame_get_num_bodies(bodyFrame))
//                 {
//                     k4abt_body_t body;
//                     VERIFY(k4abt_frame_get_body_skeleton(bodyFrame, 0, &body.skeleton), "Get skeleton from body frame failed!");
//                     for (int i = 0; i < BE.rows(); i++)
//                     {
//                         if (BE(i, 1) >= (int)i2k.size())
//                             continue; // extrimity
//                         k4a_float3_t t0 = body.skeleton.joints[i2k[BE(i, 0)]].position;
//                         k4a_float3_t t1 = body.skeleton.joints[i2k[BE(i, 1)]].position;
//                         float d[3] = {t1.xyz.x - t0.xyz.x, t1.xyz.y - t0.xyz.y, t1.xyz.z - t0.xyz.z};
//                         calibLengths[i] += sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
//                     }
//                     k4a_quaternion_t q = body.skeleton.joints[26].orientation;
//                     Quaterniond headRot = Quaterniond(q.wxyz.w, q.wxyz.x, q.wxyz.y, q.wxyz.z) * alignRot[k2i[26]];
//                     k4a_float3_t pos = body.skeleton.joints[26].position;
//                     Vector3d headPos = Vector3d(pos.xyz.x, pos.xyz.y, pos.xyz.z);

//                     pos = body.skeleton.joints[28].position;
//                     eyeL_pos += headRot.inverse() * (Vector3d(pos.xyz.x, pos.xyz.y, pos.xyz.z) - headPos);
//                     pos = body.skeleton.joints[30].position;
//                     eyeR_pos += headRot.inverse() * (Vector3d(pos.xyz.x, pos.xyz.y, pos.xyz.z) - headPos);
//                     cout << "\rCalibration frame #" << calibFrame++ << flush;
//                 }
//                 // Release the bodyFrame
//                 k4abt_frame_release(bodyFrame);
//             }

//             window3d.SetLayout3d(s_layoutMode);
//             window3d.SetJointFrameVisualization(s_visualizeJointFrame);
//             window3d.Render();
//         }
//         window3d.Delete();
//         int n(0);
//         for (auto iter : calibLengths)
//         {
//             pack[n++] = iter.first;
//             pack[n++] = iter.second;
//         }
//         pack[calibLengths.size() * 2] = -1;
//         pack[calibLengths.size() * 2 + 1] = eyeL_pos.x();
//         pack[calibLengths.size() * 2 + 2] = eyeL_pos.y();
//         pack[calibLengths.size() * 2 + 3] = eyeL_pos.z();
//         pack[calibLengths.size() * 2 + 4] = eyeR_pos.x();
//         pack[calibLengths.size() * 2 + 5] = eyeR_pos.y();
//         pack[calibLengths.size() * 2 + 6] = eyeR_pos.z();
//         pack[calibLengths.size() * 2 + 7] = calibFrame;
//         // socket.SendIntBuffer(&signal, 1);
//         socket.SendDoubleBuffer(pack.data(), 155);
//         cout << endl
//              << "Sent calibration data" << endl;
//         socket >> msg;
//         cout << msg << endl;
//     }
// #endif
//     cout << "ready" << endl;
//     vector<int> poseJoints = {0, 1, 2, 4, 5, 6, 7, 26, 11, 12, 13, 14, 18, 19, 20, 22, 23, 24, 8, 15, 21, 25, 28, 30};
//     bool calib(true);

// #ifdef DISPLAY
//     if (option & 8)
//     {
//         window3d.Create("Motion Capture", sensorCalibration);
//         window3d.SetCloseCallback(CloseCallback);
//         window3d.SetKeyCallback(ProcessKey);
//     }
// #endif

//     // variables
//     Mat color;
//     mutex m;
//     array<float, 375> pack_run;
//     int n = poseJoints.size();
//     while (1)
//     {
//         k4a_capture_t sensorCapture = nullptr;
//         k4a_wait_result_t getCaptureResult = k4a_device_get_capture(device, &sensorCapture, 0);
//         int capture_opt(0);
//         if (getCaptureResult == K4A_WAIT_RESULT_FAILED)
//         {
//             std::cout << "Get img capture returned error: " << getCaptureResult << std::endl;
//             break;
//         }
//         else if (getCaptureResult == K4A_WAIT_RESULT_TIMEOUT)
//             continue;

//         // bed
//         if (option & 2)
//         {
//         }

//         // ARUCO (0-6)
//         if (option & 4)
//         {
//             k4a_image_t color_img = k4a_capture_get_color_image(sensorCapture);
//             color = color_to_opencv(color_img);
//             k4a_image_release(color_img);
//             glassTracker.SetNewFrame(color);
//             Quaterniond q;
//             Eigen::Vector3d t;
//             bool detected = glassTracker.ProcessCurrentFrame(q, t);
//             glassTracker.Render(detected);
// #ifndef TEST
//             if (detected)
//             {
//                 capture_opt |= 4;
//                 pack_run[0] = (float)q.x();
//                 pack_run[1] = (float)q.y();
//                 pack_run[2] = (float)q.z();
//                 pack_run[3] = (float)q.w();
//                 pack_run[4] = (float)t(0);
//                 pack_run[5] = (float)t(1);
//                 pack_run[6] = (float)t(2);
//             }
// #endif
//         }

//         // MOTION TRACKING (7-)
//         if (option & 8)
//         {
//             int m = 7;
//             k4a_wait_result_t queueCaptureResult = k4abt_tracker_enqueue_capture(tracker, sensorCapture, 0);
//             k4a_capture_release(sensorCapture);
//             if (queueCaptureResult == K4A_WAIT_RESULT_FAILED)
//             {
//                 std::cout << "Error! Add capture to tracker process queue failed!" << std::endl;
//                 break;
//             }

//             k4abt_frame_t bodyFrame = nullptr;
//             k4a_wait_result_t popFrameResult = k4abt_tracker_pop_result(tracker, &bodyFrame, 0); // timeout_in_ms is set to 0
//             if (popFrameResult == K4A_WAIT_RESULT_SUCCEEDED)
//             {
// #ifdef DISPLAY
//                 VisualizeResult(bodyFrame, window3d, depthWidth, depthHeight);
// #endif
//                 if (k4abt_frame_get_num_bodies(bodyFrame))
//                 {
// #ifndef TEST
//                     capture_opt |= 8;
//                     k4abt_body_t body;
//                     k4abt_frame_get_body_skeleton(bodyFrame, 0, &body.skeleton);
//                     for (size_t i = 0; i < poseJoints.size(); i++)
//                     {
//                         pack_run[m + i] = 0;
//                         if (body.skeleton.joints[poseJoints[i]].confidence_level == K4ABT_JOINT_CONFIDENCE_MEDIUM)
//                             pack_run[m + i] = 1;

//                         pack_run[m + n + i * 3] = body.skeleton.joints[poseJoints[i]].position.xyz.x * 0.1;
//                         pack_run[m + n + i * 3 + 1] = body.skeleton.joints[poseJoints[i]].position.xyz.y * 0.1;
//                         pack_run[m + n + i * 3 + 2] = body.skeleton.joints[poseJoints[i]].position.xyz.z * 0.1;
//                     }
//                     for (int i = 0; i < BE.rows(); i++)
//                     {
//                         k4a_quaternion_t q = body.skeleton.joints[i2k[BE(i, 0)]].orientation;
//                         Quaterniond q1 = Quaterniond(q.wxyz.w, q.wxyz.x, q.wxyz.y, q.wxyz.z) * alignRot[i];
//                         q1.normalize();
//                         pack_run[m + n * 4 + i * 4] = q1.w();
//                         pack_run[m + n * 4 + i * 4 + 1] = q1.x();
//                         pack_run[m + n * 4 + i * 4 + 2] = q1.y();
//                         pack_run[m + n * 4 + i * 4 + 3] = q1.z();
//                     }
// #endif
//                 }
//                 k4abt_frame_release(bodyFrame);
//             }
// #ifdef DISPLAY
//             window3d.SetLayout3d(s_layoutMode);
//             window3d.SetJointFrameVisualization(s_visualizeJointFrame);
//             window3d.Render();
// #endif
//         }
// #ifndef TEST
//         // for(int i=0;i<375;i++) cout<<i<<": "<<pack_run[i]<<endl;
//         // cout<<"-----------------"<<endl;
//         pack_run[374] = capture_opt;
//         socket.SendFloatBuffer(pack_run.data(), 375);
//         socket.RecvIntBuffer(&signal, 1);
// #endif
//     }

//     k4abt_tracker_destroy(tracker);
//     return 1;
