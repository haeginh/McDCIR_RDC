#include <iostream>
#include <ctime>
#include <functions.hh>
#include <functional>

#include "bodytracking.hh"
#include "colorbar.hh"
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <imgui/imgui.h>
#include <igl/readTGF.h>
#include <igl/writeTGF.h>
#include <igl/readDMAT.h>
#include <igl/writeDMAT.h>
#include <igl/writeMESH.h>
#include <igl/readMESH.h>
#include <igl/readPLY.h>
#include <igl/directed_edge_parents.h>
#include <igl/copyleft/tetgen/tetrahedralize.h>
#include <igl/boundary_conditions.h>
#include <igl/directed_edge_parents.h>
#include <igl/directed_edge_orientations.h>
#include <igl/deform_skeleton.h>
#include <igl/forward_kinematics.h>
#include <igl/doublearea.h>
#include <igl/dqs.h>
#include <igl/unproject_onto_mesh.h>
#include <igl/Timer.h>
#include <igl/mat_max.h>

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/SparseCore>
#define PI 3.14159265358979323846

#include "ClientSocket.hh"
#define PORT 30303
#define BE_NUM 22
#define C_NUM 24

void PrintUsage()
{
    cout << "<Usage>" << endl;
    cout << "./2_tracker [ip] [tracking options]" << endl;
    cout << "<tracking options>" << endl;
    cout << "-m motion, -g glass, -b bed" << endl;
    exit(1);
}

using namespace Eigen;
using namespace std;
typedef Triplet<double> T;

int main(int argc, char **argv)
{
    //trackin option configuration
    if (argc < 3)
        PrintUsage();
    int option(0);
    for (int i = 2; i < argc; i++)
    {
        if (string(argv[i]) == "-m")
            option = option | 8; // 1000
        else if (string(argv[i]) == "-g")
            option = option | 4; // 0100
        else if (string(argv[i]) == "-b")
            option = option | 2; // 0010
    }

    //connection to server
    string ip(argv[1]);
    ClientSocket socket(ip, PORT);
    string msg;
    array<double, 187> pack;
    socket >> msg;
    cout << msg << endl;
    socket.SendIntBuffer(&option, 1);

    //recieve alignRot/BE
    RotationList alignRot;
    MatrixXi BE(BE_NUM, 2);
    socket.RecvDoubleBuffer(pack.data(), BE_NUM * 6);
    for (int i = 0; i < BE_NUM; i++)
    {
        alignRot.push_back(Quaterniond(pack[i * 6], pack[i * 6 + 1], pack[i * 6 + 2], pack[i * 6 + 3]));
        BE.row(i) << (int)pack[i * 6 + 4], (int)pack[i * 6 + 5];
    }
    socket << "success!";

    // joint number conv.
    vector<int> i2k = {0, 1, 2, 4, 5, 6, 7, 26, 11, 12, 13, 14, 18, 19, 20, 22, 23, 24};
    map<int, int> k2i;
    for (size_t i = 0; i < i2k.size(); i++)
        k2i[i2k[i]] = i;

    // Start camera
    k4a_device_t device = nullptr;
    VERIFY(k4a_device_open(0, &device), "Open K4A Device failed!");

    // Start camera. Make sure depth camera is enabled.
    k4a_device_configuration_t deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    deviceConfig.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_OFF;
    deviceConfig.camera_fps = K4A_FRAMES_PER_SECOND_15;
    VERIFY(k4a_device_start_cameras(device, &deviceConfig), "Start K4A cameras failed!");

    // Get calibration information
    k4a_calibration_t sensorCalibration;
    VERIFY(k4a_device_get_calibration(device, deviceConfig.depth_mode, deviceConfig.color_resolution, &sensorCalibration),
           "Get depth camera calibration failed!");
    int depthWidth = sensorCalibration.depth_camera_calibration.resolution_width;
    int depthHeight = sensorCalibration.depth_camera_calibration.resolution_height;

    // Create Body Tracker
    k4abt_tracker_t tracker = nullptr;
    k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
    tracker_config.processing_mode = K4ABT_TRACKER_PROCESSING_MODE_GPU;
    VERIFY(k4abt_tracker_create(&sensorCalibration, tracker_config, &tracker), "Body tracker initialization failed!");

    int signal(-1);
    Window3dWrapper window3d;
    while (signal < 0)
    {
        signal = -1;
        socket << "ready";
        socket.RecvIntBuffer(&signal, 1);
        if (signal > 0)
            break;
        cout << "Start cliabration" << endl;
        // Start calibration
        window3d.Create("Calibration", sensorCalibration);
        window3d.SetCloseCallback(CloseCallback);
        window3d.SetKeyCallback(ProcessKey);

        map<int, double> calibLengths;
        Vector3d eyeL_pos(0, 0, 0), eyeR_pos(0, 0, 0);
        int calibFrame(0);
        s_isRunning = true;
        while (s_isRunning)
        {
            k4a_capture_t sensorCapture = nullptr;
            k4a_wait_result_t getCaptureResult = k4a_device_get_capture(device, &sensorCapture, 0);
            if (getCaptureResult == K4A_WAIT_RESULT_SUCCEEDED)
            {
                k4a_wait_result_t queueCaptureResult = k4abt_tracker_enqueue_capture(tracker, sensorCapture, 0);
                k4a_capture_release(sensorCapture);
                if (queueCaptureResult == K4A_WAIT_RESULT_FAILED)
                {
                    std::cout << "Error! Add capture to tracker process queue failed!" << std::endl;
                    break;
                }
            }
            else if (getCaptureResult != K4A_WAIT_RESULT_TIMEOUT)
            {
                std::cout << "Get depth capture returned error: " << getCaptureResult << std::endl;
                break;
            }

            // Pop Result from Body Tracker
            k4abt_frame_t bodyFrame = nullptr;
            k4a_wait_result_t popFrameResult = k4abt_tracker_pop_result(tracker, &bodyFrame, 0); // timeout_in_ms is set to 0

            i2k[7] = 3;
            if (popFrameResult == K4A_WAIT_RESULT_SUCCEEDED)
            {
                VisualizeResult(bodyFrame, window3d, depthWidth, depthHeight);
                if (calibSwitch && k4abt_frame_get_num_bodies(bodyFrame))
                {
                    k4abt_body_t body;
                    VERIFY(k4abt_frame_get_body_skeleton(bodyFrame, 0, &body.skeleton), "Get skeleton from body frame failed!");
                    for (int i = 0; i < BE.rows(); i++)
                    {
                        if (BE(i, 1) >= (int)i2k.size())
                            continue; //extrimity
                        k4a_float3_t t0 = body.skeleton.joints[i2k[BE(i, 0)]].position;
                        k4a_float3_t t1 = body.skeleton.joints[i2k[BE(i, 1)]].position;
                        float d[3] = {t1.xyz.x - t0.xyz.x, t1.xyz.y - t0.xyz.y, t1.xyz.z - t0.xyz.z};
                        calibLengths[i] += sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
                    }
                    k4a_quaternion_t q = body.skeleton.joints[26].orientation;
                    Quaterniond headRot = Quaterniond(q.wxyz.w, q.wxyz.x, q.wxyz.y, q.wxyz.z) * alignRot[k2i[26]];
                    k4a_float3_t pos = body.skeleton.joints[26].position;
                    Vector3d headPos = Vector3d(pos.xyz.x, pos.xyz.y, pos.xyz.z);

                    pos = body.skeleton.joints[28].position;
                    eyeL_pos += headRot.inverse() * (Vector3d(pos.xyz.x, pos.xyz.y, pos.xyz.z) - headPos);
                    pos = body.skeleton.joints[30].position;
                    eyeR_pos += headRot.inverse() * (Vector3d(pos.xyz.x, pos.xyz.y, pos.xyz.z) - headPos);
                    cout << "\rCalibration frame #" << calibFrame++ << flush;
                }
                //Release the bodyFrame
                k4abt_frame_release(bodyFrame);
            }

            window3d.SetLayout3d(s_layoutMode);
            window3d.SetJointFrameVisualization(s_visualizeJointFrame);
            window3d.Render();
        }
        window3d.Delete();
        int n(0);
        for (auto iter : calibLengths)
        {
            pack[n++] = iter.first;
            pack[n++] = iter.second;
        }
        pack[calibLengths.size() * 2] = -1;
        pack[calibLengths.size() * 2 + 1] = eyeL_pos.x();
        pack[calibLengths.size() * 2 + 2] = eyeL_pos.y();
        pack[calibLengths.size() * 2 + 3] = eyeL_pos.z();
        pack[calibLengths.size() * 2 + 4] = eyeR_pos.x();
        pack[calibLengths.size() * 2 + 5] = eyeR_pos.y();
        pack[calibLengths.size() * 2 + 6] = eyeR_pos.z();
        pack[calibLengths.size() * 2 + 7] = calibFrame;
        // socket.SendIntBuffer(&signal, 1);
        socket.SendDoubleBuffer(pack.data(), 155);
        cout << endl
             << "Sent calibration data" << endl;
        socket >> msg;
        cout << msg;
    }

    cout << "ready" << endl;
    vector<int> poseJoints = {0, 1, 2, 4, 5, 6, 7, 26, 11, 12, 13, 14, 18, 19, 20, 22, 23, 24, 8, 15, 21, 25, 28, 30};
    bool calib(true);
    window3d.Create("Motion Capture", sensorCalibration);
    window3d.SetCloseCallback(CloseCallback);
    window3d.SetKeyCallback(ProcessKey);
    while (1)
    {
        window3d.SetLayout3d(s_layoutMode);
        window3d.SetJointFrameVisualization(s_visualizeJointFrame);
        window3d.Render();

        k4a_capture_t sensorCapture = nullptr;
        k4a_wait_result_t getCaptureResult = k4a_device_get_capture(device, &sensorCapture, 0);
        if (getCaptureResult == K4A_WAIT_RESULT_SUCCEEDED)
        {
            k4a_wait_result_t queueCaptureResult = k4abt_tracker_enqueue_capture(tracker, sensorCapture, 0);
            k4a_capture_release(sensorCapture);
            if (queueCaptureResult == K4A_WAIT_RESULT_FAILED)
            {
                std::cout << "Error! Add capture to tracker process queue failed!" << std::endl;
                break;
            }
        }
        else if (getCaptureResult != K4A_WAIT_RESULT_TIMEOUT)
        {
            std::cout << "Get depth capture returned error: " << getCaptureResult << std::endl;
            break;
        }

        // Pop Result from Body Tracker
        k4abt_frame_t bodyFrame = nullptr;
        k4a_wait_result_t popFrameResult = k4abt_tracker_pop_result(tracker, &bodyFrame, 0); // timeout_in_ms is set to 0
        if (popFrameResult == K4A_WAIT_RESULT_SUCCEEDED)
        {
            VisualizeResult(bodyFrame, window3d, depthWidth, depthHeight);
            if (k4abt_frame_get_num_bodies(bodyFrame))
            {
                k4abt_body_t body;
                k4abt_frame_get_body_skeleton(bodyFrame, 0, &body.skeleton);
                int n = poseJoints.size();
                for (size_t i = 0; i < poseJoints.size(); i++)
                {
                    pack[i] = 0;
                    if (body.skeleton.joints[poseJoints[i]].confidence_level == K4ABT_JOINT_CONFIDENCE_MEDIUM)
                        pack[i] = 1;
                    
                    pack[n + i * 3]= body.skeleton.joints[poseJoints[i]].position.xyz.x * 0.1;
                    pack[n + i * 3 + 1]= body.skeleton.joints[poseJoints[i]].position.xyz.y * 0.1;
                    pack[n + i * 3 + 2]= body.skeleton.joints[poseJoints[i]].position.xyz.z * 0.1;
                }
                n *= 4;
                for (int i = 0; i < BE.rows(); i++)
                {
                    k4a_quaternion_t q = body.skeleton.joints[i2k[BE(i, 0)]].orientation;
                    Quaterniond q1 = Quaterniond(q.wxyz.w, q.wxyz.x, q.wxyz.y, q.wxyz.z) * alignRot[i];
                    q1.normalize();
                    pack[n + i * 4] = q1.w();
                    pack[n + i * 4 + 1] = q1.x();
                    pack[n + i * 4 + 2] = q1.y();
                    pack[n + i * 4 + 3] = q1.z();
                }
                socket.SendDoubleBuffer(pack.data(), n+BE.rows()*4);
                socket.RecvIntBuffer(&signal, 1);
            }
            k4abt_frame_release(bodyFrame);
        }
        window3d.SetLayout3d(s_layoutMode);
        window3d.SetJointFrameVisualization(s_visualizeJointFrame);
        window3d.Render();
    }

    return 1;

    //     //main variables
    //  MatrixXd C, V;
    //  MatrixXi BE, T, F;
    //     RowVector3d sea_green(70./255.,252./255.,167./255.);
    //     RowVector3d blue(0.,0.,1.);
    //     RowVector3d red(1.,0.,0.);
    //     RowVector3d white(1.,1.,1.);

    //     //read files
    //     string prefix = argv[1];
    //     cout<<"Read "+prefix+".tgf"<<endl;
    //       igl::readTGF(prefix+".tgf",C,BE);
    //     igl::readPLY(prefix+".ply",V,F);
    //     MatrixXd W(V.rows(),BE.rows());
    //     MatrixXd Wj(V.rows(),C.rows()-1);

    //     //bbw
    //     if((!igl::readDMAT(prefix+".W",W)) || (!igl::readDMAT(prefix+".Wj",Wj))){
    //         MatrixXd boneP = GenerateBonePoints(C,BE,1.);
    //         MatrixXd V1(V.rows()+boneP.rows(),3);
    //         V1<<V, boneP;
    //         MatrixXd VT, WT_j, WT;
    //         MatrixXi TT, FT;
    //         cout<<"<Tetrahedralization>"<<endl;
    //         igl::copyleft::tetgen::tetrahedralize(V1, F, "pYq", VT, TT, FT);

    //         cout<<"<Calculate Joint Weights>"<<endl;
    //         MatrixXd C1 = C.block(0,0,C.rows()-1,3);
    //         if(!CalculateScalingWeights(C1, VT, TT, WT_j))  return EXIT_FAILURE;
    //         igl::normalize_row_sums(WT_j,WT_j);

    //         cout<<"<Calculate Bone Weights>"<<endl;
    //         MatrixXd bc; VectorXi b;
    //         igl::boundary_conditions(VT,TT,C,VectorXi(),BE,MatrixXi(),b,bc);
    //         cout<<bc.rows()<<" "<<bc.cols()<<endl;
    //         igl::BBWData bbw_data;
    //         bbw_data.active_set_params.max_iter = 10;
    //         bbw_data.verbosity = 2;
    //         if(!igl::bbw(VT,TT,b,bc,bbw_data,WT))  return EXIT_FAILURE;
    //         igl::normalize_row_sums(WT,WT);

    //         //matching between tetra & ply
    //         cout<<"matching between tetra & ply.."<<flush;
    //         auto grid = GenerateGrid(VT);
    //         int count(0);
    //         for(int i=0;i<V.rows();i++){
    //             int x = floor(V(i,0)+0.5);
    //             int y = floor(V(i,1)+0.5);
    //             int z = floor(V(i,2)+0.5);
    //             auto key = make_tuple(x,y,z);
    //             for(int n:grid[key]){
    //                 if(fabs(V(n,0)-VT(i,0))>0.01) continue;
    //                 if(fabs(V(n,1)-VT(i,1))>0.01) continue;
    //                 if(fabs(V(n,2)-VT(i,2))>0.01) continue;
    //                 W.row(i) = WT.row(n);
    //                 Wj.row(i) = WT_j.row(n);
    //                 cout<<"\rmatching between tetra & ply.."<<++count<<"/"<<V.rows()<<flush;
    //                 break;
    //             }
    //         }cout<<endl;
    //         igl::writeDMAT(prefix+".W", W, false);
    //         igl::writeDMAT(prefix+".Wj", Wj, false);
    //     }

    // //    //hand part
    // //    VectorXd dummy; VectorXi maxIdx;
    // //    igl::mat_max(W, 2, dummy, maxIdx);
    // //    set<int> handIdx;
    // //    for(int i=0;i<maxIdx.rows();i++)
    // //        if(maxIdx(i) == 6 || maxIdx(i) == 13) handIdx.insert(i);
    // //    cout<<handIdx.size()<<" hand vertices were found"<<endl;

    //     //eye part
    //     MatrixXd V_eye; MatrixXi F_eye;
    //     if(!igl::readPLY(prefix+"_eye.ply",V_eye,F_eye)){cout<<"there is no "+prefix+"_eye.ply!!"<<endl;}
    //     map<tuple<int,int,int>,vector<int>> grid = GenerateGrid(V);
    //     vector<int> eye2ply;
    //     for(int i=0;i<V_eye.rows();i++){
    //         double x = V_eye(i,0); double y = V_eye(i,1); double z = V_eye(i,2);
    //         auto ijk = make_tuple(floor(x+0.5),floor(y+0.5),floor(z+0.5));
    //         for(int n:grid[ijk]){
    //             if(fabs(x-V(n,0))>0.001) continue;
    //             if(fabs(y-V(n,1))>0.001) continue;
    //             if(fabs(z-V(n,2))>0.001) continue;
    //             eye2ply.push_back(n);
    //             break;
    //         }
    //     }
    //     vector<vector<int>> eyeFaces;
    //     for(int i=0;i<F_eye.rows();i++) {
    //         vector<int> face = {eye2ply[F_eye(i,0)],eye2ply[F_eye(i,1)],eye2ply[F_eye(i,2)]};
    //         sort(face.begin(),face.end());
    //         eyeFaces.push_back(face);
    //     } vector<int> eyeFaceIDs;
    //     for(int i=0;i<F.rows();i++){
    //         vector<int> face = {F(i,0),F(i,1),F(i,2)};
    //         sort(face.begin(),face.end());
    //         if(find(eyeFaces.begin(),eyeFaces.end(),face)!=eyeFaces.end())
    //             eyeFaceIDs.push_back(i);
    //     }

    //     vector<map<int, double>> cleanWeights;
    //     double epsilon(1e-5);
    //     for(int i=0;i<W.rows();i++){
    //         double sum(0);
    //         map<int, double> vertexWeight;
    //         for(int j=0;j<W.cols();j++){
    //             if(W(i,j)<epsilon) continue;
    //             vertexWeight[j] = W(i,j);
    //             sum += W(i,j);
    //         }
    //         for(auto &iter:vertexWeight) iter.second /= sum;
    //         cleanWeights.push_back(vertexWeight);
    //     }BE_NUM+6

    //     //additional variables
    //     VectorXi P;
    //     igl::directed_edge_parents(BE,P);
    //     map<int, double> lengths;

    //     //distance to parent joint
    //     for(int i=0;i<BE.rows();i++)
    //         lengths[i] = (C.row(BE(i,0))-C.row(BE(i,1))).norm();

    // // Start body tracking
    // k4a_device_t device = nullptr;
    // VERIFY(k4a_device_open(0, &device), "Open K4A Device failed!");

    // // Start camera. Make sure depth camera is enabled.
    // k4a_device_configuration_t deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    // deviceConfig.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    // deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_OFF;
    // deviceConfig.camera_fps = K4A_FRAMES_PER_SECOND_30;
    // VERIFY(k4a_device_start_cameras(device, &deviceConfig), "Start K4A cameras failed!");

    // // Get calibration information
    // k4a_calibration_t sensorCalibration;
    // VERIFY(k4a_device_get_calibration(device, deviceConfig.depth_mode, deviceConfig.color_resolution, &sensorCalibration),
    //        "Get depth camera calibration failed!");
    // int depthWidth = sensorCalibration.depth_camera_calibration.resolution_width;
    // int depthHeight = sensorCalibration.depth_camera_calibration.resolution_height;

    //preprocessing for KINECT draw_vidata
    // map<int, Quaterniond> kinectDataRot;
    // vector<int> groups;
    // Matrix3d tf2; tf2<<0,-1,0,0,0,-1,1,0,0;
    // for(int id:groups = {0,1,2,18,19,20,26}) kinectDataRot[id]=Quaterniond(tf2);
    // tf2<<0,1,0,0,0,1,1,0,0;
    // for(int id:groups = {22,23,24}) kinectDataRot[id]=Quaterniond(tf2);
    // tf2<<0,1,0,0,0,-1,-1,0,0;
    // for(int id:groups = {5,6}) kinectDataRot[id]=Quaterniond(tf2);
    // tf2<<0,-1,0,0,0,1,-1,0,0;
    // for(int id:groups = {12,13}) kinectDataRot[id]=Quaterniond(tf2);
    // tf2<<1,0,0,0,0,1,0,-1,0;
    // kinectDataRot[11]=Quaterniond(tf2);
    // tf2<<1,0,0,0,0,-1,0,1,0;
    // kinectDataRot[4]=Quaterniond(tf2);
    // tf2<<0,1,0,1,0,0,0,0,-1;
    // kinectDataRot[7]=Quaterniond(tf2);
    // tf2<<0,-1,0,1,0,0,0,0,1;
    // kinectDataRot[14]=Quaterniond(tf2);

    // //RotationList alignRot;
    // map<int, Vector3d> desiredOrt;
    // groups={12,13,5,6,22,23,18,19};
    // for(int id:groups) desiredOrt[id] = Vector3d(0,1,0);
    // desiredOrt[4] = Vector3d(1,0,0);
    // desiredOrt[11] = Vector3d(-1,0,0);
    // for(int i=0;i<BE.rows();i++){
    //     if(desiredOrt.find(i2k[BE(i,0)])==desiredOrt.end())
    //         alignRot.push_back(kinectDataRot[i2k[BE(i,0)]]);
    //     else {
    //         Vector3d v = (C.row(k2i[i2k[BE(i,0)]+1])-C.row(BE(i,0))).transpose();
    //         alignRot.push_back(kinectDataRot[i2k[BE(i,0)]]*GetRotMatrix(v,desiredOrt[i2k[BE(i,0)]]));
    //     }
    // }

    // int calibFrame(0);
    // //    map<int, Vec3> uprightOrien;
    // map<int, double> calibLengths;
    // Vector3d eyeL_pos(0,0,0), eyeR_pos(0,0,0);

    // while (s_isRunning)
    // {
    //     k4a_capture_t sensorCapture = nullptr;
    //     k4a_wait_result_t getCaptureResult = k4a_device_get_capture(device, &sensorCapture, 0);
    //     if (getCaptureResult == K4A_WAIT_RESULT_SUCCEEDED)
    //     {
    //         k4a_wait_result_t queueCaptureResult = k4abt_tracker_enqueue_capture(tracker, sensorCapture, 0);
    //         k4a_capture_release(sensorCapture);
    //         if (queueCaptureResult == K4A_WAIT_RESULT_FAILED){
    //             std::cout << "Error! Add capture to tracker process queue failed!" << std::endl;
    //             break;
    //         }
    //     }
    //     else if (getCaptureResult != K4A_WAIT_RESULT_TIMEOUT){
    //         std::cout << "Get depth capture returned error: " << getCaptureResult << std::endl;
    //         break;
    //     }

    //     // Pop Result from Body Tracker
    //     k4abt_frame_t bodyFrame = nullptr;
    //     k4a_wait_result_t popFrameResult = k4abt_tracker_pop_result(tracker, &bodyFrame, 0); // timeout_in_ms is set to 0

    //     i2k[7] =  3;
    //     if (popFrameResult == K4A_WAIT_RESULT_SUCCEEDED)
    //     {
    //         VisualizeResult(bodyFrame, window3d, depthWidth, depthHeight);
    //         if(calibSwitch && k4abt_frame_get_num_bodies(bodyFrame)){
    //             k4abt_body_t body;
    //             VERIFY(k4abt_frame_get_body_skeleton(bodyFrame, 0, &body.skeleton), "Get skeleton from body frame failed!");
    //             for(int i=0;i<BE.rows();i++){
    //                 if(BE(i,1)>=(int)i2k.size()) continue; //extrimity
    //                 k4a_float3_t t0 = body.skeleton.joints[i2k[BE(i,0)]].position;
    //                 k4a_float3_t t1 = body.skeleton.joints[i2k[BE(i,1)]].position;
    //                 float d[3] = {t1.xyz.x-t0.xyz.x,t1.xyz.y-t0.xyz.y,t1.xyz.z-t0.xyz.z};
    //                 calibLengths[i] += sqrt(d[0]*d[0]+d[1]*d[1]+d[2]*d[2]);
    //             }
    //             k4a_quaternion_t q = body.skeleton.joints[26].orientation;
    //             Quaterniond headRot = Quaterniond(q.wxyz.w,q.wxyz.x,q.wxyz.y,q.wxyz.z)*alignRot[k2i[26]];
    //             k4a_float3_t pos = body.skeleton.joints[26].position;
    //             Vector3d headPos = Vector3d(pos.xyz.x,pos.xyz.y,pos.xyz.z);

    //             pos = body.skeleton.joints[28].position;
    //             eyeL_pos += headRot.inverse()*(Vector3d(pos.xyz.x,pos.xyz.y,pos.xyz.z)-headPos);
    //             pos = body.skeleton.joints[30].position;
    //             eyeR_pos += headRot.inverse()*(Vector3d(pos.xyz.x,pos.xyz.y,pos.xyz.z)-headPos);
    //             cout<<"\rCalibration frame #"<<calibFrame++<<flush;
    //         }
    //         //Release the bodyFrame
    //         k4abt_frame_release(bodyFrame);
    //     }

    //     window3d.SetLayout3d(s_layoutMode);
    //     window3d.SetJointFrameVisualization(s_visualizeJointFrame);
    //     window3d.Render();
    // }
    // window3d.Delete();
    // cout<<endl;
    // i2k[7] =  26;

    //     MatrixXd V_calib;
    //     MatrixXd C_calib;
    //     MatrixXd jointTrans = MatrixXd::Zero(C.rows(),3);
    //     if(calibFrame){
    //         int headJ(24), eyeLJ(22), eyeRJ(23);
    //         for(int i=0;i<BE.rows();i++){
    //             if(calibLengths.find(i)==calibLengths.end()){
    //                 calibLengths[i] = lengths[i];
    //                 jointTrans.row(BE(i,1)) = jointTrans.row(BE(P(i),1));
    //                 continue;
    //             }
    //             calibLengths[i] /= (double)calibFrame*10;
    //             double ratio = calibLengths[i]/lengths[i];
    //             cout<<i<<" : "<<lengths[i]<<" -> "<<calibLengths[i]<<" ("<<ratio*100<<"%)"<<endl;
    //             jointTrans.row(BE(i,1)) = (1-ratio)*(C.row(BE(i,0))-C.row(BE(i,1)));
    //             if(P(i)<0) continue;
    //             jointTrans.row(BE(i,1)) += jointTrans.row(BE(P(i),1));
    //         }

    //         eyeR_pos /= (double)calibFrame*10;
    //         eyeL_pos /= (double)calibFrame*10;
    //         jointTrans.row(eyeLJ) = C.row(headJ) + jointTrans.row(headJ) + eyeL_pos.transpose() - C.row(eyeLJ);
    //         jointTrans.row(eyeRJ) = C.row(headJ) + jointTrans.row(headJ) + eyeR_pos.transpose() - C.row(eyeRJ);
    //         jointTrans.row(headJ) = MatrixXd::Zero(1,3); jointTrans(headJ, 1) = (jointTrans(eyeLJ,1)+jointTrans(eyeRJ,1))*0.5;
    //         C_calib = C+jointTrans;

    //         cout<<Wj.rows()<<"*"<<Wj.cols()<<endl;
    //         cout<<jointTrans.rows()<<"*"<<jointTrans.cols()<<endl;
    //         V_calib = V+Wj*jointTrans.block(0,0,C.rows()-1,3);
    //     }
    //     MatrixXd jt = jointTrans.block(0,0,C.rows()-1,3);

    //     // igl viewer plugin
    //     igl::opengl::glfw::imgui::ImGuiMenu menu;
    //     menu.callback_draw_viewer_window = [&](){
    //         float menu_width = 180.f * menu.menu_scaling();
    //         ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f), ImGuiCond_FirstUseEver);
    //         ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f), ImGuiCond_FirstUseEver);
    //         ImGui::SetNextWindowSizeConstraints(ImVec2(menu_width, -1.0f), ImVec2(menu_width, -1.0f));
    //         bool _viewer_menu_visible = true;
    //         ImGui::Begin("RDC Module", &_viewer_menu_visible, ImGuiWindowFlags_NoSavedSettings|ImGuiWindowFlags_AlwaysAutoResize);
    //         ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.5f);
    //         menu.callback_draw_viewer_menu();
    //         ImGui::PopItemWidth();
    //         ImGui::End();
    //     };

    //     cout<<"generating F_area, F_incident, F_area_eye matrix..."<<flush;
    //     VectorXd F_area;
    //     igl::doublearea(V,F,F_area);
    //     F_area.cwiseSqrt();
    //     VectorXd W_avgSkin = ArrayXd::Zero(V.rows());

    //     for(int i=0;i<F.rows();i++){
    //         for(int j=0;j<3;j++)
    //             W_avgSkin(F(i,j)) +=  F_area(i);
    //     }
    //     W_avgSkin = W_avgSkin/W_avgSkin.sum();

    //     VectorXd W_Lens(eye2ply.size());
    //     for(size_t i=0;i<eye2ply.size();i++)
    //         W_Lens(i) = W_avgSkin(eye2ply[i]);

    //     W_avgSkin = W_avgSkin.array()/W_avgSkin.sum();
    //     W_Lens    = W_Lens.array()/W_Lens.sum();

    //     cout<<"done"<<endl;

    //     //read map list
    //     ifstream ifsList("./doseMaps/list.txt");
    //     int mapID,mapkVp,mapRot; double mapDAP;
    //     typedef tuple<int, int> MAPIDX;
    //     map<MAPIDX, int> mapList;
    //     map<int, double> mapDAPs;
    //     while(ifsList>>mapID>>mapkVp>>mapRot>>mapDAP){
    //         mapList[MAPIDX(mapkVp, mapRot)] = mapID;
    //         mapDAPs[mapID] = mapDAP*1.e-12;
    //     }
    //     //read default map
    //     vector<double> doseMapS0, doseMapL0;
    //     int ijk[3] = {60,60,60};
    //     doseMapS0.resize(ijk[0]*ijk[1]*ijk[2]);
    //     doseMapL0.resize(ijk[0]*ijk[1]*ijk[2]);
    //     Vector3d isoCenter = Vector3d(0,0,60); // data_iso
    //     Vector3d isoRelat  = Vector3d(-150,-150,0)-Vector3d(0,0,60); //isocenter when doseMap generated. (gridstartPos - map_iso)
    //     Vector3d gridStart = isoRelat+isoCenter; // isocenter + isorelat
    //     double DAPperNPS = mapDAPs[0];
    //     ifstream ifsMap("./doseMaps/0_conf.map", ios::binary);
    //     ifsMap.read((char*) &doseMapS0[0], ijk[0]*ijk[1]*ijk[2]*sizeof(double));
    //     ifsMap.read((char*) &doseMapL0[0], ijk[0]*ijk[1]*ijk[2]*sizeof(double));
    //     ifsMap.close();
    //     double maxSkin0 = *max_element(doseMapS0.begin(),doseMapS0.end());

    //     static char rMaxChar[100]("");
    //     static char aMaxChar[100]("1");

    //     // igl viewer
    //     igl::opengl::glfw::Viewer viewer;
    //     viewer.plugins.push_back(&menu);
    //     viewer.data().set_mesh(V, F);
    //     viewer.append_mesh();
    //     viewer.load_mesh_from_file("patient3.obj");
    //     viewer.append_mesh();
    //     MatrixXd V_cArm; MatrixXi F_cArm;
    //     igl::readPLY("c-arm.ply",V_cArm,F_cArm);
    //     viewer.data().set_mesh(V_cArm,F_cArm);
    //     viewer.append_mesh();
    //     MatrixXd V_cumul(V), V_calib_cumul(V_calib);
    //     viewer.data().set_mesh(V_cumul, F);

    //     int v1=viewer.data_list[0].id;
    //     int v1_patient=viewer.data_list[1].id;
    //     int v1_cArm=viewer.data_list[2].id;
    //     int v2=viewer.data_list[3].id;

    //     if(calibFrame){
    //         viewer.data(v1).set_mesh(V_calib, F);
    //         viewer.data(v2).set_mesh(V_calib, F);
    //         viewer.data(v1).set_edges(C_calib,BE,sea_green);
    //         viewer.data(v1).set_points(C_calib,sea_green);
    //         viewer.data(v1).show_custom_labels = false;
    //     }

    //     MatrixXd V_patient = viewer.data(v1_patient).V;
    //     vector<vector<double>> recordedData;
    //     vector<MatrixXd> jointData;
    //     bool recording(false); bool loading(false);
    //     static char rec[100] = "not recording..";

    //     auto colorMap = igl::COLOR_MAP_TYPE_PARULA;
    //     static char expTime[100]("0");  double expTimeD(0);
    //     static char frameNum[100]("0"); int loadNum(0), frameNo(0);
    //     float boarder(300.f);

    //     viewer.data(v1).set_edges(C,BE,sea_green);
    //     //viewer.data(v1).set_points(C,sea_green);
    //     viewer.data(v1).show_lines = false;
    //     viewer.data(v1).show_overlay_depth = false;
    //     viewer.data(v1).line_width = 1;
    //     viewer.data(v1).point_size = 8;
    //     viewer.data(v2).point_size = 4;
    //     viewer.data(v1).double_sided = false;
    //     viewer.data(v1_patient).show_lines = true;
    //     viewer.data(v1_patient).show_overlay_depth = false;
    //     viewer.data(v1_patient).show_faces = false;
    //     viewer.data(v1_cArm).set_colors(white);
    //     viewer.data(v1_cArm).show_lines = false;

    //     viewer.data(v2).show_lines = false;
    //     viewer.data(v2).show_overlay_depth = true;
    //     viewer.data(v2).double_sided = false;

    //     int v1_view, v2_view;
    //     viewer.callback_init = [&](igl::opengl::glfw::Viewer &)
    //     {
    //         viewer.core().viewport = Eigen::Vector4f(0, 0, 640, 800);
    //         v1_view = viewer.core_list[0].id;
    //         v2_view = viewer.append_core(Eigen::Vector4f(640, 0, 640, 800));
    //         viewer.core(v1_view).background_color=Eigen::Vector4f(109./255.,100./255.,102./255.,1);
    //         viewer.core(v2_view).background_color=Eigen::Vector4f(163./255.,163./255.,163./255.,1);
    // //        viewer.core(v1_view).background_color=Eigen::Vector4f(1.,1.,1.,1.);
    // //        viewer.core(v2_view).background_color=Eigen::Vector4f(1.,1.,1.,1.);
    //         viewer.core(v1_view).camera_eye=Eigen::Vector3f(0,0,-2);
    //         viewer.core(v2_view).camera_eye=Eigen::Vector3f(0,0,-3);
    //         viewer.core(v1_view).camera_up=Eigen::Vector3f(0,-1,0);
    //         viewer.core(v2_view).camera_up=Eigen::Vector3f(0,-1,0);

    //         viewer.data(v1).set_visible(false, v2_view);
    //         viewer.data(v1_patient).set_visible(false, v2_view);
    //         viewer.data(v1_cArm).set_visible(false, v2_view);
    //         viewer.data(v2).set_visible(false, v1_view);

    //         return false;
    //     };

    //     viewer.callback_post_resize = [&](igl::opengl::glfw::Viewer &v, int w, int h) {
    //         boarder =  w * 3 / 4;
    //         v.core( v1_view).viewport = Eigen::Vector4f(0, 0, w * 3 / 4, h);
    //         v.core(v2_view).viewport = Eigen::Vector4f(w *3 / 4, 0, w/4, h);
    //         return true;
    //     };

    //     static float dap(0.01f); // Gycm2/s
    //     double doseFactor = dap/DAPperNPS;

    //     //dose data
    //     VectorXd accDoseMap = ArrayXd::Zero(V.rows());
    //     double accLensMap(0.);
    //     static char psdAcc[100], psdRate[100], avgAcc[100], avgRate[100], lensAcc[100], lensRate[100];

    //     function<void()> ResetDose = [&](){
    //         accDoseMap = ArrayXd::Zero(V.rows());
    //         accLensMap = 0; frameNo = 0; expTimeD = 0;
    //         strcpy(psdAcc,""); strcpy(psdRate,"");
    //         strcpy(avgAcc,""); strcpy(avgRate,"");
    //         strcpy(lensAcc,""); strcpy(lensRate,"");
    //         strcpy(expTime,""); strcpy(frameNum,"");
    //         strcpy(rec,"idle");
    //         viewer.core(v1_view).is_animating = false;
    //         viewer.data(v1).set_data(accDoseMap);
    //         viewer.data(v2).set_data(accDoseMap);
    //     };

    //     //switches
    //     static bool calib(true);
    //     if(calibFrame==0) calib = false;
    //     static bool doseCal(true);
    //     double preStamp = -1; float p = ImGui::GetStyle().FramePadding.x;
    //     Matrix3d rotMat = Matrix3d::Identity();
    //     Matrix3d rotInv = Matrix3d::Identity();

    //     function<void(double,double,double)> CalculateRot = [&](double yAxis, double xAxis, double zAxis){
    //         yAxis *= PI/180.;xAxis *= PI/180.;zAxis *= PI/180.;
    //         double c1=cos(yAxis); double c2=cos(xAxis); double c3=cos(zAxis);
    //         double s1=sin(yAxis); double s2=sin(xAxis); double s3=sin(zAxis);
    //         rotMat<< c1*c3+s1*s2*s3, c3*s1*s2-c1*s3, c2*s1, c2*s3, c2*c3, -s2, c1*s2*s3-c3*s1, c1*c3*s2+s1*s3, c1*c2;
    //         rotInv = rotMat.inverse();
    //     };

    //     menu.callback_draw_viewer_menu = [&](){
    //         if (ImGui::CollapsingHeader("Information", ImGuiTreeNodeFlags_None))
    //         {
    //             ImGui::Text("Author : Haegin Han");
    //             ImGui::Text("Info : RDC modult of DCIR system");
    //         }
    //         if (ImGui::CollapsingHeader("Iso center position", ImGuiTreeNodeFlags_DefaultOpen)){
    //             static float isoCenterInput[3] = {isoCenter(0),isoCenter(1),isoCenter(2)};
    //             if(ImGui::InputFloat3("isocenter pos.", isoCenterInput, "%.1f", ImGuiInputTextFlags_EnterReturnsTrue)){
    //                 isoCenter = Vector3d(isoCenterInput[0],isoCenterInput[1],isoCenterInput[2]);
    //                 gridStart = isoCenter + isoRelat;
    //                 viewer.data(v1_patient).set_vertices(V_patient.rowwise()+(isoCenter-Vector3d(0,0,60)).transpose());
    //                 viewer.data(v1_cArm).set_vertices(V_cArm.rowwise()+(isoCenter-Vector3d(0,0,60)).transpose());
    //             }
    //         }
    //         if (ImGui::CollapsingHeader("Beam Conditions", ImGuiTreeNodeFlags_DefaultOpen))
    //         {
    //             static string kVp("80");
    //             if(ImGui::InputText("kVp", kVp, ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_CharsDecimal)) cout<<"new condition: "<<kVp<<" kVp"<<endl;
    //             if(ImGui::InputFloat("Gycm2/s (DAP)", &dap, ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_CharsDecimal)){
    //                 cout<<"new condition: "<<dap<<" Gycm2/s"<<endl;
    //                 doseFactor = dap/DAPperNPS;
    //                 sprintf(rMaxChar,"%3.2f",maxSkin0*0.1*3600.e3*doseFactor);
    //             }
    //             static int cArmRot[2] = {0,0};
    //             static int lArmRot(0);
    //             if(ImGui::InputInt2("ang./rot. [deg.]",cArmRot, ImGuiInputTextFlags_EnterReturnsTrue)){
    //                 //L-arm(Y), rotation(X), angulation(Z)
    //                 CalculateRot(lArmRot,  cArmRot[1], cArmRot[0]);
    //                 viewer.data(v1_cArm).set_vertices(((rotMat*(V_cArm.rowwise()-Vector3d(0,0,60).transpose()).transpose()).colwise()+isoCenter).transpose());
    //                 if(cArmRot[0]==45){
    //                     ifstream ifsMap("./doseMaps/1_conf.map", ios::binary);
    //                     ifsMap.read((char*) &doseMapS0[0], ijk[0]*ijk[1]*ijk[2]*sizeof(double));
    //                     ifsMap.read((char*) &doseMapL0[0], ijk[0]*ijk[1]*ijk[2]*sizeof(double));
    //                     ifsMap.close();
    //                     maxSkin0 = *max_element(doseMapS0.begin(),doseMapS0.end());
    //                     CalculateRot(0, 0, 0);
    //                 }else{
    //                     ifstream ifsMap("./doseMaps/0_conf.map", ios::binary);
    //                     ifsMap.read((char*) &doseMapS0[0], ijk[0]*ijk[1]*ijk[2]*sizeof(double));
    //                     ifsMap.read((char*) &doseMapL0[0], ijk[0]*ijk[1]*ijk[2]*sizeof(double));
    //                     ifsMap.close();
    //                     maxSkin0 = *max_element(doseMapS0.begin(),doseMapS0.end());
    //                 }

    //             }
    //             enum LArmRot {right, up, left}; //patient viewpoint
    //             static LArmRot lArmRotC = up;
    //             if(ImGui::Combo("L-arm Rot.", (int*)(&lArmRotC), "-90 deg\0  0 deg\0 90 deg\0\0")){
    //                 if(lArmRotC==right)     lArmRot = -90;
    //                 else if(lArmRotC==up)   lArmRot = 0;
    //                 else if(lArmRotC==left) lArmRot = 90;
    //                 CalculateRot(lArmRot,  cArmRot[1], cArmRot[0]);
    //                 viewer.data(v1_cArm).set_vertices(((rotMat*(V_cArm.rowwise()-Vector3d(0,0,60).transpose()).transpose()).colwise()+isoCenter).transpose());
    //             }
    //             float detNum[2] = {100, 22};
    //             ImGui::InputFloat2("SID/FD [cm]", detNum,"%.1f");
    //         }
    //         if (ImGui::CollapsingHeader("C-arm Table"), ImGuiTreeNodeFlags_DefaultOpen)
    //         {
    //             static float tablePos[3] = {0,0,0};
    //             ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.75f);
    //             if(ImGui::InputFloat3("[cm]", tablePos, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue)){
    // //                patientMove = Vector3d(patientTrans[0],patientTrans[1],patientTrans[2]);
    // //                viewer.data(v1_patient).set_vertices(V_patient.rowwise()+patientMove.transpose());
    //             }

    //             ImGui::PopItemWidth();
    //             float bedAxis[2] = {0,0};
    //             ImGui::InputFloat2("long./lat. [cm]", bedAxis);
    //             static int bedRot = 0;
    //             ImGui::InputInt("Rotation [deg]", &bedRot);
    //         }
    //         if (ImGui::CollapsingHeader("Viewer option"), ImGuiTreeNodeFlags_DefaultOpen)
    //         {
    //             static bool cArmView(true);
    //             if(ImGui::Checkbox("show C-arm",&cArmView)){
    //                 if(cArmView)  viewer.data(v1_cArm).set_visible(true, v1_view);
    //                 else          viewer.data(v1_cArm).set_visible(false, v1_view);
    //             }
    //             static bool patientView(true);
    //             if(ImGui::Checkbox("show patient",&patientView)){
    //                 if(patientView)  viewer.data(v1_patient).set_visible(true, v1_view);
    //                 else          viewer.data(v1_patient).set_visible(false, v1_view);
    //             }
    //         }
    //         if (ImGui::CollapsingHeader("Record (temp.)"), ImGuiTreeNodeFlags_DefaultOpen)
    //         {
    //             float w = ImGui::GetContentRegionAvailWidth();
    //             float p = ImGui::GetStyle().FramePadding.x;
    //             if(ImGui::Button("record!", ImVec2((w-p)/2.,0))){
    //                 if(loading) cout<<"wrong status!"<<endl;
    //                 else{
    //                     recording = true;
    //                     recordedData.clear();
    //                     strcpy(rec,"recording..");
    //                 }
    //             }

    //             ImGui::SameLine(0, p);
    //             if(ImGui::Button("stop!", ImVec2((w-p)/2.,0))){
    //                 recording = false;
    //                 string fName;
    //                 cout<<"fileName: "<<flush; cin>>fName;
    //                 ofstream ofs("./records/"+fName+".dat");
    //                 for(size_t n=0;n<recordedData.size();n++){
    //                     for(int i=0;i<155;i++) ofs<<recordedData[n][i]<<"\t";
    //                     ofs<<endl;
    //                     for(int i=0;i<24;i++) ofs<<jointData[n](i,0)<<"\t"<<jointData[n](i,1)<<"\t"<<jointData[n](i,2)<<"\t";
    //                     ofs<<endl;
    //                 }ofs.close();
    //                 igl::writeDMAT("./records/"+fName + ".jt",jointTrans);
    //                 cout<<"exported ./records/"+fName+".dat,jt ("<<recordedData.size()<<" frames)"<<endl;
    //                 recordedData.clear(); jointData.clear();
    //                 strcpy(rec,"idle");
    //             }
    //             if(ImGui::Button("load", ImVec2(w,0))){
    //                 if(recording) cout<<"wrong status!"<<endl;
    //                 else{
    //                     ResetDose();
    //                     viewer.core(v1_view).is_animating = true;
    //                     strcpy(rec,"loading..");
    //                     recordedData.clear();
    //                     loading = true;
    //                 }
    //             }
    //         }
    //         ImGui::Separator();
    //         ImGui::PushItemWidth(ImGui::GetContentRegionAvailWidth());
    //         ImGui::InputText("",rec,ImGuiInputTextFlags_ReadOnly);
    //         ImGui::PopItemWidth();
    //         if(ImGui::Button("ANIMATE/STOP", ImVec2(ImGui::GetContentRegionAvailWidth(),0))){
    //             viewer.core(v1_view).is_animating = !viewer.core(v1_view).is_animating;
    //             preStamp = -1;
    //             if(viewer.core(v1_view).is_animating && !loading) strcpy(rec, "real-time animating");
    //             else strcpy(rec, "idle");
    //         }
    //         if(ImGui::Checkbox("Calculate dose", &doseCal))
    //             preStamp = -1;
    //         //if(calibFrame)
    //             if(ImGui::Checkbox("Use calibrated phantom", &calib)){
    //                 if(calib) viewer.data(v2).set_vertices(V_calib_cumul);
    //                 else      viewer.data(v2).set_vertices(V_cumul);
    //             }
    //         if(ImGui::Button("RESET DOSE", ImVec2(ImGui::GetContentRegionAvailWidth(),0)))  ResetDose();
    //     };

    //     float rotAngle_v2;
    //     menu.callback_draw_custom_window = [&](){
    //         ImGui::SetNextWindowPos(ImVec2(boarder - 180.f*menu.menu_scaling(),0.0f),ImGuiCond_FirstUseEver);
    //         ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f), ImGuiCond_FirstUseEver);
    //         ImGui::SetNextWindowSizeConstraints(ImVec2(180.f*menu.menu_scaling(), -1.0f), ImVec2(180.f*menu.menu_scaling(), -1.0f));
    //         ImGui::Begin("Current Satus", nullptr,ImGuiWindowFlags_NoSavedSettings);
    //         if (ImGui::CollapsingHeader("Status Summary", ImGuiTreeNodeFlags_DefaultOpen))
    //         {
    //             ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.25f);
    //             ImGui::InputText("Exposure time [s]", expTime, ImGuiInputTextFlags_ReadOnly);
    //             ImGui::InputText("Total Frame No.", frameNum, ImGuiInputTextFlags_ReadOnly);
    //             ImGui::PopItemWidth();
    //             ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.5f);
    //             static int maxFPS(10);
    //             viewer.core(v1).animation_max_fps = maxFPS;
    //             if(ImGui::InputInt("max FPS", &maxFPS))
    //                  viewer.core(v1).animation_max_fps = maxFPS;
    //             ImGui::PopItemWidth();
    //         }
    //         ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.25f);
    //         float p = ImGui::GetStyle().FramePadding.x;
    //         ColorbarPlugin cbar(colorMap);
    //         sprintf(rMaxChar,"%3.2f",maxSkin0*0.1*3600.e3*doseFactor);
    //         if (ImGui::CollapsingHeader("Dose Information", ImGuiTreeNodeFlags_DefaultOpen))
    //         {
    //             ImGui::BulletText("PSD (Peak skin dose)");
    //             ImGui::InputText("mGy/h" , psdRate, ImGuiInputTextFlags_ReadOnly); ImGui::SameLine(0,p);
    //             ImGui::InputText("mGy  ", psdAcc, ImGuiInputTextFlags_ReadOnly);
    //             ImGui::BulletText("Average skin dose");
    //             ImGui::InputText("mGy/h", avgRate, ImGuiInputTextFlags_ReadOnly); ImGui::SameLine(0,p);
    //             ImGui::InputText("mGy  ", avgAcc, ImGuiInputTextFlags_ReadOnly);
    //             ImGui::BulletText("Lens dose");
    //             ImGui::InputText("mGy/h", lensRate, ImGuiInputTextFlags_ReadOnly);ImGui::SameLine(0,p);
    //             ImGui::InputText("mGy  ", lensAcc, ImGuiInputTextFlags_ReadOnly);
    //             ImGui::BulletText("Skin dose color bar");
    //             cbar.draw_colorbar(rMaxChar,aMaxChar);
    //         }
    //         ImGui::PopItemWidth();
    //         ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.5f);
    //         if (ImGui::CollapsingHeader("Display Settings", ImGuiTreeNodeFlags_DefaultOpen))
    //         {
    //             Quaterniond identity = Quaterniond::Identity();
    //             RotationList vQ_hu(BE.rows(),identity);
    //             RotationList vQ_hu1;
    //             vector<Vector3d> vT_hu;
    //             static int angleA(10);
    //             static int angleH(90);
    //             static int angleL(5);
    //             function<void()> Widen=[&](){
    //                 vQ_hu[11] = AngleAxisd(angleA/180.*PI, Vector3d(0,0,1));
    //                 vQ_hu[4] = AngleAxisd(-angleA/180.*PI, Vector3d(0,0,1));
    //                 vQ_hu[12] = AngleAxisd(angleH*0.5/180.*PI, Vector3d(0,1,0));
    //                 vQ_hu[5] = AngleAxisd(-angleH*0.5/180.*PI, Vector3d(0,1,0));
    //                 vQ_hu[13] = AngleAxisd(angleH*0.5/180.*PI, Vector3d(0,1,0));
    //                 vQ_hu[6] = AngleAxisd(-angleH*0.5/180.*PI, Vector3d(0,1,0));
    //                 vQ_hu[19] = AngleAxisd(angleL/180.*PI, Vector3d(0,0,1));
    //                 vQ_hu[15] = AngleAxisd(-angleL/180.*PI, Vector3d(0,0,1));
    //                 igl::forward_kinematics(C,BE,P,vQ_hu,vQ_hu1,vT_hu);
    //                 myDqs(V,cleanWeights,vQ_hu1,vT_hu,V_cumul);
    //                 myDqs(V_calib,cleanWeights,vQ_hu1,vT_hu,V_calib_cumul);
    //                 if(calib) viewer.data(v2).set_vertices(V_calib_cumul);
    //                 else      viewer.data(v2).set_vertices(V_cumul);
    //                 viewer.data(v2).compute_normals();
    //             };
    //             Widen();
    //             if(ImGui::InputInt("hand rot.", &angleH, 10, 20)) Widen();
    //             if(ImGui::InputInt("arm rot." , &angleA,  5, 10)) Widen();
    //             if(ImGui::InputInt("leg rot." , &angleL,  5, 10)) Widen();
    //             static float rotTime(4.);
    //             rotAngle_v2 = 2.*PI/viewer.core(v1_view).animation_max_fps/rotTime;
    //             if(ImGui::InputFloat("360 rot. time", &rotTime, 0.1, 0.1)){
    //                 if(viewer.core(v1).animation_max_fps>0)
    //                     rotAngle_v2 = 2.*PI/viewer.core(v1_view).animation_max_fps/rotTime;
    //             }
    //             if(ImGui::Button("stop/start rotation",ImVec2(ImGui::GetContentRegionAvailWidth(),0))){
    //                 viewer.core(v2_view).is_animating = !viewer.core(v2_view).is_animating;
    //                 viewer.core(v2_view).camera_eye=Eigen::Vector3f(0,0,-3);
    //             }
    //         }
    //         ImGui::PopItemWidth();

    //         ImGui::End();
    //     };

    //     bool showEye(false);

    //     viewer.callback_key_down = [&](igl::opengl::glfw::Viewer &,unsigned char key, int)->bool
    //     {
    //         switch(key){
    //         case ' ': //animate
    //             viewer.data(v1).show_custom_labels = false;
    //             viewer.core(v1_view).is_animating = !viewer.core(v1_view).is_animating;
    //             if(viewer.core(v1_view).is_animating && doseCal){
    //                 preStamp = -1;
    //             }
    //             return true;
    //         }
    //         return true;
    //     };

    // vector<int> poseJoints = {0, 1, 2, 4, 5, 6, 7, 26, 11, 12, 13, 14, 18, 19, 20, 22, 23, 24, 8, 15, 21, 25, 28, 30};
    // time_t startT;
    // igl::Timer timeStamp;

    //animator
    // viewer.core().animation_max_fps = 10;
    // function<void(RotationList, vector<Vector3d>, double)> doseMapFunc = [&](RotationList vQ, vector<Vector3d> vT, double timeS){

    //     MatrixXd V_disp(V);
    //     if(calib) V_disp = V_calib;

    //     MatrixXd U;
    //     myDqs(V_disp,cleanWeights,vQ,vT,U);
    //     viewer.data(v1).set_vertices(U);
    //     viewer.data(v1).compute_normals();

    //     if(!doseCal && !recording && !loading) return;

    //     MatrixXd normals = viewer.data(v1).V_normals;
    //     MatrixXd vIso = U;
    //     vIso.rowwise() -= isoCenter.transpose();

    //     U = ((rotInv * (U.rowwise()-isoCenter.transpose()).transpose()).colwise()+isoCenter).transpose();
    //     U.rowwise() += (- gridStart).transpose();
    //     U *= 0.2; // 5cm grid
    //     VectorXd values = ArrayXd::Zero(V.rows());
    //     for(int i=0;i<U.rows();i++){
    //         double factor(1);
    //         if(vIso.row(i).squaredNorm()>900){ // do not consider cosine for 30cm-radius sphere from isocenter
    //             double dotProd = vIso.row(i).dot(normals.row(i))/vIso.row(i).norm();
    //             if(dotProd>0.5) continue;
    //             if(dotProd>0) factor = 0.5-dotProd;
    //         }
    //         int idx = floor(U(i,0))*ijk[2]*ijk[1]+floor(U(i,1))*ijk[2]+floor(U(i,2));
    //         if(idx<60*60*60)
    //             values(i) = doseMapS0[idx]*factor;
    //         else
    //             cout<<"out!!"<<endl;
    //     }
    //     ArrayXd eyedose = ArrayXd::Zero(eye2ply.size());
    //     transform(eye2ply.begin(), eye2ply.end(), &eyedose(0), [&](int i)->double{
    //         int idx = floor(U(i,0))*ijk[2]*ijk[1]+floor(U(i,1))*ijk[2]+floor(U(i,2));
    //         if(idx<60*60*60)  return doseMapL0[idx];
    //         else              return 0;
    //     });
    //     viewer.data(v1).set_data(values,0,maxSkin0*0.1,colorMap);
    //     sprintf(psdRate,"%3.2f",values.maxCoeff()*3600.e3*doseFactor);
    //     sprintf(avgRate, "%3.2f", (values.array()*W_avgSkin.array()).sum()*3600.e3*doseFactor);
    //     double lensDose = (eyedose*W_Lens.array()).sum() * doseFactor;
    //     sprintf(lensRate,"%3.2f", lensDose*3600.e3);
    //     if(doseCal){
    //         accDoseMap += values*doseFactor*timeS;   //unit gray
    //         accLensMap += lensDose*timeS; //unit gray
    //         VectorXd::Index maxRow;
    //         double psdMax = accDoseMap.maxCoeff(&maxRow);
    //         viewer.data(v2).set_data(accDoseMap,0,psdMax,colorMap);
    //         if(calib) viewer.data(v2).set_points(V_calib_cumul.row(maxRow),red);
    //         else               viewer.data(v2).set_points(V_cumul.row(maxRow),red);
    //         sprintf(psdAcc, "%3.2f", psdMax*1.e3);
    //         sprintf(aMaxChar, "%3.2f", psdMax*1.e3);
    //         sprintf(avgAcc, "%3.2f", (accDoseMap.array()*W_avgSkin.array()).sum()*1.e3);
    //         sprintf(lensAcc, "%3.2f", accLensMap*1.e3);
    //     }
    // };

    //     RotationList vQprev, vQprevprev;
    //     int rotN_v2(0); double loadTime(0);

    //     startT = time(NULL);
    //     timeStamp.start();

    //     viewer.launch(true, false, "DCIR System (RDC module)");

    //     // receiverTH.join();

    //     //quit signal
    //     int sig = 0;

    return EXIT_SUCCESS;
}
