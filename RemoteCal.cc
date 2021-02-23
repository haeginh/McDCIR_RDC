#include "ClientSocket.h"
#include "SocketException.h"
#include <iostream>
#include <ctime>
#include "functions.h"

//#include "KinectData.h"
//#include "Polygon.hh"
#include "bodytracking.hh"
//#include "bodydeformer.hh"
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
#include <igl/opengl/glfw/Viewer.h>
#include <igl/directed_edge_parents.h>
#include <igl/directed_edge_orientations.h>
#include <igl/deform_skeleton.h>
#include <igl/forward_kinematics.h>
#include <igl/dqs.h>
#include <igl/Timer.h>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

void PrintUsage(){
    cout<<"<Usage>"<<endl;
    cout<<"pre-processing    -> ./MotionCapt [phantom_prefix(.mesh, .tgf)] [outer.ply]"<<endl;
    cout<<"animating/doseCal -> ./MotionCapt [phantom_prefix(.mesh, .tgf, _o.dmat, _oj.dmat, _o.bary)] (-rst [rst.ply])"<<endl;
    cout<<"                     (PLY file is for animating visualization)"<<endl;
    exit(1);
}

using namespace Eigen;
using namespace std;

int main(int argc, char** argv){
    if(argc!=3 && argc!=2 && argc!=4) PrintUsage();
    string prefix = argv[1];


    //main variables
    MatrixXd C, V_o, W_o, W_j, V;
    MatrixXi BE, T_o, F_o, T, F;
    RowVector3d sea_green(70./255.,252./255.,167./255.);
    RowVector3d blue(0.,0.,1.);
    igl::Timer timer;

    cout<<"Read "+prefix+".tgf"<<endl;
    igl::readTGF(prefix+".tgf",C,BE);

    if(argc==3){
        string argv2 = argv[2];
        if(argv2.substr(argv2.size()-4,4)!=".ply") PrintUsage();
        //perform BBW
        cout<<"Perform BBW for "<<argv2<<endl;
        timer.start();
        MatrixXd V_PLY; MatrixXi F_PLY;
        igl::readPLY(argv2,V_PLY,F_PLY);
        MatrixXd boneP = GenerateBonePoints(C,BE,1.);
        MatrixXd V_oSurf(V_PLY.rows()+boneP.rows(),3);
        V_oSurf<<V_PLY, boneP;
        igl::copyleft::tetgen::tetrahedralize(V_oSurf, F_PLY, "pYq", V_o, T_o, F_o);
        igl::readMESH(prefix+".mesh", V, T, F);
        map<int, map<int, double>> baryCoord = GenerateBarycentricCoord(V_o, T_o, V);

        cout<<"Extract eye nodes.."<<flush;
        set<int> eyeL = {6600, 6601, 6700, 6701, 6702};
        set<int> eyeR = {6800, 6801, 6900, 6901, 6902};
        ifstream meshF(prefix+".mesh");
        string dump;
        while(getline(meshF, dump)) if(dump.substr(0,10)=="Tetrahedra") break;
        int tetN, av, bv, cv, dv, idx; meshF>>tetN;
        vector<int> eyeLV, eyeRV;
        for(int i=0;i<tetN;i++){
            meshF>>av>>bv>>cv>>dv>>idx;
            if(eyeL.find(idx)!=eyeL.end()) {
                eyeLV.push_back(av-1);eyeLV.push_back(bv-1);eyeLV.push_back(cv-1);eyeLV.push_back(dv-1);
            }else if(eyeR.find(idx)!=eyeR.end()) {
                eyeRV.push_back(av-1);eyeRV.push_back(bv-1);eyeRV.push_back(cv-1);eyeRV.push_back(dv-1);
            }
        }meshF.close();
        sort(eyeLV.begin(),eyeLV.end());sort(eyeRV.begin(),eyeRV.end());
        eyeLV.erase(unique(eyeLV.begin(),eyeLV.end()),eyeLV.end());
        eyeRV.erase(unique(eyeRV.begin(),eyeRV.end()),eyeRV.end());
        cout<<"R: "<<eyeRV.size()<<"/ L: "<<eyeLV.size()<<flush;
        vector<int> eyeLV2, eyeRV2;
        for(int i:eyeLV) for(auto iter:baryCoord[i]) eyeLV2.push_back(iter.first);
        for(int i:eyeRV) for(auto iter:baryCoord[i]) eyeRV2.push_back(iter.first);
        sort(eyeLV2.begin(),eyeLV2.end());sort(eyeRV2.begin(),eyeRV2.end());
        eyeLV2.erase(unique(eyeLV2.begin(),eyeLV2.end()),eyeLV2.end());
        eyeRV2.erase(unique(eyeRV2.begin(),eyeRV2.end()),eyeRV2.end());
        cout<<" -> R: "<<eyeRV2.size()<<"/ L: "<<eyeLV2.size()<<endl;

        cout<<"<Calculate Joint Weights>"<<endl;
        if(!CalculateScalingWeights(C, V_o, T_o, W_j, eyeLV2, eyeRV2))  return EXIT_FAILURE;
        igl::normalize_row_sums(W_j,W_j);

        MatrixXd bc; VectorXi b;
        igl::boundary_conditions(V_o,T_o,C,VectorXi(),BE,MatrixXi(),b,bc);
        cout<<bc.rows()<<" "<<bc.cols()<<endl;
        igl::BBWData bbw_data;
        bbw_data.active_set_params.max_iter = 10;
        bbw_data.verbosity = 2;
        cout<<"<Calculate Bone Weights>"<<endl;
        if(!igl::bbw(V_o,T_o,b,bc,bbw_data,W_o))  return EXIT_FAILURE;
        igl::normalize_row_sums(W_o,W_o);

        //Print outputs
        cout<<"Write "<<prefix+"_o.dmat"<<endl;
        igl::writeDMAT(prefix+"_o.dmat",W_o);
        cout<<"Write "<<prefix+"_oj.dmat"<<endl;
        igl::writeDMAT(prefix+"_oj.dmat",W_j);
        cout<<"Write "<<prefix+"_o.mesh"<<endl;
        igl::writeMESH(prefix+"_o.mesh", V_o, T_o, F_o);
        cout<<"Write "<<prefix+".bary"<<endl;
        PrintBaryCoords(prefix+".bary",baryCoord);
        timer.stop();
        cout<<"total time: "<<timer.getElapsedTimeInSec()<<endl;
        igl::opengl::glfw::Viewer viewer;
        viewer.data().set_mesh(V_o, F_o);
        viewer.data().set_edges(C,BE,sea_green);
        viewer.data().show_lines = false;
        viewer.data().show_overlay_depth = false;
        viewer.data().line_width = 1;
        viewer.data().point_size = 1;
        MatrixXd W = W_o;
        bool bone(true);
        int selected = -1;

        viewer.callback_key_down = [&](igl::opengl::glfw::Viewer &,unsigned char key, int)->bool
        {
            switch(key){
            case ',':
                selected = std::min(std::max(--selected,0),(int)W.cols()-1);
                viewer.data().set_data(W.col(selected));
                break;
            case '.':
                selected = std::min(std::max(++selected,0),(int)W.cols()-1);
                viewer.data().set_data(W.col(selected));
                break;
            case ' ':
                if(bone) W = W_j;
                else     W = W_o;
                bone = !bone;
                break;            }
            return true;
        };
        viewer.launch();
        return EXIT_SUCCESS;
    }

    //read phantom files
    cout<<"Read "+prefix+"_o.mesh"<<endl;
    igl::readMESH(prefix+"_o.mesh", V_o, T_o, F_o);
    cout<<"Read "+prefix+"_o.dmat"<<endl;
    igl::readDMAT(prefix+"_o.dmat", W_o);
    cout<<"Read "+prefix+"_oj.dmat"<<endl;
    igl::readDMAT(prefix+"_oj.dmat",W_j);
    igl::normalize_row_sums(W_j,W_j);
//    cout<<"Read "+prefix+".mesh"<<endl;
//    igl::readMESH(prefix+".mesh", V, T, F);
//    cout<<"Read "+prefix+".bary"<<endl;
//    map<int, map<int, double>> baryCoord = ReadBaryFile(prefix+".bary");
//    SparseMatrix<double> bary = GenerateBarySparse(baryCoord,V_o.rows());

    MatrixXd V_v2, T_v2, W_v2, Wj_v2; //data for version 2
    string listenerIP; int listenerPort;
    if(argc==4){
        if(string(argv[2])!="-rst") PrintUsage();
        MatrixXd V_ply; MatrixXi F_ply;
        cout<<"Set for "<<string(argv[3])<<endl;
        igl::readPLY(string(argv[3]),V_ply,F_ply);
        map<int, map<int, double>> baryCoord = GenerateBarycentricCoord(V_o,T_o,V_ply);
        SparseMatrix<double> bary = GenerateBarySparse(baryCoord,V_o.rows());

        MatrixXi F_tmp;
        igl::copyleft::tetgen::tetrahedralize(V_ply, F_ply, "pYq", V_v2, T_v2, F_tmp);
        map<int, map<int, double>> baryCoord2 = GenerateBarycentricCoord(V_o,T_o,V_v2);
        SparseMatrix<double> bary2 = GenerateBarySparse(baryCoord2,V_o.rows());
        W_v2 = bary2 * W_o; Wj_v2 = bary2*W_j;

        cout<<"Listener IP: "; cin>>listenerIP;
        cout<<"Listener port: "; cin>>listenerPort;
        cout<<"Send calib info to listener.."<<flush;
        try {
            ClientSocket client_socket(listenerIP, listenerPort );
            client_socket << "init" ;
            int vNum = V_v2.rows();
            client_socket.SendIntBuffer(&vNum,1);
            for(int i=0;i<vNum;i++){
                client_socket.SendDoubleBuffer(V_v2.row(i).data(),3);
                client_socket.SendDoubleBuffer(W_v2.row(i).data(),22);
                client_socket.SendDoubleBuffer(Wj_v2.row(i).data(),24);
            }
            int tNum = T_v2.rows();
            client_socket.SendIntBuffer(&tNum,1);
            for(int i=0;i<tNum;i++)
                client_socket.SendDoubleBuffer(T_v2.row(i).data(),4);
            int fNum = F_ply.rows();
            client_socket.SendIntBuffer(&fNum,1);
            for(int i=0;i<fNum;i++)
                client_socket.SendIntBuffer(F_ply.row(i).data(),3);
        }
        catch (SocketException& e) {cout << "Exception was caught:" << e.description() << endl;}
        cout<<"done"<<endl;

        V_o = V_ply; F_o = F_ply; W_o = bary*W_o; W_j = bary*W_j;
    }

    vector<map<int, double>> cleanWeights;
    double epsilon(1e-5);
    for(int i=0;i<W_o.rows();i++){
        double sum(0);
        map<int, double> vertexWeight;
        for(int j=0;j<W_o.cols();j++){
            if(W_o(i,j)<epsilon) continue;
            vertexWeight[j] = W_o(i,j);
            sum += W_o(i,j);
        }
        for(auto &iter:vertexWeight) iter.second /= sum;
        cleanWeights.push_back(vertexWeight);
    }

    //additional variables
    VectorXi P;
    igl::directed_edge_parents(BE,P);
    map<int, double> lengths;

    //distance to parent joint
    for(int i=0;i<BE.rows();i++)
        lengths[i] = (C.row(BE(i,0))-C.row(BE(i,1))).norm();

    // joint number conv.
    vector<int> i2k = {0, 1, 2, 4, 5, 6, 7, 26, 11, 12, 13, 14, 18, 19, 20, 22, 23, 24};
    map<int, int> k2i; for(size_t i=0;i<i2k.size();i++) k2i[i2k[i]] = i;

    // Start body tracking
    k4a_device_t device = nullptr;
    VERIFY(k4a_device_open(0, &device), "Open K4A Device failed!");

    // Start camera. Make sure depth camera is enabled.
    k4a_device_configuration_t deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    deviceConfig.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_OFF;
    deviceConfig.camera_fps = K4A_FRAMES_PER_SECOND_5;
    VERIFY(k4a_device_start_cameras(device, &deviceConfig), "Start K4A cameras failed!");

    // Get calibration information
    k4a_calibration_t sensorCalibration;
    VERIFY(k4a_device_get_calibration(device, deviceConfig.depth_mode, deviceConfig.color_resolution, &sensorCalibration),
           "Get depth camera calibration failed!");
    int depthWidth = sensorCalibration.depth_camera_calibration.resolution_width;
    int depthHeight = sensorCalibration.depth_camera_calibration.resolution_height;

    //preprocessing for KINECT data
    map<int, Quaterniond> kinectDataRot;
    vector<int> groups;
    Matrix3d tf2; tf2<<0,-1,0,0,0,-1,1,0,0;
    for(int id:groups = {0,1,2,18,19,20,26}) kinectDataRot[id]=Quaterniond(tf2);
    tf2<<0,1,0,0,0,1,1,0,0;
    for(int id:groups = {22,23,24}) kinectDataRot[id]=Quaterniond(tf2);
    tf2<<0,1,0,0,0,-1,-1,0,0;
    for(int id:groups = {5,6}) kinectDataRot[id]=Quaterniond(tf2);
    tf2<<0,-1,0,0,0,1,-1,0,0;
    for(int id:groups = {12,13}) kinectDataRot[id]=Quaterniond(tf2);
    tf2<<1,0,0,0,0,1,0,-1,0;
    kinectDataRot[11]=Quaterniond(tf2);
    tf2<<1,0,0,0,0,-1,0,1,0;
    kinectDataRot[4]=Quaterniond(tf2);
    tf2<<0,1,0,1,0,0,0,0,-1;
    kinectDataRot[7]=Quaterniond(tf2);
    tf2<<0,-1,0,1,0,0,0,0,1;
    kinectDataRot[14]=Quaterniond(tf2);

   RotationList alignRot;
    map<int, Vector3d> desiredOrt;
    groups={12,13,5,6,22,23,18,19};
    for(int id:groups) desiredOrt[id] = Vector3d(0,1,0);
    desiredOrt[4] = Vector3d(1,0,0);
    desiredOrt[11] = Vector3d(-1,0,0);
    for(int i=0;i<BE.rows();i++){
        if(desiredOrt.find(i2k[BE(i,0)])==desiredOrt.end())
            alignRot.push_back(kinectDataRot[i2k[BE(i,0)]]);
        else {
            Vector3d v = (C.row(k2i[i2k[BE(i,0)]+1])-C.row(BE(i,0))).transpose();
            alignRot.push_back(kinectDataRot[i2k[BE(i,0)]]*GetRotMatrix(v,desiredOrt[i2k[BE(i,0)]]));
        }
    }

    // Create Body Tracker
    k4abt_tracker_t tracker = nullptr;
    k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
    //tracker_config.processing_mode = K4ABT_TRACKER_PROCESSING_MODE_CPU;
    tracker_config.processing_mode = K4ABT_TRACKER_PROCESSING_MODE_GPU;
    VERIFY(k4abt_tracker_create(&sensorCalibration, tracker_config, &tracker), "Body tracker initialization failed!");

    // Start calibration
    Window3dWrapper window3d;
    window3d.Create("3D Visualization", sensorCalibration);
    window3d.SetCloseCallback(CloseCallback);
    window3d.SetKeyCallback(ProcessKey);

    int calibFrame(0);
//    map<int, Vec3> uprightOrien;
    map<int, double> calibLengths;
    Vector3d eyeL_pos(0,0,0), eyeR_pos(0,0,0);

    while (s_isRunning)
    {
        k4a_capture_t sensorCapture = nullptr;
        k4a_wait_result_t getCaptureResult = k4a_device_get_capture(device, &sensorCapture, 0);
        if (getCaptureResult == K4A_WAIT_RESULT_SUCCEEDED)
        {
            k4a_wait_result_t queueCaptureResult = k4abt_tracker_enqueue_capture(tracker, sensorCapture, 0);
            k4a_capture_release(sensorCapture);
            if (queueCaptureResult == K4A_WAIT_RESULT_FAILED){
                std::cout << "Error! Add capture to tracker process queue failed!" << std::endl;
                break;
            }
        }
        else if (getCaptureResult != K4A_WAIT_RESULT_TIMEOUT){
            std::cout << "Get depth capture returned error: " << getCaptureResult << std::endl;
            break;
        }

        // Pop Result from Body Tracker
        k4abt_frame_t bodyFrame = nullptr;
        k4a_wait_result_t popFrameResult = k4abt_tracker_pop_result(tracker, &bodyFrame, 0); // timeout_in_ms is set to 0

        i2k[7] =  3;
        if (popFrameResult == K4A_WAIT_RESULT_SUCCEEDED)
        {
            VisualizeResult(bodyFrame, window3d, depthWidth, depthHeight);
            if(calibSwitch && k4abt_frame_get_num_bodies(bodyFrame)){
                k4abt_body_t body;
                VERIFY(k4abt_frame_get_body_skeleton(bodyFrame, 0, &body.skeleton), "Get skeleton from body frame failed!");
                for(int i=0;i<BE.rows();i++){
                    if(BE(i,1)>=(int)i2k.size()) continue; //extrimity
                    k4a_float3_t t0 = body.skeleton.joints[i2k[BE(i,0)]].position;
                    k4a_float3_t t1 = body.skeleton.joints[i2k[BE(i,1)]].position;
                    float d[3] = {t1.xyz.x-t0.xyz.x,t1.xyz.y-t0.xyz.y,t1.xyz.z-t0.xyz.z};
                    calibLengths[i] += sqrt(d[0]*d[0]+d[1]*d[1]+d[2]*d[2]);
                }
                k4a_quaternion_t q = body.skeleton.joints[26].orientation;
                Quaterniond headRot = Quaterniond(q.wxyz.w,q.wxyz.x,q.wxyz.y,q.wxyz.z)*alignRot[k2i[26]];
                k4a_float3_t pos = body.skeleton.joints[26].position;
                Vector3d headPos = Vector3d(pos.xyz.x,pos.xyz.y,pos.xyz.z);

                pos = body.skeleton.joints[28].position;
                eyeL_pos += headRot.inverse()*(Vector3d(pos.xyz.x,pos.xyz.y,pos.xyz.z)-headPos);
                pos = body.skeleton.joints[30].position;
                eyeR_pos += headRot.inverse()*(Vector3d(pos.xyz.x,pos.xyz.y,pos.xyz.z)-headPos);
                cout<<"\rCalibration frame #"<<calibFrame++<<flush;
            }
            //Release the bodyFrame
            k4abt_frame_release(bodyFrame);
        }

        window3d.SetLayout3d(s_layoutMode);
        window3d.SetJointFrameVisualization(s_visualizeJointFrame);
        window3d.Render();
    }
    window3d.Delete();
    cout<<endl;
    i2k[7] =  26;

    MatrixXd V_calib;
    MatrixXd C_calib;
    MatrixXd jointTrans;
    if(calibFrame){
        int headJ(24), eyeLJ(22), eyeRJ(23);
        jointTrans = MatrixXd::Zero(C.rows(),3);
         for(int i=0;i<BE.rows();i++){
            if(calibLengths.find(i)==calibLengths.end()){
                calibLengths[i] = lengths[i];
                jointTrans.row(BE(i,1)) = jointTrans.row(BE(P(i),1));
                continue;
            }
            calibLengths[i] /= (double)calibFrame*10;
            double ratio = calibLengths[i]/lengths[i];
            cout<<i<<" : "<<lengths[i]<<" -> "<<calibLengths[i]<<" ("<<ratio*100<<"%)"<<endl;
            jointTrans.row(BE(i,1)) = (1-ratio)*(C.row(BE(i,0))-C.row(BE(i,1)));
            if(P(i)<0) continue;
            jointTrans.row(BE(i,1)) += jointTrans.row(BE(P(i),1));
        }

         eyeR_pos /= (double)calibFrame*10;
         eyeL_pos /= (double)calibFrame*10;
         jointTrans.row(eyeLJ) = C.row(headJ) + jointTrans.row(headJ) + eyeL_pos.transpose() - C.row(eyeLJ);
         jointTrans.row(eyeRJ) = C.row(headJ) + jointTrans.row(headJ) + eyeR_pos.transpose() - C.row(eyeRJ);
         jointTrans.row(headJ) = MatrixXd::Zero(1,3); jointTrans(headJ, 1) = (jointTrans(eyeLJ,1)+jointTrans(eyeRJ,1))*0.5;
         C_calib = C+jointTrans;

         cout<<W_j.rows()<<"*"<<W_j.cols()<<endl;
         cout<<jointTrans.rows()<<"*"<<jointTrans.cols()<<endl;
        V_calib = V_o+W_j*jointTrans.block(0,0,C.rows()-1,3);
    }


    // igl viewer
    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(V_o, F_o);
    viewer.data().set_edges(C,BE,sea_green);
    viewer.data().set_points(C,sea_green);
    viewer.data().show_lines = false;
    viewer.data().show_overlay_depth = false;
    viewer.data().line_width = 1;
    viewer.data().point_size = 8;
    viewer.data().double_sided = true;
    viewer.core().is_animating = false;
    for(int i=0;i<BE.rows();i++){
        Vector3d pos = (C.row(BE(i,0))+C.row(BE(i,1))).transpose()*0.5 + Vector3d(1,0,0);
        string label = to_string(i);
        viewer.data().add_label(pos,label);
     }
    viewer.data().show_custom_labels = true;
    bool calib(false), doseCal(false);

    viewer.callback_key_down = [&](igl::opengl::glfw::Viewer &,unsigned char key, int)->bool
    {
        switch(key){
        case '.': //scale to actor
            if(!calibFrame){
                cout<<"There is no calibration info!!"<<endl;
                return true;
            }
            viewer.data().set_vertices(V_calib);
            viewer.data().compute_normals();
            viewer.data().set_edges(C_calib,BE,sea_green);
            viewer.data().set_points(C_calib,sea_green);
            viewer.data().show_custom_labels = false;
            calib = true;
            return true;
        case ',': //use mrcp
            viewer.data().set_vertices(V_o);
            viewer.data().compute_normals();
            viewer.data().set_edges(C,BE,sea_green);
            viewer.data().set_points(C,sea_green);
            viewer.data().show_custom_labels = true;
            calib = false;
            return true;
        case ' ': //animate
            viewer.data().show_custom_labels = false;
            viewer.core().is_animating = !viewer.core().is_animating;
            return true;
        case '/': //calculate dose
            doseCal = !doseCal;
            if(doseCal){
                if(listenerIP.empty()){
                    cout<<"Listener IP: "; cin>>listenerIP;
                    cout<<"Listener port: "; cin>>listenerPort;
                    cout<<"Send calib info to listener.."<<flush;
                    try {
                        ClientSocket client_socket(listenerIP, listenerPort );
                        client_socket << "calib info" ;
                        client_socket.SendDoubleBuffer(jointTrans.block(0,0,C.rows()-1,3).data(),72);
                    }
                    catch (SocketException& e) {cout << "Exception was caught:" << e.description() << endl;}
                    cout<<"done"<<endl;
                }
                cout<<"Start the dose calculation!>>>>>>>>>>>>>"<<endl;
            }
            else cout<<"<<<<<<<<<<<<<Stop the dose calculation!"<<endl;
            return true;
        }
        return true;
    };

    int frameNo(0);
    vector<int> poseJoints = {0, 1, 2, 4, 5, 6, 7, 26, 11, 12, 13, 14, 18, 19, 20, 22, 23, 24, 8, 15, 21, 25, 28, 30};
    time_t startT;
    igl::Timer timeStamp;
    //double stamp;
    viewer.callback_pre_draw = [&](igl::opengl::glfw::Viewer &)->bool
    {
        if(viewer.core().is_animating){
            timer.start();
            MatrixXd V_disp, C_disp(poseJoints.size(),3), C_new(C);
            if(calib) {V_disp = V_calib;}
            else      {V_disp = V_o;}

            k4a_capture_t sensorCapture = nullptr;
            k4a_wait_result_t getCaptureResult = k4a_device_get_capture(device, &sensorCapture, 0); // timeout_in_ms is set to 0
            double stamp = timeStamp.getElapsedTimeInSec();

            if (getCaptureResult == K4A_WAIT_RESULT_SUCCEEDED)
            {
                k4a_wait_result_t queueCaptureResult = k4abt_tracker_enqueue_capture(tracker, sensorCapture, 0);
                k4a_capture_release(sensorCapture);
                if (queueCaptureResult == K4A_WAIT_RESULT_FAILED) {cerr<<"ERROR(1)!"<<endl; return false;}
            }
            else if (getCaptureResult != K4A_WAIT_RESULT_TIMEOUT) {cerr<<"ERROR(2)!"<<endl; return false;}

            // Pop Result from Body Tracker
            k4abt_frame_t bodyFrame = nullptr;
            k4a_wait_result_t popFrameResult = k4abt_tracker_pop_result(tracker, &bodyFrame, 0); // timeout_in_ms is set to 0
            if (popFrameResult == K4A_WAIT_RESULT_SUCCEEDED)
            {
                if(k4abt_frame_get_num_bodies(bodyFrame)) {
                    k4abt_body_t body;
                    k4abt_frame_get_body_skeleton(bodyFrame, 0, &body.skeleton);
                    for(size_t i=0;i<poseJoints.size();i++){
                        C_disp(i,0)=body.skeleton.joints[poseJoints[i]].position.xyz.x*0.1;
                        C_disp(i,1)=body.skeleton.joints[poseJoints[i]].position.xyz.y*0.1;
                        C_disp(i,2)=body.skeleton.joints[poseJoints[i]].position.xyz.z*0.1;
                    }
                    viewer.data().set_points(C_disp,blue);

                    vector<Vector3d> vT;
                    RotationList vQ;
                    C_new = C;
                    if(calib) C_new=C_calib;
                    C_new.row(0)=C_disp.row(0); //set root
                    for(int i=0;i<BE.rows();i++){
                        k4a_quaternion_t q = body.skeleton.joints[i2k[BE(i,0)]].orientation;
                        vQ.push_back(Quaterniond(q.wxyz.w,q.wxyz.x,q.wxyz.y,q.wxyz.z)*alignRot[i]);
                        vQ[i].normalize();
                        Affine3d a;
                        if(calib) a = Translation3d(Vector3d(C_new.row(BE(i,0)).transpose()))*vQ[i].matrix()*Translation3d(Vector3d(-C_calib.row(BE(i,0)).transpose()));
                        else      a = Translation3d(Vector3d(C_new.row(BE(i,0)).transpose()))*vQ[i].matrix()*Translation3d(Vector3d(-C.row(BE(i,0)).transpose()));
                        vT.push_back(a.translation());
                        C_new.row(BE(i,1)) = a*Vector3d(C_new.row(BE(i,1)));
                    }
                    viewer.data().set_edges(C_new, BE, sea_green);
                    MatrixXd U;
                    myDqs(V_disp,cleanWeights,vQ,vT,U);
                    viewer.data().set_vertices(U);
                    viewer.data().compute_normals();

                    if(doseCal){
                        //TCP/IP
                        try {
                            ClientSocket client_socket(listenerIP, listenerPort );
                            client_socket << "quaternion" ;
                            if(!calib) stamp = -stamp;
                            client_socket.SendDoubleBuffer(&stamp,1);

                            double arrQ[88], arrT[66];
                            for(int i=0;i<BE.rows();i++){
                                arrQ[4*i] = vQ[i].w(); arrQ[4*i+1] = vQ[i].x(); arrQ[4*i+2] = vQ[i].y(); arrQ[4*i+3] = vQ[i].z();
                                arrT[3*i] = vT[i](0); arrT[3*i+1] = vT[i](1); arrT[3*i+2] = vT[i](2);
                            }
                            client_socket.SendDoubleBuffer(arrQ,88);
                            client_socket.SendDoubleBuffer(arrT,66);
                        }
                        catch (SocketException& e) {cout << "Exception was caught:" << e.description() << endl;}
                        timer.stop();
                        cout<<"Frame #"<<frameNo<<" ("<<stamp<<"s) : "<<timer.getElapsedTimeInSec()<<"s"<<endl;
                        frameNo++;
                    }
                }

                k4abt_frame_release(bodyFrame);
            }
        }
        return false;
    };
    startT = time(NULL);
    timeStamp.start();
    viewer.launch();

    return EXIT_SUCCESS;
}
