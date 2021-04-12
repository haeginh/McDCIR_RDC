#include "ServerSocket.h"
#include "SocketException.h"

#include <iostream>
#include <ctime>
#include <thread>
#include <pthread.h>
#include <functions.h>
#include <mutex>

pthread_mutex_t sync_mutex;
pthread_cond_t  sync_cond;

pthread_mutex_t gmutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t  gcond  = PTHREAD_COND_INITIALIZER;

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

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/SparseCore>

//#include "trase.hpp"

void PrintUsage(){
    cout<<"<Usage>"<<endl;
    cout<<"pre-processing    -> ./MotionCapt [phantom_prefix(.mesh, .tgf)] [outer.ply]"<<endl;
    cout<<"animating/doseCal -> ./MotionCapt [phantom_prefix(.mesh, .tgf, _o.dmat, _oj.dmat, _o.bary)] (-rst [rst.ply])"<<endl;
    cout<<"                     (PLY file is for animating visualization)"<<endl;
    exit(1);
}

using namespace Eigen;
using namespace std;
typedef Triplet<double> T;

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
        //bbw_data.active_set_params.max_iter = 10;
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

    MatrixXd V_ply, W_ply, W_j_ply; MatrixXi F_ply;
    if(string(argv[2])!="-rst") PrintUsage();
    string skinF = argv[3];
    cout<<"Set for "<<skinF<<endl;
    igl::readPLY(string(argv[3]),V_ply,F_ply);
    map<int, map<int, double>> baryCoord = GenerateBarycentricCoord(V_o,T_o,V_ply);
    SparseMatrix<double> bary = GenerateBarySparse(baryCoord,V_o.rows());
    //V_o = V_ply; F_o = F_ply; W_o = bary*W_o; W_j = bary*W_j;
    W_ply = bary*W_o;
    W_j_ply = bary*W_j;
    //eye part
    MatrixXd V_eye; MatrixXi F_eye;
    igl::readPLY(skinF.substr(0,skinF.size()-4)+"_eye.ply",V_eye,F_eye);
    map<tuple<int,int,int>,vector<int>> grid = GenerateGrid(V_ply);
    vector<int> eye2ply;
    for(int i=0;i<V_eye.rows();i++){
        double x = V_eye(i,0); double y = V_eye(i,1); double z = V_eye(i,2);
        auto ijk = make_tuple(floor(x+0.5),floor(y+0.5),floor(z+0.5));
        for(int n:grid[ijk]){
            if(fabs(x-V_ply(n,0))>0.001) continue;
            if(fabs(y-V_ply(n,1))>0.001) continue;
            if(fabs(z-V_ply(n,2))>0.001) continue;
            eye2ply.push_back(n);
            break;
        }
    }
    vector<vector<int>> eyeFaces;
    for(int i=0;i<F_eye.rows();i++) {
        vector<int> face = {eye2ply[F_eye(i,0)],eye2ply[F_eye(i,1)],eye2ply[F_eye(i,2)]};
        sort(face.begin(),face.end());
        eyeFaces.push_back(face);
    } vector<int> eyeFaceIDs;
    for(int i=0;i<F_ply.rows();i++){
        vector<int> face = {F_ply(i,0),F_ply(i,1),F_ply(i,2)};
        sort(face.begin(),face.end());
        if(find(eyeFaces.begin(),eyeFaces.end(),face)!=eyeFaces.end())
            eyeFaceIDs.push_back(i);
    }

    vector<map<int, double>> cleanWeights;
    double epsilon(1e-5);
    for(int i=0;i<W_ply.rows();i++){
        double sum(0);
        map<int, double> vertexWeight;
        for(int j=0;j<W_ply.cols();j++){
            if(W_ply(i,j)<epsilon) continue;
            vertexWeight[j] = W_ply(i,j);
            sum += W_ply(i,j);
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

    //preprocessing for KINECT draw_vidata
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
    MatrixXd jointTrans = MatrixXd::Zero(C.rows(),3);
    if(calibFrame){
        int headJ(24), eyeLJ(22), eyeRJ(23);
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

        cout<<W_j_ply.rows()<<"*"<<W_j_ply.cols()<<endl;
        cout<<jointTrans.rows()<<"*"<<jointTrans.cols()<<endl;
        V_calib = V_ply+W_j_ply*jointTrans.block(0,0,C.rows()-1,3);
    }
    MatrixXd jt = jointTrans.block(0,0,C.rows()-1,3);

    // igl viewer plugin
    igl::opengl::glfw::imgui::ImGuiMenu menu;
    menu.callback_draw_viewer_window = [&](){
        float menu_width = 180.f * menu.menu_scaling();
        ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSizeConstraints(ImVec2(menu_width, -1.0f), ImVec2(menu_width, -1.0f));
        bool _viewer_menu_visible = true;
        ImGui::Begin(
            "RDC Module", &_viewer_menu_visible,
            ImGuiWindowFlags_NoSavedSettings
            | ImGuiWindowFlags_AlwaysAutoResize
        );
        ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.5f);
        menu.callback_draw_viewer_menu();
        ImGui::PopItemWidth();
        ImGui::End();
    };
    //data for version 2
    MatrixXd V_v2, W_v2, Wj_v2;
    MatrixXi T_v2;
    MatrixXi F_tmp;
    igl::copyleft::tetgen::tetrahedralize(V_ply, F_ply, "pYq", V_v2, T_v2, F_tmp);
    igl::writeMESH("model.mesh",V_v2,T_v2,F_ply);
    map<int, map<int, double>> baryCoord2 = GenerateBarycentricCoord(V_o,T_o,V_v2);
    SparseMatrix<double> bary2 = GenerateBarySparse(baryCoord2,V_o.rows());
    W_v2 = bary2 * W_o; Wj_v2 = bary2*W_j;
    igl::writeDMAT("model_W.dmat",W_v2,false);
    igl::writeDMAT("model_Wj.dmat",Wj_v2,false);

    cout<<"generating F_incident matrix..."<<flush;
    VectorXd F_area;
    MatrixXd W_incidentF=MatrixXd::Zero(V_ply.rows(),F_ply.rows());
    igl::doublearea(V_ply,F_ply,F_area);
    F_area.cwiseSqrt();
    for(int i=0;i<F_ply.rows();i++){
        for(int j=0;j<3;j++)
            W_incidentF(F_ply(i,j),i) =  F_area(i);
    }
    igl::normalize_row_sums(W_incidentF,W_incidentF);
    cout<<"done"<<endl;

    menu.callback_draw_viewer_menu = [&](){
        if (ImGui::CollapsingHeader("Information", ImGuiTreeNodeFlags_None))
        {
          ImGui::Text("Author : Haegin Han");
          ImGui::Text("IP/port: 166.104.155.29/30303");
        }
        static string ipAddress("localhost");
        static string userName("hurel");
        static string directory("~/RemoteCal_cal/");
        if (ImGui::CollapsingHeader("Initialization", ImGuiTreeNodeFlags_DefaultOpen))
        {
          ImGui::Text("[Accumul. dose cal.]");
          ImGui::InputText("IP", ipAddress, ImGuiInputTextFlags_AutoSelectAll);
          ImGui::InputText("user name", userName, ImGuiInputTextFlags_AutoSelectAll);
          ImGui::InputText("dir",directory, ImGuiInputTextFlags_AutoSelectAll);
          static int processNo(3), threadNo(3);
          ImGui::InputInt("process #", &processNo);
          ImGui::InputInt("thread #", &threadNo);
          static char status[64] = "before init";
          ImGui::InputText("status",status,ImGuiInputTextFlags_ReadOnly);
          if(ImGui::Button("initialize!", ImVec2(-1,0))){
              cout<<ipAddress<<": "<<directory<<endl;

              system(("scp model.mesh "+userName+"@"+ipAddress+":"+directory).c_str());
              system(("scp model_W.dmat "+userName+"@"+ipAddress+":"+directory).c_str());
              system(("scp model_Wj.dmat "+userName+"@"+ipAddress+":"+directory).c_str());

              cout<<"starting calculators..."<<endl;
              for(int i=0;i<processNo;i++){
                  system(("ssh "+userName+"@"+ipAddress+" "+directory+"run.sh "+to_string(threadNo)+" >log"+to_string(i)+" &").c_str());
              }
              cout<<"done"<<endl;
              strcpy(status,"initialized!");
          }
        }
        if (ImGui::CollapsingHeader("Beam Conditions", ImGuiTreeNodeFlags_DefaultOpen))
        {
            static string kVp("80");
            static string dap("0.1");
            if(ImGui::InputText("kVp", kVp, ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_CharsDecimal)) cout<<"new condition: "<<kVp<<" kVp"<<endl;
            if(ImGui::InputText("Gycm2/s (DAP)", dap, ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_CharsDecimal)) cout<<"new condition: "<<dap<<" Gycm2/s"<<endl;
            int cArmRot[2] = {0,0};
            ImGui::InputInt2("ang./rot. [deg.]",cArmRot);
            enum LArmRot {right, up, left}; //patient viewpoint
            static LArmRot lArmRot = left;
            ImGui::Combo("L-arm Rot.", (int*)(&lArmRot), "-90 deg\0  0 deg\0 90 deg\0\0");
            float detNum[2] = {100, 22};
            ImGui::InputFloat2("SID/FD [cm]", detNum,"%.1f");
        }
        if (ImGui::CollapsingHeader("C-arm Table"), ImGuiTreeNodeFlags_DefaultOpen)
        {
            float bedAxis[2] = {0,0};
            ImGui::InputFloat2("long./lat. [cm]", bedAxis);
            static int bedRot = 0;
            ImGui::InputInt("Rotation [deg]", &bedRot);
        }
    };
    auto colorMap = igl::COLOR_MAP_TYPE_PARULA;
    static char expTime[10]("0"); double expTimeD(0);
    static char frameNum[10]("0");
    static char calNum[10]("0");
    float boarder(300.f);
    menu.callback_draw_custom_window = [&](){
        ImGui::SetNextWindowPos(ImVec2(boarder - 180.f*menu.menu_scaling(),0.0f),ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSizeConstraints(ImVec2(180.f*menu.menu_scaling(), -1.0f), ImVec2(180.f*menu.menu_scaling(), -1.0f));
        ImGui::Begin(
            "Current Satus", nullptr,
            ImGuiWindowFlags_NoSavedSettings
        );
        ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.25f);
        ImGui::InputText("Exposure time [s]", expTime, ImGuiInputTextFlags_ReadOnly);
        ImGui::InputText("Total Frame No.", frameNum, ImGuiInputTextFlags_ReadOnly);
        ImGui::InputText("Calculated Frame#", calNum, ImGuiInputTextFlags_ReadOnly);
        ImGui::InputText("Peak Skin Dose [pGy]", calNum, ImGuiInputTextFlags_ReadOnly);
        ImGui::InputText("Avg. Skin Dose [pGy]", calNum, ImGuiInputTextFlags_ReadOnly);
        ImGui::InputText("Lens Dose Rate [pGy/s]", calNum, ImGuiInputTextFlags_ReadOnly);
        ImGui::InputText("Acc. Lens Dose [pGy]", calNum, ImGuiInputTextFlags_ReadOnly);
        ImGui::PopItemWidth();
        ColorbarPlugin cbar(colorMap);
        Eigen::Vector4f bgcolor(0, 0, 0.04, 1);
        cbar.draw_colorbar(1,0,bgcolor);
        ImGui::End();
    };
    // igl viewer
    igl::opengl::glfw::Viewer viewer;
    viewer.plugins.push_back(&menu);
    viewer.data().set_mesh(V_ply, F_ply);
    viewer.append_mesh();
    viewer.load_mesh_from_file("patient2.obj");
    viewer.append_mesh();
    viewer.data().set_mesh(V_ply, F_ply);

    int v1=viewer.data_list[0].id;
    int v1_patient=viewer.data_list[1].id;
    int v2=viewer.data_list[2].id;
    viewer.data(v1).set_edges(C,BE,sea_green);
    viewer.data(v1).set_points(C,sea_green);
    viewer.data(v1).show_lines = false;
    viewer.data(v1).show_overlay_depth = false;
    viewer.data(v1).line_width = 1;
    viewer.data(v1).point_size = 8;
    viewer.data(v1).double_sided = true;

    viewer.data(v1_patient).show_lines = true;
    viewer.data(v1_patient).show_overlay_depth = false;
    viewer.data(v1_patient).show_faces = false;

    viewer.data(v2).show_lines = false;
    viewer.data(v2).show_overlay_depth = false;
    viewer.data(v2).double_sided = true;

    int v1_view, v2_view;
    viewer.callback_init = [&](igl::opengl::glfw::Viewer &)
    {
      viewer.core().viewport = Eigen::Vector4f(0, 0, 640, 800);
      v1_view = viewer.core_list[0].id;
      v2_view = viewer.append_core(Eigen::Vector4f(640, 0, 640, 800));
      viewer.core(v1_view).background_color=Eigen::Vector4f(109./255.,100./255.,102./255.,1);
      viewer.core(v2_view).background_color=Eigen::Vector4f(163./255.,163./255.,163./255.,1);
      viewer.core(v1_view).camera_eye=Eigen::Vector3f(0,0,-2);
      viewer.core(v2_view).camera_eye=Eigen::Vector3f(0,0,-3);
      viewer.core(v1_view).camera_up=Eigen::Vector3f(0,-1,0);
      viewer.core(v2_view).camera_up=Eigen::Vector3f(0,-1,0);

      viewer.data(v1).set_visible(false, v2_view);
      viewer.data(v1_patient).set_visible(false, v2_view);
      viewer.data(v2).set_visible(false, v1_view);

      return false;
    };

    viewer.callback_post_resize = [&](igl::opengl::glfw::Viewer &v, int w, int h) {
      boarder =  w * 3 / 4;
      v.core( v1_view).viewport = Eigen::Vector4f(0, 0, w * 3 / 4, h);
      v.core(v2_view).viewport = Eigen::Vector4f(w *3 / 4, 0, w/4, h);
      return true;
    };

//    viewer.core(v1_view).is_animating = false;

//    for(int i=0;i<BE.rows();i++){
//        Vector3d pos = (C.row(BE(i,0))+C.row(BE(i,1))).transpose()*0.5 + Vector3d(1,0,0);
//        string label = to_string(i);
//        viewer.data(0).add_label(pos,label);
//    }
//    viewer.data(0).show_custom_labels = true;
    bool calib(false),doseCal(false),showEye(false);
    double preStamp = -1;

    viewer.callback_key_down = [&](igl::opengl::glfw::Viewer &,unsigned char key, int)->bool
    {
        switch(key){
        case '.': //scale to actor
            if(!calibFrame){
                cout<<"There is no calibration info!!"<<endl;
                return true;
            }
            viewer.data(v1).set_vertices(V_calib);
            viewer.data(v1).compute_normals();
            viewer.data(v1).set_edges(C_calib,BE,sea_green);
            viewer.data(v1).set_points(C_calib,sea_green);
            viewer.data(v1).show_custom_labels = false;
            calib = true;
            return true;
        case ',': //use mrcp
            viewer.data(v1).set_vertices(V_ply);
            viewer.data(v1).compute_normals();
            viewer.data(v1).set_edges(C,BE,sea_green);
            viewer.data(v1).set_points(C,sea_green);
            if(!viewer.core().is_animating)
                viewer.data(v1).show_custom_labels = true;
            calib = false;
            return true;
        case ' ': //animate
            viewer.data(v1).show_custom_labels = false;
            viewer.core(v1_view).is_animating = !viewer.core(v1_view).is_animating;
            if(viewer.core(v1_view).is_animating && doseCal){
                pthread_mutex_lock(&sync_mutex);
                pthread_cond_broadcast(&sync_cond);
                pthread_mutex_unlock(&sync_mutex);
                preStamp = -1;
            }
            return true;
        case '/': //calculate dose
            doseCal = !doseCal;
            if(doseCal) cout<<"Start the dose calculation!>>>>>>>>>>>>>"<<endl;
            else cout<<"<<<<<<<<<<<<<Stop the dose calculation!"<<endl;
            if(viewer.core(v1_view).is_animating && doseCal){
                pthread_mutex_lock(&sync_mutex);
                pthread_cond_broadcast(&sync_cond);
                pthread_mutex_unlock(&sync_mutex);
                preStamp = -1;
            }
        case '=': //show eye vertices
            showEye = !showEye;
            if(showEye){
                VectorXd eyeColor = VectorXd::Zero(V_ply.rows());
                for(int i:eye2ply) eyeColor(i)=1;
                viewer.data(v1).set_data(eyeColor, colorMap);
            }
        }
        return true;
    };

    map<int, pair<double, double>> doseMap; //<skinD, lensD>
    int ijk[3];
    //accum
    vector<array<double, 155>> poseData;
    vector<int> frameIDs;
    VectorXd doseData=VectorXd::Zero(F_ply.rows());
    mutex m, m2, m2_dose;
    int v2_calculators(0);
    //frame data
    int frameNo(0);

    thread receiverTH = thread([&](){
        ServerSocket server(30303);
        cout << "[TCP/IP] Listening..."<<endl;

        //threads
        thread* v1_cal;
        vector<thread*> v2_cals;
        int v2_calNum(100);

        //data for version 2
//        MatrixXd V_v2, W_v2, Wj_v2;
//        MatrixXi T_v2;
//        MatrixXi F_tmp;
//        igl::copyleft::tetgen::tetrahedralize(V_ply, F_ply, "pYq", V_v2, T_v2, F_tmp);
//        VectorXd F_area;
//        igl::doublearea(V_ply,F_ply,F_area);
//        F_area.cwiseSqrt();
//        MatrixXd W_incidentF=MatrixXd::Zero(V_ply.rows(),F_ply.rows());
//        for(int i=0;i<F_ply.rows();i++){
//            for(int j=0;j<3;j++)
//                W_incidentF(F_ply(i,j),i) =  F_area(i);
//        }
//        igl::normalize_row_sums(W_incidentF,W_incidentF);

//        map<int, map<int, double>> baryCoord2 = GenerateBarycentricCoord(V_o,T_o,V_v2);
//        SparseMatrix<double> bary2 = GenerateBarySparse(baryCoord2,V_o.rows());
//        W_v2 = bary2 * W_o; Wj_v2 = bary2*W_j;
        int numOfPack = floor(F_ply.rows()/180)+1;
        while(true){
            if(v1_cal && v2_cals.size()==v2_calNum) break;
            ServerSocket* _sock = new ServerSocket;
            server.accept ( *_sock );
            std::string _data("");
            (*_sock) >> _data;
            if(_data.substr(0,6)=="v1_cal"){
                cout<<"[TCP/IP] v1_cal connected.."<<flush; (*_sock) << "chk";
                v1_cal = new thread([&](ServerSocket* sock){
                        sock->RecvIntBuffer(ijk, 3); (*sock) <<"chk";
                        cout<<"World grid : "<<ijk[0]<<" * "<<ijk[1]<<" * "<<ijk[2]<<endl;
                        while(true){
                            cout<<"[v1_cal] waiting data..."<<endl;
                            int numOfData; sock->RecvIntBuffer(&numOfData,1); (*sock) << "chk";
                            cout<<"[v1_cal] recving " <<numOfData<<" data..."<<endl;
                            int numOfPack = floor(numOfData / 180)+1;
                            double buff[180], buff2[180]; int buffInt[180];
                            m.lock();
                            doseMap.clear();
                            for(int i=0, n=0;i<numOfPack;i++){
                                sock->RecvIntBuffer(buffInt, 180); (*sock)<<"chk";
                                sock->RecvDoubleBuffer(buff, 180); (*sock)<<"chk";
                                sock->RecvDoubleBuffer(buff2, 180);(*sock)<<"chk";
                                for(int j=0;j<180 && n<numOfData;j++, n++){
                                    doseMap[buffInt[j]] = make_pair(buff[j], buff2[j]);
                                }
                            }
                            m.unlock();
                       }
              }, _sock);
            }
            else if(_data.substr(0,6)=="v2_cal"){
                 int _id = v2_cals.size();
                 cout<<"[TCP/IP] v2_cal #"<<_id<<" connected!"<<endl; _sock->SendIntBuffer(&_id,1);
                 v2_cals.push_back(new thread([&](ServerSocket* sock, int id){
                    //m2.lock();
                    cout<<"[v2_cal"<<id<<"] Send eye info.."<<flush;
                    (*sock) << "init "+to_string(eye2ply.size()) ;
                    string dump; (*sock) >> dump;
                    sock->SendIntBuffer(&eye2ply[0],180);
                    cout<<"done"<<endl;
                    cout<<"[v2_cal"<<id<<"] Send calib info.."<<flush;
                    sock->SendDoubleBuffer(jt.data(), jt.size());cout<<"done"<<endl;
                    //m2.unlock();

            while(true){
            pthread_mutex_lock(&sync_mutex);
            pthread_cond_wait(&sync_cond, &sync_mutex);
            pthread_mutex_unlock(&sync_mutex);
            cout<<"[v2_cal"<<id<<"]Activate!"<<endl;
            igl::Timer timerTH;
            while(viewer.core(v1_view).is_animating && doseCal){
                m2.lock();
                if(poseData.size()==0){
                        m2.unlock();
                        sleep(1);
                        continue;
                    }
                    timerTH.start();
                    array<double, 155> aPose = poseData.back();
                    int frameID = frameIDs.back();
                    cout<<"[v2_cal"<<id<<"] Calculate frame #"<<frameID
                        <<"! (stacked: "<<poseData.size()-1<<")"<<endl;
                    frameIDs.pop_back();
                    poseData.pop_back(); m2.unlock();
                    sock->SendDoubleBuffer(aPose.data(),155);

                    double buff[180];
                    VectorXd dose(F_ply.rows());
                    for(int i=0, n=0;i<numOfPack;i++){
                        sock->RecvDoubleBuffer(buff,180);
                        for(int j=0;j<180 && n<F_ply.rows();j++, n++)
                            dose[n] = buff[j];
                        if(n<F_ply.rows())(*sock)<<"chk";
                    }
                    m2_dose.lock();
                    strcpy(calNum,to_string(frameNo-frameIDs.size()+1).c_str());
                    doseData+= dose*fabs(aPose[0]);
                    m2_dose.unlock();
                    viewer.data(v2).set_data(W_incidentF*doseData, colorMap);
                    timerTH.stop();
                    cout<<"[v2_cal"<<id<<"] Done.."<<timerTH.getElapsedTimeInSec()<<"s"<<endl;
            }
                //residual
                while(true){
                    m2.lock();
                    if(poseData.size()==0){
                            m2.unlock();
                            break;
                        }
                    array<double, 155> aPose = poseData.back();
                    int frameID = frameIDs.back();
                    cout<<"[v2_cal"<<id<<"] Calculate res. frame #"
                        <<frameID<<"! (stacked: "<<poseData.size()-1<<")"<<endl;
                    timerTH.start();
                    frameIDs.pop_back();
                    poseData.pop_back(); m2.unlock();
                    sock->SendDoubleBuffer(aPose.data(),155);

                    double buff[180];
                    VectorXd dose = VectorXd::Zero(F_ply.rows());
                    for(int i=0, n=0;i<numOfPack;i++){
                        sock->RecvDoubleBuffer(buff,180);
                        for(int j=0;j<180 && n<F_ply.rows();j++, n++)
                            dose[n] = buff[j];
                        if(n<F_ply.rows())(*sock)<<"chk";
                    }
                    m2_dose.lock();
                    strcpy(calNum,to_string(frameNo-frameIDs.size()+1).c_str());
                    doseData+= dose*fabs(aPose[0]);
                    m2_dose.unlock();
                    viewer.data(v2).set_data(((W_incidentF*doseData).array()+1e-20).log10(), colorMap);
                    timerTH.stop();
                    cout<<"[v2_cal"<<id<<"] Done.."<<timerTH.getElapsedTimeInSec()<<"s"<<endl;

                }
            }
                 }, _sock, _id));
                 v2_calculators++;
            }
            else delete _sock;
        }
        v1_cal->join();
    });

    vector<int> poseJoints = {0, 1, 2, 4, 5, 6, 7, 26, 11, 12, 13, 14, 18, 19, 20, 22, 23, 24, 8, 15, 21, 25, 28, 30};
    time_t startT;
    igl::Timer timeStamp;


    //animator
    ofstream ofs("eyeDose.txt");
    //viewer.core().animation_max_fps = 3;
    viewer.callback_pre_draw = [&](igl::opengl::glfw::Viewer &)->bool
    {
        if(viewer.core(v1_view).is_animating){
            //timer.start();
            double stamp = timeStamp.getElapsedTimeInSec();
            MatrixXd V_disp, C_disp(poseJoints.size(),3), C_new(C);
            if(calib) {V_disp = V_calib;}
            else      {V_disp = V_ply;}

            k4a_capture_t sensorCapture = nullptr;
            k4a_wait_result_t getCaptureResult = k4a_device_get_capture(device, &sensorCapture, 0); // timeout_in_ms is set to 0

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
                    viewer.data(v1).set_points(C_disp,blue);

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

                    viewer.data(v1).set_edges(C_new, BE, sea_green);
                    MatrixXd U;
                    myDqs(V_disp,cleanWeights,vQ,vT,U);
                    viewer.data(v1).set_vertices(U);
                    viewer.data(v1).compute_normals();

                    if(doseCal){
                        array<double, 155> pack; //sig, vQ, vT
                        for(int i=0;i<BE.rows();i++){
                            pack[4*i+1] = vQ[i].w(); pack[4*i+2] = vQ[i].x(); pack[4*i+3] = vQ[i].y(); pack[4*i+4] = vQ[i].z();
                            pack[3*i+89] = vT[i](0); pack[3*i+90] = vT[i](1); pack[3*i+91] = vT[i](2);
                        }
                        if(preStamp<0) pack[0] = 0.2;
                        else pack[0] = stamp-preStamp;
                        expTimeD+=pack[0];
                        if(calib) pack[0] = -pack[0];
                        strcpy(frameNum,to_string(frameNo+1).c_str());
                        strcpy(expTime,to_string(expTimeD).c_str());

                        m2.lock();
                        poseData.push_back(pack);
                        frameIDs.push_back(frameNo);
                        m2.unlock();

                        if(doseMap.size()>0){
                            MatrixXd normals = viewer.data(v1).V_normals;
                            MatrixXd vIso = U;
                            vIso.col(2) = vIso.col(2).array()-60;
                            U.col(0) = U.col(0).array() + 150;
                            U.col(1) = U.col(1).array() + 150;
                            U *= 0.2;
                            VectorXd values = ArrayXd::Zero(V_ply.rows());
                            m.lock();
                            for(int i=0;i<U.rows();i++){
                                double factor(1);
                                double dotProd = vIso.row(i).dot(normals.row(i))/vIso.row(i).norm();
                                if(dotProd>0.5) continue;
                                if(dotProd>0) factor = 0.5-dotProd;
                                int idx = floor(U(i,0))*ijk[2]*ijk[1]+floor(U(i,1))*ijk[2]+floor(U(i,2));
                                auto iter = doseMap.find(idx);
                                if(iter==doseMap.end()) continue;
                                values(i) = iter->second.first*factor;
                            }
                            double eyedose(0);
                            for(int i:eye2ply){
                                int idx = floor(U(i,0))*ijk[2]*ijk[1]+floor(U(i,1))*ijk[2]+floor(U(i,2));
                                auto iter = doseMap.find(idx);
                                if(iter==doseMap.end()) continue;
                                eyedose+=doseMap[i].second;
                            }
                            m.unlock();
                            ofs<<eyedose<<endl;
                            viewer.data(v1).set_data(values, colorMap);
                        }
                        //timer.stop();
                        cout<<"Frame #"<<frameNo++<<" ("<<stamp<<"s) : "<<pack[0]<<"s"<<endl;
                    }
                }
                k4abt_frame_release(bodyFrame);
            }
            preStamp = stamp;
        }
        return false;
    };

    startT = time(NULL);
    timeStamp.start();


//    viewer.append_core(Vector4d())
//    viewer.append_mesh();
//    viewer.data(2).set_mesh(V_o, F_o);
//    viewer.data(2).set_visible(true,);

    viewer.launch(true, false, "DCIR System (RDC module)");
    ofs.close();

   // receiverTH.join();

    //quit signal
    int sig = 0;

    return EXIT_SUCCESS;
}
