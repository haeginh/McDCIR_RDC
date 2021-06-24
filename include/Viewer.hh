#ifndef Viewer_class
#define Viewer_class

#include <iostream>
#include <mutex>
#include <condition_variable>

#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <imgui/imgui.h>
#include <igl/readPLY.h>

#include "ServerSocket.hh"
#include "PhantomAnimator.hh"

using namespace std;
using namespace Eigen;
class Viewer{
    public:
    Viewer(PhantomAnimator* _phantom);
    virtual ~Viewer();
    void SetMeshes();
    void SetCores();
    
    void Launch(){viewer.launch(true, false, "DCIR System (RDC module)");}
    private:
    void Communication_init();
    bool Communication_run(igl::opengl::glfw::Viewer &);
    
    //variables
    private:
    igl::opengl::glfw::Viewer viewer;
    igl::opengl::glfw::imgui::ImGuiMenu menu;
    MatrixXd V_cumul, V_glass;
    int v1_view, v2_view;
    int v1, v1_patient, v1_cArm, v1_charuco, v1_glass, v2;
    RowVector3d sea_green, white, red, blue;
    
    //numbers
    static char psdAcc[100], psdRate[100], avgAcc[100], avgRate[100], lensAcc[100], lensRate[100];

    //threads
    //-init.
    thread init_th;
    bool listen;
    bool calib_signal;
    bool waitingCalib;
    bool initializing;
    //-sync

    
    //socket comm.
    fd_set readfds;
    struct timeval sel_timeout;
    ServerSocket* server;
    map<int, ServerSocket *> client_sockets;
    map<int, int> sock_opts;
    int max_sd;

    //menu
    void MenuDesign();

    //phantomAnimator
    PhantomAnimator *phantom;
    vector<unsigned int> reliab_opt;


    //temp
    public:
    // void SetCharucoAff(Affine3d aff) { charucoAff = aff; }
    // void SetCoordAff(Affine3d aff) { coordAff = aff; }
    void TransformVertices(MatrixXd &V, Affine3d aff)
    {V = (V.rowwise().homogeneous()*aff.matrix().transpose()).rowwise().hnormalized();}

    private:
    map<int, Affine3d> sock_coord;
    // Affine3d charucoAff, coordAff;
};
static std::condition_variable cv;
static std::mutex m;


#endif