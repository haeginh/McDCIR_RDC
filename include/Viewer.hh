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
#include "Communicator.hh"
#include "Config.hh"

using namespace std;
using namespace Eigen;
class Communicator;
class Viewer{
    public:
    Viewer(PhantomAnimator* _phantom, string ip, int port);
    virtual ~Viewer();
    void SetMeshes(string cArm, string patient, string glass);
    void SetCores();    
    void Launch(){viewer.launch(true, false, "DCIR System (RDC module)");}

    void SetIsoCenter(Vector3d v){isoCenter = v;}

    private:
    bool Communication_run(igl::opengl::glfw::Viewer &);

    //variables
    private:
    igl::opengl::glfw::Viewer viewer;
    igl::opengl::glfw::imgui::ImGuiMenu menu;
    MatrixXd V_cumul, V_glass, V_cArm;
    int v1_view, v2_view;
    int v1, v1_patient, v1_cArm, v1_charuco, v1_glass, v2;
    RowVector3d sea_green, white, red, blue;
    
    //numbers
    static char psdAcc[100], psdRate[100], avgAcc[100], avgRate[100], lensAcc[100], lensRate[100];

    //-sync
    
    //socket comm.
    // fd_set readfds;
    // struct timeval sel_timeout;
    // ServerSocket* server;
    // map<int, ServerSocket *> client_sockets;
    // map<int, int> sock_opts;
    // int max_sd;
    Communicator *comm;

    //menu
    void MenuDesign();

    //phantomAnimator
    PhantomAnimator *phantom;
    vector<unsigned int> reliab_opt;

    //cArm
    Vector3d isoCenter;
};
static std::condition_variable cv;
static std::mutex m;


#endif