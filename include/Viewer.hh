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
#include "MapContainer.hh"
#include "Communicator.hh"
#include "Config.hh"

using namespace std;
using namespace Eigen;
class Communicator;
class Viewer{
    public:
    Viewer(PhantomAnimator* _phantom, string ip, int port);
    virtual ~Viewer();
    void SetFrame();
    void SetMeshes(string cArm, string patient, string glass);
    void SetCores();    
    void Launch(){viewer.launch(true, false, "DCIR System (RDC module)");}

    void SetIsoCenter(Vector3d v){isoCenter = v;}

    void SetRecordFile(string name)
    {
        ifs.open(name);
        playback = true;
    }

    private:
    bool Communication_run(igl::opengl::glfw::Viewer &);
    bool CalculateDoses();
    bool CalculateDoses(VectorXi idx);

    //variables
    private:
    igl::opengl::glfw::Viewer viewer;
    igl::opengl::glfw::imgui::ImGuiMenu menu;
    MatrixXd V_glass, V_cArm, V_beam, V_patient, V_table;
    int v1_view, v2_view;
    int v1, v1_apron, v1_patient, v1_table, v1_cArm, v1_beam, v1_charuco, v1_glass, v2;
    RowVector3d sea_green, white, red, blue;
    
    //numbers
    
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
    void CustomMenuDesign();

    //phantomAnimator
    PhantomAnimator *phantom;
    vector<unsigned int> reliab_opt;

    //cArm
    Vector3d isoCenter;

    //doses
    MapContainer* maps;
    ArrayXd skinRate, skinAcc;
    VectorXd doseMapS;
    Matrix3d cArm_IT;
    ArrayXd skinW;

    // map<int, double> lensW;
    igl::ColorMapType colorMap;

    MatrixXd ComputeNormal(const MatrixXd &VV, const MatrixXi &FF)
    {
        MatrixXd F_normals, V_normals;
        igl::per_face_normals(VV, FF, F_normals);
        igl::per_vertex_normals(VV, FF, F_normals, V_normals);
        return V_normals;
    }

    MatrixXf B_patient, N_patient, A_patient;
    MatrixXf B_patient1, N_patient1, A_patient1;
    void CalculateSourceFacets(RowVector3f s, RowVector3f dir, RowVector3f bed, double dist2){
        VectorXi idx = ((B_patient.rowwise() + (bed-s)).rowwise().cross(dir).rowwise().squaredNorm().array()<dist2).cast<int>();
        VectorXi sourceIdx(idx.sum());
        for(int i=0, n=0;i<idx.rows();i++) 
            if(idx(i)>0) sourceIdx(n++) = i;
        cout<<"# of source facets: "<<sourceIdx.rows()<<endl;
        B_patient1 = igl::slice(B_patient, sourceIdx, 1).rowwise() + bed;
        N_patient1 = igl::slice(N_patient, sourceIdx, 1);
        A_patient1 = igl::slice(A_patient, sourceIdx, 1);
    }

    //v2
    double v2_viewAng;

    //timer
    igl::Timer timer;
    bool beamOn;

    //playback
    bool playback;
    ifstream ifs;
    
};
static std::condition_variable cv;
static std::mutex m;


#endif