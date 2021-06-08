#ifndef Viewer_class
#define Viewer_class

#include <iostream>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <imgui/imgui.h>
#include <igl/readPLY.h>

#include "ServerSocket.hh"

using namespace std;
using namespace Eigen;
class Viewer{
    public:
    Viewer();
    void SetMeshes(const MatrixXd &V, const MatrixXi &F, const MatrixXd &C, const MatrixXi &BE);
    void SetCores();
    
    void Launch(){viewer.launch(true, false, "DCIR System (RDC module)");}
    private:
    void Communication();
    //variables
    public:
    private:
    igl::opengl::glfw::Viewer viewer;
    igl::opengl::glfw::imgui::ImGuiMenu menu;
    MatrixXd V_cumul;
    int v1_view, v2_view;
    int v1, v1_patient, v1_cArm, v2;
    RowVector3d sea_green, white, red;
    
    //numbers
    static char psdAcc[100], psdRate[100], avgAcc[100], avgRate[100], lensAcc[100], lensRate[100];

    //threads
    thread communicator_th;

    //menu
    void MenuDesign();
};


#endif