#ifndef Viewer_class
#define Viewer_class

#include <iostream>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <imgui/imgui.h>

#include <igl/readPLY.h>

using namespace Eigen;
class Viewer{
    public:
    Viewer();
    void SetMeshes(MatrixXd V, MatrixXi F, MatrixXd C, MatrixXi BE);
    void SetCores();
    void MenuDesign();
    private:
    //variables
    public:
    private:
    igl::opengl::glfw::Viewer viewer;
    igl::opengl::glfw::imgui::ImGuiMenu menu;
    MatrixXd V_cumul;
    int v1_view, v2_view;
    int v1, v1_patient, v1_cArm, v2;
    RowVector3d sea_green, white, red;
};

#endif