#ifndef RDCWindow_class
#define RDCWindow_class

#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <imgui/imgui.h>
#include "PhantomAnimator.hh"

using namespace std;
using namespace Eigen;
class RDCWindow
{
public:
    //singleton
    static RDCWindow &Instance();
    //setting window frame
    void Initialize();
    void Launch(){
        viewer.launch(true, false, "DCIR System (RDC module)");
        viewer.selected_data_index = phantom_data;
    }
    bool PreDrawFunc(igl::opengl::glfw::Viewer &_viewer);
    void AnimateAll(bool anim){
        viewer.core(0).is_animating = anim;
        viewer.core(v_left).is_animating = anim;
        viewer.core(v_middle).is_animating = anim;
        viewer.core(v_right).is_animating = anim;
    }
    void SetBodyID(int id){bodyID = id;}

    bool running;
    bool postureUpdated;
    igl::opengl::glfw::Viewer viewer;

    //data id
    unsigned int phantom_data, apron_data, extra_data, patient_data, table_data,
        cArm_data, beam_data, glass_data, phantomAcc_data, grid_data;


private:
    RDCWindow():running(false), bodyID(1){}
    void SetMeshes(string dir);
    igl::opengl::glfw::imgui::ImGuiMenu mainMenu;
    string phantomName;

    //view id
    unsigned int v_left, v_middle, v_right;

    int bodyID;
};

#endif