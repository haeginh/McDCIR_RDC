#ifndef RDCWindow_class
#define RDCWindow_class

#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
// #include <igl/opengl/glfw/imgui/ImGuizmoPlugin.h>
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
        viewer.selected_data_index = phantom_data[bodyID];
    }
    bool PreDrawFunc(igl::opengl::glfw::Viewer &_viewer);
    void AnimateAll(bool anim){
        viewer.core(0).is_animating = anim;
        viewer.core(v_left).is_animating = anim;
        viewer.core(v_middle).is_animating = anim;
        viewer.core(v_right).is_animating = anim;
    }
    void SetBodyID(int id){bodyID = id;}

    Matrix3d GetCarmRotation(double rot, double ang)
    {
        return
            (AngleAxisd(rot/180.*igl::PI, Vector3d(0, 1, 0))*
             AngleAxisd(ang/180.*igl::PI, Vector3d(1, 0, 0))).matrix();
    }

    PhantomAnimator* GetPhantom(int i) {
        if(i<indivPhantoms.size()) return indivPhantoms[i];
        else return nullptr;
    }
    bool running;
    bool postureUpdated;
    igl::opengl::glfw::Viewer viewer;

    //data id
    unsigned int apron_data, patient_data, table_data,
        cArm_data, beam_data, glass_data, phantomAcc_data, grid_data, axis_data;
    vector<unsigned int> phantom_data;
    int mainID(){return phantom_data[bodyID];}

    MatrixXd V_cArm, V_beam, V_glass, V_patient, V_table;

    bool show_C, show_BE;
    bool show_leadGlass;
    
private:
    RDCWindow():running(false), bodyID(0) {
        for(int i=0;i<5;i++)
        {
            auto phantom = new PhantomAnimator;
            indivPhantoms.push_back(phantom);
        }
    }
    void SetMeshes(string dir);
    igl::opengl::glfw::imgui::ImGuiMenu mainMenu;
    // string phantomName;

    //view id
    unsigned int v_left, v_middle, v_right;

    int bodyID;
    vector<PhantomAnimator*> indivPhantoms;
};

#endif