#ifndef RDCWindow_class
#define RDCWindow_class

#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <imgui/imgui.h>

using namespace std;
using namespace Eigen;
class RDCWindow
{
public:
    //singleton
    static RDCWindow &Instance();
    //setting window frame
    void Initialize();
    void Launch(){viewer.launch(true, false, "DCIR System (RDC module)");}
    bool Animate(igl::opengl::glfw::Viewer &_viewer);

    bool running;
    bool postureUpdated;
    igl::opengl::glfw::Viewer viewer;

private:
    RDCWindow():running(false){}
    void SetMeshes(string dir);
    igl::opengl::glfw::imgui::ImGuiMenu mainMenu;
    string phantomName;

    //data id
    int phantom_data, apron_data, patient_data, table_data,
        cArm_data, beam_data, glass_data, phantomAcc_data;
};

#endif