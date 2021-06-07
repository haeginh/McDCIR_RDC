#include "Viewer.hh"
#include "colorbar.hh"

Viewer::Viewer()
:sea_green(RowVector3d(70./255.,252./255.,167./255.)), white(RowVector3d(1.,1.,1.)), red(RowVector3d(1.,0.,0.))
{
    menu.callback_draw_viewer_window = [&]()
    {
        float menu_width = 180.f * menu.menu_scaling();
        ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSizeConstraints(ImVec2(menu_width, -1.0f), ImVec2(menu_width, -1.0f));
        bool _viewer_menu_visible = true;
        ImGui::Begin("RDC Module", &_viewer_menu_visible, ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_AlwaysAutoResize);
        ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.5f);
        menu.callback_draw_viewer_menu();
        ImGui::PopItemWidth();
        ImGui::End();
    };
    viewer.plugins.push_back(&menu);
}

void Viewer::SetMeshes(MatrixXd V, MatrixXi F, MatrixXd C, MatrixXi BE){
    viewer.data().set_mesh(V, F);
    viewer.append_mesh();
    viewer.load_mesh_from_file("patient3.obj");
    viewer.append_mesh();
    MatrixXd V_cArm; MatrixXi F_cArm;
    igl::readPLY("c-arm.ply",V_cArm,F_cArm);
    viewer.data().set_mesh(V_cArm,F_cArm);
    viewer.append_mesh();
    MatrixXd V_cumul(V);
    viewer.data().set_mesh(V, F);

    v1=viewer.data_list[0].id;
    v1_patient=viewer.data_list[1].id;
    v1_cArm=viewer.data_list[2].id;
    v2=viewer.data_list[3].id;

    viewer.data(v1).set_edges(C,BE,sea_green);
    viewer.data(v1).show_lines = false;
    viewer.data(v1).show_overlay_depth = false;
    viewer.data(v1).line_width = 1;
    viewer.data(v1).point_size = 8;
    viewer.data(v2).point_size = 4;
    viewer.data(v1).double_sided = false;
    viewer.data(v1_patient).show_lines = true;
    viewer.data(v1_patient).show_overlay_depth = false;
    viewer.data(v1_patient).show_faces = false;
    viewer.data(v1_cArm).set_colors(white);
    viewer.data(v1_cArm).show_lines = false;
    viewer.data(v2).show_lines = false;
    viewer.data(v2).show_overlay_depth = true;
    viewer.data(v2).double_sided = false;
}

void Viewer::SetCores(){
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
        viewer.data(v1_cArm).set_visible(false, v2_view);
        viewer.data(v2).set_visible(false, v1_view);

        return false;
    };

    viewer.callback_post_resize = [&](igl::opengl::glfw::Viewer &v, int w, int h) {
        float boarder =  w * 3 / 4;
        v.core( v1_view).viewport = Eigen::Vector4f(0, 0, w * 3 / 4, h);
        v.core(v2_view).viewport = Eigen::Vector4f(w *3 / 4, 0, w/4, h);
        return true;
    };
}