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
        ImGui::Begin("Settings", &_viewer_menu_visible, ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_AlwaysAutoResize);
        ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.5f);
        menu.callback_draw_viewer_menu();
        ImGui::PopItemWidth();
        ImGui::End();
    };
    viewer.plugins.push_back(&menu);
    MenuDesign();
}

void Viewer::MenuDesign(){
     menu.callback_draw_viewer_menu = [&](){
        if (ImGui::CollapsingHeader("Information", ImGuiTreeNodeFlags_None))
        {
            ImGui::Text("Author : Haegin Han");
            ImGui::Text("Info : RDC modult of DCIR system");
        }
//         if (ImGui::CollapsingHeader("Iso center position", ImGuiTreeNodeFlags_DefaultOpen)){
//             static float isoCenterInput[3] = {isoCenter(0),isoCenter(1),isoCenter(2)};
//             if(ImGui::InputFloat3("isocenter pos.", isoCenterInput, "%.1f", ImGuiInputTextFlags_EnterReturnsTrue)){
//                 isoCenter = Vector3d(isoCenterInput[0],isoCenterInput[1],isoCenterInput[2]);
//                 gridStart = isoCenter + isoRelat;
//                 viewer.data(v1_patient).set_vertices(V_patient.rowwise()+(isoCenter-Vector3d(0,0,60)).transpose());
//                 viewer.data(v1_cArm).set_vertices(V_cArm.rowwise()+(isoCenter-Vector3d(0,0,60)).transpose());
//             }
//         }
//         if (ImGui::CollapsingHeader("Beam Conditions", ImGuiTreeNodeFlags_DefaultOpen))
//         {
//             static string kVp("80");
//             if(ImGui::InputText("kVp", kVp, ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_CharsDecimal)) cout<<"new condition: "<<kVp<<" kVp"<<endl;
//             if(ImGui::InputFloat("Gycm2/s (DAP)", &dap, ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_CharsDecimal)){
//                 cout<<"new condition: "<<dap<<" Gycm2/s"<<endl;
//                 doseFactor = dap/DAPperNPS;
//                 sprintf(rMaxChar,"%3.2f",maxSkin0*0.1*3600.e3*doseFactor);
//             }
//             static int cArmRot[2] = {0,0};
//             static int lArmRot(0);
//             if(ImGui::InputInt2("ang./rot. [deg.]",cArmRot, ImGuiInputTextFlags_EnterReturnsTrue)){
//                 //L-arm(Y), rotation(X), angulation(Z)
//                 CalculateRot(lArmRot,  cArmRot[1], cArmRot[0]);
//                 viewer.data(v1_cArm).set_vertices(((rotMat*(V_cArm.rowwise()-Vector3d(0,0,60).transpose()).transpose()).colwise()+isoCenter).transpose());
//                 if(cArmRot[0]==45){
//                     ifstream ifsMap("./doseMaps/1_conf.map", ios::binary);
//                     ifsMap.read((char*) &doseMapS0[0], ijk[0]*ijk[1]*ijk[2]*sizeof(double));
//                     ifsMap.read((char*) &doseMapL0[0], ijk[0]*ijk[1]*ijk[2]*sizeof(double));
//                     ifsMap.close();
//                     maxSkin0 = *max_element(doseMapS0.begin(),doseMapS0.end());
//                     CalculateRot(0, 0, 0);
//                 }else{
//                     ifstream ifsMap("./doseMaps/0_conf.map", ios::binary);
//                     ifsMap.read((char*) &doseMapS0[0], ijk[0]*ijk[1]*ijk[2]*sizeof(double));
//                     ifsMap.read((char*) &doseMapL0[0], ijk[0]*ijk[1]*ijk[2]*sizeof(double));
//                     ifsMap.close();
//                     maxSkin0 = *max_element(doseMapS0.begin(),doseMapS0.end());
//                 }

//             }
//             enum LArmRot {right, up, left}; //patient viewpoint
//             static LArmRot lArmRotC = up;
//             if(ImGui::Combo("L-arm Rot.", (int*)(&lArmRotC), "-90 deg\0  0 deg\0 90 deg\0\0")){
//                 if(lArmRotC==right)     lArmRot = -90;
//                 else if(lArmRotC==up)   lArmRot = 0;
//                 else if(lArmRotC==left) lArmRot = 90;
//                 CalculateRot(lArmRot,  cArmRot[1], cArmRot[0]);
//                 viewer.data(v1_cArm).set_vertices(((rotMat*(V_cArm.rowwise()-Vector3d(0,0,60).transpose()).transpose()).colwise()+isoCenter).transpose());
//             }
//             float detNum[2] = {100, 22};
//             ImGui::InputFloat2("SID/FD [cm]", detNum,"%.1f");
//         }
//         if (ImGui::CollapsingHeader("C-arm Table"), ImGuiTreeNodeFlags_DefaultOpen)
//         {
//             static float tablePos[3] = {0,0,0};
//             ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.75f);
//             if(ImGui::InputFloat3("[cm]", tablePos, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue)){
// //                patientMove = Vector3d(patientTrans[0],patientTrans[1],patientTrans[2]);
// //                viewer.data(v1_patient).set_vertices(V_patient.rowwise()+patientMove.transpose());
//             }

//             ImGui::PopItemWidth();
//             float bedAxis[2] = {0,0};
//             ImGui::InputFloat2("long./lat. [cm]", bedAxis);
//             static int bedRot = 0;
//             ImGui::InputInt("Rotation [deg]", &bedRot);
//         }
//         if (ImGui::CollapsingHeader("Viewer option"), ImGuiTreeNodeFlags_DefaultOpen)
//         {
//             static bool cArmView(true);
//             if(ImGui::Checkbox("show C-arm",&cArmView)){
//                 if(cArmView)  viewer.data(v1_cArm).set_visible(true, v1_view);
//                 else          viewer.data(v1_cArm).set_visible(false, v1_view);
//             }
//             static bool patientView(true);
//             if(ImGui::Checkbox("show patient",&patientView)){
//                 if(patientView)  viewer.data(v1_patient).set_visible(true, v1_view);
//                 else          viewer.data(v1_patient).set_visible(false, v1_view);
//             }
//         }
//         if (ImGui::CollapsingHeader("Record (temp.)"), ImGuiTreeNodeFlags_DefaultOpen)
//         {
//             float w = ImGui::GetContentRegionAvailWidth();
//             float p = ImGui::GetStyle().FramePadding.x;
//             if(ImGui::Button("record!", ImVec2((w-p)/2.,0))){
//                 if(loading) cout<<"wrong status!"<<endl;
//                 else{
//                     recording = true;
//                     recordedData.clear();
//                     strcpy(rec,"recording..");
//                 }
//             }

//             ImGui::SameLine(0, p);
//             if(ImGui::Button("stop!", ImVec2((w-p)/2.,0))){
//                 recording = false;
//                 string fName;
//                 cout<<"fileName: "<<flush; cin>>fName;
//                 ofstream ofs("./records/"+fName+".dat");
//                 for(size_t n=0;n<recordedData.size();n++){
//                     for(int i=0;i<155;i++) ofs<<recordedData[n][i]<<"\t";
//                     ofs<<endl;
//                     for(int i=0;i<24;i++) ofs<<jointData[n](i,0)<<"\t"<<jointData[n](i,1)<<"\t"<<jointData[n](i,2)<<"\t";
//                     ofs<<endl;
//                 }ofs.close();
//                 igl::writeDMAT("./records/"+fName + ".jt",jointTrans);
//                 cout<<"exported ./records/"+fName+".dat,jt ("<<recordedData.size()<<" frames)"<<endl;
//                 recordedData.clear(); jointData.clear();
//                 strcpy(rec,"idle");
//             }
//             if(ImGui::Button("load", ImVec2(w,0))){
//                 if(recording) cout<<"wrong status!"<<endl;
//                 else{
//                     ResetDose();
//                     viewer.core(v1_view).is_animating = true;
//                     strcpy(rec,"loading..");
//                     recordedData.clear();
//                     loading = true;
//                 }
//             }
//         }
//         ImGui::Separator();
//         ImGui::PushItemWidth(ImGui::GetContentRegionAvailWidth());
//         ImGui::InputText("",rec,ImGuiInputTextFlags_ReadOnly);
//         ImGui::PopItemWidth();
//         if(ImGui::Button("ANIMATE/STOP", ImVec2(ImGui::GetContentRegionAvailWidth(),0))){
//             viewer.core(v1_view).is_animating = !viewer.core(v1_view).is_animating;
//             preStamp = -1;
//             if(viewer.core(v1_view).is_animating && !loading) strcpy(rec, "real-time animating");
//             else strcpy(rec, "idle");
//         }
//         if(ImGui::Checkbox("Calculate dose", &doseCal))
//             preStamp = -1;
//         //if(calibFrame)
//             if(ImGui::Checkbox("Use calibrated phantom", &calib)){
//                 if(calib) viewer.data(v2).set_vertices(V_calib_cumul);
//                 else      viewer.data(v2).set_vertices(V_cumul);
//             }
//         if(ImGui::Button("RESET DOSE", ImVec2(ImGui::GetContentRegionAvailWidth(),0)))  ResetDose();
    };
}

void Viewer::SetMeshes(const MatrixXd &V, const MatrixXi &F, const MatrixXd &C, const MatrixXi &BE){
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