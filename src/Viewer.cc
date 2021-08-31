#include "Viewer.hh"
#include "colorbar.hh"
#include <bitset>
#include <igl/writePLY.h>

static vector<string> camSock;
Viewer::Viewer(PhantomAnimator *_phantom, string ip, int port)
    : sea_green(70. / 255., 252. / 255., 167. / 255.), white(1., 1., 1.), red(1., 0., 0.), blue(0., 0., 1.),
      phantom(_phantom), v2_viewAng(0), beamOn(false), playback(false)
{
    comm = new Communicator(ip, port);
    comm->SetInitPack(phantom->GetAlignRot(), phantom->GetBE());
    comm->Initialization();
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
    menu.callback_draw_viewer_menu = std::bind(&Viewer::MenuDesign, this);
    menu.callback_draw_custom_window = std::bind(&Viewer::CustomMenuDesign, this);
    viewer.callback_pre_draw = std::bind(&Viewer::Communication_run, this, std::placeholders::_1);
    vector<int> sockets = comm->GetCamSock();
    for (int s : sockets)
        camSock.push_back(to_string(s));

    //dose initialization
    skinRate = VectorXd::Zero(phantom->GetV().rows());
    skinAcc = VectorXd::Zero(phantom->GetV().rows());

    //dosemap initialization
    maps = new MapContainer();
    colorMap = igl::COLOR_MAP_TYPE_PARULA;
    cArm_IT = Matrix3d::Identity();
}

Viewer::~Viewer()
{
    // for (auto client : client_sockets)
    //     delete client.second;
    // delete server;
}

static char runStatus[20] = "pre-init.";
static int calib_sock(0);
static int profileID(0);
static char newProfile[20] = "H_Han";

//ocr
//static vector<string> ocr_values;
// static string ocr_values;
//static char ocr_values[20][20];
//static string ocr_values[20];
static int cArm[2];
static int bed[3];
static int det[2];
static int beam_exp[2];
static int beam_fluo[3];
static float v2_rotAng(10.);

void Viewer::MenuDesign()
{
    float w = ImGui::GetContentRegionAvailWidth();
    float p = ImGui::GetStyle().FramePadding.x;
    if (ImGui::CollapsingHeader("Information", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::Text("Author : Haegin Han");
        ImGui::Text("Info : RDC module of DCIR system");
    }
    static vector<string> profileNames;
    if (ImGui::CollapsingHeader("Initialization", ImGuiTreeNodeFlags_DefaultOpen))
    {
        vector<string> names = phantom->GetProfileNames();
        profileNames.resize(names.size());
        copy(names.begin(), names.end(), profileNames.begin());
        ImGui::Combo("profile", &profileID, profileNames);
        ImGui::InputText("new profile", newProfile, ImGuiInputTextFlags_CharsNoBlank);
        ImGui::Combo("socket num.", &calib_sock, camSock);
        if (ImGui::Button("Start measurement!", ImVec2(w, 0)))
        {
            if (phantom->AlreadyExists(newProfile))
                cout << newProfile << " aready exists!" << endl;
            else
            {
                comm->BodyCalibration(comm->GetCamSock()[calib_sock], newProfile, phantom);
                profileID = profileNames.size();
            }
        }
    }

    ImGui::Separator();
    ImGui::InputText("status", runStatus, ImGuiInputTextFlags_None);
    if (ImGui::Button("START", ImVec2(w, 0)))
    {
        viewer.data(v1).set_visible(true, v1_view);
        if (profileNames[profileID] != "original")
            phantom->CalibrateTo(profileNames[profileID]);
        if (!comm->AlreadyStarted())
            comm->StartMainLoop();
        sleep(1);
        viewer.core(v1_view).is_animating = true;
        // viewer.core(v2_view).is_animating = true;
        viewer.core().animation_max_fps = 10;
        // viewer.core(v2_view).animation_max_fps = 10;
        timer.start();
        strcpy(runStatus, "running");
    }
    if (ImGui::Button("PAUSE", ImVec2(w, 0)))
    {
        timer.stop(); 
        viewer.core(v1_view).is_animating = false;
        strcpy(runStatus, "idle");
    }

    if (ImGui::Button("RECORD", ImVec2(w, 0)))
    {
        comm->Record();
        if(comm->IsRecording()) strcpy(runStatus, "recording");
        else strcpy(runStatus, "running");
    }

    if (ImGui::CollapsingHeader("OCR results", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::InputInt2("C-arm", cArm, ImGuiInputTextFlags_ReadOnly);
        ImGui::InputInt3("bed [x/y/z, cm]", bed, ImGuiInputTextFlags_ReadOnly);
        ImGui::InputInt2("SID/FD [cm]", det, ImGuiInputTextFlags_ReadOnly);
        ImGui::InputInt2("exp. [kV/mAs]", beam_exp, ImGuiInputTextFlags_ReadOnly);
        ImGui::InputInt3("flou. [kV/mA/DAP]", beam_fluo, ImGuiInputTextFlags_ReadOnly);
    }

    ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.5f);
    if (ImGui::CollapsingHeader("Display Settings", ImGuiTreeNodeFlags_DefaultOpen))
    {
        Quaterniond identity = Quaterniond::Identity();
        RotationList vQ_hu(22, identity);
        static int angleA(10);
        static int angleH(90);
        static int angleL(5);
        function<void()> Widen = [&]()
        {
            vQ_hu[11] = AngleAxisd(angleA / 180. * igl::PI, Vector3d(0, 0, 1));
            vQ_hu[4] = AngleAxisd(-angleA / 180. * igl::PI, Vector3d(0, 0, 1));
            vQ_hu[12] = AngleAxisd(angleH * 0.5 / 180. * igl::PI, Vector3d(0, 1, 0));
            vQ_hu[5] = AngleAxisd(-angleH * 0.5 / 180. * igl::PI, Vector3d(0, 1, 0));
            vQ_hu[13] = AngleAxisd(angleH * 0.5 / 180. * igl::PI, Vector3d(0, 1, 0));
            vQ_hu[6] = AngleAxisd(-angleH * 0.5 / 180. * igl::PI, Vector3d(0, 1, 0));
            vQ_hu[19] = AngleAxisd(angleL / 180. * igl::PI, Vector3d(0, 0, 1));
            vQ_hu[15] = AngleAxisd(-angleL / 180. * igl::PI, Vector3d(0, 0, 1));
            phantom->Animate(vQ_hu, viewer.data(v2).V);
            viewer.data(v2).set_vertices(viewer.data(v2).V);
            viewer.data(v2).compute_normals();
        };
        Widen();
        if (ImGui::InputInt("hand rot.", &angleH, 10, 20))
            Widen();
        if (ImGui::InputInt("arm rot.", &angleA, 5, 10))
            Widen();
        if (ImGui::InputInt("leg rot.", &angleL, 5, 10))
            Widen();
        if (ImGui::InputFloat("rot. ang./frame", &v2_rotAng, 10., 10.,1));
        if (ImGui::Button("stop/start rotation", ImVec2(ImGui::GetContentRegionAvailWidth(), 0)))
        {
            viewer.core(v2_view).is_animating = !viewer.core(v2_view).is_animating;
            viewer.core(v2_view).camera_eye = Eigen::Vector3f(0, 0, -3);
            v2_viewAng = 0;
        }
    }
    ImGui::PopItemWidth();
}
static float expTime(0.);
static int frameNum(0);
static float maxVal[2] = {0., 1.};
static float psd[2] = {0., 0.};
static float agvSkin[2] = {0., 0.};
static float lens[2] = {0., 0.};
void Viewer::CustomMenuDesign()
{
    float boarder(300.f);
    ImGui::SetNextWindowPos(ImVec2(boarder - 180.f * menu.menu_scaling(), 0.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSizeConstraints(ImVec2(180.f * menu.menu_scaling(), -1.0f), ImVec2(180.f * menu.menu_scaling(), -1.0f));
    ImGui::Begin("Current Satus", nullptr, ImGuiWindowFlags_NoSavedSettings);
    if (ImGui::CollapsingHeader("Status Summary", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.25f);
        ImGui::InputScalarN("Exposure time [s]", ImGuiDataType_Float, &expTime, 1, NULL, NULL, "%0.1f");
        ImGui::InputScalarN("Total Frame No.", ImGuiDataType_S32, &frameNum, 1, NULL, NULL, "%d");
        ImGui::PopItemWidth();
        ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.5f);
        // static int maxFPS(10);
        // if (ImGui::InputInt("max FPS", &maxFPS))
        //     viewer.core(v1_view).animation_max_fps = maxFPS;
        ImGui::PopItemWidth();
    }
    ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.25f);
    float p = ImGui::GetStyle().FramePadding.x;
    ColorbarPlugin cbar(colorMap);
    // sprintf(rMaxChar,"%3.2f",maps->GetMaxSkin()*0.1*3600.e3*beam_fluo[2]*maps->GetInvDAPperNPS());
    if (ImGui::CollapsingHeader("Dose Information", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::BulletText("PSD (Peak skin dose)");
        ImGui::InputScalarN("mGy/h", ImGuiDataType_Float, psd, 1, NULL, NULL, "%0.1f");
        ImGui::SameLine(0, p);
        ImGui::InputScalarN("mGy  ", ImGuiDataType_Float, psd + 1, 1, NULL, NULL, "%0.1f");
        ImGui::BulletText("Average skin dose");
        ImGui::InputScalarN("mGy/h", ImGuiDataType_Float, agvSkin, 1, NULL, NULL, "%0.1f");
        ImGui::SameLine(0, p);
        ImGui::InputScalarN("mGy  ", ImGuiDataType_Float, agvSkin + 1, 1, NULL, NULL, "%0.1f");
        ImGui::BulletText("Lens dose");
        ImGui::InputScalarN("mGy/h", ImGuiDataType_Float, lens, 1, NULL, NULL, "%0.1f");
        ImGui::SameLine(0, p);
        ImGui::InputScalarN("mGy  ", ImGuiDataType_Float, lens + 1, 1, NULL, NULL, "%0.1f");
        ImGui::BulletText("Skin dose color bar");
        cbar.draw_colorbar(maxVal);
    }
    ImGui::PopItemWidth();
    ImGui::End();
}

void Viewer::SetMeshes(string cArm, string patient, string glass)
{
    MatrixXd V = phantom->GetV();
    MatrixXi F = phantom->GetF();
    MatrixXd C = phantom->GetC();
    MatrixXi BE = phantom->GetBE();
    viewer.data().set_mesh(V, F);
    MatrixXi F_cArm, F_glass, F_beam, F_patient, F_table;
    igl::readPLY(cArm, V_cArm, F_cArm);
    V_cArm = V_cArm.rowwise() + isoCenter.transpose();
    igl::readPLY(patient, V_patient, F_patient);
    V_patient = V_patient.rowwise() + isoCenter.transpose();
    igl::readPLY("table.ply", V_table, F_table);
    V_table = V_table.rowwise() + isoCenter.transpose();
    igl::readPLY("beam.ply", V_beam, F_beam);
    V_beam = V_beam.rowwise() + isoCenter.transpose();
    igl::readPLY(glass, V_glass, F_glass);
    viewer.append_mesh();
    viewer.data().set_mesh(V_patient, F_patient);
    viewer.append_mesh();
    viewer.data().set_mesh(V_table, F_table);
    viewer.append_mesh();
    viewer.data().set_mesh(V_cArm, F_cArm);
    viewer.append_mesh();
    viewer.data().set_mesh(V_beam, F_beam);
    int n(5);
    double gap(50);
    MatrixXd V_grid(8 * n + 4, 3);
    MatrixXi E_grid(4 * n + 2, 2);
    for (int i = 0; i < 2 * n + 1; i++)
    {
        V_grid.row(i) = RowVector3d((i - n) * gap, n * gap, 0);
        V_grid.row(V_grid.rows() * 0.25 + i) = RowVector3d((i - n) * gap, -n * gap, 0);
        V_grid.row(V_grid.rows() * 0.5 + i) = RowVector3d(n * gap, (i - n) * gap, 0);
        V_grid.row(V_grid.rows() * 0.75 + i) = RowVector3d(-n * gap, (i - n) * gap, 0);
        E_grid.row(i) = RowVector2i(i, V_grid.rows() * 0.25 + i);
        E_grid.row(E_grid.rows() * 0.5 + i) = RowVector2i(V_grid.rows() * 0.5 + i, V_grid.rows() * 0.75 + i);
    }
    MatrixXd C_grid = RowVector3d(0.2, 0.2, 0.2).replicate(E_grid.rows(), 1);
    C_grid.row(n) = RowVector3d(1, 0, 0);
    C_grid.row(E_grid.rows() * 0.5 + n) = RowVector3d(1, 0, 0);
    viewer.append_mesh();
    viewer.data().set_edges(V_grid, E_grid, C_grid);
    viewer.append_mesh();
    viewer.data().set_mesh(V_glass, F_glass);
    MatrixXd blue(1, 4);
    blue << 0, 0, 1, 0.2;
    viewer.data().set_colors(blue);
    viewer.data().show_lines = false;
    viewer.append_mesh();
    MatrixXd V_cumul(V);
    viewer.data().set_mesh(V, F);
    v1 = viewer.data_list[0].id;
    v1_patient = viewer.data_list[1].id;
    v1_table = viewer.data_list[2].id;
    v1_cArm = viewer.data_list[3].id;
    v1_beam = viewer.data_list[4].id;
    v1_charuco = viewer.data_list[5].id;
    v1_glass = viewer.data_list[6].id;
    v2 = viewer.data_list[7].id;

    viewer.data(v1).set_edges(C, BE, sea_green);
    viewer.data(v1).show_lines = false;
    viewer.data(v1).show_overlay_depth = false;
    viewer.data(v1).line_width = 1;
    viewer.data(v1).point_size = 8;
    viewer.data(v2).point_size = 4;
    viewer.data(v1).double_sided = false;
    viewer.data(v1_patient).show_lines = true;
    viewer.data(v1_patient).show_overlay_depth = false;
    viewer.data(v1_patient).show_faces = false;
    viewer.data(v1_patient).is_visible = false;
    viewer.data(v1_table).set_colors(RowVector4d(0,0.2,0,0.5));
    viewer.data(v1_table).show_lines = false;
    viewer.data(v1_cArm).set_colors(white);
    viewer.data(v1_cArm).show_lines = false;
    viewer.data(v1_cArm).is_visible = false;
    MatrixXd red(1, 4);
    red << 1, 0, 0, 0.2;
    viewer.data(v1_beam).set_colors(red);
    viewer.data(v1_beam).show_lines = false;
    viewer.data(v2).show_lines = false;
    viewer.data(v2).double_sided = false;

    //initialize reliabilitiy options
    for (int i = 0; i < C.rows(); i++)
    {
        unsigned int opt = 1 << i;
        reliab_opt.push_back(opt);
    }
}

void Viewer::SetCores()
{
    viewer.callback_init = [&](igl::opengl::glfw::Viewer &)
    {
        viewer.core().viewport = Eigen::Vector4f(0, 0, 640, 800);
        v1_view = viewer.core_list[0].id;
        v2_view = viewer.append_core(Eigen::Vector4f(640, 0, 640, 800));
        viewer.core(v1_view).background_color = Eigen::Vector4f(109. / 255., 100. / 255., 102. / 255., 1);
        viewer.core(v2_view).background_color = Eigen::Vector4f(163. / 255., 163. / 255., 163. / 255., 1);

        viewer.core(v1_view).camera_center = Vector3f(isoCenter.x(), isoCenter.y(), isoCenter.z())*0.1;//*0.06;Vector3f(0, 0, 15);
        viewer.core(v1_view).camera_eye = viewer.core(v1_view).camera_center + Vector3f(50, 0, 10);
        viewer.core(v1_view).camera_up = Eigen::Vector3f(0, 0, 1);
        viewer.core(v1_view).set_rotation_type(igl::opengl::ViewerCore::ROTATION_TYPE_TRACKBALL);

        viewer.core(v2_view).camera_eye = Eigen::Vector3f(0, 0, -3);
        viewer.core(v2_view).camera_up = Eigen::Vector3f(0, -1, 0);

        viewer.data(v1).set_visible(false, v2_view);
        viewer.data(v1_patient).set_visible(false, v2_view);
        viewer.data(v1_table).set_visible(false, v2_view);
        viewer.data(v1_cArm).set_visible(false, v2_view);
        viewer.data(v1_glass).set_visible(false, v2_view);
        viewer.data(v1_charuco).set_visible(false, v2_view);
        viewer.data(v1_beam).set_visible(false, v2_view);

        viewer.data(v2).set_visible(false, v1_view);
        viewer.data(v1).set_visible(false, v1_view);
        viewer.data(v1_patient).set_visible(true, v1_view);
        viewer.data(v1_table).set_visible(true, v1_view);
        viewer.data(v1_cArm).set_visible(true, v1_view);
        viewer.data(v1_glass).set_visible(false, v1_view);
        viewer.data(v1_charuco).set_visible(true, v1_view);

        return false;
    };

    viewer.callback_post_resize = [&](igl::opengl::glfw::Viewer &v, int w, int h)
    {
        float boarder = w * 3 / 4;
        v.core(v1_view).viewport = Eigen::Vector4f(0, 0, w * 3 / 4, h);
        v.core(v2_view).viewport = Eigen::Vector4f(w * 3 / 4, 0, w / 4, h);
        return true;
    };
}

bool Viewer::Communication_run(igl::opengl::glfw::Viewer &)
{
    if (viewer.core(v1_view).is_animating)
    {
        // m.lock();
        vector<float> values = comm->GetValues();
        if(values.back() && !beamOn){
            beamOn = true;
            timer.start();
        }
        else if(values.back()==0 && beamOn)
        {
            beamOn = false;
            timer.stop();
        }

        // m.unlock();
        bool bedChk(false), cArmChk(false);
        if (cArm[0] != (int)values[0] | cArm[1] != (int)values[1])
        {
            cArmChk = true;
            cArm[0] = (int)values[0];
            cArm[1] = (int)values[1];
        }
        if (bed[0] != (int)values[3] | bed[1] != (int)values[2] | bed[2] != (int)values[4])
        {
            bedChk = true;
            bed[0] = (int)values[3];
            bed[1] = (int)values[2];
            bed[2] = (int)values[4];
        }
        bed[0] = (int)values[3];
        bed[1] = (int)values[2];
        bed[2] = (int)values[4];
        det[0] = (int)values[5];
        det[1] = (int)values[6];
        beam_exp[0] = (int)values[7];
        beam_exp[1] = (int)values[8];
        beam_fluo[0] = (int)values[8];
        beam_fluo[1] = (int)values[9];
        beam_fluo[1] = (int)values[10];
        if (bedChk)
        {
            viewer.data(v1_patient).set_vertices(V_patient.rowwise() + RowVector3d(bed[0], bed[1], bed[2]));
            viewer.data(v1_table).set_vertices(V_table.rowwise() + RowVector3d(bed[0], bed[1], bed[2]));
        }
        if (cArmChk)
        {
            AngleAxisd rot1(values[0] / 180. * igl::PI, Vector3d(0, 1, 0));
            AngleAxisd rot2(values[1] / 180. * igl::PI, rot1.matrix() * Vector3d(1, 0, 0));
            cArm_IT = (rot2 * rot1).matrix().inverse().transpose();
            Affine3d cArm_aff = Translation3d(isoCenter) * rot2 * rot1 * Translation3d(-isoCenter);
            viewer.data(v1_cArm).set_vertices((V_cArm.rowwise().homogeneous() * cArm_aff.matrix().transpose()).rowwise().hnormalized());
            viewer.data(v1_cArm).compute_normals();
            viewer.data(v1_beam).set_vertices((V_beam.rowwise().homogeneous() * cArm_aff.matrix().transpose()).rowwise().hnormalized());
        }

        MatrixXd C_disp;
        RotationList vQ(22, Quaterniond::Identity());
        Affine3d glass_aff;
        if(!playback)
        {
            comm->CurrentValue(glass_aff, vQ, C_disp);
            viewer.data(v1).set_points(C_disp, blue);    
        }
        else
        {
            C_disp.resize(1,3);
            ifs>> C_disp(0,0) >>C_disp(0,1)>>C_disp(0,2);
            for(int i=0;i<22;i++)
            {
                double w, x, y, z;
                ifs>>w>>x>>y>>z;
                vQ[i] = Quaterniond(w,x,y,z);
            }
        }

        // viewer.data(v1_glass).set_vertices((V_glass.rowwise().homogeneous() * glass_aff.matrix().transpose()).rowwise().hnormalized());
        // viewer.data(v1_glass).compute_normals();

        if (!playback && !comm->PostureAvailable())
            return false;  
        MatrixXd C_new;
        phantom->Animate(vQ, C_disp, C_new);
        viewer.data(v1).set_edges(C_new, phantom->GetBE(), sea_green);

        viewer.data(v1).set_vertices(phantom->GetU());
        viewer.data(v1).compute_normals();

        if (beamOn)
        {
            frameNum++;
            expTime = timer.getElapsedTime();
            viewer.data(v1_beam).set_visible(true, v1_view);
            CalculateDoses();
            viewer.data(v1).set_data(skinRate, 0, maps->GetMaxSkin() * 0.1, colorMap);
            // CalculateDoses();
            // viewer.data(v1).set_data(skinRate,0,maps->GetMaxSkin()*0.1,colorMap);
        }
        else
        {
            viewer.data(v1_beam).set_visible(false, v1_view);
            viewer.data(v1).set_data(VectorXd::Zero(skinRate.rows()), 0, maps->GetMaxSkin() * 0.1, colorMap);
        }
    }
    if (viewer.core(v2_view).is_animating)
    {
        v2_viewAng += v2_rotAng/180.*igl::PI;
        viewer.core(v2_view).camera_eye=Eigen::Vector3f(3*cos(v2_viewAng),0,3*sin(v2_viewAng));
    }
    
    return false;
}

bool Viewer::CalculateDoses()
{
    MatrixXd normals = viewer.data(v1).V_normals;
    MatrixXd U0 = phantom->GetU().rowwise() - isoCenter.transpose();
    MatrixXd U1 = ((U0 * cArm_IT).rowwise() - maps->GetIsoCenter().transpose()) * 0.2;
    for (int i = 0; i < U1.rows(); i++)
    {
        double factor(1);
        if (U0.row(i).squaredNorm() > 900)
        { // do not consider cosine for 30cm-radius sphere from isocenter
            double dotProd = U0.row(i).dot(normals.row(i)) / U0.row(i).norm();
            if (dotProd > 0.5)
                continue;
            if (dotProd > 0)
                factor = 0.5 - dotProd;
        }
        int idx = floor(abs(U1(i, 0))) * maps->GetK() * maps->GetJ() + floor(U1(i, 1)) * maps->GetK() + floor(U1(i, 2));
        if (idx < 30 * 60 * 60 && idx > 0)
            skinRate(i) = maps->GetSkinDose(idx) * factor;
    }
}