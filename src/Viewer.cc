#include "Viewer.hh"
#include "colorbar.hh"
#include <bitset>
#include <igl/writePLY.h>
// #include <igl/ambient_occlusion.h>
#include <igl/embree/ambient_occlusion.h>

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

    // dose initialization
    skinRate = VectorXd::Zero(phantom->GetV().rows());
    skinAcc = VectorXd::Zero(phantom->GetV().rows());

    // dosemap initialization
    maps = new MapContainer();
    colorMap = igl::COLOR_MAP_TYPE_PARULA;
    cArm_IT = Matrix3d::Identity();
    skinW = phantom->GetWSkin();
    // lensW = phantom->GetWLens();
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
static int anchorID(2);

static int cArm[2];
static int bed[3];
static int det[2];
static float beam_info[2] = {0, 0};
// static int beam_fluo[2];
static float dap;
static float v2_rotAng(10.);

// status manu
static float expTime(0.);
static int frameNum(0);
static float psd[2] = {0., 0.};
static float agvSkin[2] = {0., 0.};
static float lens[2] = {0., 0.};
static bool apron;
static bool glass;

void Viewer::SetFrame()
{
    
}
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
    static vector<string> anchors = {"12.2%", "15.6%", "18.9%", "22.2%", "25.6%", "28.9%", "32.0%", "32.2%", "35.6%", "38.9%", "42.3%"};
    if (ImGui::CollapsingHeader("Initialization", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::Combo("BF%", &anchorID, anchors);
        vector<string> names = phantom->GetProfileNames();
        profileNames.resize(names.size());
        copy(names.begin(), names.end(), profileNames.begin());
        ImGui::Combo("profile", &profileID, profileNames);
        ImGui::InputText("new profile", newProfile, ImGuiInputTextFlags_CharsNoBlank);
        ImGui::Combo("socket num.", &calib_sock, camSock);
        if(ImGui::Checkbox("Lead Apron", &apron))
        {
            viewer.data(v1_apron).is_visible = apron;
        }
        if(ImGui::Checkbox("Lead glass", &glass))
        {
            viewer.data(v1_glass).is_visible = glass;
        }
        if (ImGui::Button("Start measurement!", ImVec2(w, 0)))
        {
            if (phantom->AlreadyExists(newProfile))
                cout << newProfile << " already exists!" << endl;
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
        if (comm->IsRecording())
            strcpy(runStatus, "recording");
        else
            strcpy(runStatus, "running");
    }

    if (ImGui::CollapsingHeader("OCR results", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::InputInt2("C-arm", cArm, ImGuiInputTextFlags_ReadOnly);
        ImGui::InputInt3("bed [x/y/z, cm]", bed, ImGuiInputTextFlags_ReadOnly);
        ImGui::InputInt2("SID/FD [cm]", det, ImGuiInputTextFlags_ReadOnly);
        ImGui::InputFloat2("beam [kV/mA(s)]", beam_info, "%0.1f", ImGuiInputTextFlags_ReadOnly);
        // ImGui::InputInt2("flou. [kV/mA]", beam_fluo, ImGuiInputTextFlags_ReadOnly);
        ImGui::InputScalarN("DAPr [Gycm2/s]", ImGuiDataType_Float, &dap, 1, NULL, NULL, "%0.4f", ImGuiInputTextFlags_ReadOnly);
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
        if (ImGui::InputFloat("rot. ang./frame", &v2_rotAng, 10., 10., 1))
            ;
        if (ImGui::Button("stop/start rotation", ImVec2(ImGui::GetContentRegionAvailWidth(), 0)))
        {
            viewer.core(v2_view).is_animating = !viewer.core(v2_view).is_animating;
            viewer.core(v2_view).camera_eye = Eigen::Vector3f(0, 0, -3);
            v2_viewAng = 0;
        }
    }
    ImGui::PopItemWidth();
}

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
        cbar.draw_colorbar();
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
    viewer.append_mesh();
    viewer.data().set_mesh(phantom->GetUapron(), phantom->GetFapron());
    MatrixXi F_cArm, F_patient, F_glass, F_beam, F_table;
    igl::readPLY(cArm, V_cArm, F_cArm);
    V_cArm = V_cArm.rowwise() + isoCenter.transpose();
    igl::readPLY(patient, V_patient, F_patient);
    V_patient = V_patient.rowwise() + isoCenter.transpose();
    MatrixXd B_patient0, N_patient0, A_patient0;
    igl::barycenter(V_patient, F_patient, B_patient0);
    igl::per_face_normals(V_patient, F_patient, N_patient0);
    igl::doublearea(V_patient, F_patient, A_patient0);
    B_patient = B_patient0.cast<float>();
    N_patient = N_patient0.cast<float>();
    A_patient = A_patient0.cast<float>();
    igl::readPLY("table.ply", V_table, F_table);
    V_table = V_table.rowwise() + isoCenter.transpose();
    igl::readPLY("beam.ply", V_beam, F_beam);
    V_beam = V_beam.rowwise() + isoCenter.transpose();
    igl::readPLY(glass, V_glass, F_glass);
    viewer.append_mesh();
    viewer.data().set_mesh(V_patient, F_patient);
    viewer.data().set_points(B_patient1.cast<double>(), RowVector3d(1, 0, 0));
    viewer.data().point_size = 8;
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
    viewer.data().set_colors(RowVector4d(0, 0, 1, 0.2));
    viewer.data().show_lines = false;
    viewer.append_mesh();
    viewer.data().set_mesh(V, F);
    v1 = viewer.data_list[0].id;
    v1_apron = viewer.data_list[1].id;
    v1_patient = viewer.data_list[2].id;
    v1_table = viewer.data_list[3].id;
    v1_cArm = viewer.data_list[4].id;
    v1_beam = viewer.data_list[5].id;
    v1_charuco = viewer.data_list[6].id;
    v1_glass = viewer.data_list[7].id;
    v2 = viewer.data_list[8].id;

    viewer.data(v1).set_edges(C, BE, sea_green);
    viewer.data(v1).show_lines = false;
    viewer.data(v1).show_overlay_depth = false;
    viewer.data(v1).line_width = 1;
    viewer.data(v1).point_size = 8;
    viewer.data(v2).point_size = 4;
    viewer.data(v1).double_sided = false;
    viewer.data(v1_apron).double_sided = true;
    viewer.data(v1_apron).show_lines = false;
    viewer.data(v1_apron).set_colors(RowVector4d(0.2, 0.2, 0.2, 0.7));
    viewer.data(v1_patient).show_lines = true;
    viewer.data(v1_patient).show_overlay_depth = false;
    viewer.data(v1_patient).show_faces = false;
    viewer.data(v1_patient).is_visible = false;
    viewer.data(v1_patient).line_color = RowVector4f(0, 0.2, 0, 0.2);
    viewer.data(v1_table).set_colors(RowVector4d(0, 0.2, 0, 0.2));
    viewer.data(v1_table).show_lines = false;
    viewer.data(v1_cArm).set_colors(white);
    viewer.data(v1_cArm).show_lines = false;
    viewer.data(v1_cArm).double_sided = false;
    viewer.data(v1_cArm).set_colors(RowVector4d(0.8, 0.8, 0.8, 1.));
    viewer.data(v1_beam).set_colors(RowVector4d(1, 0, 0, 0.2));
    viewer.data(v1_beam).show_lines = false;
    viewer.data(v2).show_lines = false;
    viewer.data(v2).double_sided = false;
    viewer.data(v2).point_size = 4;
    viewer.data(v2).set_data(skinAcc, colorMap);

    // initialize reliabilitiy options
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
        // viewer.core(v1_view).background_color = Eigen::Vector4f(109. / 255., 100. / 255., 102. / 255., 1);
        // viewer.core(v2_view).background_color = Eigen::Vector4f(163. / 255., 163. / 255., 163. / 255., 1);
        viewer.core(v1_view).background_color = Eigen::Vector4f(1., 1., 1., 1);
        viewer.core(v2_view).background_color = Eigen::Vector4f(0., 0., 0., 1);

        viewer.core(v1_view).camera_center = Vector3f(isoCenter.x(), isoCenter.y(), isoCenter.z()) * 0.1; //*0.06;Vector3f(0, 0, 15);
        viewer.core(v1_view).camera_eye = viewer.core(v1_view).camera_center + Vector3f(50, 0, 10);
        viewer.core(v1_view).camera_up = Eigen::Vector3f(0, 0, 1);
        viewer.core(v1_view).set_rotation_type(igl::opengl::ViewerCore::ROTATION_TYPE_TRACKBALL);

        viewer.core(v2_view).camera_eye = Eigen::Vector3f(0, 0, -3);
        viewer.core(v2_view).camera_up = Eigen::Vector3f(0, -1, 0);

        viewer.data(v1).set_visible(false, v2_view);
        viewer.data(v1_apron).set_visible(false, v2_view);
        viewer.data(v1_patient).set_visible(false, v2_view);
        viewer.data(v1_table).set_visible(false, v2_view);
        viewer.data(v1_cArm).set_visible(false, v2_view);
        viewer.data(v1_glass).set_visible(false, v2_view);
        viewer.data(v1_charuco).set_visible(false, v2_view);
        viewer.data(v1_beam).set_visible(false, v2_view);

        viewer.data(v2).set_visible(false, v1_view);
        viewer.data(v1).set_visible(false, v1_view);
        viewer.data(v1_apron).set_visible(false, v1_view);
        viewer.data(v1_patient).set_visible(true, v1_view);
        viewer.data(v1_table).set_visible(true, v1_view);
        viewer.data(v1_cArm).set_visible(true, v1_view);
        viewer.data(v1_glass).set_visible(false, v1_view);
        viewer.data(v1_charuco).set_visible(true, v1_view);
        viewer.data(v1_beam).set_visible(false, v1_view);

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
extern float maxVal[2];

bool Viewer::Communication_run(igl::opengl::glfw::Viewer &)
{
    if (viewer.core(v1_view).is_animating)
    {
        timer.stop();
        double frameTime = timer.getElapsedTime();
        timer.start();

        if (comm->IsRecording() || playback)
            frameNum++;
        if (frameNum == 50)
            beamOn = true;
        dap = 0.01;
        if (beamOn)
        {
            viewer.data(v1_beam).set_visible(true, v1_view);
            viewer.data(v1_cArm).set_visible(false, v1_view);
        }
        else
        {
            viewer.data(v1_beam).set_visible(false, v1_view);
            viewer.data(v1_cArm).set_visible(true, v1_view);
        }

        // vector<float> values = comm->GetValues();
        // if (values.back() && !beamOn)
        // {
        //     beamOn = true;
        //     if (values.back() == 1)
        //         viewer.data(v1_beam).set_colors(RowVector4d(1, 0, 0, 0.2));
        //     else if (values.back() == 2)
        //         viewer.data(v1_beam).set_colors(RowVector4d(0, 1, 0, 0.2));
        //     viewer.data(v1_beam).set_visible(true, v1_view);
        // }
        // else if (values.back() == 0 && beamOn)
        // {
        //     viewer.data(v1_beam).set_visible(false, v1_view);
        //     beam_info[0] = 0;
        //     beam_info[1] = 0;
        //     beamOn = false;
        //     dap = 0;
        // }

        bool bedChk(false), cArmChk(false);
        if (frameNum == 1)
            bedChk = true;
        if (frameNum == 100)
            beamOn = false;
        if (frameNum > 110 && frameNum < 146)
        {
            cArmChk = true;
            cArm[0] = 5 * (frameNum - 110);
        }
        else
            cArmChk = false;
        if (frameNum == 160)
            beamOn = true;

        // if (cArm[0] != (int)values[0] | cArm[1] != (int)values[1])
        // {
        //     cArmChk = true;
        //     cArm[0] = (int)values[0];
        //     cArm[1] = (int)values[1];
        // }
        // if (bed[0] != (int)values[3] | bed[1] != (int)values[2] | bed[2] != (int)values[4])
        // {
        //     bedChk = true;
        //     bed[0] = (int)values[3];
        //     bed[1] = (int)values[2];
        //     bed[2] = (int)values[4];
        // }
        // bed[0] = (int)values[3];
        // bed[1] = (int)values[2];
        // bed[2] = (int)values[4];
        // det[0] = (int)values[5];
        // det[1] = (int)values[6];
        if (bedChk)
        {
            bed[1] = -30;
            bed[2] = -10;
            viewer.data(v1_patient).set_vertices(V_patient.rowwise() + RowVector3d(bed[0], bed[1], bed[2]));
            viewer.data(v1_table).set_vertices(V_table.rowwise() + RowVector3d(bed[0], bed[1], bed[2]));
        }
        if (cArmChk)
        {
            AngleAxisd rot1(cArm[0] / 180. * igl::PI, Vector3d(0, 1, 0));
            AngleAxisd rot2(cArm[1] / 180. * igl::PI, rot1.matrix() * Vector3d(1, 0, 0));
            cArm_IT = (rot2 * rot1).matrix().inverse().transpose();
            Affine3d cArm_aff = Translation3d(isoCenter) * rot2 * rot1 * Translation3d(-isoCenter);
            viewer.data(v1_cArm).set_vertices((V_cArm.rowwise().homogeneous() * cArm_aff.matrix().transpose()).rowwise().hnormalized());
            viewer.data(v1_cArm).compute_normals();
            MatrixXd V_beam1 = (V_beam.rowwise().homogeneous() * cArm_aff.matrix().transpose()).rowwise().hnormalized();
            viewer.data(v1_beam).set_vertices(V_beam1);
            viewer.data(v1_beam).compute_normals();
            CalculateSourceFacets(V_beam1.row(0).cast<float>(), (cArm_aff.rotation() * Vector3d(0, 0, 1)).cast<float>(), RowVector3f(bed[0], bed[1], bed[2]), 200);
            viewer.data(v1_patient).set_points(B_patient1.cast<double>(), RowVector3d(1, 0, 0));
        }

        MatrixXd C_disp;
        RotationList vQ(22, Quaterniond::Identity());
        Affine3d glass_aff;
        if (!playback)
        {
            comm->CurrentValue(glass_aff, vQ, C_disp);
            // viewer.data(v1).set_points(C_disp, blue);
        }
        else
        {
            C_disp.resize(1, 3);
            ifs >> C_disp(0, 0) >> C_disp(0, 1) >> C_disp(0, 2);
            for (int i = 0; i < 22; i++)
            {
                double w, x, y, z;
                ifs >> w >> x >> y >> z;
                vQ[i] = Quaterniond(w, x, y, z);
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
        viewer.data(v1_apron).set_vertices(phantom->GetUapron());
        viewer.data(v1_apron).compute_normals();

        if (beamOn)
        {
            // if (values.back() == 1)
            // {
            // frameNum++;
            expTime += frameTime * 0.5;
            // beam_info[0] = values[9];
            // beam_info[1] = values[10];
            // dap = values[11];
            if(B_patient1.rows()==0)
            {
                CalculateSourceFacets(V_beam.row(0).cast<float>(), RowVector3f(0, 0, 1), RowVector3f(bed[0], bed[1], bed[2]), 200);
                viewer.data(v1_patient).set_points(B_patient1.cast<double>(), RowVector3d(1, 0, 0));
            }
            CalculateDoses();
            if(apron) skinRate *= phantom->GetApronMask();
            // if(apron) CalculateDoses(phantom->GetOutApron());
            // else CalculateDoses();
            viewer.data(v1).set_data(skinRate, 0, maps->GetMaxSkin() * 0.001, colorMap);
            skinRate *= 3600.e3 * dap * maps->GetInvDAPperNPS();
            skinAcc += skinRate * (frameTime / 3600.);
            viewer.data(v2).set_data(skinAcc, 0, psd[1], colorMap);
            psd[0] = skinRate.array().maxCoeff();
            VectorXd::Index maxRow;
            psd[1] = skinAcc.array().maxCoeff(&maxRow);
            viewer.data(v2).set_points(viewer.data(v2).V.row(maxRow), RowVector3d(1, 0, 0));
            agvSkin[0] = (skinRate.array() * skinW).sum();
            agvSkin[1] = (skinAcc.array() * skinW).sum();
            maxVal[0] = maps->GetMaxSkin() * 0.001 * 3600.e3 * dap * maps->GetInvDAPperNPS();
            maxVal[1] = psd[1];

            cout<<C_new.row(23)<<endl;
            cout<<C_new.row(22)<<endl;


            int R, L;
            phantom->GetEyeIdx(R, L);
            Vector3i rightL = ((phantom->GetU().row(R) - isoCenter.transpose()) * cArm_IT * 0.2).array().floor().cast<int>();
            Vector3i leftL =((phantom->GetU().row(L) - isoCenter.transpose()) * cArm_IT  * 0.2).array().floor().cast<int>();
            double rightLD = maps->GetLensDose(rightL(0)* maps->GetK() * maps->GetJ() + rightL(1)* maps->GetK() + rightL(2) + maps->GetIdxBase());
            double leftLD = maps->GetLensDose(leftL(0)* maps->GetK() * maps->GetJ() + leftL(1)* maps->GetK() + leftL(2) + maps->GetIdxBase());
            lens[0] = (rightLD + leftLD)*0.5* 3600.e3 * dap * maps->GetInvDAPperNPS();
            lens[1] += lens[0] * (frameTime/3600.);
            // // maxVal[1] = maps->GetMaxSkin() * 0.1 * 3600.e3 * dap * maps->GetInvDAPperNPS();
            // lens[0] *= 3600.e3 * dap * maps->GetInvDAPperNPS();
            // lens[1] += lens[0] * (frameTime/3600.);
            // }
            // else if (values.back() == 2)
            // {
            //     beam_info[0] = values[7];
            //     beam_info[1] = values[8];
            // }
            // CalculateDoses();
            // viewer.data(v1).set_data(skinRate,0,maps->GetMaxSkin()*0.1,colorMap);
        }
        else
        {
            psd[0] = 0;
            agvSkin[0] = 0;
            viewer.data(v1).set_data(VectorXd::Zero(skinRate.rows()), 0, maps->GetMaxSkin() * 0.1, colorMap);
        }
    }
    if (viewer.core(v2_view).is_animating)
    {
        v2_viewAng += v2_rotAng / 180. * igl::PI;
        viewer.core(v2_view).camera_eye = Eigen::Vector3f(3 * cos(v2_viewAng), 0, 3 * sin(v2_viewAng));
    }

    return false;
}

bool Viewer::CalculateDoses()
{
    MatrixXd normals = viewer.data(v1).V_normals;
    MatrixXd U = phantom->GetU();
    MatrixXi gridM = (((U.rowwise() - isoCenter.transpose()) * cArm_IT) * 0.2).array().floor().cast<int>();
    VectorXi idx = (gridM.col(0) * maps->GetK() * maps->GetJ() + gridM.col(1) * maps->GetK() + gridM.col(2)).array() + maps->GetIdxBase();

    int R, L;
    phantom->GetEyeIdx(R, L);
    lens[0] = (maps->GetLensDose(idx(R)) + maps->GetLensDose(idx(L)) )*0.5*3600.e3 * dap * maps->GetInvDAPperNPS();

    maps->GetSkinDoseMap(doseMapS);

    // VectorXd s;
    igl::embree::ambient_occlusion(U, phantom->GetF(), U, normals, 20, skinRate);
    skinRate = 1-skinRate;

    skinRate *= igl::slice(doseMapS, idx, 1).array();

    igl::embree::EmbreeIntersector ei;
    if(!glass) ei.init(U.template cast<float>(), phantom->GetF().template cast<int>());
    else{
        MatrixXd U1(U.rows() + V_glass.rows(), 3);
        MatrixXi F1(phantom->GetF().rows() + viewer.data(v1_glass).F.rows(), 3);
        U1 << U, V_glass;
        F1 << phantom->GetF(), (viewer.data(v1_glass).F).array()+U.rows();
        ei.init(U1.template cast<float>(), F1.template cast<int>());
    }

#pragma omp for
    for (int i = 0; i < U.rows(); i++)
    {
        double hitN(0);
        MatrixXf dir = (-B_patient1).rowwise() + U.row(i).cast<float>();
        VectorXf weight = (N_patient1.array() * dir.rowwise().normalized().array()).rowwise().sum();
        double tot(0);
        for (int j = 0; j < weight.rows(); j++)
        {
            if (weight(j) < 0)
                continue;
            weight(j) *= A_patient1(j);
            tot += weight(j);
            igl::Hit hit;
            // if(aabb.intersect_ray(V_f, F,B_patientF.row(j), dir.row(j)*0.9999,hit)) hitN += weight(j);
            if (ei.intersectSegment(B_patient1.row(j), dir.row(j) * 0.9, hit))
            {
                hitN += weight(j);
            } // cout<<B_patientF.row(j)<<endl<<dir.row(j)+B_patientF.row(j); getchar();}
        }
        skinRate(i) *= (1-hitN / tot);    // VectorXd s;
        // igl::ambient_occlusion(U, phantom->GetF(), U, normals, 10, s); tot);
        // cout<<"\rgenerating shadows.."<<i+1<<"/"<<U.rows()<<flush;
    }

    // skinRate = skinRate.array() * (1-s.array());
    return true;

    // MatrixXd normals = viewer.data(v1).V_normals;
    // MatrixXd U0 = phantom->GetU().rowwise() - isoCenter.transpose();
    // MatrixXd U1 = ((U0 * cArm_IT).rowwise() - maps->GetIsoCenter().transpose()) * 0.2;
    // double lensAgv(0.);
    // skinRate = VectorXd::Zero(U1.rows());

    // for (int i = 0; i < U1.rows(); i++)
    // {
    //     double factor(1);
    //     if (U0.row(i).squaredNorm() > 900)
    //     { // do not consider cosine for 30cm-radius sphere from isocenter
    //         double dotProd = U0.row(i).dot(normals.row(i)) / U0.row(i).norm();
    //         if (dotProd > 0.5)
    //             continue;
    //         if (dotProd > 0)
    //             factor = 0.5 - dotProd;
    //     }
    //     int idx = floor(abs(U1(i, 0))) * maps->GetK() * maps->GetJ() + floor(U1(i, 1)) * maps->GetK() + floor(U1(i, 2));
    //     if (idx < 30 * 60 * 60 && idx > 0)
    //     {
    //         skinRate(i) = maps->GetSkinDose(idx) * factor;
    //         // if(lensW.find(i)!=lensW.end()) lensAgv += maps->GetLensDose(idx) * lensW[i] * factor;
    //     }
    // }
    // lens[0] = lensAgv;
}


bool Viewer::CalculateDoses(VectorXi _idx)
{
    MatrixXd normals = igl::slice(viewer.data(v1).V_normals, _idx, 1);
    MatrixXd U = igl::slice(phantom->GetU(), _idx, 1);
    MatrixXi gridM = (((U.rowwise() - isoCenter.transpose()) * cArm_IT) * 0.2).array().floor().cast<int>();
    VectorXi idx = (gridM.col(0) * maps->GetK() * maps->GetJ() + gridM.col(1) * maps->GetK() + gridM.col(2)).array() + maps->GetIdxBase();

    int R, L;
    phantom->GetEyeIdx(R, L);
    lens[0] = (maps->GetLensDose(idx(R)) + maps->GetLensDose(idx(L)) )*0.5*3600.e3 * dap * maps->GetInvDAPperNPS();

    maps->GetSkinDoseMap(doseMapS);

    // VectorXd s;
    ArrayXd skinRate1;
    igl::embree::ambient_occlusion(phantom->GetU(), phantom->GetF(), U, normals, 20, skinRate1);
    skinRate1 = 1-skinRate1;

    skinRate1 *= igl::slice(doseMapS, idx, 1).array();

    igl::embree::EmbreeIntersector ei;
    if(!glass) ei.init(U.template cast<float>(), phantom->GetF().template cast<int>());
    else{
        MatrixXd U1(U.rows() + V_glass.rows(), 3);
        MatrixXi F1(phantom->GetF().rows() + viewer.data(v1_glass).F.rows(), 3);
        U1 << U, viewer.data(v1_glass).V;
        F1 << phantom->GetF(), (viewer.data(v1_glass).F).array()+U.rows();
        ei.init(U1.template cast<float>(), F1.template cast<int>());
    }

#pragma omp for
    for (int i = 0; i < U.rows(); i++)
    {
        double hitN(0);
        MatrixXf dir = (-B_patient1).rowwise() + U.row(i).cast<float>();
        VectorXf weight = (N_patient1.array() * dir.rowwise().normalized().array()).rowwise().sum();
        double tot(0);
        for (int j = 0; j < weight.rows(); j++)
        {
            if (weight(j) < 0)
                continue;
            weight(j) *= A_patient1(j);
            tot += weight(j);
            igl::Hit hit;
            // if(aabb.intersect_ray(V_f, F,B_patientF.row(j), dir.row(j)*0.9999,hit)) hitN += weight(j);
            if (ei.intersectSegment(B_patient1.row(j), dir.row(j) * 0.9, hit))
            {
                hitN += weight(j);
            } // cout<<B_patientF.row(j)<<endl<<dir.row(j)+B_patientF.row(j); getchar();}
        }
        skinRate1(i) *= (1-hitN / tot);    // VectorXd s;
        // igl::ambient_occlusion(U, phantom->GetF(), U, normals, 10, s); tot);
        // cout<<"\rgenerating shadows.."<<i+1<<"/"<<U.rows()<<flush;
    }
    skinRate = VectorXd::Zero(phantom->GetU().rows());
    igl::slice_into(VectorXd(skinRate1), _idx, skinRate);
    
    // skinRate = skinRate.array() * (1-s.array());
    return true;
}
