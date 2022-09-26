#include "RDCWindow.hh"
#include <igl/readPLY.h>
#include <external/imgui/backends/imgui_impl_glfw.h>
#include <implot.h>
#include <igl/Timer.h>
#include <mysql/mysql.h>

RDCWindow &RDCWindow::Instance()
{
    static RDCWindow RDCWindow;
    return RDCWindow;
}

static void MainMenuBar(RDCWindow *_window);
static void ShowMachineStatus(bool *p_open, RDCWindow *_window);
static void ShowViewOptions(bool *p_open, RDCWindow *_window);
static void ShowSettingsWindow(bool *p_open, RDCWindow *_window);
static void ShowManualSettingPopup(bool *p_open, RDCWindow *_window);
static void DoseResultWindow(RDCWindow *_window);
static void ShowProgramInformationPopUp(bool *p_open);
void ColorInfoWindow(float min, float max, string unit);
static void ShowColorInfoPopUp(bool *p_open);

void RDCWindow::Initialize()
{
    mainMenu.callback_draw_viewer_window = [this]() -> bool
    {
        MainMenuBar(this);
        return true;
    };
    viewer.plugins.push_back(&mainMenu);

    // multi view
    viewer.callback_init = [&](igl::opengl::glfw::Viewer &_viewer)
    {
        int w, h;
        glfwGetWindowSize(_viewer.window, &w, &h);
        float phantomViewX = h * 0.35;
        // radiolRate
        // _viewer.core().viewport = Eigen::Vector4f(0, h * 0.29, w - phantomViewX * 2, h * 0.71);
        _viewer.core().viewport = Eigen::Vector4f(0, h * 0.29, w - phantomViewX,  h - 230);
        this->v_left = viewer.core_list[0].id;
        _viewer.core(this->v_left).background_color = Eigen::Vector4f(0.95, 0.95, 0.95, 1.);
        _viewer.core(this->v_left).camera_center = Vector3f(0, 0, 10);
        _viewer.core(this->v_left).camera_up = Vector3f(0, 0, 1);
        _viewer.core(this->v_left).camera_eye = Vector3f(60, 0, 0);
        // radiolAcc
        // this->v_middle = viewer.append_core(Eigen::Vector4f(w - phantomViewX * 2, h * 0.29, phantomViewX, h * 0.71));
        this->v_middle = viewer.append_core(Eigen::Vector4f(w - phantomViewX, h * 0.29, phantomViewX,  h - 230));
        _viewer.core(this->v_middle).background_color = Eigen::Vector4f(255. / 255., 255. / 255., 255. / 255., 0.);
        _viewer.core(this->v_middle).camera_center = Vector3f(0, 0, 0);
        _viewer.core(this->v_middle).camera_up = Vector3f(0, -1, 0);
        _viewer.core(this->v_middle).camera_dfar = 300;
        _viewer.core(this->v_middle).camera_eye = Vector3f(0, 0, -250);
        // patient
        // this->v_right = viewer.append_core(Eigen::Vector4f(w - phantomViewX, h * 0.29, phantomViewX, h * 0.71));
        // _viewer.core(this->v_right).background_color = Eigen::Vector4f(82. / 255., 82. / 255., 82. / 255., 0.);

        // visiblity
        _viewer.data(this->apron_data).is_visible = 0;
        _viewer.data(this->patient_data).is_visible = 0;
        _viewer.data(this->table_data).is_visible = 0;
        _viewer.data(this->cArm_data).is_visible = 0;
        _viewer.data(this->beam_data).is_visible = 0;
        _viewer.data(this->glass_data).is_visible = 0;
        _viewer.data(this->grid_data).is_visible = 0;
        _viewer.data(this->axis_data).is_visible = 0;
        _viewer.data(this->phantomAcc_data).is_visible = 0;
        _viewer.data(this->sphere_data).is_visible = 0;
        for (int id : this->phantom_data)
            _viewer.data(id).is_visible = 0;

        // _viewer.data(this->phantom_data).is_visible |= this->v_left;
        // _viewer.data(this->apron_data).is_visible   |= this->v_left;
        _viewer.data(this->patient_data).is_visible |= this->v_left;
        _viewer.data(this->table_data).is_visible |= this->v_left;
        _viewer.data(this->cArm_data).is_visible |= this->v_left;
        _viewer.data(this->beam_data).is_visible |= this->v_left;
        _viewer.data(this->grid_data).is_visible |= this->v_left;
        _viewer.data(this->phantomAcc_data).is_visible |= this->v_middle;

        _viewer.selected_core_index = this->v_left;

        _viewer.core(this->v_left).set_rotation_type(igl::opengl::ViewerCore::ROTATION_TYPE_TRACKBALL);
        return false;
    };
    viewer.callback_post_resize = [&](igl::opengl::glfw::Viewer &v, int w, int h)
    {
        float phantomViewX = h * 0.35;
        // radiolRate
        // v.core(this->v_left).viewport = Eigen::Vector4f(0, 235, w - phantomViewX * 2, h - 235);
        v.core(this->v_left).viewport = Eigen::Vector4f(0, 230, w - phantomViewX, h - 230);
        // radiolAcc
        // v.core(this->v_middle).viewport = Eigen::Vector4f(w - phantomViewX * 2, 235, phantomViewX, h - 235);
        v.core(this->v_middle).viewport = Eigen::Vector4f(w - phantomViewX, 230, phantomViewX, h - 230);
        // patient
        // v.core(this->v_right).viewport = Eigen::Vector4f(w - phantomViewX, 235, phantomViewX, h - 235);
        return true;
    };

    // set meshes
    SetMeshes(".");

    // callback functions
    viewer.callback_pre_draw = bind(&RDCWindow::PreDrawFunc, this, placeholders::_1);
    viewer.callback_post_draw = [&](igl::opengl::glfw::Viewer &_viewer) -> bool
    {
        // to prevent program to be idle
        // if (RDCWindow::Instance().beamOn)
        _viewer.core(0).is_animating = true;
        // else
        //     _viewer.core(0).is_animating = false;
        // _viewer.core().is_animating = true;
        return true;
    };

    // set everything always animating
    viewer.core(0).is_animating = true;
    viewer.core(v_left).is_animating = true;
    viewer.core(v_middle).is_animating = true;
    // viewer.core(v_right).is_animating = true;
}
static int boneID(0);
static bool showAxis(false);
bool RDCWindow::PreDrawFunc(igl::opengl::glfw::Viewer &_viewer)
{
    if (stop)
        return false;
    // if (showAxis)
    // {
    //     _viewer.data(phantomAcc_data).set_data(indivPhantoms[bodyID]->GetWeight(boneID));

    //     MatrixXd CE = MatrixXd::Zero(4, 3);
    //     CE.bottomRows(3) = Matrix3d::Identity() * 10;
    //     MatrixXi BE(3, 2);
    //     BE << 0, 1, 0, 2, 0, 3;
    //     DataSet data = Communicator::Instance().current;
    //     _viewer.data(phantom_data[bodyID]).set_edges((CE * data.bodyMap[bodyID].posture[boneID].normalized().matrix().transpose()).rowwise() + data.bodyMap[bodyID].jointC.row(boneID), BE, Matrix3d::Identity());
    // }
    float frameTimeInMSEC(0);
    if (loadNum >= 0) // loading
    {
        if (loadNum == recordData.size())
        {
            loadNum = -1;
            return false;
        }
        currentFrame = ReadAframeData(recordData[loadNum++]);
        frameTimeInMSEC = currentFrame.time;
        //////////////////
        // if(currentFrame.bodyMap.find(0)==currentFrame.bodyMap.end()) return false;
        // currentFrame.bodyMap[0].jointC(0) -= 20;
        // currentFrame.beamOn = true;
    }
    else
    { // not loading
        Communicator::Instance().SetCurrentFrame(currentFrame, bodyDelayTol);
        if (currentFrame.beamOn || useManualBeam)
        {
            if (lastFrameT > 0)
                frameTimeInMSEC = float(currentFrame.time - lastFrameT) / CLOCKS_PER_SEC * 1000;
            lastFrameT = currentFrame.time;
            if (recording)
                recordData.push_back(SetAframeData(currentFrame, frameTimeInMSEC));
        }
        else if (lastFrameT > 0) // last frame
        {
            frameTimeInMSEC = currentFrame.time - lastFrameT;
            lastFrameT = -1;
            if (recording)
            {
                recordData.push_back(SetAframeData(currentFrame, frameTimeInMSEC));
                recordData.back().push_back(0); // last frame flag
            }
        }
    }

    if (_viewer.core(v_left).is_animating)
    {
        // machine
        if (!useManualBeam)
        {
            _viewer.data(patient_data).set_vertices(V_patient.rowwise() + currentFrame.bed.cast<double>());
            _viewer.data(table_data).set_vertices(V_table.rowwise() + currentFrame.bed.cast<double>());
            SetMap(currentFrame.kVp, currentFrame.cArm(0), currentFrame.cArm(1));
            // SetDAP(currentFrame.dap);
            MatrixXf rot = GetCarmRotation(currentFrame.cArm(0), currentFrame.cArm(1));
            _viewer.data(cArm_data).set_vertices(V_cArm * rot.cast<double>().transpose());
            _viewer.data(beam_data).set_vertices(V_beam * rot.cast<double>().transpose());
            _viewer.data(cArm_data).compute_normals();
            _viewer.data(beam_data).compute_normals();
            CalculateSourceFacets(currentFrame.bed, rot);
            _viewer.data(patient_data).set_points(B_patient1.cast<double>(), RowVector3d(1, 0, 0));
        }
        else
            currentFrame.dap = manualDAP;

        if (!currentFrame.beamOn && !useManualBeam)
        {
            _viewer.data(patient_data).set_visible(false, v_left);
            return false;
        }
        _viewer.data(patient_data).set_visible(show_beam, v_left);

        if (currentFrame.glassChk)
        {
            if (show_leadGlass)
                _viewer.data(glass_data).is_visible |= v_left;
            _viewer.data(glass_data).set_vertices((V_glass.rowwise().homogeneous() * currentFrame.glass_aff.matrix().transpose()).rowwise().hnormalized());
            _viewer.data(glass_data).compute_normals();
        }
        else
            _viewer.data(glass_data).is_visible = 0;

        if (!currentFrame.bodyMap.size())
        {
            for (int i = 0; i < phantom_data.size(); i++)
                _viewer.data(phantom_data[i]).is_visible = 0;
            return false;
        }

        MatrixXd P, C, V;
        MatrixXi BE, F;
        int extraID;
        // posture deformation
        for (auto iter : currentFrame.bodyMap)
        {
            if (iter.first<0) continue;
            if (indivPhantoms[iter.first]->V.rows() > 0)
                indivPhantoms[iter.first]
                    ->Animate(iter.second.posture, iter.second.jointC, C, false);
        }
        // shadow generation
        igl::embree::EmbreeIntersector ei;
        MatrixXf totalV = InitTree(ei, currentFrame.bodyMap);
        VectorXd dose = VectorXd::Zero(totalV.rows());
        if (B_patient1.rows() && (currentFrame.beamOn || useManualBeam))
        {
            dose = GenerateShadow(ei, totalV);
            SetDoseRate(totalV, dose, currentFrame.dap);
            // cout<<dose.maxCoeff()<<endl;
        }
        // else if(data.beamOn||useManualBeam) // to-do
        // {

        // }

        // visualization & dose calculation
        for (int i = 0, vNum = 0; i < phantom_data.size(); i++)
        {
            if(indivPhantoms[i]->V.rows() ==0) continue;
            if (currentFrame.bodyMap.find(i) == currentFrame.bodyMap.end())
            {
                _viewer.data(phantom_data[i]).is_visible = 0;
                continue;
            }
            indivPhantoms[i]->SetDose(dose.block(vNum, 0, indivPhantoms[i]->U.rows(), 1), frameTimeInMSEC);
            _viewer.data(phantom_data[i]).is_visible |= v_left;
            _viewer.data(phantom_data[i]).set_vertices(indivPhantoms[i]->U);
            // indivPhantoms[i]->accD += dose.block(vNum, 0, indivPhantoms[i]->U.rows(), 1);
            if (i == bodyID)
            {
                if (show_C)
                    _viewer.data(phantom_data[i]).set_points(currentFrame.bodyMap[i].jointC, RowVector3d(0, 0, 1));
                if (show_BE)
                    _viewer.data(phantom_data[i]).set_edges(C, indivPhantoms[i]->BE, RowVector3d(70. / 255., 252. / 255., 167. / 255.));

                _viewer.data(phantom_data[i]).set_data(indivPhantoms[i]->rateD);
                _viewer.data(phantom_data[i]).compute_normals();
                // _viewer.data(phantomAcc_data).set_data(indivPhantoms[i]->accD);
            }
            vNum += indivPhantoms[i]->U.rows();
        }
    }

    return false;
}

void RDCWindow::SetMeshes(string dir)
{
    // radiologist phantom
    //  auto phantom  = indivPhantoms[0];
    //  viewer.data().set_mesh(phantom->U_apron, phantom->F_apron);
    // apron_data = viewer.selected_data_index;
    viewer.append_mesh();
    // viewer.data().set_mesh(phantom->V, phantom->F);
    phantomAcc_data = viewer.selected_data_index;

    // read other meshes
    MatrixXi F_cArm, F_patient, F_glass, F_beam, F_table, F_sphere;
    igl::readPLY(dir + "/c-arm.ply", V_cArm, F_cArm);
    igl::readPLY(dir + "/patient.ply", V_patient, F_patient);
    igl::readPLY(dir + "/table.ply", V_table, F_table);
    igl::readPLY(dir + "/beam.ply", V_beam, F_beam);
    igl::readPLY(dir + "/leadGlass.ply", V_glass, F_glass);
    igl::readPLY(dir + "/sphere.ply", V_sphere, F_sphere);
    V_sphere.rowwise().normalize();

    // for shadow factors
    MatrixXd B_patient0, Nface_patient;
    igl::barycenter(V_patient, F_patient, B_patient0);
    igl::per_face_normals(V_patient, F_patient, Nface_patient);
    B_patient = (B_patient0 + Nface_patient * 0.1).cast<float>();
    N_patient = Nface_patient.cast<float>();
    igl::doublearea(V_patient, F_patient, A_patient);
    CalculateSourceFacets(RowVector3f::Zero(), Matrix3f::Identity());

    // append other meshes
    viewer.append_mesh();
    viewer.data().set_mesh(V_patient, F_patient);
    viewer.data().show_texture = true;
    viewer.data().double_sided = true;
    // viewer.data().set_points(B_patient1.cast<double>(), RowVector3d(1, 0, 0));
    // viewer.data().point_size = 8;
    patient_data = viewer.selected_data_index;
    viewer.append_mesh();
    viewer.data().set_mesh(V_table, F_table);
    table_data = viewer.selected_data_index;
    viewer.append_mesh();
    viewer.data().set_mesh(V_cArm, F_cArm);
    cArm_data = viewer.selected_data_index;
    viewer.append_mesh();
    viewer.data().set_mesh(V_beam, F_beam);
    beam_data = viewer.selected_data_index;
    viewer.append_mesh();
    viewer.data().set_mesh(V_glass, F_glass);
    viewer.data().set_colors(RowVector4d(0, 0, 1, 0.2));
    viewer.data().show_lines = false;
    glass_data = viewer.selected_data_index;
    viewer.append_mesh();

    for (int i = 0; i < maxNumPeople; i++)
    {
        phantom_data.push_back(viewer.selected_data_index);
        viewer.append_mesh();
        viewer.append_mesh(); // for apron
    }

    // draw grids
    int n(5);
    double gap(50);
    double floorZ(-113.5);
    MatrixXd V_grid(8 * n + 4, 3);
    MatrixXi E_grid(4 * n + 2, 2);
    for (int i = 0; i < 2 * n + 1; i++)
    {
        V_grid.row(i) = RowVector3d((i - n) * gap, n * gap, floorZ);
        V_grid.row(V_grid.rows() * 0.25 + i) = RowVector3d((i - n) * gap, -n * gap, floorZ);
        V_grid.row(V_grid.rows() * 0.5 + i) = RowVector3d(n * gap, (i - n) * gap, floorZ);
        V_grid.row(V_grid.rows() * 0.75 + i) = RowVector3d(-n * gap, (i - n) * gap, floorZ);
        E_grid.row(i) = RowVector2i(i, V_grid.rows() * 0.25 + i);
        E_grid.row(E_grid.rows() * 0.5 + i) = RowVector2i(V_grid.rows() * 0.5 + i, V_grid.rows() * 0.75 + i);
    }
    MatrixXd C_grid = RowVector3d(0.2, 0.2, 0.2).replicate(E_grid.rows(), 1);
    C_grid.row(n) = RowVector3d(1, 0, 0);
    C_grid.row(E_grid.rows() * 0.5 + n) = RowVector3d(1, 0, 0);
    viewer.data().set_edges(V_grid, E_grid, C_grid);
    grid_data = viewer.selected_data_index;

    // axis
    MatrixXd CE = MatrixXd::Zero(4, 3);
    CE.bottomRows(3) = Matrix3d::Identity() * 50;
    MatrixXi BE(3, 2);
    BE << 0, 1, 0, 2, 0, 3;
    viewer.append_mesh();
    viewer.data().set_edges(CE, BE, Matrix3d::Identity());
    viewer.data().line_width = 10;
    axis_data = viewer.selected_data_index;

    // sphere
    viewer.append_mesh();
    viewer.data().set_mesh(V_sphere * 20, F_sphere);
    viewer.data().show_faces = false;
    viewer.data().show_lines = true;
    viewer.data().line_width = 1;
    viewer.data().line_color = RowVector4f(0.5, 0.5, 0.5, 0.5);
    sphere_data = viewer.selected_data_index;

    // viewer settings
    viewer.data(apron_data).double_sided = true;
    viewer.data(apron_data).show_lines = false;
    viewer.data(apron_data).set_colors(RowVector4d(0.2, 0.2, 0.2, 0.7));
    for (int id : phantom_data)
    {
        viewer.data(id).double_sided = true;
        viewer.data(id).show_overlay_depth = true;
        viewer.data(id).line_width = 1;
        viewer.data(id).point_size = 8;
        viewer.data(id).show_texture = true;
        viewer.data(id).show_lines = false;
        viewer.data(id).show_faces = true;
        viewer.data(id).set_colors(RowVector4d(0.2, 0.2, 0.2, 0.2));
        if (id == mainID())
            viewer.data(id).show_texture = false;
    }

    viewer.data(patient_data).show_lines = true;
    viewer.data(patient_data).show_overlay_depth = true;
    viewer.data(patient_data).show_faces = false;
    viewer.data(patient_data).line_color = RowVector4f(0, 0.2, 0, 0.2);
    viewer.data(patient_data).set_colors(RowVector4d(0.8, 1., 0.8, 1.));
    viewer.data(patient_data).show_texture = true;
    viewer.data(patient_data).point_size = 5;
    viewer.data(table_data).set_colors(RowVector4d(0, 0.2, 0, 0.2));
    viewer.data(table_data).show_overlay_depth = true;
    viewer.data(table_data).show_lines = false;
    viewer.data(cArm_data).set_colors(RowVector4d(1., 1., 1., 1.));
    viewer.data(cArm_data).show_lines = false;
    viewer.data(cArm_data).show_overlay_depth = false;
    viewer.data(cArm_data).double_sided = false;
    viewer.data(cArm_data).point_size = 4;
    viewer.data(cArm_data).set_colors(RowVector4d(0.8, 0.8, 0.8, 1.));
    viewer.data(beam_data).set_colors(RowVector4d(1, 0, 0, 0.2));
    viewer.data(beam_data).show_lines = false;
    viewer.data(beam_data).show_overlay_depth = false;
    viewer.data(phantomAcc_data).point_size = 4;
    viewer.data(phantomAcc_data).show_lines = false;
    viewer.data(phantomAcc_data).double_sided = false;
}

void mysql_finish_with_error(MYSQL* conn)
{
    cerr << "SQL Error !! - " + string(mysql_error(conn)) << endl;
    mysql_close(conn);
}

void MainMenuBar(RDCWindow *_window)
{
    static bool show_status_window = true;
    static bool show_view_options = false;
    static bool show_settings_window = true;
    static bool show_manual_setting_popup = false;
    static bool show_color_popup = true;
    static bool show_info_popup = false;
    static bool show_machine_popup = false;
    static bool connect_PDC = true;
    DoseResultWindow(_window);
    if (show_view_options)
        ShowViewOptions(&show_view_options, _window);
    if (show_color_popup)
        ShowColorInfoPopUp(&show_color_popup);
    if (show_settings_window)
        ShowSettingsWindow(&show_settings_window, _window);
    if (show_manual_setting_popup)
        ShowManualSettingPopup(&show_manual_setting_popup, _window);
    if (show_info_popup)
        ShowProgramInformationPopUp(&show_info_popup);
    if (show_machine_popup)
        ShowMachineStatus(&show_machine_popup, _window);
    if (ImGui::BeginMainMenuBar())
    {
        if (ImGui::BeginMenu("View"))
        {
            ImGui::MenuItem("show machine status", "", &show_machine_popup);
            ImGui::MenuItem("show view options", "", &show_view_options);
            ImGui::MenuItem("show color bar", "", &show_color_popup);
            ImGui::MenuItem("show info.", "", &show_info_popup);
            if (ImGui::BeginMenu("show bone weight."))
            {
                ImGui::Checkbox("show axis and weight", &showAxis);
                ImGui::InputInt("bone ID", &boneID, 1, 1);
                ImGui::EndMenu();
            }
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Settings"))
        {
            ImGui::SetNextItemWidth(ImGui::GetFontSize() * 7.f);
            static double bdtol(_window->bodyDelayTol);
            if(ImGui::InputDouble("body delay tol.", &bdtol,0.1, 1, "%.1f")) _window->bodyDelayTol = bdtol;
            ImGui::SetNextItemWidth(ImGui::GetFontSize() * 7.f);
            ImGui::DragFloat("font size", &ImGui::GetFont()->Scale, 0.005f, 0.3f, 2.0f, "%.1f");
            if (ImGui::MenuItem("open settings", ""))
            {
                show_settings_window = true;
            }
            if (ImGui::MenuItem("manual beam", ""))
            {
                show_manual_setting_popup = true;
            }
            ImGui::EndMenu();
        }
        static int pdcProcess(0), pdcDataSize(0);
        static thread pdcSender;
        if (ImGui::BeginMenu("Export"))
        {
            static string name;
            ImGui::SetNextItemWidth(110);
            ImGui::InputText("", name);
            ImGui::SameLine();
            if (ImGui::Button("export"))
            {
                if (_window->recording)
                    cout << "stop recording first" << endl;
                else
                {
                    ofstream ofs(name, ios::binary);
                    for (auto frame : _window->recordData)
                        ofs.write((char *)frame.data(), _window->dataSize * sizeof(float));
                    ofs.close();
                    _window->recordData.clear();
                    cout << name << " recorded!" << endl;
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("load"))
            {
                if (_window->loadNum < 0)
                {
                    if (_window->recording)
                        cout << "recording is on currently!!" << endl;
                    else
                    {
                        ifstream ifs(name, ios::binary);
                        if (!ifs.is_open())
                            cout << name << " is not open!" << endl;
                        vector<float> frameData(_window->dataSize);
                        while (ifs.read((char *)frameData.data(), _window->dataSize * sizeof(float)))
                            _window->recordData.push_back(frameData);
                        ifs.close();
                        if (_window->recordData.size())
                            _window->loadNum = 0;
                    }
                }
                else
                {
                    _window->recordData.clear();
                    _window->loadNum = -1;
                }
            }
            static int pdcFrameT(500);
            ImGui::InputInt("ms", &pdcFrameT, 100);
            ImGui::SameLine();
            if (ImGui::Button("Send to NAS"))
            {
                if (pdcSender.joinable())
                {
                    cout << "wait for pevious sender!" << endl;
                }
                else
                {
                    vector<vector<float>> data;
                    float time(0);
                    for (auto frame : _window->recordData)
                    {
                        time += frame[0];
                        if ((frame.size() > _window->dataSize) || (time > pdcFrameT)) // last frame || time>min. frame time
                        {
                            frame[0] = time;
                            data.push_back(frame);
                            time = 0;
                        }
                    }
                    pdcDataSize = data.size();
                    pdcSender = thread(
                        [&pdcProcess](vector<vector<float>> _data)
                        {
                            string DB_HOST("166.104.155.16");
                            string DB_USER("sungho");
                            string DB_PASS("sungho");
                            string DB_NAME("McDCIR_PDC");
                            string DB_TABLE("FrameMain");
                            int PORT_ID = 3307;

                            vector<string> body_TABLE;
                            for (int i=0;i<20;i++) {
                                body_TABLE.push_back("id" + to_string(i));
                            }
                            vector<string> main_colNameVec, body_colNameVec;
                            string main_colNameStr, main_colValueStr, body_colNameStr;
                            vector<string> body_colValueStr;
                            string main_colNameStr_tmp, main_colValueStr_tmp;

                            main_colNameVec = {"time","kVp","mA","dap","cArm0","cArm1","cArm2","bed0","bed1","bed2",
                                            "glassChk","glass_qx","glass_qy","glass_qz","glass_qw","glass_tx","glass_ty","glass_tz",
                                            "id0","id1","id2","id3","id4","id5","id6","id7","id8","id9",
                                            "id10","id11","id12","id13","id14","id15","id16","id17","id18","id19","flag"};
                            
                            cout << "#: " << pdcProcess << endl;

                            
                            for (int i=0;i<18;i++) {
                                main_colNameStr  += "`" + main_colNameVec[i] + "`,";
                            }
                            main_colNameStr_tmp = main_colNameStr;
                            
                            for (int i=0;i<18*4;i++) {
                                if      (i%4 == 0) body_colNameVec.push_back("qx_" + to_string(i/4));
                                else if (i%4 == 1) body_colNameVec.push_back("qy_" + to_string(i/4));
                                else if (i%4 == 2) body_colNameVec.push_back("qz_" + to_string(i/4));
                                else if (i%4 == 3) body_colNameVec.push_back("qw_" + to_string(i/4));
                                body_colNameStr += "`" + body_colNameVec[i] + "`,";
                            }
                            for (int i=0;i<24*3;i++) {
                                if      (i%3 == 0) body_colNameVec.push_back("tx_" + to_string(i/3));
                                else if (i%3 == 1) body_colNameVec.push_back("ty_" + to_string(i/3));
                                else if (i%3 == 2) body_colNameVec.push_back("tz_" + to_string(i/3));
                                body_colNameStr += "`" + body_colNameVec[18*4+i] + "`,";
                                if (i == 24*3-1) {
                                    body_colNameStr.pop_back();
                                }
                            }
                            // ==========================================================

                            MYSQL* conn = mysql_init(NULL);
                            if( mysql_real_connect(conn, DB_HOST.c_str(), DB_USER.c_str(), DB_PASS.c_str(), DB_NAME.c_str(), PORT_ID, NULL, 0) != NULL ) {
                                cout << "web-server is successfully connected !!" << endl;
                            } //else mysql_finish_with_error(conn);

                            for(int i=0;i<_data.size();i++)
                            {
                                sleep(1);
                                //send
                                pdcProcess = i+1;

                                for (int j=0; j<18; j++) { // before first worker id
                                    main_colValueStr += "'" + to_string(_data[i][j]) + "',";
                                }
                                
                                int maxWorkerNo(5);
                                int body_start_idx(18);
                                int idx(0); int bodyNo(0);
                                bool isFirst(false);
                                for (int j=body_start_idx; j<body_start_idx+maxWorkerNo*145; j++) {
                                    int body_id = (j-body_start_idx)/145*145+body_start_idx;
                                    if (_data[i][body_id] == false) {
                                        continue;
                                    }
                                    if (j == 18 + idx * 145) {
                                        main_colNameStr += "`" + main_colNameVec[(body_start_idx)+idx] + "`,";
                                        main_colValueStr += "'" + to_string((int)_data[i][j]) + "',";
                                        body_colValueStr.push_back("");
                                        idx++;
                                        if (_data[i][body_id] == true) bodyNo++;
                                        continue;
                                    }
                                    
                                    body_colValueStr[bodyNo-1] += "'" + to_string(_data[i][j]) + "',";
                                }
                                main_colNameStr += "`flag`";
                                main_colValueStr += "'0'"; // set flag status as '0'
                                
                                
                                string sql;
                                sql = "INSERT INTO `" + DB_NAME + "`.`" + DB_TABLE + "` (" + main_colNameStr + ") VALUES (" + main_colValueStr + ");";
                               // if (mysql_query(conn, sql.c_str()) != false) mysql_finish_with_error(conn);
                                main_colNameStr = main_colNameStr_tmp;
                                main_colValueStr = "";

                                for(int j=0;j<bodyNo;j++) {
                                    body_colValueStr[j].pop_back();
                                    sql = "INSERT INTO `" + DB_NAME + "`.`" + body_TABLE[j] + "` (" + body_colNameStr + ") VALUES (" + body_colValueStr[j] + ");";
                                  //  if (mysql_query(conn, sql.c_str()) != false) mysql_finish_with_error(conn);
                                    body_colValueStr[j].clear();
                                }
                                
                            }
                            
                            //close
                            mysql_close(conn);
                            return;
                        },
                        data);
                }
                // _window->PDCsender->
            }
            ImGui::EndMenu();
        }
        // if (ImGui::Checkbox("beamOn", &RDCWindow::Instance().beamOn))
        // {
        //     if (RDCWindow::Instance().beamOn)
        //     {
        //         _window->viewer.core(0).is_animating = true;
        //         _window->viewer.core(_window->v_left).is_animating = true;
        //         _window->viewer.core(_window->v_middle).is_animating = true;
        //         _window->viewer.core(_window->v_right).is_animating = true;
        //         _window->viewer.data(_window->beam_data).set_visible(_window->show_beam, _window->v_left);
        //         _window->viewer.data(_window->patient_data).set_points(_window->B_patient1.cast<double>(), RowVector3d(1, 0, 0));

        //     }
        //     else
        //     {
        //         _window->viewer.core(0).is_animating = false;
        //         _window->viewer.core(_window->v_left).is_animating = false;
        //         _window->viewer.core(_window->v_middle).is_animating = false;
        //         _window->viewer.core(_window->v_right).is_animating = false;
        //         _window->viewer.data(_window->beam_data).set_visible(false, _window->v_left);
        //         _window->viewer.data(_window->patient_data).clear_points();
        //     }
        //     // else
        //     //     RDCWindow::Instance().viewer.core().is_animating = false;
        // }
        // ImGui::InputText("",name);
        if (!_window->stop)
        {
            if (ImGui::SmallButton("stop"))
                _window->stop = true;
        }
        else
        {
            if (ImGui::SmallButton(" run "))
                _window->stop = false;
        }
        if (ImGui::SmallButton("record"))
        {
            _window->loadNum = -1;
            _window->recording = !_window->recording;
        }
        if (_window->recordData.size() && (!_window->recording))
        {
            if (ImGui::SmallButton("replay"))
            {
                if (_window->loadNum < 0)
                    _window->loadNum = 0;
                else
                    _window->loadNum = -1;
            }
            if (ImGui::SmallButton("clear"))
            {
                _window->recordData.clear();
            }
        }
        if (_window->stop)
            ImGui::Text("program paused");
        else if (_window->recording)
            ImGui::Text("recording #%d..", _window->recordData.size());
        else if (_window->loadNum >= 0)
            ImGui::Text("loading %d/%d..", _window->loadNum, _window->recordData.size());
        if (pdcSender.joinable())
        {
            if (pdcProcess < pdcDataSize)
                ImGui::Text(" sending PDC data %d/%d..", pdcProcess, pdcDataSize);
            else
            {
                pdcSender.join();
                pdcDataSize = 0;
                pdcProcess = 0;
                cout << "PDC sender joined" << endl;
            }
        }
        ImGui::EndMainMenuBar();
    }
}

void ShowMachineStatus(bool *p_open, RDCWindow *_window)
{
    const ImGuiViewport *viewport = ImGui::GetMainViewport();
    int sx(viewport->Size.x), sy(viewport->Size.y);
    float phantomViewX = sy * 0.35;
    // ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.f, 0.f, 0.f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(110, -1));
    ImGui::SetNextWindowPos(ImVec2(sx - phantomViewX * 2 - 10, viewport->WorkPos.y + 10), ImGuiCond_Always, ImVec2(1.f, 0.f));
    ImGuiWindowFlags window_flags = ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoMove;
    if (ImGui::Begin("machine state", p_open, window_flags))
    {
        if (ImGui::CollapsingHeader("C-arm    ", ImGuiTreeNodeFlags_DefaultOpen))
        {
            if (_window->currentFrame.beamOn)
            {
                ImGui::Text("BEAM ON");
                ImGui::Text("DAP  :  %.2f Gycm2/s", _window->currentFrame.dap);
            }
            else
            {
                ImGui::Text("BEAM OFF");
                ImGui::Text("DAP  :  %.2f Gycm2", _window->currentFrame.dap);
            }
            ImGui::Text("volt.:  %d kVp", (int)_window->currentFrame.kVp);
            ImGui::Text("curr.:  %.1f mA", _window->currentFrame.mA);
            ImGui::Text("rot  :  %d deg.", (int)_window->currentFrame.cArm(0));
            ImGui::Text("ang  :  %d deg.", (int)_window->currentFrame.cArm(1));
            ImGui::Text("SID  :  %d cm", (int)_window->currentFrame.cArm(2));
            ImGui::Text("FD   :  %d cm", (int)_window->currentFrame.FD);
        }
        if (ImGui::CollapsingHeader("Bed    ", ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::Text("long. :  %d cm", (int)_window->currentFrame.bed(1));
            ImGui::Text("lat.  :  %d cm", (int)_window->currentFrame.bed(0));
            ImGui::Text("height:  %d cm", (int)_window->currentFrame.bed(2));
        }
    }
    ImGui::End();
    // ImGui::PopStyleColor();
}

void ShowViewOptions(bool *p_open, RDCWindow *_window)
{
    ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoMove;
    const ImGuiViewport *viewport = ImGui::GetMainViewport();
    ImVec2 work_pos = viewport->WorkPos;
    ImVec2 work_size = viewport->WorkSize;
    ImVec2 window_pos, window_pos_pivot;
    window_pos = ImVec2(work_pos.x, work_pos.y);
    window_pos_pivot = ImVec2(0.f, 0.f);
    ImGui::SetNextWindowPos(window_pos, ImGuiCond_Always, window_pos_pivot);
    ImGui::SetNextWindowSize(ImVec2(128, work_size.y - 236));
    if (ImGui::Begin("View options", p_open, flags))
    {
        ImGui::BulletText("RADIOLOGIST");
        static int bodyID(0);
        ImGui::SetNextItemWidth(64);
        if (ImGui::InputInt("body ID", &bodyID))
        {
            bodyID = bodyID % _window->maxNumPeople;
            _window->SetBodyID(bodyID);
            for (int id : _window->phantom_data)
            {
                if (id == _window->mainID())
                {
                    _window->viewer.data(id).show_texture = false;
                    auto phantom = _window->GetPhantom(bodyID);
                    _window->viewer.data(_window->phantomAcc_data).set_mesh(phantom->V, phantom->F);
                    continue;
                }
                _window->viewer.data(id).set_colors(RowVector4d(0.2, 0.2, 0.2, 0.2));
                _window->viewer.data(id).show_texture = true;
                _window->viewer.data(id).clear_points();
                _window->viewer.data(id).clear_edges();
            }
        }
        static bool show_C(_window->show_C), show_BE(_window->show_BE);
        if (ImGui::Checkbox("joint", &show_C))
        {
            _window->show_C = show_C;
            if (!show_C)
                _window->viewer.data(_window->mainID()).clear_points();
        }
        if (ImGui::Checkbox("skeleton", &show_BE))
        {
            _window->show_BE = show_BE;
            if (!show_BE)
                _window->viewer.data(_window->mainID()).clear_edges();
        }
        ImGui::BulletText("PATIENT");
        static int patientOpt(1);
        if (ImGui::RadioButton("none", (patientOpt == 0)))
        {
            patientOpt = 0;
            _window->viewer.data(_window->patient_data).is_visible = false;
        }
        if (ImGui::RadioButton("wire", (patientOpt == 1)))
        {
            patientOpt = 1;
            _window->viewer.data(_window->patient_data).is_visible |= _window->v_left;
            _window->viewer.data(_window->patient_data).show_lines = true;
            _window->viewer.data(_window->patient_data).show_faces = false;
        }
        if (ImGui::RadioButton("transparent", (patientOpt == 2)))
        {
            patientOpt = 2;
            _window->viewer.data(_window->patient_data).is_visible |= _window->v_left;
            _window->viewer.data(_window->patient_data).show_lines = false;
            _window->viewer.data(_window->patient_data).show_faces = true;
        }
        ImGui::BulletText("MACHINE");
        static bool show_cArm(true), show_leadGlass(_window->show_leadGlass), show_beam(_window->show_beam), show_table(true), show_grid(true), show_axis(false), show_sphere(false);
        if (ImGui::Checkbox("C-arm", &show_cArm))
            _window->viewer.data(_window->cArm_data).set_visible(show_cArm, _window->v_left);
        if (ImGui::Checkbox("beam", &show_beam))
        {
            _window->show_beam = show_beam;
            _window->viewer.data(_window->beam_data).set_visible(show_beam, _window->v_left);
        }
        if (ImGui::Checkbox("lead glass", &show_leadGlass))
        {
            _window->show_leadGlass = show_leadGlass;
            _window->viewer.data(_window->glass_data).set_visible(show_leadGlass, _window->v_left);
        }
        if (ImGui::Checkbox("table", &show_table))
            _window->viewer.data(_window->table_data).set_visible(show_table, _window->v_left);
        ImGui::BulletText("ACCESSORIES");
        if (ImGui::Checkbox("grid", &show_grid))
            _window->viewer.data(_window->grid_data).set_visible(show_grid, _window->v_left);
        if (ImGui::Checkbox("axis", &show_axis))
            _window->viewer.data(_window->axis_data).set_visible(show_axis, _window->v_left);

        static int sphereR(20);
        if (ImGui::Checkbox("sphere", &show_sphere))
            _window->viewer.data(_window->sphere_data).set_visible(show_sphere, _window->v_left);
        ImGui::SetNextItemWidth(64);
        if (ImGui::DragInt("sphere R", &sphereR, 0.5))
            _window->viewer.data(_window->sphere_data).set_vertices(_window->V_sphere * sphereR);
    }
    ImGui::End();
}

// void ShowStatusWindow(bool *p_open, RDCWindow *_window)
// {
//     ImGuiWindowFlags flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing;
//     const ImGuiViewport *viewport = ImGui::GetMainViewport();
//     ImGui::SetNextWindowPos(ImVec2(0.f, viewport->WorkPos.y + viewport->WorkSize.y), ImGuiCond_Always, ImVec2(0., 1.f));
//     ImGui::SetNextWindowSize(ImVec2(viewport->WorkSize.x, 235));
//     ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(8. / 255., 15. / 255., 26. / 255., 1.0f));
//     if (ImGui::Begin("Status", p_open, flags))
//     {
//         if (ImGui::BeginTabBar("StatusTabBar", ImGuiTabBarFlags_None))
//         {
//             DoseResultTab(_window);
//             DoseGraphTab();
//             ProgramLogTab();
//             ImGui::EndTabBar();
//         }
//     }
//     ImGui::PopStyleColor();
//     ImGui::End();
// }

void DoseResultWindow(RDCWindow *_window)
{
    ImGuiWindowFlags flags = ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing;
    const ImGuiViewport *viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(ImVec2(0.f, viewport->WorkPos.y + viewport->WorkSize.y), ImGuiCond_Always, ImVec2(0., 1.f));
    ImGui::SetNextWindowSize(ImVec2(viewport->WorkSize.x, 230));
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(8. / 255., 15. / 255., 26. / 255., 1.0f));
    vector<string> radiologistDose = {"Whole skin", "Right hand (palm)", "Left hand (palm)", "PSD", "Lens"};
    vector<string> patientDose = {"Whole skin", "PSD"};
    char buf[128];
    sprintf(buf, "Radiologist #%d Dose", _window->bodyID);
    bool open(true);
    if (ImGui::Begin(buf, &open, flags))
    {
        // ImGui::BulletText("Radiologist #%d Dose", _window->bodyID);
        ImGui::Columns(2, "", false);
        ImGuiTableFlags flags = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_Reorderable;
        static int graphOpt(0);
        if (ImGui::BeginTable("Dose Table", 4, flags))
        {
            ImGui::TableSetupColumn("", ImGuiTableColumnFlags_NoReorder | ImGuiTableColumnFlags_WidthFixed);
            ImGui::TableSetupColumn("Dose Rate (mGy/hr)");
            ImGui::TableSetupColumn("Cumulative Dose (uGy)");
            ImGui::TableSetupColumn("Graph");
            ImGui::TableHeadersRow();
            // //set dose
            //     cout<<4<<endl;
            auto phantom = _window->GetMainPhantomHandle();
            vector<double> rateVal = {phantom->GetAvgSkinDoseRate() * 1000. * 3600., 0, 0, phantom->GetMaxSkinDoseRate() * 1000. * 3600., 0, 0};
            vector<double> accVal = {phantom->GetAvgAccSkinDose() * 1.e6, 0, 0, phantom->GetMaxAccSkinDose() * 1.e6, 0, 0};
            for (size_t row = 0; row < radiologistDose.size(); row++)
            {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::BulletText(radiologistDose[row].c_str());
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%f", rateVal[row]);
                ImGui::TableSetColumnIndex(2);
                ImGui::Text("%f", accVal[row]);
                ImGui::TableSetColumnIndex(3);
                ImGui::PushID(row * 10 + 1);
                ImGui::RadioButton("", &graphOpt, row);
                ImGui::PopID();
            }
            ImGui::EndTable();
        }
        ImGui::NextColumn();
        ImPlot::CreateContext();
        float x_data[1000], y_data[1000];
        if (ImPlot::BeginPlot(radiologistDose[graphOpt].c_str(), ImVec2(-1, 200)))
        {
            ImPlot::PlotLine("My Line Plot", x_data, y_data, 1000);
            ImPlot::EndPlot();
        }
        ImPlot::DestroyContext();
        // ImGui::BulletText("Patient Dose");
        // if (ImGui::BeginTable("patient", 3, flags))
        // {
        //     ImGui::TableSetupColumn("", ImGuiTableColumnFlags_NoReorder | ImGuiTableColumnFlags_WidthFixed);
        //     ImGui::TableSetupColumn("Dose Rate (mGy/hr)");
        //     ImGui::TableSetupColumn("Cumulative Dose (uGy)");
        //     ImGui::TableHeadersRow();
        //     for (size_t row = 0; row < patientDose.size(); row++)
        //     {
        //         ImGui::TableNextRow();
        //         ImGui::TableSetColumnIndex(0);
        //         ImGui::BulletText(patientDose[row].c_str());
        //     }
        //     ImGui::EndTable();
        // }
        ImGui::NextColumn();
        ImGui::End();
    }
}

// void DoseGraphTab()
// {
//     static int offset = 0;
//     if (ImGui::BeginTabItem("Dose rate gragphs"))
//     {
//         static int frameTime = 2;
//         const ImGuiViewport *viewport = ImGui::GetMainViewport();
//         ImGui::SetNextItemWidth(viewport->WorkSize.x * 0.5);
//         ImGui::SliderInt("", &frameTime, 1, 10);
//         char buf[200];
//         sprintf(buf, "frame time: %d sec / plot length: %d sec", frameTime, frameTime * 100);
//         ImGui::SameLine();
//         ImGui::Text(buf);
//         static float whole_skin[100] = {};
//         static float hands[100] = {};
//         static float lens[100] = {};
//         static float patient[100] = {};
//         for (int i = 0; i < 100; i++)
//         {
//             whole_skin[i] = 0.1 * i;
//             hands[i] = 0.1 * i;
//             lens[i] = 0.1 * i;
//             patient[i] = 0.1 * i;
//         }
//         offset = ++offset % 100;
//         ImGui::Columns(2, "", false);
//         ImGui::BulletText("Whole skin dose rate (radiologist)");
//         ImGui::PlotLines("", whole_skin, 100, offset, "", 0, 10, ImVec2(viewport->WorkSize.x * 0.48, viewport->WorkSize.y * 0.085));
//         ImGui::BulletText("Hands dose rate (radiologist)");
//         ImGui::PlotLines("", hands, 100, offset, "", 0, 10, ImVec2(viewport->WorkSize.x * 0.48, viewport->WorkSize.y * 0.085));
//         ImGui::NextColumn();
//         ImGui::BulletText("Lens dose rate (radiologist)");
//         ImGui::PlotLines("", lens, 100, offset, "", 0, 10, ImVec2(viewport->WorkSize.x * 0.48, viewport->WorkSize.y * 0.085));
//         ImGui::BulletText("Patient PSD");
//         ImGui::PlotLines("", patient, 100, offset, "", 0, 10, ImVec2(viewport->WorkSize.x * 0.48, viewport->WorkSize.y * 0.085));
//         ImGui::NextColumn();
//         ImGui::EndTabItem();
//     }
// }

void ShowSettingsWindow(bool *p_open, RDCWindow *_window)
{
    ImGuiWindowFlags flags = ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize;
    const ImGuiViewport *viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowSize(ImVec2(ImGui::GetItemRectSize().x * 3, -1), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Settings", p_open, flags))
    {
        ImGui::BulletText("Phantom settings");

        // if (ImGui::Button("Load Phantom"))
        //     PhantomAnimator::Instance().LoadPhantom("./phantoms/" + phantomlist[phantomNum]);

        vector<string> names = RDCWindow::Instance().GetPhantom(0)->GetProfileNames();
        static vector<string> profileNames(names.size());
        copy(names.begin(), names.end(), profileNames.begin());

        ImGuiTableFlags flags = ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_NoBordersInBody;
        // | ImGuiTableFlags_SizingFixedFit;
        static int bfID[20], profileID[20];// idJoints[20][2];
        static float color[20][3]={2,};
        if(color[0][0]>1){
            fill(color[0], color[20], 1);
        }
        static bool blueMask[20];
        static bool trackOpt[20];
        if (ImGui::BeginTable("phantom settings", 6, flags))
        {
            ImGui::TableSetupColumn("#");
            ImGui::TableSetupColumn("BF%");
            ImGui::TableSetupColumn("profile");
            ImGui::TableSetupColumn("neck color");
            ImGui::TableSetupColumn("mask color"); // or red
            ImGui::TableSetupColumn("track");
            ImGui::TableHeadersRow();

            for (int i = 0; i < _window->maxNumPeople; i++)
            {
                ImGui::PushID(i * 10);
                ImGui::TableNextRow();
                if (ImGui::TableNextColumn())
                    ImGui::Text(std::to_string(i).c_str());
                if (ImGui::TableNextColumn())
                {
                    ImGui::SetNextItemWidth(60);
                    ImGui::Combo("BF%", &bfID[i], BFlist);
                }
                if (ImGui::TableNextColumn())
                {
                    ImGui::SetNextItemWidth(60);
                    ImGui::Combo("profile", &profileID[i], profileNames);
                }
                if (ImGui::TableNextColumn())
                {
                    auto flag = ImGuiColorEditFlags_InputHSV | ImGuiColorEditFlags_HSV|ImGuiColorEditFlags_Float|ImGuiColorEditFlags_NoSidePreview|ImGuiColorEditFlags_NoAlpha|ImGuiColorEditFlags_NoOptions|ImGuiColorEditFlags_PickerHueWheel|ImGuiColorEditFlags_NoInputs;
                    ImGui::ColorEdit3("", color[i], flag);
                }
                if (ImGui::TableNextColumn())
                {
                    ImGui::Checkbox("blue", &blueMask[i]);
                }
                    // ImGui::ColorEdit3("", color[i]);
                if (ImGui::TableNextColumn())
                    if (ImGui::Checkbox("", &trackOpt[i]))
                    {
                        int id = _window->phantom_data[i];
                        auto phantom = _window->GetPhantom(i);
                        if (trackOpt[i])
                        {
                            phantom->LoadPhantom("./phantoms/" + phantomlist[bfID[i]]);
                            phantom->ClearDose();
                            if (profileNames[profileID[i]] != "original")
                                phantom->CalibrateTo(profileNames[profileID[i]]);
                            _window->viewer.data(id).set_mesh(phantom->V, phantom->F);

                            _window->workerIdData[i] = make_pair(int(floor(color[i][0]*180.+0.5)), blueMask[i]);
                            // texture setting
                            if (id == _window->mainID())
                                _window->viewer.data(id).show_texture = false;
                            else
                            {
                                _window->viewer.data(id).set_colors(RowVector4d(0.2, 0.2, 0.2, 0.2));
                                _window->viewer.data(id).show_texture = true;
                                _window->viewer.data(id).clear_points();
                                _window->viewer.data(id).clear_edges();
                            }
                            _window->viewer.data(id).set_visible(true, _window->v_left);
                            if (id == _window->mainID())
                                _window->viewer.data(_window->phantomAcc_data).set_mesh(phantom->V, phantom->F);
                        }
                        else
                        {
                            phantom->Clear();
                            _window->viewer.data(id).is_visible = false;
                            _window->workerIdData.erase(i);
                        }
                    }
                ImGui::PopID();
            }
        }
        ImGui::EndTable();
        static string fileBuff("track.data"), loadFile("track.data");
        ImGui::InputText("read", loadFile);
        ImGui::SameLine();
        if (ImGui::Button("LOAD"))
        {
            ifstream ifs(loadFile);
            for (int i = 0; i < _window->maxNumPeople; i++)
            {
                int id;
                string bfData, profile;
                ifs >> id >> bfData >> profile >> color[id][0]>> color[id][1]>> color[id][2] >>blueMask[id];
                color[id][0] /= 180.;
                for (int b = 0; b < BFlist.size(); b++)
                    if (BFlist[b] == bfData)
                        bfID[i] = b;
                for (int p = 0; p < profileNames.size(); p++)
                    if (profileNames[p] == profile)
                        profileID[i] = p;
            }
            ifs.close();
        }

        ImGui::InputText("write", fileBuff);
        ImGui::SameLine();
        if (ImGui::Button("SAVE"))
        {
            ofstream ofs(fileBuff);
            for (int i = 0; i < _window->maxNumPeople; i++)
                ofs << i << "\t" << BFlist[bfID[i]] << "\t" << profileNames[profileID[i]]
                    << "\t"<< color[i][0]*180 <<"\t"<<color[i][1]<<"\t" << color[i][2]<<"\t" << int(blueMask[i])<< endl;
            ofs.close();
        }

        ImGui::BulletText("Server machine settings");
        static int serverPort(30303);
        ImGui::SetNextItemWidth(ImGui::GetWindowSize().x * 0.4);
        ImGui::InputInt("Server port", &serverPort);
        ImGui::SameLine();
        static bool listen(false);
        if (ImGui::Checkbox("listen", &listen))
        {
            if (listen)
            {
                Communicator::Instance().StartServer(serverPort);
                Communicator::Instance().StartWorkers();
            }
            else
                Communicator::Instance().CloseServer();
        }
        ImGui::BulletText("Client machine settings");
        static string ip; //, port("22");
        ImGui::SetNextItemWidth(ImGui::GetWindowSize().x * 0.4);
        ImGui::InputText("ip", ip);
        // ImGui::SameLine();
        // ImGui::SetNextItemWidth(ImGui::GetWindowSize().x * 0.2);
        // ImGui::InputText("port", port);
        static bool ocrChk, bedChk, glassChk, motionChk;
        ImGui::Checkbox("ocr", &ocrChk);
        ImGui::SameLine();
        ImGui::Checkbox("bed", &bedChk);
        ImGui::SameLine();
        ImGui::Checkbox("glass", &glassChk);
        ImGui::SameLine();
        ImGui::Checkbox("motion", &motionChk);
        ImGui::SameLine();
        if (ImGui::Button("Establish Connection"))
        {
            if (!listen)
                cout << "activate listen button first" << endl;
            else
            {
                int option = 8 * (int)motionChk + 4 * (int)glassChk + 2 * (int)bedChk + 1 * (int)ocrChk;
                if(_window->workerIdData.size()) Communicator::Instance().StartWorker(ip, option, _window->workerIdData);
                else Communicator::Instance().StartWorker(ip, option);
            }
        }
        // | ImGuiTableFlags_SizingFixedFit;
        if (ImGui::BeginTable("worker machines", 8, flags))
        {
            ImGui::TableSetupColumn("ID");
            ImGui::TableSetupColumn("IP               ");
            // ImGui::TableSetupColumn("port");
            ImGui::TableSetupColumn("ocr");
            ImGui::TableSetupColumn("bed");
            ImGui::TableSetupColumn("glass");
            ImGui::TableSetupColumn("motion");
            ImGui::TableSetupColumn("status");
            ImGui::TableSetupColumn("delete");
            ImGui::TableHeadersRow();
            int del(-1), id(9);
            for (auto iter : Communicator::Instance().workerData)
            {
                ImGui::PushID(id++);
                ImGui::TableNextRow();
                if (ImGui::TableNextColumn())
                    ImGui::Text(std::to_string(iter.first).c_str());
                if (ImGui::TableNextColumn())
                    ImGui::Text(get<0>(iter.second).c_str());
                int opt = get<1>(iter.second);
                if (ImGui::TableNextColumn() && (opt & 1))
                    ImGui::Text("*");
                if (ImGui::TableNextColumn() && (opt & 2))
                    ImGui::Text("*");
                if (ImGui::TableNextColumn() && (opt & 4))
                    ImGui::Text("*");
                if (ImGui::TableNextColumn() && (opt & 8))
                    ImGui::Text("*");
                if (ImGui::TableNextColumn() && Communicator::Instance().GetDelay(iter.first) < 0.1)
                    ImGui::Text("*");
                if (ImGui::TableNextColumn() && ImGui::SmallButton("delete"))
                    del = iter.first;
                ImGui::PopID();
            }
            ImGui::EndTable();
            if (del >= 0)
                Communicator::Instance().DeleteWorker(del);
        }
        ImGui::PushStyleVar(ImGuiStyleVar_IndentSpacing, viewport->WorkSize.x * 0.5 - ImGui::GetFontSize() * 10.f);
        ImGui::PopStyleVar();
    }
    ImGui::End();
}

void ShowManualSettingPopup(bool *p_open, RDCWindow *_window)
{
    ImGuiWindowFlags flags = ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize;
    const ImGuiViewport *viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowSize(ImVec2(ImGui::GetItemRectSize().x * 2, -1), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Manual settings", p_open, flags))
    {
        static float dap;
        static int kvp(80), rotation(0), angulation(0);
        static int bedPos[3];
        static bool useManualBeam(false);
        Matrix3f rot = Matrix3f::Identity();
        ImGui::BulletText("C-arm");
        ImGui::SameLine();
        if (ImGui::Button("Set beam"))
            _window->SetMap(kvp, rotation, angulation);
        ImGui::SameLine();
        if (ImGui::Checkbox("use manual beam", &useManualBeam))
            _window->useManualBeam = useManualBeam;
        if (ImGui::InputFloat("DAP(Gycm2)", &dap, 1))
            _window->manualDAP = dap;

        ImGui::InputInt("peak voltage (kVp)", &kvp, 5);
        if (ImGui::DragInt("RAO(-) / LAO(+)", &rotation, 1, -180, 180))
        {
            rot = _window->GetCarmRotation(rotation, angulation);
            _window->viewer.data(_window->cArm_data).set_vertices(_window->V_cArm * rot.cast<double>().transpose());
            _window->viewer.data(_window->beam_data).set_vertices(_window->V_beam * rot.cast<double>().transpose());
            _window->viewer.data(_window->cArm_data).compute_normals();
            _window->viewer.data(_window->beam_data).compute_normals();
            _window->CalculateSourceFacets(RowVector3f(bedPos[0], bedPos[1], bedPos[2]), rot);
            // if (RDCWindow::Instance().beamOn)
            _window->viewer.data(_window->patient_data).set_points(_window->B_patient1.cast<double>(), RowVector3d(1, 0, 0));
        }
        if (ImGui::DragInt("CRAN(-) / CAUD(+)", &angulation, 1, -90, 90))
        {
            rot = _window->GetCarmRotation(rotation, angulation);
            _window->viewer.data(_window->cArm_data).set_vertices(_window->V_cArm * rot.cast<double>().transpose());
            _window->viewer.data(_window->beam_data).set_vertices(_window->V_beam * rot.cast<double>().transpose());
            _window->viewer.data(_window->cArm_data).compute_normals();
            _window->viewer.data(_window->beam_data).compute_normals();
            _window->CalculateSourceFacets(RowVector3f(bedPos[0], bedPos[1], bedPos[2]), rot);
            // if (RDCWindow::Instance().beamOn)
            _window->viewer.data(_window->patient_data).set_points(_window->B_patient1.cast<double>(), RowVector3d(1, 0, 0));
        }
        ImGui::BulletText("Glass");
        static int glassPos[3], glassRot[3];
        if (ImGui::DragInt3("glass pos.", glassPos))
        {
            MatrixXd TT(4, 3);
            TT.topRows(3) = (AngleAxisd((double)glassRot[0] / 180. * igl::PI, Vector3d(1, 0, 0)) * AngleAxisd((double)glassRot[1] / 180. * igl::PI, Vector3d(0, 1, 0)) * AngleAxisd((double)glassRot[2] / 180. * igl::PI, Vector3d(0, 0, 1))).matrix().transpose();
            TT.bottomRows(1) << glassPos[0], glassPos[1], glassPos[2];
            _window->viewer.data(_window->glass_data).set_vertices(_window->V_glass.rowwise().homogeneous() * TT);
        }
        if (ImGui::DragInt3("glass rot.", glassRot))
        {
            MatrixXd TT(4, 3);
            TT.topRows(3) = (AngleAxisd((double)glassRot[0] / 180. * igl::PI, Vector3d(1, 0, 0)) * AngleAxisd((double)glassRot[1] / 180. * igl::PI, Vector3d(0, 1, 0)) * AngleAxisd((double)glassRot[2] / 180. * igl::PI, Vector3d(0, 0, 1))).matrix().transpose();
            TT.bottomRows(1) << glassPos[0], glassPos[1], glassPos[2];
            _window->viewer.data(_window->glass_data).set_vertices(_window->V_glass.rowwise().homogeneous() * TT);
        }
        if (ImGui::Button("  Print glass data (V.rowwise().homogeneous()*TT)  "))
        {
            MatrixXd TT(4, 3);
            TT.topRows(3) = (AngleAxisd((double)glassRot[0] / 180. * igl::PI, Vector3d(1, 0, 0)) * AngleAxisd((double)glassRot[1] / 180. * igl::PI, Vector3d(0, 1, 0)) * AngleAxisd((double)glassRot[2] / 180. * igl::PI, Vector3d(0, 0, 1))).matrix().transpose();
            TT.bottomRows(1) << glassPos[0], glassPos[1], glassPos[2];
            cout << endl
                 << TT << endl;
        }
        ImGui::BulletText("Bed");
        if (ImGui::DragInt3("bed trans.", bedPos))
        {
            RowVector3d trans(bedPos[0], bedPos[1], bedPos[2]);
            _window->viewer.data(_window->table_data).set_vertices(_window->V_table.rowwise() + trans);
            _window->viewer.data(_window->patient_data).set_vertices(_window->V_patient.rowwise() + trans);
            _window->CalculateSourceFacets(RowVector3f(bedPos[0], bedPos[1], bedPos[2]), rot.cast<float>());
            // if (RDCWindow::Instance().beamOn)
            _window->viewer.data(_window->patient_data).set_points(_window->B_patient1.cast<double>(), RowVector3d(1, 0, 0));
        }
    }
    ImGui::End();
}

void ColorInfoWindow(float min, float max, string unit)
{
    float fs = ImGui::GetFontSize() * ImGui::GetFont()->Scale;
    ImDrawList *draw_list = ImGui::GetWindowDrawList();
    ImVec2 p0 = ImGui::GetCursorScreenPos();
    draw_list->AddRectFilled(p0, ImVec2(p0.x + fs, p0.y + fs), IM_COL32(255, 192, 0, 255));
    ImGui::Dummy(ImVec2(fs, fs));
    ImGui::SameLine();
    ImGui::Text("%1.f %s", min, unit.c_str());
    draw_list->AddRectFilled(ImVec2(p0.x, p0.y + fs + ImGui::GetStyle().ItemSpacing.y), ImVec2(p0.x + fs, p0.y + 2 * fs + ImGui::GetStyle().ItemSpacing.y), IM_COL32(32, 56, 100, 255));
    ImGui::Dummy(ImVec2(fs, fs));
    ImGui::SameLine();
    ImGui::Text("%1.f %s", max, unit.c_str());
}

void ShowColorInfoPopUp(bool *p_open)
{
    const ImGuiViewport *viewport = ImGui::GetMainViewport();
    int sx(viewport->Size.x), sy(viewport->Size.y);
    float phantomViewX = sy * 0.35;
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.f, 0.f, 0.f, 0.5f));
    ImGui::SetNextWindowPos(ImVec2(sx - phantomViewX * 2 - 10, sy - 245), ImGuiCond_Always, ImVec2(1.f, 1.f));
    ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoMove;
    if (ImGui::Begin("RadiolRate_color", p_open, window_flags))
        ColorInfoWindow(0, 10, "uGy/hr");
    ImGui::End();
    ImGui::SetNextWindowPos(ImVec2(sx - phantomViewX - 10, sy - 245), ImGuiCond_Always, ImVec2(1.f, 1.f));
    if (ImGui::Begin("RadiolAcc_color", p_open, window_flags))
        ColorInfoWindow(3, 20, "uGy");
    ImGui::End();
    ImGui::SetNextWindowPos(ImVec2(sx - 10, sy - 245), ImGuiCond_Always, ImVec2(1.f, 1.f));
    if (ImGui::Begin("Patient_color", p_open, window_flags))
        ColorInfoWindow(10, 50, "mGy");
    ImGui::End();
    ImGui::PopStyleColor();
}

void ShowProgramInformationPopUp(bool *p_open)
{
    ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoMove;
    const ImGuiViewport *viewport = ImGui::GetMainViewport();
    ImVec2 work_pos = viewport->WorkPos;
    ImVec2 work_size = viewport->WorkSize;
    ImVec2 window_pos, window_pos_pivot;
    window_pos = ImVec2(work_pos.x + work_size.x * 0.5f, work_pos.y + work_size.y * 0.5f);
    window_pos_pivot = ImVec2(0.5f, 0.5f);
    ImGui::SetNextWindowPos(window_pos, ImGuiCond_Always, window_pos_pivot);

    if (ImGui::Begin("info. window", p_open, window_flags))
    {
        ImGui::Text("Author : Haegin Han\n"
                    "Info : RDC module of DCIR system\n"
                    "(right-click to close)");
        if (ImGui::BeginPopupContextWindow())
        {
            if (p_open && ImGui::MenuItem("Close"))
                *p_open = false;
            ImGui::EndPopup();
        }
    }
    ImGui::End();
}

vector<float> RDCWindow::SetAframeData(DataSet data, float frameTimeInMSEC)
{
    vector<float> frameData(dataSize, 0);
    frameData[0] = frameTimeInMSEC;
    frameData[1] = data.kVp;
    frameData[2] = data.mA;
    frameData[3] = data.dap;
    frameData[4] = data.cArm(0);
    frameData[5] = data.cArm(1);
    frameData[6] = data.cArm(2);
    frameData[7] = data.bed(0);
    frameData[8] = data.bed(1);
    frameData[9] = data.bed(2);
    frameData[10] = data.glassChk;
    Quaterniond q(data.glass_aff.rotation());
    frameData[11] = q.x();
    frameData[12] = q.y();
    frameData[13] = q.z();
    frameData[14] = q.w();
    frameData[15] = data.glass_aff.translation().x();
    frameData[16] = data.glass_aff.translation().y();
    frameData[17] = data.glass_aff.translation().z();
    int pos = 18;
    for (int i = 0; i < maxNumPeople; i++)
    {
        if (data.bodyMap.find(i) == data.bodyMap.end())
            continue;
        int num = pos + i * 145;
        frameData[num++] = 1;
        for (int b = 0; b < data.bodyMap[i].posture.size(); b++)
        {
            frameData[num++] = data.bodyMap[i].posture[b].x();
            frameData[num++] = data.bodyMap[i].posture[b].y();
            frameData[num++] = data.bodyMap[i].posture[b].z();
            frameData[num++] = data.bodyMap[i].posture[b].w();
        }
        for (int c = 0; c < data.bodyMap[i].jointC.rows(); c++)
        {
            frameData[num++] = data.bodyMap[i].jointC(c, 0);
            frameData[num++] = data.bodyMap[i].jointC(c, 1);
            frameData[num++] = data.bodyMap[i].jointC(c, 2);
        }
    }
    return frameData;
}

DataSet RDCWindow::ReadAframeData(vector<float> frameData)
{
    DataSet data;
    if ((frameData.size() != dataSize) && (frameData.size() != dataSize + 1))
    {
        cout << "Wrong frame data size!" << endl;
        return data;
    }
    data.time = frameData[0];
    data.kVp = frameData[1];
    data.mA = frameData[2];
    data.dap = frameData[3];
    data.cArm(0) = frameData[4];
    data.cArm(1) = frameData[5];
    data.cArm(2) = frameData[6];
    data.bed(0) = frameData[7];
    data.bed(1) = frameData[8];
    data.bed(2) = frameData[9];
    data.glassChk = frameData[10];
    Affine3d glassAff = Affine3d::Identity();
    glassAff.rotate(Quaterniond(frameData[11], frameData[12], frameData[13], frameData[14]));
    glassAff.translate(Vector3d(frameData[15], frameData[16], frameData[17]));
    data.glass_aff = glassAff;
    int pos = 18;
    for (int i = 0; i < 5; i++)
    {
        int num = pos + i * 145;
        if (!frameData[num++])
            continue;
        for (int b = 0; b < data.bodyMap[i].posture.size(); b++)
        {
            data.bodyMap[i].posture[b].x() = frameData[num++];
            data.bodyMap[i].posture[b].y() = frameData[num++];
            data.bodyMap[i].posture[b].z() = frameData[num++];
            data.bodyMap[i].posture[b].w() = frameData[num++];
        }
        for (int c = 0; c < data.bodyMap[i].jointC.rows(); c++)
        {
            data.bodyMap[i].jointC(c, 0) = frameData[num++];
            data.bodyMap[i].jointC(c, 1) = frameData[num++];
            data.bodyMap[i].jointC(c, 2) = frameData[num++];
        }
    }
    return data;
}

// void RDCWindow::SendPDCData()
// {
//     for(PDCprocess=0;PDCprocess<recordData.size();PDCprocess++)
//     {
//         cout<<PDCprocess<<endl;
//         // sleep(0.4);
//     }
//     return;
// }