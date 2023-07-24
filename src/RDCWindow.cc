#include "RDCWindow.hh"
#include <igl/readPLY.h>
#include <igl/writePLY.h>
#include <backends/imgui_impl_glfw.h>
#include <implot.h>
#include <igl/Timer.h>
#include <mysql/mysql.h>

RDCWindow::RDCWindow() : bodyID(0), stop(false), recording(false), loadNum(-1), playOnce(false),show_beam(true), 
show_leadGlass(true), bodyDelayTol(10), animBeamOnOnly(true), externalOBJ_data(-1), show_externalDose(false), force_manualBeam(false), use_curtain(true)
{
    mapContainer = new MapContainer(81);
    manualFrame.kVp = 80;
    manualFrame.dap = 0.001;
    manualFrame.bed = RowVector3f(0, 0, 0);
    manualFrame.cArm = RowVector3f(0, 0, 120);
    manualFrame.FD = 48;
    manualFrame.beamOn = false;
    extraChk = true;
    MatrixXd base(4, 2);
    base<<-1, 1, -1, -1, 1, -1, 1, 1;
    fdSize[16] = base * 5.5;
    fdSize[19] = base * 6.75;
    fdSize[22] = base * 9;
    fdSize[27] = base * 9.5;
    fdSize[31] = base * 11;
    fdSize[37] = base * 13;
    fdSize[42] = base * 15;
    fdSize[48] = base.array().rowwise() * Array2d(19, 15).transpose();
}

void RDCWindow::SetConfig(Config config)
{
    if(extraPhantom != nullptr || indivPhantoms.size())
    {
        cerr << "[ERROR] RDCWindow::SetConfig was called more than once" << endl;
        exit(100);
    }
    SetPhantoms(config.GetProfileFile());
}

static void MainMenuBar(RDCWindow *_window);
static void ShowMachineStatus(bool *p_open, RDCWindow *_window);
static void ShowViewOptions(bool *p_open, RDCWindow *_window);
static void ShowSettingsWindow(bool *p_open, RDCWindow *_window);
static void ShowManualSettingPopup(bool *p_open, RDCWindow *_window);
static void DoseResultWindow(RDCWindow *_window);
static void ShowProgramInformationPopUp(bool *p_open);
void ColorInfoWindow(float min, float max, string unit);
static void ShowColorInfoPopUp(bool *p_open, RDCWindow *_window);

void RDCWindow::InitializeGUI()
{
    mainMenu.callback_draw_viewer_window = [this]() -> bool
    {
        MainMenuBar(this);
        return true;
    };
    viewer.plugins.push_back(&plugin);
    plugin.widgets.push_back(&mainMenu);

    // multi view
    viewer.callback_init = [&](igl::opengl::glfw::Viewer &_viewer)
    {
        int w, h;
        glfwGetWindowSize(_viewer.window, &w, &h);
        float phantomViewX = h * 0.35;
        // radiolRate
        // _viewer.core().viewport = Eigen::Vector4f(0, h * 0.29, w - phantomViewX * 2, h * 0.71);
        _viewer.core().viewport = Eigen::Vector4f(0, h * 0.29, w - phantomViewX, h - 230);
        this->v_left = viewer.core_list[0].id;
        _viewer.core(this->v_left).background_color = Eigen::Vector4f(0.95, 0.95, 0.95, 1.);
        _viewer.core(this->v_left).camera_center = Vector3f(0, 0, 10);
        _viewer.core(this->v_left).camera_up = Vector3f(0, 0, 1);
        _viewer.core(this->v_left).camera_eye = Vector3f(60, 0, 0);
        // radiolAcc
        // this->v_middle = viewer.append_core(Eigen::Vector4f(w - phantomViewX * 2, h * 0.29, phantomViewX, h * 0.71));
        this->v_middle = viewer.append_core(Eigen::Vector4f(w - phantomViewX, h * 0.29, phantomViewX, h - 230));
        _viewer.core(this->v_middle).background_color = Eigen::Vector4f(1., 1., 1., 0.);
        _viewer.core(this->v_middle).camera_center = Vector3f(0, 0, 0);
        _viewer.core(this->v_middle).camera_up = Vector3f(0, -1, 0);
        _viewer.core(this->v_middle).camera_dfar = 300;
        _viewer.core(this->v_middle).camera_eye = Vector3f(0, 0, -250);
        _viewer.core(this->v_middle).rotation_type = igl::opengl::ViewerCore::ROTATION_TYPE_NO_ROTATION;
        // patient
        // this->v_right = viewer.append_core(Eigen::Vector4f(w - phantomViewX, h * 0.29, phantomViewX, h * 0.71));
        // _viewer.core(this->v_right).background_color = Eigen::Vector4f(82. / 255., 82. / 255., 82. / 255., 0.);

        // visiblity
        // _viewer.data(this->apron_data).is_visible = 0;
        _viewer.data(this->patient_data).is_visible = 0;
        _viewer.data(this->table_data).is_visible = 0;
        _viewer.data(this->cArm_data).is_visible = 0;
        _viewer.data(this->beam_data).is_visible = 0;
        _viewer.data(this->glass_data).is_visible = 0;
        _viewer.data(this->grid_data).is_visible = 0;
        _viewer.data(this->axis_data).is_visible = 0;
        _viewer.data(this->phantomAcc_data).is_visible = 0;
        _viewer.data(this->extra_data).is_visible = 0;
        _viewer.data(this->sphere_data).is_visible = 0;
        // for (int id : this->phantom_data)
        //     _viewer.data(id).is_visible = 0;

        // _viewer.data(this->phantom_data).is_visible |= this->v_left;
        // _viewer.data(this->apron_data).is_visible   |= this->v_left;
        // if(this->extraChk) _viewer.data(this->extra_data).is_visible |= this->v_left;
        _viewer.data(this->patient_data).is_visible |= this->v_left;
        _viewer.data(this->table_data).is_visible |= this->v_left;
        _viewer.data(this->cArm_data).is_visible |= this->v_left;
        _viewer.data(this->grid_data).is_visible |= this->v_left;
        _viewer.data(this->phantomAcc_data).is_visible |= this->v_middle;

        // _viewer.core(v_left).animation_max_fps = 5;

        _viewer.selected_core_index = this->v_left;

        _viewer.core(this->v_left).set_rotation_type(igl::opengl::ViewerCore::ROTATION_TYPE_TRACKBALL);
        _viewer.core(this->v_left).animation_max_fps = 5;
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
    SetMeshes(string(getenv("VIR_PHANTOM_DIR")) + "rdc/");

    // callback functions
    viewer.callback_pre_draw = bind(&RDCWindow::PreDrawFunc, this, placeholders::_1);
    viewer.callback_post_draw = [&](igl::opengl::glfw::Viewer &_viewer) -> bool
    {
        _viewer.core(0).is_animating = true;
        return true;
    };

    // set everything always animating
    viewer.core(0).is_animating = true;
    viewer.core(v_left).is_animating = true;
    viewer.core(v_middle).is_animating = true;
}
static int boneID(0);
static bool showAxis(false);
bool RDCWindow::PreDrawFunc(igl::opengl::glfw::Viewer &_viewer)
{
    if (stop && !playOnce)
        return false;
    playOnce = false;
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
    //***SET FRAME***//
    if (!keepCurr)
    {
        if (loadNum >= 0) // loading
        {
            if (loadNum == recordData.size())
            {
                loadNum = -1;
                return false;
            }
            currentFrame = ReadAframeData(recordData[loadNum++]);

            // currentFrame.bed = RowVector3f(-3, -10, 5);
            currentFrame.FD = 48;            
            if(force_manualBeam || force_withDAP)
            {
                if(force_manualBeam)
                {
                    currentFrame.kVp = manualFrame.kVp;
                    currentFrame.FD = manualFrame.FD;
                }
                currentFrame.beamOn = manualFrame.beamOn;
                currentFrame.dap = manualFrame.dap;
                // currentFrame.time = clock();
            }
            recordData[loadNum-1] = SetAframeData(currentFrame, currentFrame.time); //move into if.

            if (currentFrame.beamOn)
                lastFrameT = currentFrame.time;
            else if (lastFrameT > 0) // last frame
                lastFrameT = -1;
            else currentFrame.time = 0;
        }
        else
        { // not loading
            Communicator::Instance().SetCurrentFrame(currentFrame, bodyDelayTol);
            // manual settings
            if (manualFrame.beamOn)
            {
                currentFrame.beamOn = true;
                currentFrame.cArm = manualFrame.cArm;
                currentFrame.kVp = manualFrame.kVp;
                currentFrame.FD = manualFrame.FD;
                currentFrame.dap = manualFrame.dap;
                currentFrame.bed = manualFrame.bed;
                currentFrame.time = clock();
            }
            if (manualFrame.glassChk)
            {
                currentFrame.glassChk = true;
                currentFrame.glass_aff = manualFrame.glass_aff;
            }

            // frame time settings
            float frameTimeInMSEC(0);
            if (currentFrame.beamOn)
            {
                if (lastFrameT > 0)
                    frameTimeInMSEC = float(currentFrame.time - lastFrameT) / CLOCKS_PER_SEC * 1000;
                lastFrameT = currentFrame.time;
            }
            else if (lastFrameT > 0) // last frame
            {
                frameTimeInMSEC = currentFrame.time - lastFrameT;
                lastFrameT = -1;
            }
            currentFrame.time = frameTimeInMSEC;

            if (recording)
                recordData.push_back(SetAframeData(currentFrame, frameTimeInMSEC));
        }
    }
    keepCurr = false;

    //***COMPARE WITH PREVIDOUS FRAME && PREPARE DOSE MAP && MACHINE ANIMATION***//
    bool changeMap(false);
    MatrixXf rot = GetCarmRotation(currentFrame.cArm(0), currentFrame.cArm(1));

    if (prevFrame.cArm != currentFrame.cArm || prevFrame.FD != currentFrame.FD)
    {
        MatrixXd cArmTmp = V_cArm;
        cArmTmp.bottomRows(det_v).col(2) = V_cArm.bottomRows(det_v).col(2).array() + (currentFrame.cArm(2) - 120);
        _viewer.data(cArm_data).set_vertices(cArmTmp * rot.cast<double>().transpose());
        MatrixXd beamTmp = V_beam;
        beamTmp.bottomRows(4).col(2) = V_beam.bottomRows(4).col(2).array() + (currentFrame.cArm(2) - 120);
        if (fdSize.find(currentFrame.FD) == fdSize.end())
            cout << "wrong FD!" << endl;
        else
            beamTmp.bottomRows(4).leftCols(2) = fdSize[currentFrame.FD];
        _viewer.data(beam_data).set_vertices(beamTmp * rot.cast<double>().transpose());
        _viewer.data(cArm_data).compute_normals();
        _viewer.data(beam_data).compute_normals();
        changeMap = true;
    }

    if (prevFrame.bed != currentFrame.bed)
    {
        _viewer.data(patient_data).set_vertices(V_patient.rowwise() + currentFrame.bed.cast<double>());
        _viewer.data(table_data).set_vertices(V_table.rowwise() + currentFrame.bed.cast<double>());
        changeMap = true;
    }

    if (changeMap)
    {
        CalculateSourceFacets(currentFrame.bed, rot);
        mapContainer->SetCArm(currentFrame.kVp, currentFrame.cArm, currentFrame.bed, currentFrame.FD);
    }
    prevFrame = currentFrame;

    if (currentFrame.glassChk)
    {
        if (show_leadGlass)
            _viewer.data(glass_data).is_visible |= v_left;
        _viewer.data(glass_data).set_vertices((V_glass.rowwise().homogeneous() * currentFrame.glass_aff.matrix().transpose()).rowwise().hnormalized());
        _viewer.data(glass_data).compute_normals();
    }
    else
        _viewer.data(glass_data).is_visible = 0;

    if (currentFrame.time>0)
    {
        if (show_beam)
            _viewer.data(beam_data).set_visible(true, v_left);
        _viewer.data(patient_data).set_points(B_patient1.cast<double>(), RowVector3d(1, 0, 0));
    }
    else
    {
        _viewer.data(patient_data).clear_points();
        _viewer.data(beam_data).set_visible(false, v_left);
        if (animBeamOnOnly)
            return false;
    }

    // if (_viewer.core(v_left).is_animating)
    // {

    //***PHANTOM ANIMATION***//
    if (!currentFrame.bodyMap.size() && !show_externalDose)
    {
        for (auto iter : phantomDataID)
        {
            _viewer.data(iter.second.first).is_visible = 0;
            _viewer.data(iter.second.second).is_visible = 0;
            if(indivPhantoms.find(iter.first)!= indivPhantoms.end())
                indivPhantoms[iter.first]->SetZeroDose();
        }
        _viewer.data(extra_data).is_visible = 0;
        return false;
    }

    MatrixXd P;
    // posture deform & shadow generation
    igl::embree::EmbreeIntersector ei;
    MatrixXi extraF;
    MatrixXd extraV;
    VectorXd dose;

    for (auto id : currentFrame.bodyMap)
    {
        PhantomAnimator *phantom;
        if (indivPhantoms.find(id.first) != indivPhantoms.end())
            phantom = indivPhantoms[id.first];
        else // extras
            phantom = extraPhantom;
        MatrixXd C;
        int numF = phantom->F.rows();
        int numV = phantom->V.rows();
        if (phantom->useApron)
            phantom->Animate(id.second.posture, id.second.jointC, true);
        else
            phantom->Animate(id.second.posture, id.second.jointC);
        if (phantom == extraPhantom)
        {
            extraF.conservativeResize(extraF.rows() + numF, 3);
            extraF.bottomRows(numF) = phantom->F.array() + extraV.rows();
            extraV.conservativeResize(extraV.rows() + numV, 3);
            extraV.bottomRows(numV) = phantom->U;
        }
    }
    if ((currentFrame.time>0) && B_patient1.rows())
    {
        MatrixXf totalV = InitTree(ei, extraV, extraF);
        dose = VectorXd::Zero(totalV.rows());
        dose = GenerateShadow(ei, totalV);
        mapContainer->SetDoseRate(totalV, dose, currentFrame.dap);
    }

    // visualization & dose calculation
    if (extraChk && extraV.rows() > 0) // extra
    {
        if (_viewer.data(extra_data).F.rows() != extraF.rows())
        {
            _viewer.data(extra_data).clear();
            _viewer.data(extra_data).set_mesh(extraV, extraF);
            _viewer.data(extra_data).set_colors(RowVector4d(0.4, 0.4, 0.4, 0.2));
        }
        else
            _viewer.data(extra_data).set_vertices(extraV);
        _viewer.data(extra_data).is_visible |= v_left;
    }
    else
        _viewer.data(extra_data).is_visible = 0;

    int vNum(0);
    for (auto iter : indivPhantoms)
    {
        if (currentFrame.bodyMap.find(iter.first) == currentFrame.bodyMap.end())
        {
            _viewer.data(phantomDataID[iter.first].first).is_visible = 0;
            _viewer.data(phantomDataID[iter.first].second).is_visible = 0;
            indivPhantoms[iter.first]->SetZeroDose();
            continue;
        }
        _viewer.data(phantomDataID[iter.first].first).is_visible |= v_left;
        _viewer.data(phantomDataID[iter.first].first).set_vertices(iter.second->U);
        if ((currentFrame.time>0) && B_patient1.rows())
        {
            int num;
            if (iter.second->useApron)
                num = iter.second->outApron.rows();
            else
                num = iter.second->U.rows();
            VectorXd eyeD;
            MatrixXf eyeP = currentFrame.bodyMap[iter.first].jointC.block(22, 0, 2, 3).cast<float>();
            eyeD = GenerateShadow(ei, eyeP);
            mapContainer->SetDoseRate(eyeP, eyeD, currentFrame.dap);
            iter.second->SetDose(dose.block(vNum, 0, num, 1), eyeD, currentFrame.time);
            vNum += num;
        }
        else
            iter.second->SetZeroDose();
        if (iter.second->useApron)
        {
            _viewer.data(phantomDataID[iter.first].second).is_visible |= v_left;
            _viewer.data(phantomDataID[iter.first].second).set_vertices(iter.second->U_apron);
        }
        if (iter.first == bodyID)
        {
            if (show_C)
                _viewer.data(phantomDataID[iter.first].first).set_points(currentFrame.bodyMap[iter.first].jointC, RowVector3d(1, 0.813, 0));
            if (show_BE)
                _viewer.data(phantomDataID[iter.first].first).set_edges(iter.second->C1, iter.second->BE, RowVector3d(70. / 255., 252. / 255., 167. / 255.));
            _viewer.data(phantomDataID[iter.first].first).compute_normals();
            if (iter.second->useApron)
                _viewer.data(phantomDataID[iter.first].second).compute_normals();
            if ((currentFrame.time>0) && B_patient1.rows())
                _viewer.data(phantomAcc_data).set_data(iter.second->accD, igl::COLOR_MAP_TYPE_PARULA, 50);
            _viewer.data(phantomDataID[iter.first].first).set_data(iter.second->rateD, igl::COLOR_MAP_TYPE_PARULA, 50);
        }
    }
    if(show_externalDose)
    {
        _viewer.data(externalOBJ_data).set_visible(true, v_left);
        if(currentFrame.time>0) externalD = dose.block(vNum, 0, _viewer.data(externalOBJ_data).V.rows(), 1);
        else externalD.setZero();
        _viewer.data(externalOBJ_data).set_data(externalD, igl::COLOR_MAP_TYPE_PARULA, 50);
    }

    // }
    if(loadNum>=0 && loadNum<recordVal.rows())
    {
        recordVal(loadNum, 0) = indivPhantoms[bodyID]->GetAvgSkinDoseRate();
        recordVal(loadNum, 1) = indivPhantoms[bodyID]->GetLeftHandDoseRate();
        recordVal(loadNum, 2) = indivPhantoms[bodyID]->GetRightHandDoseRate();
        recordVal(loadNum, 3) = indivPhantoms[bodyID]->GetMaxSkinDoseRate();
        recordVal(loadNum, 4) = indivPhantoms[bodyID]->rateD_eye.sum() * 0.5;
    }
    return false;
}

void RDCWindow::SetMeshes(string dir)
{
    // radiologist phantom
    //  auto phantom  = indivPhantoms[0];
    //  viewer.data().set_mesh(phantom->U_apron, phantom->F_apron);
    // apron_data = viewer.selected_data_index;
    // viewer.append_mesh();
    // viewer.data().set_mesh(phantom->V, phantom->F);
    // main phantoms
    for(int i=0;i<numStaff;i++)
    {
        phantomDataID[i].first = viewer.selected_data_index;
        viewer.append_mesh();
        phantomDataID[i].second = viewer.selected_data_index;
        viewer.append_mesh();
    }
    phantomAcc_data = viewer.selected_data_index;
    cout<<33<<endl;

    // read other meshes
    MatrixXi F_cArm, F_patient, F_glass, F_beam, F_sphere, F_det, F_curtain;
    MatrixXd V_det, V_curtain;
    igl::readPLY(dir + "/c-arm.ply", V_cArm, F_cArm);
    igl::readPLY(dir + "/FPD.ply", V_det, F_det);
    det_v = V_det.rows();
    det_f = F_det.rows();
    F_cArm.conservativeResize(F_cArm.rows() + det_f, 3);
    F_cArm.bottomRows(det_f) = F_det.array() + V_cArm.rows();
    V_cArm.conservativeResize(V_cArm.rows() + det_v, 3);
    V_cArm.bottomRows(det_v) = V_det;
    igl::readPLY(dir + "/table.ply", V_table, F_table);
    igl::readPLY(dir + "/pbShield.ply", V_curtain, F_curtain);
    Vector3d tableCen = (V_table.colwise().maxCoeff() + V_table.colwise().minCoeff())*0.5;
    F_table.conservativeResize(F_table.rows()+2, 3);
    F_table.bottomRows(2) = F_curtain.array() + V_table.rows();
    V_table.conservativeResize(V_table.rows()+4, 3);
    V_table.bottomRows(4) = V_curtain;
    cout<<44<<endl;

    igl::readPLY(dir + "/patient.ply", V_patient, F_patient);
    // igl::readPLY(dir + "/box_phantom.ply", V_patient, F_patient);
    Vector3d pMax = V_patient.colwise().maxCoeff();
    Vector3d pMin = V_patient.colwise().minCoeff();
    V_patient = V_patient.rowwise() + RowVector3d(tableCen(0)-(pMax(0)+pMin(0))*0.5, V_table.col(1).maxCoeff() - pMax(1), V_table.col(2).maxCoeff() - pMin(2));
    cout<<(V_patient.colwise().maxCoeff() + V_patient.colwise().minCoeff())*0.5<<endl; 
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
    viewer.data().set_mesh(V_cArm, F_cArm);
    cArm_data = viewer.selected_data_index;
    viewer.append_mesh();
    viewer.data().set_mesh(V_beam, F_beam);
    beam_data = viewer.selected_data_index;
    viewer.append_mesh();

    // for (int i = 0; i < numStaff; i++)
    // {
    //     phantom_data.push_back(viewer.selected_data_index);
    //     viewer.append_mesh();
    //     viewer.append_mesh(); // for apron
    // }

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

    // extra phantom
    viewer.append_mesh();
    viewer.data().show_lines = false;
    viewer.data().show_texture = true;
    viewer.data().set_colors(RowVector4d(0.4, 0.4, 0.4, 0.2));
    extra_data = viewer.selected_data_index;
    // for (int id : phantom_data)
    // {
    //     vieewer.data(id).double_sided = true;
    //     viewer.data(id).show_overlay_depth = true;
    //     viewer.data(id).line dose"ine_width = 1;
    //     viewer.data(id).point_size = 8;
    //     viewer.data(id).show_texture = true;
    //     viewer.data(id).show_lines = false;
    //     viewer.data(id).show_faces = true;
    //     viewer.data(id).face_based = false;
    //     viewer.data(id).set_colors(RowVector4d(0.2, 0.2, 0.2, 0.2));
    //     // if (id == mainID())
    //     //     viewer.data(id).show_texture = false;
    // }

    viewer.append_mesh();
    viewer.data().set_mesh(V_table, F_table);
    table_data = viewer.selected_data_index;

    viewer.append_mesh();
    viewer.data().set_mesh(V_glass, F_glass);
    viewer.data().set_colors(RowVector4d(0, 0, 1, 0.2));
    glass_data = viewer.selected_data_index;


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
    viewer.data(cArm_data).show_lines = false;
    viewer.data(cArm_data).show_overlay_depth = false;
    viewer.data(cArm_data).double_sided = false;
    viewer.data(cArm_data).point_size = 4;
    viewer.data(cArm_data).set_colors(RowVector4d(0.8, 0.8, 0.8, 1.));
    viewer.data(beam_data).set_colors(RowVector4d(1, 0, 0, 0.2));
    viewer.data(beam_data).show_lines = false;
    viewer.data(beam_data).show_overlay_depth = false;
    viewer.data(glass_data).show_lines = false;
    viewer.data(glass_data).double_sided = true;
    viewer.data(glass_data).show_overlay_depth = true;
    viewer.data(phantomAcc_data).point_size = 4;
    viewer.data(phantomAcc_data).show_lines = false;
    viewer.data(phantomAcc_data).double_sided = false;
}

void mysql_finish_with_error(MYSQL *conn)
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
    if (show_settings_window)
        ShowSettingsWindow(&show_settings_window, _window);
    if (show_view_options)
        ShowViewOptions(&show_view_options, _window);
    if (show_color_popup)
        ShowColorInfoPopUp(&show_color_popup, _window);
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
            static double bdtol(_window->bodyDelayTol);
            static int distLimit(sqrt(Communicator::Instance().dist2Limit));
            ImGui::SetNextItemWidth(ImGui::GetFontSize() * 7.f);
            if (ImGui::InputDouble("body delay tol.", &bdtol, 0.1, 1, "%.1f"))
                _window->bodyDelayTol = bdtol;
            ImGui::SetNextItemWidth(ImGui::GetFontSize() * 7.f);
            if (ImGui::InputInt("dist limit (cm)", &distLimit, 50, 100))
                Communicator::Instance().dist2Limit = distLimit * distLimit;
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
            static bool force_manualBeam(_window->force_manualBeam);
            if (ImGui::MenuItem("force manual beam", "", &force_manualBeam))
            {
                _window->force_manualBeam = force_manualBeam;
            }
            static bool animBeamOnly(_window->animBeamOnOnly);
            if (ImGui::MenuItem("animate beamOn only", "", &animBeamOnly))
            {
                _window->animBeamOnOnly = animBeamOnly;
            }
            static bool useCurtain(_window->use_curtain);
            if (ImGui::MenuItem("use curtain", "", &useCurtain))
            {
                _window->use_curtain = useCurtain;
                MatrixXd v = _window->viewer.data(_window->table_data).V;
                _window->viewer.data(_window->table_data).clear();
                if(useCurtain) _window->viewer.data(_window->table_data).set_mesh(v, _window->F_table);
                else _window->viewer.data(_window->table_data).set_mesh(v, _window->F_table.topRows(_window->F_table.rows()-2));
                _window->viewer.data(_window->table_data).set_colors(RowVector4d(0, 0.2, 0, 0.2));
            } 
            ImGui::EndMenu();
        }
        static int pdcProcess(0), pdcDataSize(0);
        static thread pdcSender;
        if (ImGui::BeginMenu("data"))
        {
            if (ImGui::MenuItem("load frame data"))
            {
                if (_window->loadNum >= 0)
                    _window->recordData.clear();
                if (_window->recording)
                    cout << "recording is on currently!!" << endl;
                else
                {
                    string name = igl::file_dialog_open();
                    ifstream ifs(name, ios::binary);
                    if (!ifs.is_open())
                        cout << name << " is not open!" << endl;
                    vector<float> frameData(19);
                    while (ifs.read((char *)frameData.data(), 19 * sizeof(float)))
                    {
                        frameData.resize(19 + frameData[18] * 145);
                        ifs.read((char *)&frameData[19], frameData[18] * 145 * sizeof(float));
                        _window->recordData.push_back(frameData);
                    }
                    ifs.close();
                    if (_window->recordData.size())
                        _window->loadNum = 0;
                    _window->stop = true;
                    _window->recordVal = MatrixXd::Zero(_window->recordData.size(), 5);
                }
            }
            if (_window->loadNum>=0 && _window->stop && ImGui::MenuItem("export time stamp"))
            {
                string name = igl::file_dialog_save();
                ofstream ofs(name);
                for(int i=0;i<_window->recordData.size();i++)
                {
                    auto frame = _window->ReadAframeData(_window->recordData[i]);
                    ofs<<i<<"\t"<<frame.time<<"\t"<<frame.dap<<"\t"<<frame.kVp<<endl;
                }
                ofs.close();
                ofstream ofs1(name+".dose");
                ofs1<<_window->recordVal<<endl;
                ofs1.close();
            }
            if (_window->stop&&ImGui::MenuItem("export machines"))
            {
                igl::writePLY("machine_glass.ply",_window->viewer.data(_window->glass_data).V*10,
                _window->viewer.data(_window->glass_data).F);
                igl::writePLY("machine_cArm.ply",_window->viewer.data(_window->cArm_data).V*10,
                _window->viewer.data(_window->cArm_data).F);
                igl::writePLY("machine_bed.ply",_window->viewer.data(_window->table_data).V*10,
                _window->viewer.data(_window->table_data).F);
                igl::writePLY("machine_patient.ply",_window->viewer.data(_window->patient_data).V*10,
                _window->viewer.data(_window->patient_data).F);
            }
            static bool mmChk(false);
            if (ImGui::MenuItem("export main OBJ"))
            {
                string name = igl::file_dialog_save();
                MatrixXd V =  _window->GetMainPhantomHandle()->U;
                if(mmChk) V *= 10;
                MatrixXi F = _window->GetMainPhantomHandle()->F;
                // for(int i=0;i<_window->GetMainPhantomHandle()->C.rows();i++)
                // {
                //     F.conservativeResize(F.rows()+_window->viewer.data(_window->sphere_data).F.rows(), 3);
                //     F.bottomRows(_window->viewer.data(_window->sphere_data).F.rows()) = _window->viewer.data(_window->sphere_data).F.array() + V.rows();
                //     V.conservativeResize(V.rows() + _window->V_sphere.rows(), 3);
                //     V.bottomRows(_window->V_sphere.rows()) = _window->V_sphere.rowwise() + _window->GetMainPhantomHandle()->C1.row(i);
                // }
                igl::writeOBJ(name + ".obj", V, F);
                cout << "exported " + name + ".obj" << endl;
            }
            if (_window->stop && _window->loadNum >= 0 && ImGui::BeginMenu("delet data"))
            {
                for (auto iter : _window->currentFrame.bodyMap)
                {
                    if (ImGui::MenuItem(to_string(iter.first).c_str()))
                    {
                        _window->currentFrame.bodyMap.erase(iter.first);
                        _window->keepCurr = true;
                        _window->playOnce = true;
                    }
                }
                ImGui::EndMenu();
            }
            if (ImGui::MenuItem("Export dose file"))
            {
                string name = igl::file_dialog_save();
                MatrixXd D(_window->GetMainPhantomHandle()->V.rows(), 2);
                D.col(0) = _window->GetMainPhantomHandle()->rateD;
                D.col(1) = _window->GetMainPhantomHandle()->accD;
                igl::writeDMAT(name+".dose", D);
                VectorXd DA;
                igl::doublearea(_window->GetMainPhantomHandle()->V, _window->GetMainPhantomHandle()->F, DA);
                VectorXd fRateD = VectorXd::Zero(_window->GetMainPhantomHandle()->F.rows());
                VectorXd fAccD = VectorXd::Zero(_window->GetMainPhantomHandle()->F.rows());
                for(int i=0;i<fRateD.rows();i++)
                {
                    fRateD(i) = igl::slice(_window->GetMainPhantomHandle()->rateD, _window->GetMainPhantomHandle()->F.row(i).transpose(), 1).sum() / 3.;
                    fAccD(i) = igl::slice(_window->GetMainPhantomHandle()->accD, _window->GetMainPhantomHandle()->F.row(i).transpose(), 1).sum()/ 3.;
                }
                D.resize(_window->GetMainPhantomHandle()->F.rows(), 3);
                D.col(0) = DA*0.5;
                D.col(1) = fRateD;
                D.col(2) = fAccD;
                ofstream ofs(name+".Fdose");
                ofs<<D<<endl;
                ofs.close();
            }
            if (ImGui::MenuItem("Import dose file"))
            {
                if(!_window->GetMainPhantomHandle())
                {
                    cout<<"set main phantom first!"<<endl;
                }
                else
                {
                    string file =igl::file_dialog_open();
                    MatrixXd D;
                    if(!igl::readDMAT(file, D))
                        cout<<file + " was not properly opened!"<<endl;
                    else if(D.rows()!=_window->viewer.data(_window->phantomAcc_data).V.rows())
                        cout<<"Wrong D size (D: "<<D.rows()<<"/V: "<<_window->viewer.data(_window->phantomAcc_data).V.rows()<<endl;
                    else
                        _window->viewer.data(_window->phantomAcc_data).set_data(D.col(0), igl::COLOR_MAP_TYPE_PARULA, 100);
                    cout<<D.col(0).maxCoeff()<<endl;
                    _window->viewer.selected_data_index = _window->phantomDataID[_window->bodyID].first;
                    // cout<<D.maxCoeff()<<endl;
                }
            }
            if (_window->stop && ImGui::MenuItem("Import external OBJ"))
            {
                if (_window->externalOBJ_data < 0)
                {
                    _window->viewer.append_mesh();
                    _window->externalOBJ_data = _window->viewer.selected_data_index;
                }
                else if (_window->viewer.data(_window->externalOBJ_data).V.rows() > 0)
                    _window->viewer.data(_window->externalOBJ_data).clear();
                string name = igl::file_dialog_open();
                MatrixXd _V;
                MatrixXi _F;
                if (igl::readOBJ(name, _V, _F))
                {
                    cout << "open " << _V.rows() << " vertices and " << _F.rows() << " facets " << flush;
                    if (mmChk)
                    {
                        cout << " in mm scale" << endl;
                        _V *= 0.1;
                    }
                    else
                        cout << " in cm scale" << endl;
                    _window->viewer.data(_window->externalOBJ_data).set_visible(false, _window->v_middle);
                    _window->viewer.data(_window->externalOBJ_data).set_visible(true, _window->v_left);
                    _window->viewer.data(_window->externalOBJ_data).set_mesh(_V, _F);
                    // _window->viewer.data(_window->externalOBJ_data).show_texture = true;
                    _window->viewer.data(_window->externalOBJ_data).set_data(VectorXd::Zero(_V.rows()), igl::COLOR_MAP_TYPE_PARULA);
                    _window->playOnce = true;
                    _window->keepCurr = true;
                    _window->externalD.resize(_V.rows());
                }
                else
                {
                    cout << name + " is not open" << endl;
                }
            }
            ImGui::MenuItem("use mm scales", "", &mmChk);
            if (ImGui::MenuItem("clear external OBJ"))
            {
                if (_window->externalOBJ_data >= 0)
                    _window->viewer.data(_window->externalOBJ_data).clear();
            }

            static int pdcFrameT(500);
            ImGui::SetNextItemWidth(100);
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
                    float time(0), prevTime(-1), dap(0), num(-1);
                    vector<int> saved;
                    for (auto frame : _window->recordData)
                    {
                        num++;
                        if(prevTime<0 && frame[0]<0) continue;
                        prevTime = frame[0];
                        time += fabs(frame[0]);
                        dap +=  fabs(frame[0]) * frame[3];
                        if ((frame[0] < 0) || (time > pdcFrameT)) // last frame || time>min. frame time
                        {
                            frame[0] = time;
                            frame[3] = dap / time;
                            data.push_back(frame);
                            time = 0;
                            dap = 0;
                            saved.push_back(num);
                        }
                    }
                    pdcDataSize = data.size();
                    string name = igl::file_dialog_save();
                    ofstream ofs(name, ios::binary);
                    for (auto frame : data)
                        ofs.write((char *)frame.data(), frame.size() * sizeof(float));
                    ofs.close();
                    _window->recordData.clear();
                    cout << name << " recorded!" << endl;
                    ofstream frameOut(name+".frame");
                    for(int i:saved) frameOut<<i<<endl;
                    frameOut.close();

                    // pdcSender = thread(
                    //     [&pdcProcess](vector<vector<float>> _data)
                    //     {
                    //         string DB_HOST("166.104.155.16");
                    //         string DB_USER("sungho");
                    //         string DB_PASS("sungho");
                    //         string DB_NAME("McDCIR_PDC");
                    //         string DB_TABLE("FrameMain");
                    //         int PORT_ID = 3307;

                    //         vector<string> body_TABLE;
                    //         for (int i = 0; i < 20; i++)
                    //         {
                    //             body_TABLE.push_back("id" + to_string(i));
                    //         }
                    //         vector<string> main_colNameVec, body_colNameVec;
                    //         string main_colNameStr, main_colValueStr, body_colNameStr;
                    //         vector<string> body_colValueStr;
                    //         string main_colNameStr_tmp, main_colValueStr_tmp;

                    //         main_colNameVec = {"time", "kVp", "mA", "dap", "cArm0", "cArm1", "cArm2", "bed0", "bed1", "bed2",
                    //                            "glassChk", "glass_qx", "glass_qy", "glass_qz", "glass_qw", "glass_tx", "glass_ty", "glass_tz",
                    //                            "id0", "id1", "id2", "id3", "id4", "id5", "id6", "id7", "id8", "id9",
                    //                            "id10", "id11", "id12", "id13", "id14", "id15", "id16", "id17", "id18", "id19", "flag"};

                    //         cout << "#: " << pdcProcess << endl;

                    //         for (int i = 0; i < 18; i++)
                    //         {
                    //             main_colNameStr += "`" + main_colNameVec[i] + "`,";
                    //         }
                    //         main_colNameStr_tmp = main_colNameStr;

                    //         for (int i = 0; i < 18 * 4; i++)
                    //         {
                    //             if (i % 4 == 0)
                    //                 body_colNameVec.push_back("qx_" + to_string(i / 4));
                    //             else if (i % 4 == 1)
                    //                 body_colNameVec.push_back("qy_" + to_string(i / 4));
                    //             else if (i % 4 == 2)
                    //                 body_colNameVec.push_back("qz_" + to_string(i / 4));
                    //             else if (i % 4 == 3)
                    //                 body_colNameVec.push_back("qw_" + to_string(i / 4));
                    //             body_colNameStr += "`" + body_colNameVec[i] + "`,";
                    //         }
                    //         for (int i = 0; i < 24 * 3; i++)
                    //         {
                    //             if (i % 3 == 0)
                    //                 body_colNameVec.push_back("tx_" + to_string(i / 3));
                    //             else if (i % 3 == 1)
                    //                 body_colNameVec.push_back("ty_" + to_string(i / 3));
                    //             else if (i % 3 == 2)
                    //                 body_colNameVec.push_back("tz_" + to_string(i / 3));
                    //             body_colNameStr += "`" + body_colNameVec[18 * 4 + i] + "`,";
                    //             if (i == 24 * 3 - 1)
                    //             {
                    //                 body_colNameStr.pop_back();
                    //             }
                    //         }
                    //         // ==========================================================

                    //         MYSQL *conn = mysql_init(NULL);
                    //         if (mysql_real_connect(conn, DB_HOST.c_str(), DB_USER.c_str(), DB_PASS.c_str(), DB_NAME.c_str(), PORT_ID, NULL, 0) != NULL)
                    //         {
                    //             cout << "web-server is successfully connected !!" << endl;
                    //         } // else mysql_finish_with_error(conn);

                    //         for (int i = 0; i < _data.size(); i++)
                    //         {
                    //             sleep(1);
                    //             // send
                    //             pdcProcess = i + 1;

                    //             for (int j = 0; j < 18; j++)
                    //             { // before first worker id
                    //                 main_colValueStr += "'" + to_string(_data[i][j]) + "',";
                    //             }

                    //             int maxWorkerNo(5);
                    //             int body_start_idx(18);
                    //             int idx(0);
                    //             int bodyNo(0);
                    //             bool isFirst(false);
                    //             for (int j = body_start_idx; j < body_start_idx + maxWorkerNo * 145; j++)
                    //             {
                    //                 int body_id = (j - body_start_idx) / 145 * 145 + body_start_idx;
                    //                 if (_data[i][body_id] == false)
                    //                 {
                    //                     continue;
                    //                 }
                    //                 if (j == 18 + idx * 145)
                    //                 {
                    //                     main_colNameStr += "`" + main_colNameVec[(body_start_idx) + idx] + "`,";
                    //                     main_colValueStr += "'" + to_string((int)_data[i][j]) + "',";
                    //                     body_colValueStr.push_back("");
                    //                     idx++;
                    //                     if (_data[i][body_id] == true)
                    //                         bodyNo++;
                    //                     continue;
                    //                 }

                    //                 body_colValueStr[bodyNo - 1] += "'" + to_string(_data[i][j]) + "',";
                    //             }
                    //             main_colNameStr += "`flag`";
                    //             main_colValueStr += "'0'"; // set flag status as '0'

                    //             string sql;
                    //             sql = "INSERT INTO `" + DB_NAME + "`.`" + DB_TABLE + "` (" + main_colNameStr + ") VALUES (" + main_colValueStr + ");";
                    //             // if (mysql_query(conn, sql.c_str()) != false) mysql_finish_with_error(conn);
                    //             main_colNameStr = main_colNameStr_tmp;
                    //             main_colValueStr = "";

                    //             for (int j = 0; j < bodyNo; j++)
                    //             {
                    //                 body_colValueStr[j].pop_back();
                    //                 sql = "INSERT INTO `" + DB_NAME + "`.`" + body_TABLE[j] + "` (" + body_colNameStr + ") VALUES (" + body_colValueStr[j] + ");";
                    //                 //  if (mysql_query(conn, sql.c_str()) != false) mysql_finish_with_error(conn);
                    //                 body_colValueStr[j].clear();
                    //             }
                    //         }

                    //         // close
                    //         mysql_close(conn);
                    //         return;
                    //     },
                    //     data);
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

        if (!_window->recording)
        {
            if (ImGui::SmallButton("record"))
            {
                _window->loadNum = -1;
                _window->recording = true;
            }
        }
        else if (ImGui::SmallButton("stop recording"))
        {
            _window->loadNum = -1;
            _window->recording = false;
        }

        if (_window->recordData.size() && (!_window->recording))
        {
            if (ImGui::SmallButton("export"))
            {
                string name = igl::file_dialog_save();
                ofstream ofs(name, ios::binary);
                if (!ofs.is_open())
                    cout << "failed to open file" << endl;
                else
                {
                    for (auto frame : _window->recordData)
                        ofs.write((char *)frame.data(), frame.size() * sizeof(float));
                    ofs.close();
                    _window->recordData.clear();
                    cout << name << " recorded!" << endl;
                }
            }
            if (ImGui::SmallButton("replay"))
            {
                if (_window->loadNum < 0)
                    _window->loadNum = 0;
                else
                    _window->loadNum = -1;
            }
            if (ImGui::SmallButton("clear"))
                _window->recordData.clear();
        }

        if (_window->loadNum >= 0 && _window->stop)
        {
            static int frameN;
            frameN = _window->loadNum;
            ImGui::SetNextItemWidth(100);
            if(ImGui::InputInt("go to", &frameN))
            {
                _window->loadNum=frameN;
                _window->playOnce = true;
            }
        }

        if (_window->recording)
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
    ImGui::SetNextWindowPos(ImVec2(sx - phantomViewX - 10, viewport->WorkPos.y + 10), ImGuiCond_Always, ImVec2(1.f, 0.f));
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
        static bool eChk(true);
        if (ImGui::Checkbox("extra", &eChk))
        {
            _window->extraChk = eChk;
        }
        ImGui::SetNextItemWidth(64);

        static vector<string> bodyIDs;
        bodyIDs.resize(_window->workerIdData.size());
        int tmpI(0);
        for (auto iter : _window->workerIdData)
            bodyIDs[tmpI++] = to_string(iter.first);
        if (_window->externalOBJ_data >= 0 && _window->viewer.data(_window->externalOBJ_data).V.rows() > 0)
            bodyIDs.push_back("external");
        static int selectedID(0);
        if (ImGui::Combo("main ID", &selectedID, bodyIDs))
        {
            if (bodyIDs[selectedID] == "external")
                _window->show_externalDose = true;
            else
            {
                _window->show_externalDose = false;
                _window->SetBodyID(atoi(bodyIDs[selectedID].c_str()));
                for (auto id : _window->phantomDataID)
                {
                    if (id.first == _window->bodyID)
                    {
                        _window->viewer.selected_data_index = id.second.first;
                        _window->viewer.data(id.second.first).show_texture = false;
                        _window->viewer.data(id.second.first).double_sided = false;
                        _window->viewer.data(id.second.first).show_overlay = true;
                        _window->viewer.data(id.second.first).show_overlay_depth = true;
                        auto phantom = _window->GetMainPhantomHandle();
                        _window->viewer.data(_window->phantomAcc_data).set_mesh(phantom->GetAccV(), phantom->F);
                        _window->viewer.data(_window->phantomAcc_data).set_data(phantom->accD, igl::COLOR_MAP_TYPE_PARULA, 50);
                        _window->viewer.data(_window->phantomDataID[id.first].second).set_colors(RowVector4d(0.5, 0.5, 1, 0.7));
                        _window->viewer.data(id.second.first).is_visible = 0;
                        _window->viewer.data(id.second.second).is_visible = 0;
                        continue;
                    }

                    // _window->viewer.data(_window->phantomAcc_data).set_data(phantom->accD, igl::COLOR_MAP_TYPE_PARULA, 50);
                    _window->viewer.data(id.second.first).set_colors(RowVector4d(0.2, 0.2, 0.2, 0.2));
                    _window->viewer.data(id.second.first).show_texture = true;
                    _window->viewer.data(id.second.first).is_visible = 0;
                    _window->viewer.data(id.second.second).is_visible = 0;
                    _window->viewer.data(id.second.first).clear_points();
                    _window->viewer.data(id.second.first).clear_edges();
                }
            }
        }
        static bool show_C(_window->show_C), show_BE(_window->show_BE);
        if (ImGui::Checkbox("joint", &show_C))
        {
            _window->show_C = show_C;
            if (!show_C)
                _window->viewer.data(_window->phantomDataID[_window->bodyID].first).clear_points();
        }
        if (ImGui::Checkbox("skeleton", &show_BE))
        {
            _window->show_BE = show_BE;
            if (!show_BE)
                _window->viewer.data(_window->phantomDataID[_window->bodyID].first).clear_edges();
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
            _window->viewer.data(_window->patient_data).show_lines = true;
            _window->viewer.data(_window->patient_data).show_faces = true;            
            _window->viewer.data(_window->patient_data).face_based = true;            
            _window->viewer.data(_window->patient_data).clear_points();
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
struct ScrollingBuffer
{
    int MaxSize;
    int Offset;
    ImVector<ImVec2> Data;
    ScrollingBuffer(int max_size = 1000)
    {
        MaxSize = max_size;
        Offset = 0;
        Data.reserve(MaxSize);
        Data.push_back(ImVec2(0, 0));
    }
    void AddPoint(float x, float y)
    {
        if (Data.size() < MaxSize)
            Data.push_back(ImVec2(x, y));
        else
        {
            Data[Offset] = ImVec2(x, y);
            Offset = (Offset + 1) % MaxSize;
        }
    }
    void Erase()
    {
        if (Data.size() > 0)
        {
            Data.shrink(0);
            Offset = 0;
        }
    }
};

void DoseResultWindow(RDCWindow *_window)
{
    ImGuiWindowFlags flags = ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing;
    const ImGuiViewport *viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(ImVec2(0.f, viewport->WorkPos.y + viewport->WorkSize.y), ImGuiCond_Always, ImVec2(0., 1.f));
    ImGui::SetNextWindowSize(ImVec2(viewport->WorkSize.x, 230));
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(8. / 255., 15. / 255., 26. / 255., 1.0f));
    vector<string> radiologistDose = {"Whole skin dose", "Right hand dose", "Left hand dose", "PSD", "Lens dose"};
    char buf[128];
    if (_window->show_externalDose)
        sprintf(buf, "External OBJ Dose");
    else
        sprintf(buf, "Radiologist #%d Dose", _window->bodyID);
    bool open(true);
    if (ImGui::Begin(buf, &open, flags))
    {
        if (_window->show_externalDose)
        {
            ImGui::BulletText("Examine dose");
            static float center[3], radius;
            ImGui::InputFloat3("center (cm)",center);
            ImGui::InputFloat("radius (cm)", &radius);
            static float result(0);
            if(ImGui::Button("Calculate"))
            {
                auto data = &_window->viewer.data(_window->externalOBJ_data);
                VectorXd A, W = VectorXd::Zero(data->V.rows());
                igl::doublearea(data->V,data->F, A);
                for(int i=0;i<A.rows();i++)
                {
                    W(data->F(i, 0))+=A(i);
                    W(data->F(i, 1))+=A(i);
                    W(data->F(i, 2))+=A(i);
                }
                ArrayXd fov = ((data->V.rowwise()-RowVector3d(center[0], center[1], center[2])).rowwise().squaredNorm().array()<radius*radius).cast<double>();
                // cout<<fov.sum()<<endl;
                _window->viewer.data(_window->sphere_data).set_vertices((_window->V_sphere*radius).rowwise() + RowVector3d(center[0], center[1], center[2]));
                _window->viewer.data(_window->sphere_data).set_visible(true, _window->v_left);
                result = (_window->externalD.array() * fov * W.array()).sum() * 1000. * 3600. / (W.array()*fov.array()).sum();
            }
            ImGui::SameLine();
            ImGui::Text("-> %f mGy/hr", result);
        }
        else
        {
            auto phantom = _window->GetMainPhantomHandle();
            if (phantom)
            {
                ImGui::Columns(2, "", false);
                ImGuiTableFlags flags = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_Reorderable;
                static int graphOpt(-1);
                if (graphOpt < 0)
                {
                    graphOpt = 0;
                }
                static ScrollingBuffer rate, avg;
                VectorXd rateVal(radiologistDose.size()), accVal(radiologistDose.size());
                rateVal << phantom->GetAvgSkinDoseRate(), phantom->GetRightHandDoseRate(), phantom->GetLeftHandDoseRate(), phantom->GetMaxSkinDoseRate(), phantom->rateD_eye.sum() * 0.5;
                rateVal *= 1000. * 3600.; // mGy/hr
                accVal << phantom->GetAvgAccSkinDose(), phantom->GetRightHandAccDose(), phantom->GetLeftHandAccDose(), phantom->GetMaxAccSkinDose(), phantom->accD_eye.sum() * 0.5;
                accVal *= 1.e6; // uGy

                if (ImGui::BeginTable("Dose Table", 4, flags))
                {
                    ImGui::TableSetupColumn("", ImGuiTableColumnFlags_NoReorder | ImGuiTableColumnFlags_WidthFixed);
                    ImGui::TableSetupColumn("Dose Rate (mGy/hr)");
                    ImGui::TableSetupColumn("Cumulative Dose (uGy)");
                    ImGui::TableSetupColumn("Graph");
                    ImGui::TableHeadersRow();
                    // //set dose

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
                        ImGui::PushID(row + 1);
                        if (ImGui::RadioButton("", &graphOpt, row))
                        {
                            rate.Erase();
                            avg.Erase();
                        }
                        ImGui::PopID();
                    }
                    ImGui::EndTable();
                }
                // cout<<time(NULL) <<" "<< startT<<" "<<t<<endl;
                double avgV(0);
                if (phantom->irrTime > 0)
                {
                    avgV = accVal(graphOpt) / phantom->irrTime * 0.001 * 3600;
                    avg.AddPoint(phantom->irrTime, avgV);
                    rate.AddPoint(phantom->irrTime, rateVal(graphOpt));
                }
                static int history = 30;
                if (ImGui::SliderInt("history time (sec)", &history, 10, 100))
                    ;
                ImGui::NextColumn();
                ImPlot::CreateContext();
                if (ImPlot::BeginPlot(radiologistDose[graphOpt].c_str(), ImVec2(-1, 200), ImPlotFlags_NoTitle))
                {
                    ImPlot::SetupAxes("sec", "mGy/hr");
                    ImPlot::SetupAxisLimits(ImAxis_X1, phantom->irrTime - history, phantom->irrTime, ImGuiCond_Always);
                    if (avgV > 0)
                        ImPlot::SetupAxisLimits(ImAxis_Y1, 0, max(avgV * 2, rateVal(graphOpt) * 1.3));
                    else
                        ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 0.1);
                    ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
                    ImPlot::PlotShaded("avg. skin dose rate", &avg.Data[0].x, &avg.Data[0].y, avg.Data.size(), -INFINITY, 0, avg.Offset, 2 * sizeof(float));
                    ImPlot::PlotLine((radiologistDose[graphOpt] + " rate").c_str(), &rate.Data[0].x, &rate.Data[0].y, rate.Data.size(), 0, rate.Offset, 2 * sizeof(float));
                    ImPlot::EndPlot();
                }
                ImPlot::DestroyContext();
                ImGui::NextColumn();
            }
        }
        ImGui::End();
    }
    // ImPlot::CreateContext();
    // ImPlot::ShowDemoWindow();
    // ImPlot::DestroyContext();
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

        vector<string> names = _window->profileNames;
        static vector<string> profileNames(names.size());
        copy(names.begin(), names.end(), profileNames.begin());

        ImGuiTableFlags flags = ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_NoBordersInBody;
        // | ImGuiTableFlags_SizingFixedFit;
        static int bfID[20], profileID[20]; // idJoints[20][2];
        static float height[20];
        static float color[20][3] = { 2,};
        if (color[0][0] > 1)
        {
            fill(color[0], color[20], 1);
        }
        static bool blueMask[20];
        static bool useApron[20];
        static bool trackOpt[20];
        if (ImGui::BeginTable("phantom settings", 6, flags))
        {
            ImGui::TableSetupColumn("#");
            ImGui::TableSetupColumn("BMI");
            ImGui::TableSetupColumn("profile");
            ImGui::TableSetupColumn("mask color");
            ImGui::TableSetupColumn("use apron");
            ImGui::TableSetupColumn("track");
            ImGui::TableHeadersRow();

            for (int i = 0; i < _window->numStaff; i++)
            {
                ImGui::PushID(i * 10);
                ImGui::TableNextRow();
                if (ImGui::TableNextColumn())
                    ImGui::Text(std::to_string(i).c_str());
                if (ImGui::TableNextColumn())
                {
                    ImGui::SetNextItemWidth(60);
                    ImGui::Combo("", &bfID[i], BFlist);
                }
                if (ImGui::TableNextColumn())
                {
                    ImGui::SetNextItemWidth(60);
                    ImGui::Combo(" ", &profileID[i], profileNames);
                    ImGui::SameLine();
                    ImGui::SetNextItemWidth(100);
                    ImGui::InputFloat("cm", &height[i], 1, 10, "%.1f");
                }
                if (ImGui::TableNextColumn())
                {
                    auto flag = ImGuiColorEditFlags_InputHSV | ImGuiColorEditFlags_Float | ImGuiColorEditFlags_NoSidePreview | ImGuiColorEditFlags_NoAlpha | ImGuiColorEditFlags_NoOptions | ImGuiColorEditFlags_PickerHueWheel | ImGuiColorEditFlags_NoInputs;
                    ImGui::ColorEdit3("", color[i], flag);
                }
                // if (ImGui::TableNextColumn())
                // {
                //     ImGui::Checkbox("blue", &blueMask[i]);
                // }
                if (ImGui::TableNextColumn())
                {
                    if (ImGui::Checkbox("apron", &useApron[i]))
                    {
                        if (_window->GetPhantom(i))
                            _window->GetPhantom(i)->useApron = useApron[i];
                    }
                }
                // ImGui::ColorEdit3("", color[i]);
                if (ImGui::TableNextColumn())
                {
                    if (ImGui::Checkbox("##trackOpt", &trackOpt[i]))
                    {
                        // int id = _window->phantom_data[i];
                        if (trackOpt[i])
                //  if(false)
                        {
                            auto phantom = _window->AddNewPhantom(i);
                            bool isMale(false);
                            if (bfID[i] < 5)
                                isMale = true;
                            phantom->LoadPhantom(BFlist[bfID[i]], isMale);
                            phantom->useApron = useApron[i];
                            if (profileNames[profileID[i]] != "original")
                            {
                                phantom->CalibrateTo2(_window->jointLengths[profileID[i]], height[i]);
                                // phantom->CalibrateTo(_window->jointLengths[profileID[i]], _window->eyeL_vec[profileID[i]], _window->eyeL_vec[profileID[i]]);
                            }
                            // _window->viewer.append_mesh();
                            _window->viewer.data(_window->phantomDataID[i].first).set_mesh(phantom->V, phantom->F);
                            _window->viewer.data(_window->phantomDataID[i].first).show_lines = false;
                            _window->viewer.data(_window->phantomDataID[i].first).double_sided = false;
                            // _window->phantomDataID[i].first = _window->viewer.selected_data_index;
                            // _window->viewer.append_mesh();
                            if (phantom->useApron)
                                _window->viewer.data(_window->phantomDataID[i].second).set_mesh(phantom->V_apron, phantom->F_apron);
                            // _window->phantomDataID[i].second = _window->viewer.selected_data_index;
                            _window->workerIdData[i] = make_pair(int(floor(color[i][0] * 180. + 0.5)), blueMask[i]);

                            // texture setting
                            if (i == _window->bodyID)
                            {
                                _window->viewer.data(_window->phantomDataID[i].first).show_texture = false;
                                _window->viewer.data(_window->phantomAcc_data).set_mesh(phantom->GetAccV(), phantom->F);
                                _window->viewer.data(_window->phantomAcc_data).set_data(phantom->accD, igl::COLOR_MAP_TYPE_PARULA, 50);
                                _window->viewer.data(_window->phantomDataID[i].second).set_colors(RowVector4d(0.5, 0.5, 1, 0.7));
                            }
                            else
                            {
                                _window->viewer.data(_window->phantomDataID[i].first).set_colors(RowVector4d(0.2, 0.2, 0.2, 0.2));
                                _window->viewer.data(_window->phantomDataID[i].second).set_colors(RowVector4d(0.5, 0.5, 1, 0.2));
                                _window->viewer.data(_window->phantomDataID[i].first).show_texture = true;
                                _window->viewer.data(_window->phantomDataID[i].first).clear_points();
                                _window->viewer.data(_window->phantomDataID[i].first).clear_edges();
                            }
                            _window->viewer.data(_window->phantomDataID[i].first).point_size = 8;
                            _window->viewer.data(_window->phantomDataID[i].first).line_width = 2;
                            _window->viewer.data(_window->phantomDataID[i].first).show_overlay_depth = false;
                            _window->viewer.data(_window->phantomDataID[i].second).show_texture = true;
                            _window->viewer.data(_window->phantomDataID[i].second).show_lines = false;
                            _window->viewer.data(_window->phantomDataID[i].second).face_based = false;
                            _window->viewer.selected_data_index = _window->phantomDataID[i].second;
                            // _window->viewer.data(_window->phantomDataID[i].first).set_visible(true, _window->v_left);
                            // _window->viewer.data(_window->phantomDataID[i].second).set_visible(phantom->useApron, _window->v_left);
                        }
                        else
                        {
                            _window->viewer.data(_window->phantomDataID[i].first).clear();
                            _window->viewer.data(_window->phantomDataID[i].second).clear();
                            _window->viewer.data(_window->phantomDataID[i].first).is_visible = false;
                            _window->viewer.data(_window->phantomDataID[i].second).is_visible = false;
                            _window->workerIdData.erase(i);
                            _window->DeletePhantom(i);
                        }
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
            for (int i = 0; i < _window->numStaff; i++)
            {
                int id;
                string bfData, profile;
                ifs >> id >> bfData >> profile >> color[id][0] >> color[id][1] >> color[id][2] >> blueMask[id];
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
            for (int i = 0; i < _window->numStaff; i++)
                ofs << i << "\t" << BFlist[bfID[i]] << "\t" << profileNames[profileID[i]]
                    << "\t" << color[i][0] * 180 << "\t" << color[i][1] << "\t" << color[i][2] << "\t" << int(blueMask[i]) << endl;
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
                if (_window->workerIdData.size())
                    Communicator::Instance().StartWorker(ip, option, _window->workerIdData);
                else
                    Communicator::Instance().StartWorker(ip, option);
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
                if (ImGui::TableNextColumn() )//&& Communicator::Instance().GetDelay(iter.first) < 0.1)
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
        static float dap(_window->manualFrame.dap);
        static int kvpFD[2] = {_window->manualFrame.kVp, _window->manualFrame.FD};
        static int bedPos[3] = {_window->manualFrame.bed(0), _window->manualFrame.bed(1), _window->manualFrame.bed(2)};
        static int cArm[3] = {_window->manualFrame.cArm(0), _window->manualFrame.cArm(1), _window->manualFrame.cArm(2)};
        static bool useManualBeam(_window->manualFrame.beamOn);
        static bool force_withDAP(_window->force_withDAP);
        
        if (ImGui::Checkbox("beamOn w/ DAP", &force_withDAP))
        {
            _window->force_withDAP = force_withDAP;
            _window->manualFrame.beamOn = force_withDAP;
            _window->manualFrame.dap = dap;
        }
        ImGui::BulletText("C-arm");
        ImGui::SameLine();
        // if (ImGui::Button("Set beam"))
        //     _window->SetMap(kvp, rotation, angulation, RowVector3f(bedPos[0], bedPos[1], bedPos[2]), sidfd[0], sidfd[1]);
        ImGui::SameLine();
        if (ImGui::Checkbox("use manual beam/bed", &useManualBeam))
            _window->manualFrame.beamOn = useManualBeam;
        if (ImGui::InputFloat("DAP(Gycm2)", &dap, 1))
            _window->manualFrame.dap = dap;

        if (ImGui::InputInt2("kVp / FD", kvpFD))
        {
            _window->manualFrame.kVp = kvpFD[0];
            _window->manualFrame.FD = kvpFD[1];
        }

        if (ImGui::DragInt3("rot/ang/sid", cArm))
        {
            if (cArm[0] > 180)
                cArm[0] = 180;
            else if (cArm[0] < -180)
                cArm[0] = -180;

            if (cArm[1] > 90)
                cArm[1] = 90;
            else if (cArm[1] < -90)
                cArm[1] = -90;

            if (cArm[2] > 120)
                cArm[2] = 120;
            else if (cArm[2] < 90)
                cArm[2] = 90;

            _window->manualFrame.cArm = RowVector3f(cArm[0], cArm[1], cArm[2]);
            // rot = _window->GetCarmRotation(rotation, angulation);
            // _window->viewer.data(_window->cArm_data).set_vertices(_window->V_cArm * rot.cast<double>().transpose());
            // _window->viewer.data(_window->beam_data).set_vertices(_window->V_beam * rot.cast<double>().transpose());
            // _window->viewer.data(_window->cArm_data).compute_normals();
            // _window->viewer.data(_window->beam_data).compute_normals();
            // _window->CalculateSourceFacets(RowVector3f(bedPos[0], bedPos[1], bedPos[2]), rot);
            // if (RDCWindow::Instance().beamOn)
            // _window->viewer.data(_window->patient_data).set_points(_window->B_patient1.cast<double>(), RowVector3d(1, 0, 0));
        }
        ImGui::BulletText("Bed");
        if (ImGui::DragInt3("bed trans.", bedPos))
        {
            RowVector3f trans(bedPos[0], bedPos[1], bedPos[2]);
            _window->manualFrame.bed = trans;
        }
        ImGui::BulletText("Glass");
        ImGui::SameLine();
        static bool useManualGlass(false);
        if (ImGui::Checkbox("use manual glass", &useManualGlass))
            _window->manualFrame.glassChk = useManualGlass;
        static int glassPos[3], glassRot[3];
        if (ImGui::DragInt3("glass pos.", glassPos))
        {
            _window->manualFrame.glass_aff.setIdentity();
            _window->manualFrame.glass_aff.translate(Vector3d(glassPos[0], glassPos[1], glassPos[2]));
            _window->manualFrame.glass_aff.rotate(AngleAxisd((double)glassRot[0] / 180. * igl::PI, Vector3d(1, 0, 0)) * AngleAxisd((double)glassRot[1] / 180. * igl::PI, Vector3d(0, 1, 0)) * AngleAxisd((double)glassRot[2] / 180. * igl::PI, Vector3d(0, 0, 1)));
        }
        if (ImGui::DragInt3("glass rot.", glassRot))
        {
            _window->manualFrame.glass_aff.setIdentity();
            _window->manualFrame.glass_aff.translate(Vector3d(glassPos[0], glassPos[1], glassPos[2]));
            _window->manualFrame.glass_aff.rotate(AngleAxisd((double)glassRot[0] / 180. * igl::PI, Vector3d(1, 0, 0)) * AngleAxisd((double)glassRot[1] / 180. * igl::PI, Vector3d(0, 1, 0)) * AngleAxisd((double)glassRot[2] / 180. * igl::PI, Vector3d(0, 0, 1)));
        }
        // if (ImGui::Button("  Print glass data (V.rowwise().homogeneous()*TT)  "))
        // {
        //     MatrixXd TT(4, 3);
        //     TT.topRows(3) = (AngleAxisd((double)glassRot[0] / 180. * igl::PI, Vector3d(1, 0, 0)) * AngleAxisd((double)glassRot[1] / 180. * igl::PI, Vector3d(0, 1, 0)) * AngleAxisd((double)glassRot[2] / 180. * igl::PI, Vector3d(0, 0, 1))).matrix().transpose();
        //     TT.bottomRows(1) << glassPos[0], glassPos[1], glassPos[2];
        //     cout << endl
        //          << TT << endl;
        // }
    }
    ImGui::End();
}

void ColorInfoWindow(float min, float max, string unit)
{
    float fs = ImGui::GetFontSize() * ImGui::GetFont()->Scale;
    ImDrawList *draw_list = ImGui::GetWindowDrawList();
    ImVec2 p0 = ImGui::GetCursorScreenPos();
    float f(0), r, g, b, a;
    draw_list->AddRectFilled(p0, ImVec2(p0.x + fs, p0.y + fs), IM_COL32(igl::parula_cm[0][0]*255, igl::parula_cm[0][1]*255, igl::parula_cm[0][2]*255, 255));
    ImGui::Dummy(ImVec2(fs, fs));
    ImGui::SameLine();
    ImGui::Text("%1.f %s", min, unit.c_str());
    draw_list->AddRectFilled(ImVec2(p0.x, p0.y + fs + ImGui::GetStyle().ItemSpacing.y), ImVec2(p0.x + fs, p0.y + 2 * fs + ImGui::GetStyle().ItemSpacing.y), IM_COL32(igl::parula_cm[255][0]*255, igl::parula_cm[255][1]*255, igl::parula_cm[255][2]*255, 255));
    ImGui::Dummy(ImVec2(fs, fs));
    ImGui::SameLine();
    ImGui::Text("%1.f %s", max, unit.c_str());
}

void ShowColorInfoPopUp(bool *p_open, RDCWindow* _window)
{
    const ImGuiViewport *viewport = ImGui::GetMainViewport();
    int sx(viewport->Size.x), sy(viewport->Size.y);
    float phantomViewX = sy * 0.35;
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.f, 0.f, 0.f, 0.5f));
    // ImGui::SetNextWindowPos(ImVec2(sx - phantomViewX * 2 - 10, sy - 245), ImGuiCond_Always, ImVec2(1.f, 1.f));
    ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoMove;
    // if (ImGui::Begin("RadiolRate_color", p_open, window_flags))
    //     ColorInfoWindow(0, 10, "uGy/hr");
    // ImGui::End();
    ImGui::SetNextWindowPos(ImVec2(sx - phantomViewX - 10, sy - 245), ImGuiCond_Always, ImVec2(1.f, 1.f));
    int min(0), max(0), min1(0), max1(0);
    if(_window->show_externalDose)
    {
        min = _window->externalD.minCoeff()*1000*3600*1000;
        max = _window->externalD.maxCoeff()*1000*3600*1000;
    }
    else if(_window->GetMainPhantomHandle())
    {
        min = _window->GetMainPhantomHandle()->rateD.minCoeff()*1000*3600*1000;
        max = _window->GetMainPhantomHandle()->rateD.maxCoeff()*1000*3600*1000;
        min1 = _window->GetMainPhantomHandle()->accD.minCoeff()*1.e6;
        max1 = _window->GetMainPhantomHandle()->accD.maxCoeff()*1.e6;
    }
    if (ImGui::Begin("RadiolRate_color", p_open, window_flags))
        ColorInfoWindow(min, max, "uGy/hr");
    ImGui::End();
    ImGui::SetNextWindowPos(ImVec2(sx - 10, sy - 245), ImGuiCond_Always, ImVec2(1.f, 1.f));
    if (ImGui::Begin("RadiolAcc_color", p_open, window_flags))
        ColorInfoWindow(min1, max1, "uGy");
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
    vector<float> frameData(19);
    if(data.beamOn) frameData[0] = fabs(frameTimeInMSEC);
    else frameData[0] = -fabs(frameTimeInMSEC);
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
    frameData[18] = data.bodyMap.size();
    int pos = 19;

    for (auto iter : data.bodyMap)
    {
        frameData.push_back(iter.first);                     // 1
        for (int b = 0; b < iter.second.posture.size(); b++) // 18*4 (72)
        {
            frameData.push_back(iter.second.posture[b].x());
            frameData.push_back(iter.second.posture[b].y());
            frameData.push_back(iter.second.posture[b].z());
            frameData.push_back(iter.second.posture[b].w());
        }
        for (int c = 0; c < iter.second.jointC.rows(); c++) // 25*3 (75)
        {
            frameData.push_back(iter.second.jointC(c, 0));
            frameData.push_back(iter.second.jointC(c, 1));
            frameData.push_back(iter.second.jointC(c, 2));
        }
    }
    return frameData;
}

DataSet RDCWindow::ReadAframeData(vector<float> frameData)
{
    DataSet data;
    if (frameData.size() < 19)
    {
        cout << "Wrong frame data size!" << endl;
        return data;
    }
    data.beamOn = (frameData[0]>0);
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
    glassAff.translate(Vector3d(frameData[15], frameData[16], frameData[17]));
    glassAff.rotate(Quaterniond(frameData[14], frameData[11], frameData[12], frameData[13]));
    data.glass_aff = glassAff;
    int bodyNum = frameData[18];
    int pos = 19;
    for (int i = 0; i < bodyNum; i++)
    {
        int id = frameData[pos++];
        for (int b = 0; b < data.bodyMap[id].posture.size(); b++)
        {
            data.bodyMap[id].posture[b].x() = frameData[pos++];
            data.bodyMap[id].posture[b].y() = frameData[pos++];
            data.bodyMap[id].posture[b].z() = frameData[pos++];
            data.bodyMap[id].posture[b].w() = frameData[pos++];
        }
        for (int c = 0; c < data.bodyMap[id].jointC.rows(); c++)
        {
            data.bodyMap[id].jointC(c, 0) = frameData[pos++];
            data.bodyMap[id].jointC(c, 1) = frameData[pos++];
            data.bodyMap[id].jointC(c, 2) = frameData[pos++];
        }
    }
    return data;
}

// vector<float> RDCWindow::SetAframeData(DataSet data, float frameTimeInMSEC)
// {
//     vector<float> frameData(dataSize, 0);
//     frameData[0] = frameTimeInMSEC;
//     frameData[1] = data.kVp;
//     frameData[2] = data.mA;
//     frameData[3] = data.dap;
//     frameData[4] = data.cArm(0);
//     frameData[5] = data.cArm(1);
//     frameData[6] = data.cArm(2);
//     frameData[7] = data.bed(0);
//     frameData[8] = data.bed(1);
//     frameData[9] = data.bed(2);
//     frameData[10] = data.glassChk;
//     Quaterniond q(data.glass_aff.rotation());
//     frameData[11] = q.x();
//     frameData[12] = q.y();
//     frameData[13] = q.z();
//     frameData[14] = q.w();
//     frameData[15] = data.glass_aff.translation().x();
//     frameData[16] = data.glass_aff.translation().y();
//     frameData[17] = data.glass_aff.translation().z();
//     frameData[18] = data.bodyMap.size();
//     int pos = 19;

//     if(data.bodyMap.size()>numStaff) //if there are more than MAX
//     {
//         map<double, int> distMap;
//         for(auto iter:data.bodyMap)
//             distMap[iter.second.jointC.row(0).leftCols(2).squaredNorm()] = iter.first;

//         int i=0;
//         for(auto iter:distMap)
//         {
//             int num = pos + i * 145;
//             int id = iter.second;
//             frameData[num++] = id;
//             for (int b = 0; b < data.bodyMap[id].posture.size(); b++)
//             {
//                 frameData[num++] = data.bodyMap[id].posture[b].x();
//                 frameData[num++] = data.bodyMap[id].posture[b].y();
//                 frameData[num++] = data.bodyMap[id].posture[b].z();
//                 frameData[num++] = data.bodyMap[id].posture[b].w();
//             }
//             for (int c = 0; c < data.bodyMap[id].jointC.rows(); c++)
//             {
//                 frameData[num++] = data.bodyMap[id].jointC(c, 0);
//                 frameData[num++] = data.bodyMap[id].jointC(c, 1);
//                 frameData[num++] = data.bodyMap[id].jointC(c, 2);
//             }
//             if(++i==numStaff) break;
//         }
//         return frameData;
//     }

//     int i=0;
//     for (auto iter:data.bodyMap)
//     {
//         int num = pos + ++i * 145;
//         frameData[num++] = iter.first;
//         for (int b = 0; b < iter.second.posture.size(); b++)
//         {
//             frameData[num++] = iter.second.posture[b].x();
//             frameData[num++] = iter.second.posture[b].y();
//             frameData[num++] = iter.second.posture[b].z();
//             frameData[num++] = iter.second.posture[b].w();
//         }
//         for (int c = 0; c < iter.second.jointC.rows(); c++)
//         {
//             frameData[num++] = iter.second.jointC(c, 0);
//             frameData[num++] = iter.second.jointC(c, 1);
//             frameData[num++] = iter.second.jointC(c, 2);
//         }
//     }
//     return frameData;
// }

// DataSet RDCWindow::ReadAframeData(vector<float> frameData)
// {
//     DataSet data;
//     if ((frameData.size() != dataSize) && (frameData.size() != dataSize + 1))
//     {
//         cout << "Wrong frame data size!" << endl;
//         return data;
//     }
//     data.time = frameData[0];
//     data.kVp = frameData[1];
//     data.mA = frameData[2];
//     data.dap = frameData[3];
//     data.cArm(0) = frameData[4];
//     data.cArm(1) = frameData[5];
//     data.cArm(2) = frameData[6];
//     data.bed(0) = frameData[7];
//     data.bed(1) = frameData[8];
//     data.bed(2) = frameData[9];
//     data.glassChk = frameData[10];
//     Affine3d glassAff = Affine3d::Identity();
//     glassAff.rotate(Quaterniond(frameData[11], frameData[12], frameData[13], frameData[14]));
//     glassAff.translate(Vector3d(frameData[15], frameData[16], frameData[17]));
//     data.glass_aff = glassAff;
//     int bodyNum = frameData[18];
//     int pos = 19;

//     for (int i = 0; i < min(numStaff, bodyNum); i++)
//     {
//         int num = pos + i * 145;
//         int id = frameData[num++];

//         for (int b = 0; b < data.bodyMap[i].posture.size(); b++)
//         {
//             data.bodyMap[id].posture[b].x() = frameData[num++];
//             data.bodyMap[id].posture[b].y() = frameData[num++];
//             data.bodyMap[id].posture[b].z() = frameData[num++];
//             data.bodyMap[id].posture[b].w() = frameData[num++];
//         }
//         for (int c = 0; c < data.bodyMap[i].jointC.rows(); c++)
//         {
//             data.bodyMap[id].jointC(c, 0) = frameData[num++];
//             data.bodyMap[id].jointC(c, 1) = frameData[num++];
//             data.bodyMap[id].jointC(c, 2) = frameData[num++];
//         }
//     }
//     return data;
// }

bool RDCWindow::SetPhantoms(string fileName)
{
    // for (int i = 0; i < numStaff; i++)
    // {
    //     auto phantom = new PhantomAnimator;
    //     indivPhantoms.push_back(phantom);
    // }
    extraPhantom = new PhantomAnimator;
    extraPhantom->LoadPhantom("M26", true);

    ifstream ifs(fileName);
    if (!ifs.is_open())
    {
        cout << "There is no " + fileName << endl;
        return false;
    }

    ifs >> numStaff;
    string firstLine;
    getline(ifs, firstLine);
    jointLengths.resize(numStaff);
    eyeR_vec.resize(numStaff);
    eyeL_vec.resize(numStaff);

    getline(ifs, firstLine);
    stringstream ss(firstLine);
    vector<int> boneIDs;
    string dump;
    ss >> dump >> dump >> dump;
    for (int i = 0; i < 17; i++)
    {
        int boneID;
        ss >> boneID;
        boneIDs.push_back(boneID);
    }

    for (int i = 0; i < numStaff; i++)
    {
        string aLine;
        getline(ifs, aLine);
        stringstream ss1(aLine);
        string name;
        double d;
        ss1 >> name;

        profileNames.push_back(name);
        for (int j = 0; j < 17; j++)
        {
            double d;
            ss1 >> d;
            jointLengths[i][boneIDs[j]] = d;
        }
        for (int j = 0; j < 3; j++)
        {
            ss1 >> d;
            eyeL_vec[i](j) = d;
        }
        for (int j = 0; j < 3; j++)
        {
            ss1 >> d;
            eyeR_vec[i](j) = d;
        }
    }

    ifs.close();
    return true;
}