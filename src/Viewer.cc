#include "Viewer.hh"
#include "colorbar.hh"
#include <bitset>
#define PORT 30303

Viewer::Viewer(PhantomAnimator *_phantom)
    : sea_green(70. / 255., 252. / 255., 167. / 255.), white(1., 1., 1.), red(1., 0., 0.), blue(0., 0., 1.),
      listen(true), calib_signal(false), waitingCalib(false), initializing(true), phantom(_phantom)
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
    menu.callback_draw_viewer_menu = std::bind(&Viewer::MenuDesign, this);
}

Viewer::~Viewer()
{
    for (auto client : client_sockets)
        delete client.second;
    delete server;
}

static char num_client[3] = "0";
static char status[30] = "not listening";
static char runStatus[20] = "pre-init.";
static int calib_sock(20);
static int profileID(0);
static char newProfile[20] = "H_Han";
void Viewer::MenuDesign()
{
    float w = ImGui::GetContentRegionAvailWidth();
    float p = ImGui::GetStyle().FramePadding.x;
    if (ImGui::CollapsingHeader("Information", ImGuiTreeNodeFlags_None))
    {
        ImGui::Text("Author : Haegin Han");
        ImGui::Text("Info : RDC modult of DCIR system");
    }
    if (ImGui::CollapsingHeader("Connections", ImGuiTreeNodeFlags_DefaultOpen))
    {
        if (ImGui::Button("Start listening", ImVec2(w, 0)))
        {
            if (initializing)
            {
                strcpy(status, "listening");
                listen = true;
                if (!init_th.joinable())
                    init_th = thread(&Viewer::Communication_init, this);
            }
        }
        if (ImGui::Button("Stop listening", ImVec2(w, 0)))
        {
            if (initializing)
            {
                listen = false;
                strcpy(status, "not listening");
            }
        }
        ImGui::InputText("status", status, ImGuiInputTextFlags_None);
        ImGui::InputText("connected", num_client, ImGuiInputTextFlags_None);
    }
    static vector<string> profileNames;
    if (ImGui::CollapsingHeader("Initialization", ImGuiTreeNodeFlags_DefaultOpen))
    {
        vector<string> names = phantom->GetProfileNames();
        profileNames.resize(names.size());
        copy(names.begin(), names.end(), profileNames.begin());
        ImGui::Combo("profile", &profileID, profileNames);
        ImGui::InputText("add new profile", newProfile, ImGuiInputTextFlags_CharsNoBlank);
        ImGui::InputInt("socket num.", &calib_sock);
        if (ImGui::Button("Start measurement!", ImVec2(w, 0)))
        {
            if (phantom->AlreadyExists(newProfile))
                cout << newProfile << " aready exists!" << endl;
            else if (initializing)
                calib_signal = true;
        }
    }
    ImGui::Separator();
    ImGui::InputText("status", runStatus, ImGuiInputTextFlags_None);
    if (ImGui::Button("START!", ImVec2(w, 0)))
    {
        if (init_th.joinable())
        {
            initializing = false;
            init_th.join();
            phantom->CalibrateTo(profileNames[profileID]);
            viewer.callback_pre_draw = std::bind(&Viewer::Communication_run, this, std::placeholders::_1);
            viewer.core(v1_view).animation_max_fps = 10;
            viewer.core(v1_view).is_animating = true;
            strcpy(runStatus, "running");
        }
        else if (initializing)
        {
            strcpy(status, "Plz init first!");
        }
    }
    if (ImGui::Button("run / pause", ImVec2(w, 0)))
    {
        viewer.core(v1_view).is_animating = !viewer.core(v1_view).is_animating;
        if (viewer.core(v1_view).is_animating)
        {
            strcpy(runStatus, "running");
            // cv.notify_all();
        }
        else
            strcpy(runStatus, "idle");
    }

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
    //   };
}

void Viewer::SetMeshes()
{
    MatrixXd V = phantom->GetV();
    MatrixXi F = phantom->GetF();
    MatrixXd C = phantom->GetC();
    MatrixXi BE = phantom->GetBE();
    viewer.data().set_mesh(V, F);
    viewer.append_mesh();
    viewer.load_mesh_from_file("patient3.obj");
    viewer.append_mesh();
    MatrixXd V_cArm, V_charuco;
    MatrixXi F_cArm, F_charuco, F_glass;
    igl::readPLY("c-arm.ply", V_cArm, F_cArm);
    igl::readPLY("charuco.ply", V_charuco, F_charuco);
    igl::readPLY("glass.ply", V_glass, F_glass);
    viewer.data().set_mesh(V_cArm, F_cArm);
    viewer.append_mesh();
    viewer.data().set_mesh(V_charuco, F_charuco);
    viewer.append_mesh();
    viewer.data().set_mesh(V_glass, F_glass);
    viewer.append_mesh();
    MatrixXd V_cumul(V);
    viewer.data().set_mesh(V, F);

    v1 = viewer.data_list[0].id;
    v1_patient = viewer.data_list[1].id;
    v1_cArm = viewer.data_list[2].id;
    v1_charuco = viewer.data_list[3].id;
    v1_glass = viewer.data_list[4].id;
    v2 = viewer.data_list[5].id;

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
    viewer.data(v1_cArm).set_colors(white);
    viewer.data(v1_cArm).show_lines = false;
    viewer.data(v1_cArm).is_visible = false;
    viewer.data(v2).show_lines = false;
    viewer.data(v2).show_overlay_depth = true;
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

        viewer.core(v1_view).camera_eye = Eigen::Vector3f(20, -20, 20);
        viewer.core(v1_view).camera_center = Eigen::Vector3f(0, 0, 10);
        viewer.core(v1_view).camera_up = Eigen::Vector3f(0, 0, 1);

        viewer.core(v2_view).camera_eye = Eigen::Vector3f(0, 0, -3);
        viewer.core(v2_view).camera_up = Eigen::Vector3f(0, -1, 0);

        viewer.data(v1).set_visible(false, v2_view);
        viewer.data(v1_patient).set_visible(false, v2_view);
        viewer.data(v1_cArm).set_visible(false, v2_view);
        viewer.data(v1_charuco).set_visible(false, v2_view);
        viewer.data(v2).set_visible(false, v1_view);

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

void Viewer::Communication_init()
{
    fd_set tmp;
    server = new ServerSocket(PORT);
    cout << "waiting for connections..." << endl;
    FD_ZERO(&readfds);
    FD_SET(server->GetSocket(), &readfds);
    max_sd = server->GetSocket();
    //bitwise operation (motion, class, bed, c-arm): ex) motion + bed = 10

    //packages to broadcast
    array<double, 155> pack, pack_recv;
    RotationList alignRot = phantom->GetAlignRot();
    MatrixXi BE = phantom->GetBE();
    int num_bone = phantom->GetBE().rows();
    for (int i = 0; i < num_bone; i++)
    {
        pack[6 * i] = alignRot[i].w();
        pack[6 * i + 1] = alignRot[i].x();
        pack[6 * i + 2] = alignRot[i].y();
        pack[6 * i + 3] = alignRot[i].z();
        pack[6 * i + 4] = BE(i, 0);
        pack[6 * i + 5] = BE(i, 1);
    }

    //before init.
    while (initializing)
    {
        tmp = readfds;

        sel_timeout.tv_sec = 3;
        sel_timeout.tv_usec = 0;
        int activity = select(max_sd + 1, &tmp, NULL, NULL, &sel_timeout);
        if (activity < 0 && errno != EINTR)
            cout << "" << flush;
        if (FD_ISSET(server->GetSocket(), &tmp))
        {
            ServerSocket *_sock = new ServerSocket();
            server->accept(*_sock);
            if (!listen)
            {
                (*_sock) << "connection refused!";
                //delete _sock;
            }
            else
            {
                int sd = _sock->GetSocket();
                FD_SET(sd, &readfds);
                if (sd > max_sd)
                    max_sd = sd;

                strcpy(num_client, to_string(atoi(num_client) + 1).c_str());
                cout << "new connection from "
                     << inet_ntoa(_sock->GetAdrrInfo().sin_addr)
                     << " (sock #" << sd << ") / opt: ";
                (*_sock) << "welcome socket #" + to_string(sd) + "!";

                _sock->RecvDoubleBuffer(pack_recv.data(), 8);
                int tracking_id = (int)pack_recv[0];
                sock_opts[sd] = tracking_id;
                client_sockets[sd] = _sock;
                if (tracking_id & 8)
                    cout << "motion ";
                if (tracking_id & 4)
                    cout << "glass ";
                if (tracking_id & 2)
                    cout << "bed ";
                if (tracking_id & 1)
                    cout << "c-arm ";
                cout << endl;

                Affine3d aff = Affine3d::Identity();
                aff.rotate(Quaterniond(pack_recv[4], pack_recv[1], pack_recv[2], pack_recv[3]).normalized().toRotationMatrix().transpose());
                aff.translate(-Vector3d(pack_recv[5], pack_recv[6], pack_recv[7]));
                sock_coord[sd] = aff;

                if (tracking_id > 1)
                {
                    cout << "-> Sending alignRot/BE data.." << flush;
                    _sock->SendDoubleBuffer(pack.data(), num_bone * 6);
                    cout << "success" << endl;
                }
            }
        }
        if (calib_signal && (!waitingCalib))
        {
            if (!FD_ISSET(calib_sock, &tmp))
                cout << "socket #" << calib_sock << " is unavailable!" << endl;
            else
            {
                string msg;
                (*client_sockets[calib_sock]) >> msg;
                int signal(-1); //signal for calib.
                client_sockets[calib_sock]->SendIntBuffer(&signal, 1);
                cout << "Start calibration in socket #" << calib_sock << endl;
                waitingCalib = true;
            }
            calib_signal = false;
        }
        else if (waitingCalib && FD_ISSET(calib_sock, &tmp))
        {
            cout << "Get calibration data.." << flush;
            int signal(-1);
            //client_sockets[calib_sock]->RecvIntBuffer(&signal, 1);
            client_sockets[calib_sock]->RecvDoubleBuffer(pack.data(), 155);
            map<int, double> calibLengths;
            Vector3d eyeL_pos(0, 0, 0), eyeR_pos(0, 0, 0);
            int i = 0;
            for (; i < 155; i += 2)
            {
                if (pack[i] < 0)
                    break;
                calibLengths[pack[i]] = pack[i + 1];
            }
            eyeL_pos = Vector3d(pack[i + 1], pack[i + 2], pack[i + 3]);
            eyeR_pos = Vector3d(pack[i + 4], pack[i + 5], pack[i + 6]);
            int calibFrame = pack[i + 7];
            cout << "done" << endl;
            waitingCalib = false;

            for (i = 0; i < BE.rows(); i++)
            {
                if (calibLengths.find(i) == calibLengths.end())
                    continue;
                calibLengths[i] /= (double)calibFrame * 10;
            }
            eyeR_pos /= (double)calibFrame * 10;
            eyeL_pos /= (double)calibFrame * 10;
            profileID = phantom->AddProfile(calibLengths, eyeL_pos, eyeR_pos, newProfile);
            phantom->WriteProfileData("profile.txt");
            (*client_sockets[calib_sock]) << "Profile data for " + string(newProfile) + " was successfully transmitted!";
        }
    }
    string msg;
    for (auto client : client_sockets)
    {
        (*client.second) >> msg;
        int signal(1);
        client.second->SendIntBuffer(&signal, 1);
    }
}

bool Viewer::Communication_run(igl::opengl::glfw::Viewer &)
{
    // igl::Timer timer;
    // timer.start();
    fd_set tmp;
    array<float, 375> pack; //max ()
    int numBE = phantom->GetBE().rows();
    int dataNum = (24 + numBE) * 4;
    if (viewer.core(v1_view).is_animating)
    {
        // if (!running)
        // {
        //     unique_lock<mutex> lock(m);
        //     cv.wait(lock);
        // }
        tmp = readfds;
        sel_timeout.tv_sec = 0;
        sel_timeout.tv_usec = 1;

        int activity = select(max_sd + 1, &tmp, NULL, NULL, &sel_timeout);
        if (activity < 0 && errno != EINTR)
            return true;
        if (FD_ISSET(server->GetSocket(), &tmp))
        {
            ServerSocket _sock;
            server->accept(_sock);
            _sock << "connection refused!";
        }

        Affine3d glass_aff = Affine3d::Identity();
        RotationList vQ;
        vQ.resize(numBE);
        MatrixXd C_disp(24, 3);
        MatrixXi BE = phantom->GetBE();
        unsigned int reliability(0), r(0);
        vector<int> eraseSID;
        for (auto sid : sock_opts)
        {
            if (!FD_ISSET(sid.first, &tmp))
                continue;

            if (!client_sockets[sid.first]->RecvFloatBuffer(pack.data(), 375))
            {
                eraseSID.push_back(sid.first);
                continue;
            }
            // for (int i = 0; i < 375; i++)
            //     cout << i << ": " << pack[i] << endl;
            // cout << "-----------------" << endl;
            int capture_opt = pack[374];

            int signal(1);
            client_sockets[sid.first]->SendIntBuffer(&signal, 1);

            if (capture_opt & 2) //bed
            {
            }
            if (capture_opt & 4) //glass (0-7)
            {
                Quaterniond q;
                Vector3d t;
                q.x() = pack[0]; q.y() = pack[1]; q.z() = pack[2]; q.w() = pack[3];
                t(0) = pack[4]; t(1) = pack[5]; t(2) = pack[6];

                Affine3d aff = Affine3d::Identity();
                aff.translate(t);
                aff.rotate(q);
                glass_aff = sock_coord[sid.first] * aff;

                //cout << glass_aff.matrix() << endl;
                viewer.data(v1_glass).set_vertices((V_glass.rowwise().homogeneous() * glass_aff.matrix().transpose()).rowwise().hnormalized());
                viewer.data(v1_glass).compute_normals();
            }
            if (capture_opt & 8) //motion (7-)
            {
                if (!reliability)
                {
                    int i = 7;
                    for (int n = 0; n < 24; n++)
                        reliability |= bool(pack[i]) << i++;
                    for (int n = 0; n < 24; n++)
                    {
                        C_disp(n, 0) = pack[i++];
                        C_disp(n, 1) = pack[i++];
                        C_disp(n, 2) = pack[i++];
                    }
                    TransformVertices(C_disp, sock_coord[sid.first]);
                    for (int r = 0; r < numBE; i += 4, r++)
                        vQ[r] = sock_coord[sid.first].rotation() * Quaterniond(pack[i], pack[i + 1], pack[i + 2], pack[i + 3]);
                }
                else
                {
                    int i = 7;
                    for (int n = 0; n < 24; n++)
                        r |= bool(pack[i]) << i++;
                    unsigned int r_chk = (~reliability) & r; //not existing, but exists in the new data
                    for (int row = 0; row < 24; row++)
                    {
                        if (!(r_chk & reliab_opt[row]))
                        {
                            i += 3;
                            continue;
                        }
                        C_disp(row, 0) = pack[i++];
                        C_disp(row, 1) = pack[i++];
                        C_disp(row, 2) = pack[i++];
                    }
                    TransformVertices(C_disp, sock_coord[sid.first]);
                    for (int row = 0; row < numBE; i += 4, row++)
                    {
                        if (!(r_chk & reliab_opt[BE(row, 0)]))
                        {
                            i += 4;
                            continue;
                        }
                        vQ[row] = sock_coord[sid.first].rotation() * Quaterniond(pack[i], pack[i + 1], pack[i + 2], pack[i + 3]);
                    }

                    reliability |= r;
                }
            }

            if (sid.second & 1) //c-arm
            {
            }
        }
        for (int i : eraseSID)
        {
            client_sockets.erase(i);
            sock_opts.erase(i);
        }
        // timer.stop();
        // cout << "data transfer: " << timer.getElapsedTime() << endl;
        // timer.start();
        if (reliability)
        {
            MatrixXd C_new, V_new;
            phantom->Animate(vQ, C_disp, C_new, V_new);
            // timer.stop();
            // cout << "animation: " << timer.getElapsedTime() << endl;
            // timer.start();

            viewer.data(v1).set_points(C_disp, blue);
            viewer.data(v1).set_edges(C_new, BE, sea_green);

            viewer.data(v1).set_vertices(V_new);
            viewer.data(v1).compute_normals();

            // timer.stop();
            // cout << "vis: " << timer.getElapsedTime() << endl;
        }
        //cout<<bitset<22>(reliability)<<endl;
    }
    return false;
}
