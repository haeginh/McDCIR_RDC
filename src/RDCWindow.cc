#include "RDCWindow.hh"
#include <igl/readPLY.h>
#include <external/imgui/backends/imgui_impl_glfw.h>
#include "PhantomAnimator.hh"
#include "Communicator.hh"

RDCWindow &RDCWindow::Instance()
{
    static RDCWindow RDCWindow;
    return RDCWindow;
}

static void MainMenuBar();
static void ShowSettingsWindow(bool *p_open);
static void ShowStatusWindow(bool *p_open);
static void DoseResultTab();
static void DoseGraphTab();
static void ProgramLogTab();
static void ShowProgramInformationPopUp(bool *p_open);
void ColorInfoWindow(float min, float max, string unit);
static void ShowColorInfoPopUp(bool *p_open);

void RDCWindow::Initialize()
{
    // MatrixXd V;
    // MatrixXi F;
    // igl::readPLY("../glass.ply", V, F);
    // viewer.data().set_mesh(V, F);

    mainMenu.callback_draw_viewer_window = []() -> bool
    {
        MainMenuBar();
        return true;
    };
    viewer.plugins.push_back(&mainMenu);

    // multi view
    static unsigned int radiolRate_view, radiolAcc_view, patient_view;
    viewer.callback_init = [&](igl::opengl::glfw::Viewer &_viewer)
    {
        int w, h;
        glfwGetWindowSize(_viewer.window, &w, &h);
        float phantomViewX = h * 0.35;
        // radiolRate
        _viewer.core().viewport = Eigen::Vector4f(0, h * 0.29, w - phantomViewX * 2, h * 0.71);
        radiolRate_view = viewer.core_list[0].id;
        _viewer.core(radiolRate_view).background_color = Eigen::Vector4f(0.95, 0.95, 0.95, 1.);
        _viewer.core(radiolRate_view).camera_center = Vector3f(0, 0, 0);
        _viewer.core(radiolRate_view).camera_up = Vector3f(0, 0, 1);
        _viewer.core(radiolRate_view).camera_eye = Vector3f(60, 0, 1);
        // radiolAcc
        radiolAcc_view = viewer.append_core(Eigen::Vector4f(w - phantomViewX * 2, h * 0.29, phantomViewX, h * 0.71));
        _viewer.core(radiolAcc_view).background_color = Eigen::Vector4f(255. / 255., 255. / 255., 255. / 255., 0.);
        // patient
        patient_view = viewer.append_core(Eigen::Vector4f(w - phantomViewX, h * 0.29, phantomViewX, h * 0.71));
        _viewer.core(patient_view).background_color = Eigen::Vector4f(82. / 255., 82. / 255., 82. / 255., 0.);

        //visiblity
        viewer.data(this->phantom_data).is_visible = 0;
        viewer.data(this->apron_data).is_visible = 0;
        viewer.data(this->patient_data).is_visible = 0;
        viewer.data(this->table_data).is_visible = 0;
        viewer.data(this->cArm_data).is_visible = 0;
        viewer.data(this->beam_data).is_visible = 0;
        viewer.data(this->glass_data).is_visible = 0;
        viewer.data(this->phantomAcc_data).is_visible = 0;
        
        viewer.data(this->phantom_data).is_visible |= radiolRate_view;
        viewer.data(this->apron_data).is_visible |= radiolRate_view;
        viewer.data(this->patient_data).is_visible |= radiolRate_view;
        viewer.data(this->table_data).is_visible |= radiolRate_view;
        viewer.data(this->cArm_data).is_visible |= radiolRate_view;
        viewer.data(this->beam_data).is_visible |= radiolRate_view;
        viewer.data(this->glass_data).is_visible |= radiolRate_view;
        viewer.data(this->phantomAcc_data).is_visible |= radiolAcc_view;
        return false;
    };
    viewer.callback_post_resize = [&](igl::opengl::glfw::Viewer &v, int w, int h)
    {
        float phantomViewX = h * 0.35;
        // radiolRate
        v.core(radiolRate_view).viewport = Eigen::Vector4f(0, h * 0.29, w - phantomViewX * 2, h * 0.71);
        // radiolAcc
        v.core(radiolAcc_view).viewport = Eigen::Vector4f(w - phantomViewX * 2, h * 0.29, phantomViewX, h * 0.71);
        // patient
        v.core(patient_view).viewport = Eigen::Vector4f(w - phantomViewX, h * 0.29, phantomViewX, h * 0.71);
        return true;
    };

    //set meshes
    SetMeshes(".");
  
    //callback functions
    viewer.callback_pre_draw = bind(&RDCWindow::Animate, this, placeholders::_1);
    
    viewer.callback_post_draw = [](igl::opengl::glfw::Viewer &_viewer) -> bool
    {
        // to prevent program to be idle
        if(RDCWindow::Instance().running) _viewer.core().is_animating = true;
        else _viewer.core().is_animating = false;
        // _viewer.core().is_animating = true;
        return true;
    };
}

bool RDCWindow::Animate(igl::opengl::glfw::Viewer &_viewer)
{
    DataSet data = Communicator::Instance().current;
    PhantomAnimator::Instance().Animate(data.posture, data.jointC, _viewer.data(phantom_data).points);
    auto phantom  = PhantomAnimator::Instance();
    _viewer.data(phantom_data).V = phantom.U;
    _viewer.data(phantom_data).compute_normals();

    return false;  
}

void RDCWindow::SetMeshes(string dir)
{
    //radiologist phantom
    auto phantom  = PhantomAnimator::Instance();
    viewer.data().set_mesh(phantom.U, phantom.F);
    viewer.data().set_edges(phantom.C, phantom.BE, RowVector3d(70. / 255., 252. / 255., 167. / 255.));
    phantom_data = viewer.selected_data_index;
    viewer.append_mesh();
    viewer.data().set_mesh(phantom.U_apron, phantom.F_apron);
    apron_data = viewer.selected_data_index;
    viewer.append_mesh();
    viewer.data().set_mesh(phantom.V, phantom.F);
    phantomAcc_data = viewer.selected_data_index;

    //read other meshes
    MatrixXd V_cArm, V_patient, V_table, V_beam, V_glass;
    MatrixXi F_cArm, F_patient, F_glass, F_beam, F_table;
    igl::readPLY(dir + "/c-arm.ply", V_cArm, F_cArm);
    igl::readPLY(dir + "/patient.ply", V_patient, F_patient);
    igl::readPLY(dir + "/table.ply", V_table, F_table);
    igl::readPLY(dir + "/beam.ply", V_beam, F_beam);
    igl::readPLY(dir + "/glass.ply", V_glass, F_glass);

    //for shadow factors
    // MatrixXd B_patient0, N_patient0, A_patient0;
    // igl::barycenter(V_patient, F_patient, B_patient0);
    // igl::per_face_normals(V_patient, F_patient, N_patient0);
    // igl::doublearea(V_patient, F_patient, A_patient0);
    // B_patient = B_patient0.cast<float>();
    // N_patient = N_patient0.cast<float>();
    // A_patient = A_patient0.cast<float>();
    
    //append other meshes
    viewer.append_mesh();
    viewer.data().set_mesh(V_patient, F_patient);
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

    //draw grids
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
    viewer.append_mesh();
    viewer.data().set_edges(V_grid, E_grid, C_grid);

    //viewer settings
    viewer.data(phantom_data).show_lines = true;
    viewer.data(phantom_data).show_overlay_depth = false;
    viewer.data(phantom_data).line_width = 1;
    viewer.data(phantom_data).point_size = 8;
    viewer.data(phantom_data).double_sided = false;
    viewer.data(apron_data).double_sided = true;
    viewer.data(apron_data).show_lines = false;
    viewer.data(apron_data).set_colors(RowVector4d(0.2, 0.2, 0.2, 0.7));
    viewer.data(patient_data).show_lines = true;
    viewer.data(patient_data).show_overlay_depth = false;
    viewer.data(patient_data).show_faces = false;
    viewer.data(patient_data).line_color = RowVector4f(0, 0.2, 0, 0.2);
    viewer.data(table_data).set_colors(RowVector4d(0, 0.2, 0, 0.2));
    viewer.data(table_data).show_lines = false;
    viewer.data(cArm_data).set_colors(RowVector4d(1., 1., 1., 1.));
    viewer.data(cArm_data).show_lines = false;
    viewer.data(cArm_data).double_sided = false;
    viewer.data(cArm_data).set_colors(RowVector4d(0.8, 0.8, 0.8, 1.));
    viewer.data(beam_data).set_colors(RowVector4d(1, 0, 0, 0.2));
    viewer.data(beam_data).show_lines = false;
    viewer.data(phantomAcc_data).point_size = 4;
    viewer.data(phantomAcc_data).show_lines = false;
    viewer.data(phantomAcc_data).double_sided = false;
}

void MainMenuBar()
{
    static bool show_status_window = true;
    static bool show_settings_window = true;
    static bool show_color_popup = true;
    static bool show_info_popup = false;
    if (show_status_window)
        ShowStatusWindow(&show_status_window);
    if (show_color_popup)
        ShowColorInfoPopUp(&show_color_popup);
    if (show_settings_window)
        ShowSettingsWindow(&show_settings_window);
    if (show_info_popup)
        ShowProgramInformationPopUp(&show_info_popup);
    if (ImGui::BeginMainMenuBar())
    {
        if (ImGui::BeginMenu("View"))
        {
            if (ImGui::MenuItem("show status", "", &show_status_window))
                ;
            if (ImGui::MenuItem("show color bar", "", &show_color_popup))
                ;
            if (ImGui::MenuItem("show info.", "", &show_info_popup))
                ;
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Settings"))
        {
            ImGui::SetNextItemWidth(ImGui::GetFontSize() * 7.f);
            ImGui::DragFloat("font size", &ImGui::GetFont()->Scale, 0.005f, 0.3f, 2.0f, "%.1f");
            if (ImGui::MenuItem("open settings", ""))
            {
                show_settings_window = true;
            }
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Export"))
        {
            ImGui::EndMenu();
        }
        if (ImGui::Checkbox("running", &RDCWindow::Instance().running))
        {
            if (RDCWindow::Instance().running)
                RDCWindow::Instance().viewer.core().is_animating = true;
            // else
            //     RDCWindow::Instance().viewer.core().is_animating = false;
        }
        ImGui::EndMainMenuBar();
    }
}

void ShowStatusWindow(bool *p_open)
{
    ImGuiWindowFlags flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing;
    const ImGuiViewport *viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(ImVec2(0.f, viewport->WorkPos.y + viewport->WorkSize.y), ImGuiCond_Always, ImVec2(0., 1.f));
    ImGui::SetNextWindowSize(ImVec2(viewport->WorkSize.x, viewport->WorkSize.y * 0.3f));
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(8. / 255., 15. / 255., 26. / 255., 1.0f));
    if (ImGui::Begin("Status", p_open, flags))
    {
        if (ImGui::BeginTabBar("StatusTabBar", ImGuiTabBarFlags_None))
        {
            DoseResultTab();
            DoseGraphTab();
            ProgramLogTab();
            ImGui::EndTabBar();
        }
    }
    ImGui::PopStyleColor();
    ImGui::End();
}

void DoseResultTab()
{
    vector<string> radiologistDose = {"Whole skin", "Right hand (palm)", "Left hand (palm)", "PSD", "Lens"};
    vector<string> patientDose = {"Whole skin", "PSD"};
    if (ImGui::BeginTabItem("Dose values"))
    {
        ImGui::Columns(2, "", false);
        ImGuiTableFlags flags = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_Reorderable;
        ImGui::BulletText("Radiologist Dose");
        if (ImGui::BeginTable("radiologist", 3, flags))
        {
            ImGui::TableSetupColumn("", ImGuiTableColumnFlags_NoReorder | ImGuiTableColumnFlags_WidthFixed);
            ImGui::TableSetupColumn("Dose Rate (mGy/hr)");
            ImGui::TableSetupColumn("Cumulative Dose (uGy)");
            ImGui::TableHeadersRow();
            for (size_t row = 0; row < radiologistDose.size(); row++)
            {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::BulletText(radiologistDose[row].c_str());
            }
            ImGui::EndTable();
        }
        ImGui::NextColumn();
        ImGui::BulletText("Patient Dose");
        if (ImGui::BeginTable("patient", 3, flags))
        {
            ImGui::TableSetupColumn("", ImGuiTableColumnFlags_NoReorder | ImGuiTableColumnFlags_WidthFixed);
            ImGui::TableSetupColumn("Dose Rate (mGy/hr)");
            ImGui::TableSetupColumn("Cumulative Dose (uGy)");
            ImGui::TableHeadersRow();
            for (size_t row = 0; row < patientDose.size(); row++)
            {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::BulletText(patientDose[row].c_str());
            }
            ImGui::EndTable();
        }
        ImGui::NextColumn();
        ImGui::EndTabItem();
    }
}

void DoseGraphTab()
{
    static int offset = 0;
    if (ImGui::BeginTabItem("Dose rate gragphs"))
    {
        static int frameTime = 2;
        const ImGuiViewport *viewport = ImGui::GetMainViewport();
        ImGui::SetNextItemWidth(viewport->WorkSize.x * 0.5);
        ImGui::SliderInt("", &frameTime, 1, 10);
        char buf[200];
        sprintf(buf, "frame time: %d sec / plot length: %d sec", frameTime, frameTime * 100);
        ImGui::SameLine();
        ImGui::Text(buf);
        static float whole_skin[100] = {};
        static float hands[100] = {};
        static float lens[100] = {};
        static float patient[100] = {};
        for (int i = 0; i < 100; i++)
        {
            whole_skin[i] = 0.1 * i;
            hands[i] = 0.1 * i;
            lens[i] = 0.1 * i;
            patient[i] = 0.1 * i;
        }
        offset = ++offset % 100;
        ImGui::Columns(2, "", false);
        ImGui::BulletText("Whole skin dose rate (radiologist)");
        ImGui::PlotLines("", whole_skin, 100, offset, "", 0, 10, ImVec2(viewport->WorkSize.x * 0.48, viewport->WorkSize.y * 0.085));
        ImGui::BulletText("Hands dose rate (radiologist)");
        ImGui::PlotLines("", hands, 100, offset, "", 0, 10, ImVec2(viewport->WorkSize.x * 0.48, viewport->WorkSize.y * 0.085));
        ImGui::NextColumn();
        ImGui::BulletText("Lens dose rate (radiologist)");
        ImGui::PlotLines("", lens, 100, offset, "", 0, 10, ImVec2(viewport->WorkSize.x * 0.48, viewport->WorkSize.y * 0.085));
        ImGui::BulletText("Patient PSD");
        ImGui::PlotLines("", patient, 100, offset, "", 0, 10, ImVec2(viewport->WorkSize.x * 0.48, viewport->WorkSize.y * 0.085));
        ImGui::NextColumn();
        ImGui::EndTabItem();
    }
}

//-----------------------------------------------------------------------------
// [SECTION] Example App: Debug Log / ShowExampleAppLog()
//-----------------------------------------------------------------------------

// Usage:
//  static ExampleAppLog my_log;
//  my_log.AddLog("Hello %d world\n", 123);
//  my_log.Draw("title");

struct DCIRAppLog
{
    ImGuiTextBuffer Buf;
    ImGuiTextFilter Filter;
    ImVector<int> LineOffsets; // Index to lines offset. We maintain this with AddLog() calls.
    bool AutoScroll;           // Keep scrolling if already at the bottom.

    DCIRAppLog()
    {
        AutoScroll = true;
        Clear();
    }

    void Clear()
    {
        Buf.clear();
        LineOffsets.clear();
        LineOffsets.push_back(0);
    }

    void AddLog(const char *fmt, ...) IM_FMTARGS(2)
    {
        int old_size = Buf.size();
        va_list args;
        va_start(args, fmt);
        Buf.appendfv(fmt, args);
        va_end(args);
        for (int new_size = Buf.size(); old_size < new_size; old_size++)
            if (Buf[old_size] == '\n')
                LineOffsets.push_back(old_size + 1);
    }

    void Draw()
    {
        // Options menu
        if (ImGui::BeginPopup("Options"))
        {
            ImGui::Checkbox("Auto-scroll", &AutoScroll);
            ImGui::EndPopup();
        }

        // Main window
        bool clear = ImGui::Button("Clear");
        ImGui::SameLine();
        bool copy = ImGui::Button("Copy");
        ImGui::SameLine();
        Filter.Draw("Filter", -100.0f);

        ImGui::BeginChild("scrolling", ImVec2(0, 0), false, ImGuiWindowFlags_HorizontalScrollbar);

        if (clear)
            Clear();
        if (copy)
            ImGui::LogToClipboard();

        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0, 0));
        const char *buf = Buf.begin();
        const char *buf_end = Buf.end();
        if (Filter.IsActive())
        {
            // In this example we don't use the clipper when Filter is enabled.
            // This is because we don't have a random access on the result on our filter.
            // A real application processing logs with ten of thousands of entries may want to store the result of
            // search/filter.. especially if the filtering function is not trivial (e.g. reg-exp).
            for (int line_no = 0; line_no < LineOffsets.Size; line_no++)
            {
                const char *line_start = buf + LineOffsets[line_no];
                const char *line_end = (line_no + 1 < LineOffsets.Size) ? (buf + LineOffsets[line_no + 1] - 1) : buf_end;
                if (Filter.PassFilter(line_start, line_end))
                    ImGui::TextUnformatted(line_start, line_end);
            }
        }
        else
        {
            // The simplest and easy way to display the entire buffer:
            //   ImGui::TextUnformatted(buf_begin, buf_end);
            // And it'll just work. TextUnformatted() has specialization for large blob of text and will fast-forward
            // to skip non-visible lines. Here we instead demonstrate using the clipper to only process lines that are
            // within the visible area.
            // If you have tens of thousands of items and their processing cost is non-negligible, coarse clipping them
            // on your side is recommended. Using ImGuiListClipper requires
            // - A) random access into your data
            // - B) items all being the  same height,
            // both of which we can handle since we an array pointing to the beginning of each line of text.
            // When using the filter (in the block of code above) we don't have random access into the data to display
            // anymore, which is why we don't use the clipper. Storing or skimming through the search result would make
            // it possible (and would be recommended if you want to search through tens of thousands of entries).
            ImGuiListClipper clipper;
            clipper.Begin(LineOffsets.Size);
            while (clipper.Step())
            {
                for (int line_no = clipper.DisplayStart; line_no < clipper.DisplayEnd; line_no++)
                {
                    const char *line_start = buf + LineOffsets[line_no];
                    const char *line_end = (line_no + 1 < LineOffsets.Size) ? (buf + LineOffsets[line_no + 1] - 1) : buf_end;
                    ImGui::TextUnformatted(line_start, line_end);
                }
            }
            clipper.End();
        }
        ImGui::PopStyleVar();

        if (AutoScroll && ImGui::GetScrollY() >= ImGui::GetScrollMaxY())
            ImGui::SetScrollHereY(1.0f);

        ImGui::EndChild();
    }
};

void ProgramLogTab()
{
    if (ImGui::BeginTabItem("System sataus"))
    {
        ImGui::Columns(2, "", false);
        const ImGuiViewport *viewport = ImGui::GetMainViewport();
        ImGuiStyle &style = ImGui::GetStyle();
        ImGui::SetColumnWidth(0, (viewport->WorkSize.x - style.FramePadding.x * 2) * 0.58f);
        ImGui::SetColumnWidth(1, (viewport->WorkSize.x - style.FramePadding.x * 2) * 0.4f);
        ImGui::BulletText("Tracking information");
        ImGuiTableFlags flags = ImGuiTableFlags_Borders | ImGuiTableFlags_Reorderable;
        if (ImGui::BeginTable("Tracking_info", 3, flags))
        {
            ImGui::TableSetupColumn("OCR");
            ImGui::TableSetupColumn("Glass");
            ImGui::TableSetupColumn("Posture");
            ImGui::TableHeadersRow();
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::BulletText("RAO/LAO: RAO 30");
            ImGui::BulletText("CRAN/CAUD: CRAN 20");
            ImGui::BulletText("SID: 110 cm");
            ImGui::BulletText("Beam: 80 kVp, 20 mA");
            ImGui::BulletText("Table Pos: (10, 5, 21) cm");
            ImGui::TableSetColumnIndex(1);
            ImGui::BulletText("Pos: (10, 20, 22) cm");
            ImGui::BulletText("Quat: (0.5, -0.2, 0.5, 0.6)");
            ImGui::TableSetColumnIndex(2);
            ImGui::BulletText("# of detected people: 2");
            ImGui::BulletText("P1: (0, 20, -15) cm / 0.89");
            ImGui::BulletText("P2: (10, 22, -25) cm / 0.71");
            ImGui::EndTable();
        }
        ImGui::NextColumn();
        ImGui::BulletText("Program log");
        static DCIRAppLog logApp;
        logApp.AddLog("[%05d] [%.1f] This is log!!\n", ImGui::GetFrameCount(), ImGui::GetTime());
        logApp.Draw();
        ImGui::EndTabItem();
    }
}

void ShowSettingsWindow(bool *p_open)
{
    ImGuiWindowFlags flags = ImGuiWindowFlags_NoCollapse|ImGuiWindowFlags_AlwaysAutoResize;
    const ImGuiViewport *viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowSize(ImVec2(ImGui::GetItemRectSize().x * 2, -1), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Settings", p_open, flags))
    {
        ImGui::BulletText("Phantom settings");
        int phantomNum(2);
        ImGui::Combo("", &phantomNum, BFlist);
        ImGui::SameLine();
        if (ImGui::Button("Load Phantom"))
        {
            PhantomAnimator::Instance().LoadPhantom("./phantoms/" + phantomlist[phantomNum]);
        }
        
        static int profileID(0);
        vector<string> names = PhantomAnimator::Instance().GetProfileNames();
        static vector<string> profileNames(names.size());
        copy(names.begin(), names.end(), profileNames.begin());
        ImGui::Combo("profile", &profileID, profileNames);
        ImGui::BulletText("Server machine settings");
        static int serverPort(30303);
        ImGui::SetNextItemWidth(ImGui::GetWindowSize().x * 0.4);
        ImGui::InputInt("Server port", &serverPort);
        ImGui::SameLine();
        static bool listen(false);
        if(ImGui::Checkbox("listen", &listen))
        {
            if(listen)
            {
                Communicator::Instance().StartServer(serverPort);
                Communicator::Instance().StartWorkers();
            }
            else Communicator::Instance().CloseServer();
        }
        ImGui::BulletText("Client machine settings");
        static string ip;//, port("22");
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
            if(!listen)
                cout<<"activate listen button first"<<endl;
            else
            {
                int option = 8 * (int)motionChk + 4 * (int)glassChk + 2 * (int)bedChk + 1 * (int)ocrChk;
                Communicator::Instance().StartWorker(ip, option);
            }
        }
        ImGuiTableFlags flags =  ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_NoBordersInBody;
            // | ImGuiTableFlags_SizingFixedFit;
        if (ImGui::BeginTable("worker machines", 8, flags))
        {
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
            for (auto iter:Communicator::Instance().workerData)
            {
                ImGui::PushID(id++);
                ImGui::TableNextRow();
                if(ImGui::TableNextColumn()) ImGui::Text(get<0>(iter.second).c_str());
                int opt = get<1>(iter.second);
                if(ImGui::TableNextColumn() && (opt&1)) ImGui::Text("*");
                if(ImGui::TableNextColumn() && (opt&2)) ImGui::Text("*");
                if(ImGui::TableNextColumn() && (opt&4)) ImGui::Text("*");
                if(ImGui::TableNextColumn() && (opt&8)) ImGui::Text("*");
                if(ImGui::TableNextColumn() && Communicator::Instance().GetDelay(iter.first)<0.01) ImGui::Text("*");
                if(ImGui::TableNextColumn() && ImGui::SmallButton("delete")) del = iter.first;
                ImGui::PopID();
            }
            ImGui::EndTable();
            if(del>=0) Communicator::Instance().DeleteWorker(del);
        }
        ImGui::PushStyleVar(ImGuiStyleVar_IndentSpacing, viewport->WorkSize.x * 0.5 - ImGui::GetFontSize() * 10.f);
        ImGui::PopStyleVar();
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
    ImGui::SetNextWindowPos(ImVec2(sx - phantomViewX * 2 - 10, sy * 0.71 - 10), ImGuiCond_Always, ImVec2(1.f, 1.f));
    ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoMove;
    if (ImGui::Begin("RadiolRate_color", p_open, window_flags))
        ColorInfoWindow(0, 10, "uGy/hr");
    ImGui::End();
    ImGui::SetNextWindowPos(ImVec2(sx - phantomViewX - 10, sy * 0.71 - 10), ImGuiCond_Always, ImVec2(1.f, 1.f));
    if (ImGui::Begin("RadiolAcc_color", p_open, window_flags))
        ColorInfoWindow(3, 20, "uGy");
    ImGui::End();
    ImGui::SetNextWindowPos(ImVec2(sx - 10, sy * 0.71 - 10), ImGuiCond_Always, ImVec2(1.f, 1.f));
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