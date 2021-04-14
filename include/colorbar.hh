#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <igl/opengl/glfw/Viewer.h>
#include <imgui/imgui.h>

void texture_from_colormap(const Eigen::MatrixXd &rgb, GLuint & id)
{
  int width = 1;
  int height = (int) rgb.rows();

  Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> cmap;
  if (rgb.maxCoeff() > 1.0) {
    cmap = rgb.cast<unsigned char>();
  } else {
    cmap = (rgb.array() * 255.f).cast<unsigned char>();
  }
  assert(cmap.cols() == 3);
  cmap.conservativeResize(cmap.rows(), 4);
  cmap.col(3).setConstant(255);

  if (id == 0) {
    glGenTextures(1, &id);
  }
  glBindTexture(GL_TEXTURE_2D, id);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexImage2D(
    GL_TEXTURE_2D, 0, GL_RGBA,
    width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, cmap.data());
  glGenerateMipmap(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, 0);
}

class ColorbarPlugin {
public:
    ColorbarPlugin(igl::ColorMapType type, int _num_steps=21){
        colormap_type_ = type;
        init_colormap();
        num_steps = _num_steps;
    }
    void draw_colorbar(
    const Eigen::Vector4f & background_color, char* rMax, char* aMax) const;
protected:
    void init_colormap();

protected:
    GLuint colormaps_;
    igl::ColorMapType colormap_type_;
    int num_steps;
};

void ColorbarPlugin::init_colormap() {
    Eigen::MatrixXd rgb;
    GLuint id = 0;
    igl::colormap(colormap_type_,Eigen::VectorXd::LinSpaced(256,1,0).eval(),0,1,rgb);
    texture_from_colormap(rgb, id);
    colormaps_ = id;
}

// Draws the actual colorbar with min/max values
void ColorbarPlugin::draw_colorbar(
    const Eigen::Vector4f & background_color, char* rMax, char* aMax) const
{
    ImVec4 color(1,1,1,1);
    auto rgb = background_color;
    // http://stackoverflow.com/a/3943023/112731
    if (rgb[0] * 0.299 + rgb[1] * 0.587 + rgb[2] * 0.114 > 186) {
        color = ImVec4(1,1,1,1);
    }
    float w = 20;
    float h = 200;
    ImGui::Columns(3,0);
    ImGui::SetColumnWidth(0,ImGui::GetWindowWidth()*0.4);
    ImGui::SetColumnWidth(1,ImGui::GetWindowWidth()*0.2);
    ImGui::SetColumnWidth(2,ImGui::GetWindowWidth()*0.4);
//    ImGui::BeginGroup();
//    ImGui::BeginGroup();
    ImGui::Text("[pGy/h]");
    ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.3f);
    static char rMin[3]("0");
    ImGui::InputText("",rMax, ImGuiInputTextFlags_ReadOnly);
//    ImGui::TextColored(color, "%.3g", xmin);
    ImGui::Dummy(ImVec2(0, h - 2.2 * ImGui::GetItemRectSize().y));
//    ImGui::InputText("",test, ImGuiInputTextFlags_ReadOnly);
    ImGui::InputText("",rMin, ImGuiInputTextFlags_ReadOnly);
    ImGui::PopItemWidth();
//    ImGui::EndGroup();
//    ImGui::SameLine();
    ImGui::NextColumn();
//    ImGui::BeginGroup();
//    ImGui::BeginGroup();
    ImGui::Dummy(ImVec2(0, ImGui::GetFontSize()));
//    ImGui::EndGroup();
//    ImGui::SameLine();
    ImGui::Image(reinterpret_cast<ImTextureID>(colormaps_), ImVec2(w, h));
//    ImGui::EndGroup();
    ImGui::NextColumn();
    ImGui::Text("[pGy]");
    ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.3f);
    static char aMin[10]("1E-13(0)");
    ImGui::InputText("",aMax, ImGuiInputTextFlags_ReadOnly);
//    ImGui::TextColored(color, "%.3g", xmin);
    ImGui::Dummy(ImVec2(0, h - 2.2 * ImGui::GetItemRectSize().y));
//    ImGui::InputText("",test, ImGuiInputTextFlags_ReadOnly);
    ImGui::InputText("",aMin, ImGuiInputTextFlags_ReadOnly);

    ImGui::PopItemWidth();
   // ImGui::EndGroup();
}
