// Copyright 2017-2019, Nicholas Sharp and the Polyscope contributors.
// http://polyscope.run.
#include "BPMShader.h"

#include "polyscope/file_helpers.h"
#include "polyscope/polyscope.h"
#include "polyscope/render/engine.h"

#include "imgui.h"

using std::cout;
using std::endl;

namespace polyscope {

// == Custom shader rule for projective texture interpolation
const render::ShaderReplacementRule MESH_PROPAGATE_BLENDED_MOBIUS_VALUE2(
    /* rule name */ "MESH_PROPAGATE_BLENDED_MOBIUS_VALUE2",
    {
        /* replacement sources */
        {"VERT_DECLARATIONS", R"(
          in vec2 v_z, v_coords;
          in vec3 v_bary, v_edge_lengths;
          in vec3 v_Mijk_re, v_Mijk_im, v_lij_re, v_lij_im, v_ljk_re, v_ljk_im, v_lki_re, v_lki_im;
          out vec2 f_z, f_coords;
          out vec3 f_bary, f_edge_lengths;
          out vec3 f_Mijk_re, f_Mijk_im, f_lij_re, f_lij_im, f_ljk_re, f_ljk_im, f_lki_re, f_lki_im;
        )"},
        {"VERT_ASSIGNMENTS", R"(
          f_z = v_z; f_coords = v_coords;
          f_bary = v_bary; f_edge_lengths = v_edge_lengths;
          f_Mijk_re = v_Mijk_re; f_Mijk_im = v_Mijk_im;
          f_lij_re = v_lij_re; f_lij_im = v_lij_im;
          f_ljk_re = v_ljk_re; f_ljk_im = v_ljk_im;
          f_lki_re = v_lki_re; f_lki_im = v_lki_im;
        )"},
        {"FRAG_FUNCTIONS", R"(
          vec2 mul(vec2 a, vec2 b) {
              return vec2(a.x * b.x - a.y * b.y, a.x * b.y + a.y * b.x);
          }
          vec2 inv(vec2 a) {
              return vec2(a.x, -a.y) / dot(a, a);
          }
          vec2 div(vec2 a, vec2 b) {
              return mul(a, inv(b));
          }
          float arg(vec2 z) {
              return atan(z.y, z.x);
          }
          void buildMoebiusTransformation(in vec3 m_re, in vec3 m_im, out mat2 M_re, out mat2 M_im) {
              vec2 a = vec2(m_re.x, m_im.x);
              vec2 b = vec2(m_re.y, m_im.y);
              vec2 c = vec2(m_re.z, m_im.z);
              vec2 d = div(mul(b, c) + vec2(1., 0.), a);
              M_re = mat2(a.x, b.x, c.x, d.x);
              M_im = mat2(a.y, b.y, c.y, d.y);
          }
          void buildLogMoebiusTransformation(in vec3 m_re, in vec3 m_im, out mat2 M_re, out mat2 M_im) {
              vec2 a = vec2(m_re.x, m_im.x);
              vec2 b = vec2(m_re.y, m_im.y);
              vec2 c = vec2(m_re.z, m_im.z);
              vec2 d = -a;
              M_re = mat2(a.x, b.x, c.x, d.x);
              M_im = mat2(a.y, b.y, c.y, d.y);
          }
          // https://math.stackexchange.com/a/44500
          vec2 sqrt(in vec2 z) {
              float r = length(z);
              return sqrt(r) * normalize(z + vec2(r, 0.));
          }
          vec2 exp(in vec2 z) {
              return exp(z.x) * vec2(cos(z.y), sin(z.y));
          }
          vec2 sinh(in vec2 z) {
              return (exp(z) - exp(-z)) / 2.;
          }
          vec2 cosh(in vec2 z) {
              return (exp(z) + exp(-z)) / 2.;
          }
          void exp(in mat2 X_re, in mat2 X_im, out mat2 expX_re, out mat2 expX_im) {
              vec2 a = vec2(X_re[0][0], X_im[0][0]);
              vec2 b = vec2(X_re[0][1], X_im[0][1]);
              vec2 c = vec2(X_re[1][0], X_im[1][0]);
              vec2 d = vec2(X_re[1][1], X_im[1][1]);

              // formula from Mathematica (TODO: find reference)
              vec2 e  = exp((a + d) / 2.);
              vec2 r  = sqrt(4. * mul(b, c) + mul(a - d, a - d));
              vec2 ch = cosh(r / 2.);
              vec2 sh = sinh(r / 2.);

              vec2 result_a = mul(e, (ch + div(mul(a - d , sh), r)));
              vec2 result_b = 2. * div(mul(mul(b, e), sh), r);
              vec2 result_c = 2. * div(mul(mul(c, e), sh), r);
              vec2 result_d = mul(e, (ch + div(mul(d - a, sh), r)));
              expX_re[0][0] = result_a.x; expX_im[0][0] = result_a.y;
              expX_re[0][1] = result_b.x; expX_im[0][1] = result_b.y;
              expX_re[1][0] = result_c.x; expX_im[1][0] = result_c.y;
              expX_re[1][1] = result_d.x; expX_im[1][1] = result_d.y;
          }
          vec2 applyMoebius(in mat2 M_re, in mat2 M_im, in vec2 z) {
              vec2 a = vec2(M_re[0][0], M_im[0][0]);
              vec2 b = vec2(M_re[0][1], M_im[0][1]);
              vec2 c = vec2(M_re[1][0], M_im[1][0]);
              vec2 d = vec2(M_re[1][1], M_im[1][1]);

              return div(mul(a, z) + b, mul(c, z) + d);
          }
        )"},
        {"FRAG_DECLARATIONS", R"(
          uniform float u_interpolation;
          in vec2 f_z, f_coords;
          in vec3 f_bary, f_edge_lengths;
          in vec3 f_Mijk_re, f_Mijk_im, f_lij_re, f_lij_im, f_ljk_re, f_ljk_im, f_lki_re, f_lki_im;
        )"},
        {"GENERATE_SHADE_VALUE", R"(
          vec2 shadeValue2;
          if (u_interpolation < 0.5) { // Mobius interpolation
              mat2 Mijk_re, Mijk_im, lij_re, lij_im, ljk_re, ljk_im, lki_re, lki_im;
              buildMoebiusTransformation(f_Mijk_re, f_Mijk_im, Mijk_re, Mijk_im);
              buildLogMoebiusTransformation(f_lij_re, f_lij_im, lij_re, lij_im);
              buildLogMoebiusTransformation(f_ljk_re, f_ljk_im, ljk_re, ljk_im);
              buildLogMoebiusTransformation(f_lki_re, f_lki_im, lki_re, lki_im);

              // edge coordinates
              vec3 ec = vec3( f_edge_lengths[0] / f_bary[2],
                              f_edge_lengths[1] / f_bary[0],
                              f_edge_lengths[2] / f_bary[1] );
              ec /= ec[0] + ec[1] + ec[2];
              mat2 blendedRatio_re = ec[0] * lij_re + ec[1] * ljk_re + ec[2] * lki_re;
              mat2 blendedRatio_im = ec[0] * lij_im + ec[1] * ljk_im + ec[2] * lki_im;
              mat2 M_re, M_im;
              exp(blendedRatio_re / 2., blendedRatio_im / 2., M_re, M_im);
              shadeValue2 = applyMoebius(Mijk_re, Mijk_im, (applyMoebius(M_re, M_im, f_z)));
          } else { // linear interpolation
              shadeValue2 = f_coords;
          }
        )"},
    },
    /* uniforms */
    {
        {"u_interpolation", render::DataType::Float},
    },
    /* attributes */
    {{"v_z", render::DataType::Vector2Float},
     {"v_coords", render::DataType::Vector2Float},
     {"v_bary", render::DataType::Vector3Float},
     {"v_edge_lengths", render::DataType::Vector3Float},
     {"v_Mijk_re", render::DataType::Vector3Float},
     {"v_Mijk_im", render::DataType::Vector3Float},
     {"v_lij_re", render::DataType::Vector3Float},
     {"v_lij_im", render::DataType::Vector3Float},
     {"v_ljk_re", render::DataType::Vector3Float},
     {"v_ljk_im", render::DataType::Vector3Float},
     {"v_lki_re", render::DataType::Vector3Float},
     {"v_lki_im", render::DataType::Vector3Float}},
    /* textures */ {});

// ==============================================================
// ================  Base Parameterization  =====================
// ==============================================================

SurfaceBlendedMobiusParameterizationQuantity::
    SurfaceBlendedMobiusParameterizationQuantity(std::string name,
                                                 ParamCoordsType type_,
                                                 ParamVizStyle style_,
                                                 SurfaceMesh& mesh_)
    : SurfaceMeshQuantity(name, mesh_, true), coordsType(type_),
      useMobiusInterpolation(uniquePrefix() + "#useMobiusInterpolatino", true),
      checkerSize(uniquePrefix() + "#checkerSize", 0.02),
      vizStyle(uniquePrefix() + "#vizStyle", style_),
      checkColor1(uniquePrefix() + "#checkColor1", render::RGB_PINK),
      checkColor2(uniquePrefix() + "#checkColor2", glm::vec3(.976, .856, .885)),
      gridLineColor(uniquePrefix() + "#gridLineColor", render::RGB_WHITE),
      gridBackgroundColor(uniquePrefix() + "#gridBackgroundColor",
                          render::RGB_PINK),
      altDarkness(uniquePrefix() + "#altDarkness", 0.5),
      cMap(uniquePrefix() + "#cMap", "phase")

{

    // Register a custom shader rule for projective interpolation
    render::engine->registerShaderRule("MESH_PROPAGATE_BLENDED_MOBIUS_VALUE2",
                                       MESH_PROPAGATE_BLENDED_MOBIUS_VALUE2);
}

void SurfaceBlendedMobiusParameterizationQuantity::draw() {
    if (!isEnabled()) return;

    if (program == nullptr) {
        createProgram();
    }

    // Set uniforms
    setProgramUniforms(*program);
    parent.setStructureUniforms(*program);
    parent.setSurfaceMeshUniforms(*program);

    program->draw();
}

void SurfaceBlendedMobiusParameterizationQuantity::createProgram() {
    // Create the program to draw this quantity

    switch (getStyle()) {
    case ParamVizStyle::CHECKER:
        // program = render::engine->generateShaderProgram(
        //{render::PARAM_SURFACE_VERT_SHADER,
        // render::PARAM_CHECKER_SURFACE_FRAG_SHADER}, DrawMode::Triangles);
        program = render::engine->requestShader(
            "MESH",
            parent.addSurfaceMeshRules({"MESH_PROPAGATE_BLENDED_MOBIUS_VALUE2",
                                        "SHADE_CHECKER_VALUE2"}));
        break;
    case ParamVizStyle::GRID:
        // program = render::engine->generateShaderProgram(
        //{render::PARAM_SURFACE_VERT_SHADER,
        // render::PARAM_GRID_SURFACE_FRAG_SHADER}, DrawMode::Triangles);
        program = render::engine->requestShader(
            "MESH",
            parent.addSurfaceMeshRules(
                {"MESH_PROPAGATE_BLENDED_MOBIUS_VALUE2", "SHADE_GRID_VALUE2"}));
        break;
    case ParamVizStyle::LOCAL_CHECK:
        // program = render::engine->generateShaderProgram(
        //{render::PARAM_SURFACE_VERT_SHADER,
        // render::PARAM_LOCAL_CHECKER_SURFACE_FRAG_SHADER},
        // DrawMode::Triangles);
        program = render::engine->requestShader(
            "MESH", parent.addSurfaceMeshRules(
                        {"MESH_PROPAGATE_BLENDED_MOBIUS_VALUE2",
                         "SHADE_COLORMAP_ANGULAR2", "CHECKER_VALUE2COLOR"}));
        program->setTextureFromColormap("t_colormap", cMap.get());
        break;
    case ParamVizStyle::LOCAL_RAD:
        // program = render::engine->generateShaderProgram(
        //{render::PARAM_SURFACE_VERT_SHADER,
        // render::PARAM_LOCAL_RAD_SURFACE_FRAG_SHADER}, DrawMode::Triangles);
        program = render::engine->requestShader(
            "MESH", parent.addSurfaceMeshRules(
                        {"MESH_PROPAGATE_BLENDED_MOBIUS_VALUE2",
                         "SHADE_COLORMAP_ANGULAR2", "SHADEVALUE_MAG_VALUE2",
                         "ISOLINE_STRIPE_VALUECOLOR"}));
        program->setTextureFromColormap("t_colormap", cMap.get());
        break;
    }

    // Fill color buffers
    fillColorBuffers(*program);
    parent.fillGeometryBuffers(*program);

    render::engine->setMaterial(*program, parent.getMaterial());
}


// Update range uniforms
void SurfaceBlendedMobiusParameterizationQuantity::setProgramUniforms(
    render::ShaderProgram& program) {
    program.setUniform("u_interpolation", getUseMobius() ? 0. : 1.);
    // Interpretatin of modulo parameter depends on data type
    switch (coordsType) {
    case ParamCoordsType::UNIT:
        program.setUniform("u_modLen", getCheckerSize());
        break;
    case ParamCoordsType::WORLD:
        program.setUniform("u_modLen", getCheckerSize() * state::lengthScale);
        break;
    }

    // Set other uniforms needed
    switch (getStyle()) {
    case ParamVizStyle::CHECKER:
        program.setUniform("u_color1", getCheckerColors().first);
        program.setUniform("u_color2", getCheckerColors().second);
        break;
    case ParamVizStyle::GRID:
        program.setUniform("u_gridLineColor", getGridColors().first);
        program.setUniform("u_gridBackgroundColor", getGridColors().second);
        break;
    case ParamVizStyle::LOCAL_CHECK:
    case ParamVizStyle::LOCAL_RAD:
        program.setUniform("u_angle", localRot);
        program.setUniform("u_modDarkness", getAltDarkness());
        break;
    }
}

namespace {
// Helper to name styles
std::string styleName(ParamVizStyle v) {
    switch (v) {
    case ParamVizStyle::CHECKER:
        return "checker";
        break;
    case ParamVizStyle::GRID:
        return "grid";
        break;
    case ParamVizStyle::LOCAL_CHECK:
        return "local grid";
        break;
    case ParamVizStyle::LOCAL_RAD:
        return "local dist";
        break;
    }
    throw std::runtime_error("broken");
}

} // namespace

void SurfaceBlendedMobiusParameterizationQuantity::buildCustomUI() {
    ImGui::PushItemWidth(100);

    ImGui::SameLine(); // put it next to enabled

    // Choose viz style
    if (ImGui::BeginCombo("style", styleName(getStyle()).c_str())) {
        for (ParamVizStyle s :
             {ParamVizStyle::CHECKER, ParamVizStyle::GRID,
              ParamVizStyle::LOCAL_CHECK, ParamVizStyle::LOCAL_RAD}) {
            if (ImGui::Selectable(styleName(s).c_str(), s == getStyle())) {
                setStyle(s);
            }
        }
        ImGui::EndCombo();
    }


    // Modulo stripey width
    if (ImGui::DragFloat(
            "period", &checkerSize.get(), .001, 0.0001, 1.0, "%.4f",
            ImGuiSliderFlags_Logarithmic | ImGuiSliderFlags_NoRoundToFormat)) {
        setCheckerSize(getCheckerSize());
    }


    ImGui::PopItemWidth();

    if (ImGui::Checkbox("Use Mobius Interpolation",
                        &useMobiusInterpolation.get()))
        setUseMobius(getUseMobius());

    switch (getStyle()) {
    case ParamVizStyle::CHECKER:
        if (ImGui::ColorEdit3("##colors2", &checkColor1.get()[0],
                              ImGuiColorEditFlags_NoInputs))
            setCheckerColors(getCheckerColors());
        ImGui::SameLine();
        if (ImGui::ColorEdit3("colors", &checkColor2.get()[0],
                              ImGuiColorEditFlags_NoInputs))
            setCheckerColors(getCheckerColors());
        break;
    case ParamVizStyle::GRID:
        if (ImGui::ColorEdit3("base", &gridBackgroundColor.get()[0],
                              ImGuiColorEditFlags_NoInputs))
            setGridColors(getGridColors());
        ImGui::SameLine();
        if (ImGui::ColorEdit3("line", &gridLineColor.get()[0],
                              ImGuiColorEditFlags_NoInputs))
            setGridColors(getGridColors());
        break;
    case ParamVizStyle::LOCAL_CHECK:
    case ParamVizStyle::LOCAL_RAD: {
        // Angle slider
        ImGui::PushItemWidth(100);
        ImGui::SliderAngle("angle shift", &localRot, -180,
                           180); // displays in degrees, works in radians TODO
                                 // refresh/update/persist
        if (ImGui::DragFloat("alt darkness", &altDarkness.get(), 0.01, 0.,
                             1.)) {
            altDarkness.manuallyChanged();
            requestRedraw();
        }
        ImGui::PopItemWidth();

        // Set colormap
        if (render::buildColormapSelector(cMap.get())) {
            setColorMap(getColorMap());
        }
    }

    break;
    }
}


SurfaceBlendedMobiusParameterizationQuantity*
SurfaceBlendedMobiusParameterizationQuantity::setStyle(ParamVizStyle newStyle) {
    vizStyle = newStyle;
    program.reset();
    requestRedraw();
    return this;
}

ParamVizStyle SurfaceBlendedMobiusParameterizationQuantity::getStyle() {
    return vizStyle.get();
}

SurfaceBlendedMobiusParameterizationQuantity*
SurfaceBlendedMobiusParameterizationQuantity::setCheckerColors(
    std::pair<glm::vec3, glm::vec3> colors) {
    checkColor1 = colors.first;
    checkColor2 = colors.second;
    requestRedraw();
    return this;
}

std::pair<glm::vec3, glm::vec3>
SurfaceBlendedMobiusParameterizationQuantity::getCheckerColors() {
    return std::make_pair(checkColor1.get(), checkColor2.get());
}

SurfaceBlendedMobiusParameterizationQuantity*
SurfaceBlendedMobiusParameterizationQuantity::setGridColors(
    std::pair<glm::vec3, glm::vec3> colors) {
    gridLineColor       = colors.first;
    gridBackgroundColor = colors.second;
    requestRedraw();
    return this;
}

std::pair<glm::vec3, glm::vec3>
SurfaceBlendedMobiusParameterizationQuantity::getGridColors() {
    return std::make_pair(gridLineColor.get(), gridBackgroundColor.get());
}

SurfaceBlendedMobiusParameterizationQuantity*
SurfaceBlendedMobiusParameterizationQuantity::setCheckerSize(double newVal) {
    checkerSize = newVal;
    requestRedraw();
    return this;
}

double SurfaceBlendedMobiusParameterizationQuantity::getCheckerSize() {
    return checkerSize.get();
}

SurfaceBlendedMobiusParameterizationQuantity*
SurfaceBlendedMobiusParameterizationQuantity::setUseMobius(bool useMobius) {
    useMobiusInterpolation = useMobius;
    requestRedraw();
    return this;
}

double SurfaceBlendedMobiusParameterizationQuantity::getUseMobius() {
    return useMobiusInterpolation.get();
}

SurfaceBlendedMobiusParameterizationQuantity*
SurfaceBlendedMobiusParameterizationQuantity::setColorMap(std::string name) {
    cMap = name;
    program.reset();
    requestRedraw();
    return this;
}
std::string SurfaceBlendedMobiusParameterizationQuantity::getColorMap() {
    return cMap.get();
}

SurfaceBlendedMobiusParameterizationQuantity*
SurfaceBlendedMobiusParameterizationQuantity::setAltDarkness(double newVal) {
    altDarkness = newVal;
    requestRedraw();
    return this;
}

double SurfaceBlendedMobiusParameterizationQuantity::getAltDarkness() {
    return altDarkness.get();
}

void SurfaceBlendedMobiusParameterizationQuantity::refresh() {
    program.reset();
    Quantity::refresh();
}

// ==============================================================
// ===============  Vertex Parameterization  ====================
// ==============================================================


SurfaceVertexBlendedMobiusParameterizationQuantity::
    SurfaceVertexBlendedMobiusParameterizationQuantity(
        std::string name, std::vector<glm::vec2> coords_,
        std::vector<std::array<glm::vec2, 3>> z_,
        std::array<std::vector<glm::mat2>, 8> matrices, ParamCoordsType type_,
        ParamVizStyle style_, SurfaceMesh& mesh_)
    : SurfaceBlendedMobiusParameterizationQuantity(name, type_, style_, mesh_),
      coords(std::move(coords_)), z(std::move(z_)) {

    for (const glm::mat2& m : matrices[0])
        Mijk_re.push_back(glm::vec3(m[0][0], m[0][1], m[1][0]));
    for (const glm::mat2& m : matrices[1])
        Mijk_im.push_back(glm::vec3(m[0][0], m[0][1], m[1][0]));
    for (const glm::mat2& m : matrices[2])
        lij_re.push_back(glm::vec3(m[0][0], m[0][1], m[1][0]));
    for (const glm::mat2& m : matrices[3])
        lij_im.push_back(glm::vec3(m[0][0], m[0][1], m[1][0]));
    for (const glm::mat2& m : matrices[4])
        ljk_re.push_back(glm::vec3(m[0][0], m[0][1], m[1][0]));
    for (const glm::mat2& m : matrices[5])
        ljk_im.push_back(glm::vec3(m[0][0], m[0][1], m[1][0]));
    for (const glm::mat2& m : matrices[6])
        lki_re.push_back(glm::vec3(m[0][0], m[0][1], m[1][0]));
    for (const glm::mat2& m : matrices[7])
        lki_im.push_back(glm::vec3(m[0][0], m[0][1], m[1][0]));
}

std::string SurfaceVertexBlendedMobiusParameterizationQuantity::niceName() {
    return name + " (vertex parameterization)";
}

void SurfaceVertexBlendedMobiusParameterizationQuantity::fillColorBuffers(
    render::ShaderProgram& p) {
    std::vector<glm::vec2> c_z, c_coords;
    std::vector<glm::vec3> c_bary, c_edge_lengths, c_Mijk_re, c_Mijk_im,
        c_lij_re, c_lij_im, c_ljk_re, c_ljk_im, c_lki_re, c_lki_im;

    c_z.reserve(3 * parent.nFacesTriangulation());
    c_coords.reserve(3 * parent.nFacesTriangulation());
    c_bary.reserve(3 * parent.nFacesTriangulation());
    c_edge_lengths.reserve(3 * parent.nFacesTriangulation());
    c_Mijk_re.reserve(3 * parent.nFacesTriangulation());
    c_Mijk_im.reserve(3 * parent.nFacesTriangulation());
    c_lij_re.reserve(3 * parent.nFacesTriangulation());
    c_lij_im.reserve(3 * parent.nFacesTriangulation());
    c_ljk_re.reserve(3 * parent.nFacesTriangulation());
    c_ljk_im.reserve(3 * parent.nFacesTriangulation());
    c_lki_re.reserve(3 * parent.nFacesTriangulation());
    c_lki_im.reserve(3 * parent.nFacesTriangulation());

    for (size_t iF = 0; iF < parent.nFaces(); iF++) {
        auto& face = parent.faces[iF];
        size_t D   = face.size();

        // implicitly triangulate from root
        size_t vi = face[0];
        for (size_t j = 1; (j + 1) < D; j++) {
            size_t vj = face[j];
            size_t vk = face[j + 1];
            // linear data
            c_z.push_back(z[iF][0]);
            c_z.push_back(z[iF][j]);
            c_z.push_back(z[iF][j + 1]);
            c_coords.push_back(coords[vi]);
            c_coords.push_back(coords[vj]);
            c_coords.push_back(coords[vk]);
            c_bary.push_back(glm::vec3(1., 0., 0.));
            c_bary.push_back(glm::vec3(0., 1., 0.));
            c_bary.push_back(glm::vec3(0., 0., 1.));

            glm::vec3 lengths =
                glm::vec3(length(parent.vertices[vj] - parent.vertices[vi]),
                          length(parent.vertices[vk] - parent.vertices[vj]),
                          length(parent.vertices[vi] - parent.vertices[vk]));
            // constant data
            for (size_t iC = 0; iC < 3; iC++) {
                c_edge_lengths.push_back(lengths);
                c_Mijk_re.push_back(Mijk_re[iF]);
                c_Mijk_im.push_back(Mijk_im[iF]);
                c_lij_re.push_back(lij_re[iF]);
                c_lij_im.push_back(lij_im[iF]);
                c_ljk_re.push_back(ljk_re[iF]);
                c_ljk_im.push_back(ljk_im[iF]);
                c_lki_re.push_back(lki_re[iF]);
                c_lki_im.push_back(lki_im[iF]);
            }
        }
    }

    // Store data in buffers
    p.setAttribute("v_z", c_z);
    p.setAttribute("v_coords", c_coords);
    p.setAttribute("v_bary", c_bary);
    p.setAttribute("v_edge_lengths", c_edge_lengths);
    p.setAttribute("v_Mijk_re", c_Mijk_re);
    p.setAttribute("v_Mijk_im", c_Mijk_im);
    p.setAttribute("v_lij_re", c_lij_re);
    p.setAttribute("v_lij_im", c_lij_im);
    p.setAttribute("v_ljk_re", c_ljk_re);
    p.setAttribute("v_ljk_im", c_ljk_im);
    p.setAttribute("v_lki_re", c_lki_re);
    p.setAttribute("v_lki_im", c_lki_im);
}

void SurfaceVertexBlendedMobiusParameterizationQuantity::buildVertexInfoGUI(
    size_t vInd) {
    ImGui::TextUnformatted(name.c_str());
    ImGui::NextColumn();
    ImGui::Text("<%g,%g>", coords[vInd].x, coords[vInd].y);
    ImGui::NextColumn();
}


} // namespace polyscope
