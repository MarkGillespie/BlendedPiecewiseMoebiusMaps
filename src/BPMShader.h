#pragma once

#include "polyscope/affine_remapper.h"
#include "polyscope/histogram.h"
#include "polyscope/render/color_maps.h"
#include "polyscope/render/engine.h"
#include "polyscope/surface_mesh.h"

namespace polyscope {
class SurfaceBlendedMobiusParameterizationQuantity
    : public SurfaceMeshQuantity {

  public:
    SurfaceBlendedMobiusParameterizationQuantity(std::string name,
                                                 ParamCoordsType type_,
                                                 ParamVizStyle style,
                                                 SurfaceMesh& mesh_);

    void draw() override;
    virtual void buildCustomUI() override;

    virtual void refresh() override;


    // === Members
    const ParamCoordsType coordsType;

    // === Viz stuff
    // to keep things simple, has settings for all of the viz styles, even
    // though not all are used at all times


    // === Getters and setters for visualization options

    // What visualization scheme to use
    SurfaceBlendedMobiusParameterizationQuantity*
    setStyle(ParamVizStyle newStyle);
    ParamVizStyle getStyle();

    // Colors for checkers
    SurfaceBlendedMobiusParameterizationQuantity*
    setCheckerColors(std::pair<glm::vec3, glm::vec3> colors);
    std::pair<glm::vec3, glm::vec3> getCheckerColors();

    // Colors for checkers
    SurfaceBlendedMobiusParameterizationQuantity*
    setGridColors(std::pair<glm::vec3, glm::vec3> colors);
    std::pair<glm::vec3, glm::vec3> getGridColors();

    // The size of checkers / stripes
    SurfaceBlendedMobiusParameterizationQuantity* setCheckerSize(double newVal);
    double getCheckerSize();

    // interpolation mode
    SurfaceBlendedMobiusParameterizationQuantity* setUseMobius(bool useMobius);
    double getUseMobius();

    // Color map for radial visualization
    SurfaceBlendedMobiusParameterizationQuantity* setColorMap(std::string val);
    std::string getColorMap();

    // Darkness for checkers (etc)
    SurfaceBlendedMobiusParameterizationQuantity* setAltDarkness(double newVal);
    double getAltDarkness();


  protected:
    // === Visualiztion options
    PersistentValue<bool> useMobiusInterpolation;
    PersistentValue<float> checkerSize;
    PersistentValue<ParamVizStyle> vizStyle;
    PersistentValue<glm::vec3> checkColor1,
        checkColor2; // for checker (two colors to use)
    PersistentValue<glm::vec3> gridLineColor,
        gridBackgroundColor; // for GRID (two colors to use)
    PersistentValue<float> altDarkness;

    PersistentValue<std::string> cMap;
    float localRot = 0.; // for LOCAL (angular shift, in radians)
    std::shared_ptr<render::ShaderProgram> program;

    // Helpers
    void createProgram();
    void setProgramUniforms(render::ShaderProgram& program);
    virtual void fillColorBuffers(render::ShaderProgram& p) = 0;
};

// ==============================================================
// ===============  Vertex Parameterization  ====================
// ==============================================================

class SurfaceVertexBlendedMobiusParameterizationQuantity
    : public SurfaceBlendedMobiusParameterizationQuantity {
  public:
    SurfaceVertexBlendedMobiusParameterizationQuantity(
        std::string name, std::vector<glm::vec2> values_,
        std::vector<std::array<glm::vec2, 3>> z_,
        std::array<std::vector<glm::mat2>, 8> matrices, ParamCoordsType type_,
        ParamVizStyle style, SurfaceMesh& mesh_);

    virtual void buildVertexInfoGUI(size_t vInd) override;
    virtual std::string niceName() override;

    // === Members
    std::vector<glm::vec2> coords; // on vertices
    std::vector<std::array<glm::vec2, 3>> z;

    // complex matrices on faces
    std::vector<glm::vec3> Mijk_re, Mijk_im, lij_re, lij_im, ljk_re, ljk_im,
        lki_re, lki_im;

  protected:
    virtual void fillColorBuffers(render::ShaderProgram& p) override;
};

} // namespace polyscope
