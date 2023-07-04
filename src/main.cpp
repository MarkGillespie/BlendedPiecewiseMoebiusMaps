#include "geometrycentral/surface/boundary_first_flattening.h"
#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/meshio.h"
#include "geometrycentral/surface/vertex_position_geometry.h"

#include "polyscope/point_cloud.h"
#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"

#include "args/args.hxx"
#include "imgui.h"

#include "BPM.h"
#include "BPMShader.h"
#include "utils.h"

using namespace geometrycentral;
using namespace geometrycentral::surface;

// == Geometry-central data
std::unique_ptr<ManifoldSurfaceMesh> mesh;
std::unique_ptr<VertexPositionGeometry> geom;

// Polyscope visualization handle, to quickly add data to the surface
polyscope::SurfaceMesh* psMesh;

// A user-defined callback, for creating control panels (etc)
// Use ImGUI commands to build whatever you want here, see
// https://github.com/ocornut/imgui/blob/master/imgui.h
void myCallback() {}

int main(int argc, char** argv) {

    // Configure the argument parser
    args::ArgumentParser parser("Blended Mobius Shader");
    args::Positional<std::string> inputFilename(parser, "mesh",
                                                "Mesh to be processed.");

    // Parse args
    try {
        parser.ParseCLI(argc, argv);
    } catch (args::Help) {
        std::cout << parser;
        return 0;
    } catch (args::ParseError e) {
        std::cerr << e.what() << std::endl;
        std::cerr << parser;
        return 1;
    }

    // Make sure a mesh name was given
    if (!inputFilename) {
        std::cout << "Error: no input mesh" << vendl;
        std::cout << parser << vendl;
        return 1;
    }
    std::string filename = args::get(inputFilename);

    // Initialize polyscope
    polyscope::init();

    // Set the callback function
    polyscope::state::userCallback = myCallback;

    // Load mesh
    std::tie(mesh, geom) = readManifoldSurfaceMesh(filename);

    // Register the mesh with polyscope
    psMesh = polyscope::registerSurfaceMesh(
        polyscope::guessNiceNameFromPath(filename), geom->vertexPositions,
        mesh->getFaceVertexList(), polyscopePermutations(*mesh));
    // psMesh->setEnabled(false);

    VertexData<Vector2> param = parameterizeBFF(*mesh, *geom);
    for (Vertex v : mesh->vertices()) {
        param[v].x *= -1; // need to flip orientation for some reason
    }

    BPM bpm(*mesh, *geom, param);

    polyscope::SurfaceVertexBlendedMobiusParameterizationQuantity q(
        "param", polyscope::standardizeVectorArray<glm::vec2, 2>(param),
        bpm.computePolyscopeCoordinates(), bpm.computePolyscopeMatrices(),
        polyscope::ParamCoordsType::UNIT, polyscope::ParamVizStyle::GRID,
        *psMesh);
    psMesh->addQuantity(&q);
    q.setEnabled(true);


    // Give control to the polyscope gui
    polyscope::show();

    return EXIT_SUCCESS;
}
