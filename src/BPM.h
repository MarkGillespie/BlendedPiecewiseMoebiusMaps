#pragma once

#include "geometrycentral/surface/barycentric_coordinate_helpers.h"
#include "geometrycentral/surface/common_subdivision.h" // niceColors
#include "geometrycentral/surface/intrinsic_geometry_interface.h"
#include "geometrycentral/surface/manifold_surface_mesh.h"

#include "glm/glm.hpp"

#include "Mobius.h"
#include "utils.h"

using namespace geometrycentral;
using namespace geometrycentral::surface;

class BPM {
  public:
    BPM(ManifoldSurfaceMesh& mesh, IntrinsicGeometryInterface& geom,
        const VertexData<Vector2>& param);

    ManifoldSurfaceMesh& mesh;
    IntrinsicGeometryInterface& geom;
    VertexData<Vector2> param;
    HalfedgeData<Eigen::Matrix2cd> transitionMap;
    FaceData<Eigen::Matrix2cd> paramMap;

    void computeParamMap();

    std::array<std::vector<glm::mat2>, 8> computePolyscopeMatrices();
    std::vector<std::array<glm::vec2, 3>> computePolyscopeCoordinates();

    // visualize (discontinuous) param of random points in triangle
    std::pair<std::vector<Vector2>, std::vector<double>>
    getBrokenParamPointCloud(size_t res);

    // visualize (continuous) param of random points in triangle
    std::pair<std::vector<Vector2>, std::vector<double>>
    getBlendedParamPointCloud(size_t res);

    // transitionMap[ij] maps from ij.face()'s coordinates system to
    // ij.faces()'s vertex positions to ij.twin().face()'s coordinate system
    void computeTransitionMaps();
    void computeTriangleScales();

    // lay out three vertices of ijk (ordered by appearance in ijk)
    // in ijk's coordinate system
    std::array<Vector2, 3> layOutTriangle(Face ijk);

    // lay out three vertices of he.twin().face() (ordered by appearance in the
    // face, not starting from he.vertex()) in the coordinate system of face
    // he.face()
    std::array<Vector2, 3> layOutNeighboringTriangle(Halfedge he);

    Vector3 computeEdgeCoordinates(Face ijk, Vector3 bary);
};
