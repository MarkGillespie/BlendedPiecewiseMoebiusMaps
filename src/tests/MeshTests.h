#include "geometrycentral/surface/vertex_position_geometry.h"
#include "test_utils.h"

#include "BPM.h"

class MeshTest : public ::testing::Test {};

TEST_F(MeshTest, layOutNeighboringTriangle) {
    std::vector<std::vector<size_t>> triangles;
    triangles.push_back({0, 1, 2});
    triangles.push_back({3, 2, 1});
    triangles.push_back({4, 0, 2});
    triangles.push_back({5, 1, 0});

    ManifoldSurfaceMesh mesh(triangles);

    VertexData<Vector2> positions(mesh);
    positions[0] = {0, 0};
    positions[1] = {2, 0};
    positions[2] = {1, 1};
    positions[3] = {3, 3};
    positions[4] = {-1, 1};
    positions[5] = {1, -1};

    VertexData<Vector3> p3D(mesh);
    for (Vertex v : mesh.vertices()) {
        Vector2 p = positions[v];
        p3D[v]    = {p.x, p.y, 0};
    }

    VertexPositionGeometry geom(mesh, p3D);

    BPM bpm(mesh, geom, positions);

    Halfedge h5 = mesh.face(0).halfedge();
    Halfedge h3 = h5.next();
    Halfedge h4 = h3.next();

    std::array<Vector2, 3> n3 = bpm.layOutNeighboringTriangle(h3);
    std::array<Vector2, 3> n4 = bpm.layOutNeighboringTriangle(h4);
    std::array<Vector2, 3> n5 = bpm.layOutNeighboringTriangle(h5);

    EXPECT_VEC2_NEAR(n3[0], positions[3], 1e-5);
    EXPECT_VEC2_NEAR(n3[1], positions[2], 1e-5);
    EXPECT_VEC2_NEAR(n3[2], positions[1], 1e-5);

    EXPECT_VEC2_NEAR(n4[0], positions[4], 1e-5);
    EXPECT_VEC2_NEAR(n4[1], positions[0], 1e-5);
    EXPECT_VEC2_NEAR(n4[2], positions[2], 1e-5);

    EXPECT_VEC2_NEAR(n5[0], positions[5], 1e-5);
    EXPECT_VEC2_NEAR(n5[1], positions[1], 1e-5);
    EXPECT_VEC2_NEAR(n5[2], positions[0], 1e-5);
}
