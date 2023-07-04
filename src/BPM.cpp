#include "BPM.h"

BPM::BPM(ManifoldSurfaceMesh& mesh_, IntrinsicGeometryInterface& geom_,
         const VertexData<Vector2>& param_)
    : mesh(mesh_), geom(geom_), param(param_) {
    computeTransitionMaps();
    computeParamMap();
}

void BPM::computeParamMap() {
    paramMap = FaceData<Eigen::Matrix2cd>(mesh);

    for (Face ijk : mesh.faces()) {
        Vertex i = ijk.halfedge().tailVertex();
        Vertex j = ijk.halfedge().tipVertex();
        Vertex k = ijk.halfedge().next().tipVertex();
        std::array<Vector2, 3> fq{param[i], param[j], param[k]};
        std::array<Vector2, 3> p = layOutTriangle(ijk);

        paramMap[ijk] = findMobiusTransformation(p, fq);
    }
}

std::array<std::vector<glm::mat2>, 8> BPM::computePolyscopeMatrices() {
    std::array<std::vector<glm::mat2>, 8> mats;
    for (Face ijk : mesh.faces()) {
        Halfedge ij = ijk.halfedge();
        Halfedge jk = ij.next();
        Halfedge ki = jk.next();

        Eigen::Matrix2cd Mijk = paramMap[ijk];
        Eigen::Matrix2cd Mij =
            paramMap[ij.twin().face()] * transitionMap[ij.twin()];
        Eigen::Matrix2cd Mjk =
            paramMap[jk.twin().face()] * transitionMap[jk.twin()];
        Eigen::Matrix2cd Mki =
            paramMap[ki.twin().face()] * transitionMap[ki.twin()];

        Eigen::Matrix2cd lij = pmlog(Mij * Mijk.inverse()); // "log ratio"
        Eigen::Matrix2cd ljk = pmlog(Mjk * Mijk.inverse()); // "log ratio"
        Eigen::Matrix2cd lki = pmlog(Mki * Mijk.inverse()); // "log ratio"

        // no contribution from boundary faces
        if (ij.edge().isBoundary()) lij = Eigen::Matrix2cd::Zero();
        if (jk.edge().isBoundary()) ljk = Eigen::Matrix2cd::Zero();
        if (ki.edge().isBoundary()) lki = Eigen::Matrix2cd::Zero();

        if (ijk.getIndex() == 194 || ijk.getIndex() == 266) {
            WATCH(ijk.getIndex());
            WATCH(Mijk);
            WATCH(Mij);
            // WATCH(Mjk);
            // WATCH(Mki);
            WATCH(lij);
            // WATCH(ljk);
            // WATCH(lki);
            std::cout << vendl;
        }

        mats[0].push_back(glm::mat2(Mijk(0, 0).real(), Mijk(0, 1).real(),
                                    Mijk(1, 0).real(), Mijk(1, 1).real()));
        mats[1].push_back(glm::mat2(Mijk(0, 0).imag(), Mijk(0, 1).imag(),
                                    Mijk(1, 0).imag(), Mijk(1, 1).imag()));
        mats[2].push_back(glm::mat2(lij(0, 0).real(), lij(0, 1).real(),
                                    lij(1, 0).real(), lij(1, 1).real()));
        mats[3].push_back(glm::mat2(lij(0, 0).imag(), lij(0, 1).imag(),
                                    lij(1, 0).imag(), lij(1, 1).imag()));
        mats[4].push_back(glm::mat2(ljk(0, 0).real(), ljk(0, 1).real(),
                                    ljk(1, 0).real(), ljk(1, 1).real()));
        mats[5].push_back(glm::mat2(ljk(0, 0).imag(), ljk(0, 1).imag(),
                                    ljk(1, 0).imag(), ljk(1, 1).imag()));
        mats[6].push_back(glm::mat2(lki(0, 0).real(), lki(0, 1).real(),
                                    lki(1, 0).real(), lki(1, 1).real()));
        mats[7].push_back(glm::mat2(lki(0, 0).imag(), lki(0, 1).imag(),
                                    lki(1, 0).imag(), lki(1, 1).imag()));
    }
    return mats;
}

std::vector<std::array<glm::vec2, 3>> BPM::computePolyscopeCoordinates() {
    std::vector<std::array<glm::vec2, 3>> coords;
    for (Face f : mesh.faces()) {
        std::array<Vector2, 3> p = layOutTriangle(f);
        coords.push_back({glm::vec2(p[0].x, p[0].y), glm::vec2(p[1].x, p[1].y),
                          glm::vec2(p[2].x, p[2].y)});
    }
    return coords;
}

// visualize (discontinuous) param of random points in triangle
std::pair<std::vector<Vector2>, std::vector<double>>
BPM::getBrokenParamPointCloud(size_t res) {
    std::vector<Vector2> points;
    std::vector<double> faceColors;

    FaceData<double> color = niceColors(mesh);

    for (Face ijk : mesh.faces()) {
        std::array<Vector2, 3> p = layOutTriangle(ijk);
        for (size_t iP = 0; iP < res; iP++) {
            Vector3 b =
                normalizeBarycentric({unitRand(), unitRand(), unitRand()});
            Vector2 z = b[0] * p[0] + b[1] * p[1] + b[2] * p[2];
            points.push_back(applyMobiusTransformation(paramMap[ijk], z));
            faceColors.push_back(color[ijk]);
        }
    }

    return std::make_tuple(points, faceColors);
}

// visualize (continuous) param of random points in triangle
std::pair<std::vector<Vector2>, std::vector<double>>
BPM::getBlendedParamPointCloud(size_t res) {
    std::vector<Vector2> points;
    std::vector<double> faceColors;

    FaceData<double> color = niceColors(mesh);

    for (Face ijk : mesh.faces()) {
        Halfedge ij = ijk.halfedge();
        Halfedge jk = ij.next();
        Halfedge ki = jk.next();

        Eigen::Matrix2cd Mijk = paramMap[ijk];
        Eigen::Matrix2cd Mij =
            paramMap[ij.twin().face()] * transitionMap[ij.twin()];
        Eigen::Matrix2cd Mjk =
            paramMap[jk.twin().face()] * transitionMap[jk.twin()];
        Eigen::Matrix2cd Mki =
            paramMap[ki.twin().face()] * transitionMap[ki.twin()];

        Eigen::Matrix2cd lij = pmlog(Mij * Mijk.inverse()); // "log ratio"
        Eigen::Matrix2cd ljk = pmlog(Mjk * Mijk.inverse()); // "log ratio"
        Eigen::Matrix2cd lki = pmlog(Mki * Mijk.inverse()); // "log ratio"

        // no contribution from boundary faces
        if (ij.edge().isBoundary()) lij = Eigen::Matrix2cd::Zero();
        if (jk.edge().isBoundary()) ljk = Eigen::Matrix2cd::Zero();
        if (ki.edge().isBoundary()) lki = Eigen::Matrix2cd::Zero();

        std::array<Vector2, 3> p = layOutTriangle(ijk);
        for (size_t iP = 0; iP < res; iP++) {
            Vector3 bary =
                normalizeBarycentric({unitRand(), unitRand(), unitRand()});
            Vector2 z = bary[0] * p[0] + bary[1] * p[1] + bary[2] * p[2];

            std::array<Vector2, 3> q = layOutNeighboringTriangle(ij.twin());
            Vector2 zij = bary[0] * q[0] + bary[1] * q[1] + bary[2] * q[2];
            std::array<Vector2, 3> pij = layOutTriangle(ij.twin().face());
            Vector2 neighbor_zij =
                bary[0] * pij[0] + bary[1] * pij[1] + bary[2] * pij[2];

            Vector3 ec = computeEdgeCoordinates(ijk, bary);
            Eigen::Matrix2cd blendedRatio =
                ec[0] * lij + ec[1] * ljk + ec[2] * lki;

            Eigen::Matrix2cd M = exp(blendedRatio / 2.) * Mijk;
            points.push_back(applyMobiusTransformation(M, z));
            faceColors.push_back(color[ijk]);
        }
    }

    return std::make_tuple(points, faceColors);
}

void BPM::computeTransitionMaps() {
    transitionMap = HalfedgeData<Eigen::Matrix2cd>(mesh);

    for (Face ijk : mesh.faces()) {

        Halfedge ij = ijk.halfedge();
        Halfedge jk = ij.next();
        Halfedge ki = jk.next();

        std::array<Vector2, 3> p = layOutTriangle(ijk);

        if (ij.edge().isBoundary()) { // map across ij
            transitionMap[ij.twin()] = Eigen::Matrix2cd::Zero();
        } else {
            // lay out ijk from perspective of neighbor
            std::array<Vector2, 3> q = layOutNeighboringTriangle(ij.twin());
            transitionMap[ij.twin()] = findMobiusTransformation(p, q);
        }
        if (jk.edge().isBoundary()) { // map across jk
            transitionMap[jk.twin()] = Eigen::Matrix2cd::Zero();
        } else {
            // lay out ijk from perspective of neighbor
            std::array<Vector2, 3> q = layOutNeighboringTriangle(jk.twin());
            transitionMap[jk.twin()] = findMobiusTransformation(p, q);
        }
        if (ki.edge().isBoundary()) { // map across ki
            transitionMap[ki.twin()] = Eigen::Matrix2cd::Zero();
        } else {
            // lay out ijk from perspective of neighbor
            std::array<Vector2, 3> q = layOutNeighboringTriangle(ki.twin());
            transitionMap[ki.twin()] = findMobiusTransformation(p, q);
        }
    }
}

// lay out three vertices of ijk (ordered by appearance in ijk)
// in ijk's coordinate system
std::array<Vector2, 3> BPM::layOutTriangle(Face ijk) {
    geom.requireHalfedgeVectorsInFace();
    Vector2 pi = Vector2::zero();
    Vector2 pj = geom.halfedgeVectorsInFace[ijk.halfedge()];
    Vector2 pk = pj + geom.halfedgeVectorsInFace[ijk.halfedge().next()];
    geom.unrequireHalfedgeVectorsInFace();

    return std::array<Vector2, 3>{pi, pj, pk};
}

// lay out three vertices of he.twin().face() (ordered by appearance in the
// face, not starting from he.vertex()) in the coordinate system of face
// he.face()
std::array<Vector2, 3> BPM::layOutNeighboringTriangle(Halfedge he) {
    geom.requireHalfedgeVectorsInFace();
    geom.requireCornerAngles();
    Face ijk = he.face();

    Halfedge ij = ijk.halfedge();
    Halfedge jk = ij.next();
    Halfedge ki = jk.next();

    std::array<Vector2, 3> p = layOutTriangle(ijk);

    size_t iH     = halfedgeIndexInTriangle(he.twin());
    Halfedge next = he.twin().next();
    double l      = geom.edgeLengths[next.edge()];
    double theta  = geom.cornerAngles[next.corner()];
    std::array<Vector2, 3> result;
    if (he == ij) {
        result[(iH + 0) % 3] = p[1];
        result[(iH + 1) % 3] = p[0];
        result[(iH + 2) % 3] =
            l * Vector2::fromAngle(-theta) * (p[1] - p[0]).normalize() + p[0];
    } else if (he == jk) {
        result[(iH + 0) % 3] = p[2];
        result[(iH + 1) % 3] = p[1];
        result[(iH + 2) % 3] =
            l * Vector2::fromAngle(-theta) * (p[2] - p[1]).normalize() + p[1];
    } else if (he == ki) {
        result[(iH + 0) % 3] = p[0];
        result[(iH + 1) % 3] = p[2];
        result[(iH + 2) % 3] =
            l * Vector2::fromAngle(-theta) * (p[0] - p[2]).normalize() + p[2];
    }
    geom.unrequireCornerAngles();
    geom.unrequireHalfedgeVectorsInFace();

    return result;
}

Vector3 BPM::computeEdgeCoordinates(Face ijk, Vector3 bary) {
    geom.requireEdgeLengths();
    Halfedge ij = ijk.halfedge();
    Halfedge jk = ij.next();
    Halfedge ki = jk.next();

    Vector3 edgeCoords{geom.edgeLengths[ij.edge()] / bary[2],
                       geom.edgeLengths[jk.edge()] / bary[0],
                       geom.edgeLengths[ki.edge()] / bary[1]};
    geom.unrequireEdgeLengths();

    return normalizeBarycentric(edgeCoords);
}
