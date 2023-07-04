#pragma once

#include "geometrycentral/utilities/vector2.h"
#include <Eigen/Dense>

using namespace geometrycentral;

Eigen::Matrix2cd exp(const Eigen::Matrix2cd& X);
Eigen::Matrix2cd log(const Eigen::Matrix2cd& X);

// "plus-or-minus log": log(X) or log(-X), whichever is closest to the identity
// This is given by log(sgn(Tr(X)) X)
// (Note that our matrices are normalized to have determinant 1, so the
// eigenvalues must form a conjugate pair. Hence the trace (which is their sum)
// must be real)
inline Eigen::Matrix2cd pmlog(const Eigen::Matrix2cd& X) {
    return log(copysign(1., X.trace().real()) * X);
}

// compute Mobius transformation sending z->
// (normalized so to have determinant 1)
Eigen::Matrix2cd
findMobiusTransformation(const std::array<std::complex<double>, 3>& z,
                         const std::array<std::complex<double>, 3>& w);
Eigen::Matrix2cd findMobiusTransformation(const std::array<Vector2, 3>& z,
                                          const std::array<Vector2, 3>& w);

inline std::complex<double> applyMobiusTransformation(Eigen::Matrix2cd f,
                                                      std::complex<double> z) {
    return (f(0, 0) * z + f(0, 1)) / (f(1, 0) * z + f(1, 1));
}

inline Vector2 applyMobiusTransformation(Eigen::Matrix2cd f, Vector2 z) {
    std::complex<double> cz = z;
    return Vector2::fromComplex((f(0, 0) * cz + f(0, 1)) /
                                (f(1, 0) * cz + f(1, 1)));
}
