#include "Mobius.h"

Eigen::Matrix2cd exp(const Eigen::Matrix2cd& X) {
    std::complex<double> a = X(0, 0);
    std::complex<double> b = X(0, 1);
    std::complex<double> c = X(1, 0);
    std::complex<double> d = X(1, 1);

    // formula from Mathematica (TODO: find reference)
    std::complex<double> e  = exp((a + d) / 2.);
    std::complex<double> r  = sqrt(4. * b * c + pow(a - d, 2.));
    std::complex<double> ch = cosh(r / 2.);
    std::complex<double> sh = sinh(r / 2.);

    Eigen::Matrix2cd result;
    result(0, 0) = e * (ch + (a - d) * sh / r);
    result(0, 1) = 2. * b * e * sh / r;
    result(1, 0) = 2. * c * e * sh / r;
    result(1, 1) = e * (ch + (d - a) * sh / r);

    return result;
}

Eigen::Matrix2cd log(const Eigen::Matrix2cd& X) {
    std::complex<double> a = X(0, 0);
    std::complex<double> b = X(0, 1);
    std::complex<double> c = X(1, 0);
    std::complex<double> d = X(1, 1);

    // formula from Mathematica (TODO: find reference)
    std::complex<double> r  = sqrt(4. * b * c + pow(a - d, 2.));
    std::complex<double> lm = log((a - r + d) / 2.);
    std::complex<double> lp = log((a + r + d) / 2.);

    Eigen::Matrix2cd result;
    result(0, 0) = ((d - a + r) * lm + (a - d + r) * lp) / (2. * r);
    result(0, 1) = b * (lp - lm) / r;
    result(1, 0) = c * (lp - lm) / r;
    result(1, 1) = ((a - d + r) * lm + (d - a + r) * lp) / (2. * r);
    return result;
}

// compute Mobius transformation sending z-> w
// (normalized so to have determinant 1)
Eigen::Matrix2cd
findMobiusTransformation(const std::array<std::complex<double>, 3>& z,
                         const std::array<std::complex<double>, 3>& w) {
    // https://en.wikipedia.org/wiki/M%C3%B6bius_transformation#Explicit_determinant_formula
    // clang-format off
    std::complex<double> a = Eigen::Matrix3cd{
        {z[0] * w[0], w[0], 1.},
        {z[1] * w[1], w[1], 1.},
        {z[2] * w[2], w[2], 1.}
    }.determinant();

    std::complex<double> b = Eigen::Matrix3cd{
      {z[0] * w[0], z[0], w[0]},
      {z[1] * w[1], z[1], w[1]},
      {z[2] * w[2], z[2], w[2]}
    }.determinant();

    std::complex<double> c = Eigen::Matrix3cd{
      {z[0], w[0], 1.},
      {z[1], w[1], 1.},
      {z[2], w[2], 1.}
    }.determinant();

    std::complex<double> d = Eigen::Matrix3cd{
      {z[0] * w[0], z[0], 1.},
      {z[1] * w[1], z[1], 1.},
      {z[2] * w[2], z[2], 1.}
    }.determinant();
    // clang-format on

    std::complex<double> det = a * d - b * c;

    return Eigen::Matrix2cd{{a, b}, {c, d}} / sqrt(det);
}

// compute Mobius transformation sending z->
// (normalized so to have determinant 1)
Eigen::Matrix2cd findMobiusTransformation(const std::array<Vector2, 3>& z,
                                          const std::array<Vector2, 3>& w) {
    // https://en.wikipedia.org/wiki/M%C3%B6bius_transformation#Explicit_determinant_formula
    // clang-format off
    std::complex<double> a = Eigen::Matrix3cd{
        {z[0] * w[0], w[0], 1.},
        {z[1] * w[1], w[1], 1.},
        {z[2] * w[2], w[2], 1.}
    }.determinant();

    std::complex<double> b = Eigen::Matrix3cd{
      {z[0] * w[0], z[0], w[0]},
      {z[1] * w[1], z[1], w[1]},
      {z[2] * w[2], z[2], w[2]}
    }.determinant();

    std::complex<double> c = Eigen::Matrix3cd{
      {z[0], w[0], 1.},
      {z[1], w[1], 1.},
      {z[2], w[2], 1.}
    }.determinant();

    std::complex<double> d = Eigen::Matrix3cd{
      {z[0] * w[0], z[0], 1.},
      {z[1] * w[1], z[1], 1.},
      {z[2] * w[2], z[2], 1.}
    }.determinant();
    // clang-format on

    std::complex<double> det = a * d - b * c;

    return Eigen::Matrix2cd{{a, b}, {c, d}} / sqrt(det);
}
