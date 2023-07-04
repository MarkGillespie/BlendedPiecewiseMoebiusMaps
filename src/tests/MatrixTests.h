#include "geometrycentral/surface/barycentric_coordinate_helpers.h"
#include "test_utils.h"

#include "Mobius.h"

class MatrixTest : public ::testing::Test {};

TEST_F(MatrixTest, ExpLogIsId) {
    for (size_t iR = 0; iR < 10; iR++) {
        Eigen::Matrix2cd X       = Eigen::Matrix2cd::Random(2, 2);
        Eigen::Matrix2cd ExpLogX = exp(log(X));
        EXPECT_MATC_NEAR(X, ExpLogX, 1e-5 * X.norm());
    }
}

TEST_F(MatrixTest, LogExpIsId) {
    for (size_t iR = 0; iR < 10; iR++) {
        Eigen::Matrix2cd X       = Eigen::Matrix2cd::Random(2, 2);
        Eigen::Matrix2cd LogExpX = log(exp(X));
        EXPECT_MATC_NEAR(X, LogExpX, 1e-5 * X.norm());
    }
}

TEST_F(MatrixTest, RigidMobiusConstruction) {
    for (size_t iR = 0; iR < 10; iR++) {
        std::array<std::complex<double>, 3> z, w;
        for (size_t iP = 0; iP < 3; iP++)
            z[iP] = std::complex<double>{fRand(0, 1), fRand(0, 1)};

        std::complex<double> scale{fRand(0, 1), fRand(0, 1)};
        std::complex<double> translate{fRand(0, 1), fRand(0, 1)};
        for (size_t iP = 0; iP < 3; iP++) w[iP] = scale * z[iP] + translate;

        Eigen::Matrix2cd f = findMobiusTransformation(z, w);

        for (size_t iP = 0; iP < 3; iP++) {
            Vector3 bary =
                normalizeBarycentric({unitRand(), unitRand(), unitRand()});
            std::complex<double> zi =
                bary[0] * z[0] + bary[1] * z[1] + bary[2] * z[2];
            std::complex<double> wi =
                bary[0] * w[0] + bary[1] * w[1] + bary[2] * w[2];

            EXPECT_CPLX_NEAR(wi, applyMobiusTransformation(f, zi), 1e-5);
        }
        EXPECT_CPLX_NEAR(f.determinant(), 1., 1e-5);
    }
}

TEST_F(MatrixTest, MobiusConstruction) {
    for (size_t iR = 0; iR < 10; iR++) {
        std::array<std::complex<double>, 3> z, w;
        for (size_t iP = 0; iP < 3; iP++) {
            z[iP] = std::complex<double>{fRand(0, 1), fRand(0, 1)};
            w[iP] = std::complex<double>{fRand(0, 1), fRand(0, 1)};
        }
        Eigen::Matrix2cd f = findMobiusTransformation(z, w);
        for (size_t iP = 0; iP < 3; iP++) {
            EXPECT_CPLX_NEAR(w[iP], applyMobiusTransformation(f, z[iP]), 1e-5);
        }
        EXPECT_CPLX_NEAR(f.determinant(), 1., 1e-5);
    }
}
