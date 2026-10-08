//
//  ikbt_types.h --  the types and constants every generated IKBT module uses
//
//  SOURCE, not a generated artifact.  Cpp_src/ is to CodeGen/Cpp/ what
//  LaTex_src/ is to LaTex/:  hand-written material that generated files
//  reach by a relative #include, so that a generated .cpp compiles with
//
//      g++ -std=c++17 -O2 <Robot>.cpp -o ik   (plus -lm)
//
//  and no -I.  It lives apart from CodeGen/Cpp/ so that wiping the artifacts
//  (CodeGen/cleanCodeGenOutput) cannot take the sources with it.
//
//  Each header here is ALSO independently compilable, which is the whole
//  reason they are files rather than python string literals: the numerics can
//  be unit-tested as C++ instead of only through a generated module.
//
//  THE TYPES ARE EIGEN'S.  An application that does anything with an IK answer
//  -- a norm, a comparison, an inverse, a product -- wants a real linear
//  algebra library, and hand-rolled types would mean re-writing one.  Eigen
//  is header-only, so it costs no link step, and a copy is VENDORED in
//  third_party/eigen3/ so a user installs nothing.  See VENDORED.md there.
//
//  C++17, not C++11:  a fixed-size Eigen type (Mat4) inside a std::vector or
//  a struct needs over-aligned allocation, which C++17's operator new provides
//  and C++11 does not.  Under C++11 that is a crash waiting for the right
//  allocation;  under C++17 it simply works.
//
//  Copyright 2026 University of Washington
//
//  Developed by Blake Hannaford
//  BioRobotics Lab, University of Washington

#ifndef IKBT_TYPES_H
#define IKBT_TYPES_H

#if __cplusplus < 201703L
#error "IKBT's generated C++ needs C++17:  compile with -std=c++17"
#endif

#include <cmath>
#include <limits>
#include <vector>

//  The vendored copy, by a path relative to THIS file, so no -I is needed.
//  An application with its own Eigen defines IKBT_SYSTEM_EIGEN, so that one
//  program never holds two Eigens.
#ifdef IKBT_SYSTEM_EIGEN
#include <Eigen/Dense>
#else
#include "third_party/eigen3/Eigen/Dense"
#endif

//  M_PI and friends are POSIX, not ISO C++, so -std=c++17 (as opposed to
//  -std=gnu++17) can leave them undefined.  sympy's C++ printer emits them for
//  pi, pi/2 and pi/4, so define what it can emit.
#ifndef M_PI
#define M_PI   3.14159265358979323846
#endif
#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif
#ifndef M_PI_4
#define M_PI_4 0.78539816339744830962
#endif
#ifndef M_E
#define M_E    2.71828182845904523536
#endif

namespace ikbt {

//  A homogeneous transform.  T(row, col);  T(0, 3) is x.  The same indexing
//  as the numpy 4x4 the python modules pass around.  Eigen stores it column
//  major, which matters only to code reading the raw memory -- from_array()
//  and to_array() below do that properly.
using Mat4 = Eigen::Matrix4d;
using Mat3 = Eigen::Matrix3d;

//  A 6-vector:  a pose residual [dp ; w_rot*theta*axis].
using Vec6 = Eigen::Matrix<double, 6, 1>;

//  One joint vector.  Dynamic size, not Matrix<double, NDOF, 1>:  the shared
//  numerics below have to work for any arm without being templated on its DOF.
using JointVec = Eigen::VectorXd;

//  One row per solution branch.  The python twin returns a list of lists and
//  False for an unreachable pose;  here an unreachable pose is an EMPTY list,
//  which is the same information without a second return type.
//
//  A std::vector, not a matrix:  the number of branches is a property of the
//  pose, and a caller iterates over them one posture at a time.
using SolutionList = std::vector<JointVec>;

//  A dense matrix of any shape -- what a Jacobian comes back as (6 x NDOF).
using Matrix = Eigen::MatrixXd;

const double INF = std::numeric_limits<double>::infinity();


//  Interop with a plain row-major C array, for a caller holding a
//  double[4][4].  Element by element, so the storage order of Mat4 does not
//  matter.
inline Mat4 from_array(const double T[4][4])
{
    Mat4 M;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            M(i, j) = T[i][j];
    return M;
}


inline void to_array(const Mat4 &M, double T[4][4])
{
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            T[i][j] = M(i, j);
}

}   // namespace ikbt

#endif   // IKBT_TYPES_H
