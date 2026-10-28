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
//  C++17, not C++11:  a fixed-size Eigen type (Eigen::Matrix4d) inside a std::vector or
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

//  WHICH EIGEN.  One program must never hold two:  the module guards
//  (EIGEN_CORE_H, EIGEN_LU_MODULE_H, ...) are the same in every 3.x, so a
//  second copy included after a first is silently half-skipped, and the
//  program ends up with one version's Core under another's LU.  So the
//  APPLICATION's Eigen wins whenever there is one:
//
//    - IKBT_VENDORED_EIGEN       forces the copy shipped here
//    - IKBT_SYSTEM_EIGEN         forces <Eigen/Dense>
//    - EIGEN_WORLD_VERSION       Eigen is already included -- use that one
//    - __has_include(<Eigen/Dense>)   the build has an Eigen on its include
//                                path (ROS, or CMake's find_package(Eigen3))
//    - otherwise                 the vendored copy, by a path relative to
//                                THIS file, so a bare g++ needs no -I
//
//  A distribution's Eigen lives in /usr/include/eigen3/, which is not on the
//  default search path, so a bare `g++ -std=c++17 Robot.cpp` still gets the
//  vendored copy:  having Eigen installed is not the same as using it.
#if defined(IKBT_VENDORED_EIGEN)
#include "third_party/eigen3/Eigen/Dense"
#elif defined(IKBT_SYSTEM_EIGEN) || defined(EIGEN_WORLD_VERSION) \
      || __has_include(<Eigen/Dense>)
#include <Eigen/Dense>
#else
#include "third_party/eigen3/Eigen/Dense"
#endif

#if !EIGEN_VERSION_AT_LEAST(3, 3, 0)
#error "IKBT's generated C++ needs Eigen 3.3 or newer"
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

//  NO TYPE ALIASES.  Every signature spells out the Eigen type, so a reader
//  sees at once that an answer is an Eigen object and needs no conversion:
//
//    Eigen::Matrix4d               an end-effector-pose.  T(row, col);
//                                  T(0, 3) is x, as in the numpy 4x4.
//                                  Column-major storage matters only to code
//                                  reading raw memory -- from_array() and
//                                  to_array() below do that properly.
//    Eigen::VectorXd               one joint-pose.  Dynamic size, so the
//                                  shared numerics work for any DOF.
//    std::vector<Eigen::VectorXd>  one joint-pose per solution branch.  An
//                                  unreachable end-effector-pose is an EMPTY
//                                  list (python returns False).
//    Eigen::MatrixXd               a Jacobian, 6 x NDOF.
//    Eigen::Matrix<double, 6, 1>   a pose residual [dp ; w_rot*theta*axis].

const double INF = std::numeric_limits<double>::infinity();


//  Interop with a plain row-major C array, for a caller holding a
//  double[4][4].  Element by element, so the storage order of Eigen::Matrix4d does not
//  matter.
inline Eigen::Matrix4d from_array(const double T[4][4])
{
    Eigen::Matrix4d M;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            M(i, j) = T[i][j];
    return M;
}


inline void to_array(const Eigen::Matrix4d &M, double T[4][4])
{
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            T[i][j] = M(i, j);
}


//  J_0 = blkdiag(R_06, R_06) . J_66 -- frame 6 into the base frame.
//
//  Here, not with the numerics, because every generated file's jacobian()
//  returns J_66 and every generated file includes this header:  so the
//  rotation is callable whichever solution path wrote the file.
inline Eigen::MatrixXd jacobian_base(const Eigen::Matrix4d &T, const Eigen::MatrixXd &J66)
{
    const Eigen::Matrix3d R = T.topLeftCorner<3, 3>();
    Eigen::MatrixXd J(6, J66.cols());
    J.topRows<3>() = R * J66.topRows<3>();
    J.bottomRows<3>() = R * J66.bottomRows<3>();
    return J;
}

}   // namespace ikbt

#endif   // IKBT_TYPES_H
