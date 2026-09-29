//
//  ikbt_types.h --  the types and constants every generated IKBT module uses
//
//  SOURCE, not a generated artifact.  Cpp_src/ is to CodeGen/Cpp/ what
//  LaTex_src/ is to LaTex/:  hand-written material that the generator READS
//  AND INLINES, so that a generated .cpp is self-contained and compiles with
//
//      g++ -std=c++11 -O2 IK_equations<Robot>.cpp -o ik   (plus -lm)
//
//  and no -I.  It lives apart from CodeGen/Cpp/ so that wiping the artifacts
//  (CodeGen/cleanCodeGenOutput) cannot take the sources with it.
//
//  Each header here is ALSO independently compilable, which is the whole
//  reason they are files rather than python string literals: the numerics can
//  be unit-tested as C++ instead of only through a generated module.
//
//  Copyright 2026 University of Washington
//
//  Developed by Blake Hannaford
//  BioRobotics Lab, University of Washington

#ifndef IKBT_TYPES_H
#define IKBT_TYPES_H

#include <array>
#include <cmath>
#include <limits>
#include <vector>

//  M_PI and friends are POSIX, not ISO C++, so -std=c++11 (as opposed to
//  -std=gnu++11) can leave them undefined.  sympy's C++ printer emits them for
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

//  A homogeneous transform, ROW MAJOR:  T[row][col], T[0][3] is x.
//  The same layout as the numpy 4x4 the python modules pass around, and the
//  same layout as the `double T[4][4]` the older generated C++ took.
typedef std::array<std::array<double, 4>, 4> Mat4;

//  One joint vector.  std::vector, not std::array<double, NDOF>:  the shared
//  numerics below have to work for any arm without being templated on its DOF.
typedef std::vector<double> JointVec;

//  One row per solution branch.  The python twin returns a list of lists and
//  False for an unreachable pose;  here an unreachable pose is an EMPTY list,
//  which is the same information without a second return type.
typedef std::vector<JointVec> SolutionList;

const double INF = std::numeric_limits<double>::infinity();


inline Mat4 identity4()
{
    Mat4 T;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            T[i][j] = (i == j) ? 1.0 : 0.0;
    return T;
}


//  Interop with the plain C array the first generation of this code used, so
//  a caller holding a double[4][4] does not have to be rewritten.
inline Mat4 from_array(const double T[4][4])
{
    Mat4 M;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            M[i][j] = T[i][j];
    return M;
}


inline void to_array(const Mat4 &M, double T[4][4])
{
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            T[i][j] = M[i][j];
}


inline Mat4 mat_mul(const Mat4 &A, const Mat4 &B)
{
    Mat4 C;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            double s = 0.0;
            for (int k = 0; k < 4; ++k)
                s += A[i][k] * B[k][j];
            C[i][j] = s;
        }
    return C;
}


//  Every entry finite?  The python side tests np.all(np.isfinite(T)) before
//  trusting an FK result, and so does this.
inline bool all_finite(const Mat4 &T)
{
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            if (!std::isfinite(T[i][j]))
                return false;
    return true;
}


inline bool all_finite(const JointVec &q)
{
    for (size_t i = 0; i < q.size(); ++i)
        if (!std::isfinite(q[i]))
            return false;
    return true;
}

}   // namespace ikbt

#endif   // IKBT_TYPES_H
