//
//  ikbt_linalg.h --  the only linear algebra a generated module needs
//
//  numpy's np.linalg.solve, for the one shape the damped-least-squares step
//  actually uses:  a 6x6 system, once per iteration.  Gaussian elimination
//  with partial pivoting, ~40 lines.
//
//  NO EIGEN, NO BLAS.  A generated artifact must not make the user install
//  anything -- the same rule that says a generated python module stands on
//  numpy alone.  A 6x6 solve is 216 multiplies;  a dependency to avoid
//  writing it would cost more than it saves.
//
//  RETURNS false RATHER THAN THROWING on a singular system.  numpy raises
//  LinAlgError there and solve_numeric() catches it, raises the damping and
//  tries again -- a singular solve is a REJECTED STEP, not an error.  That is
//  the whole point of damping: at a singularity an undamped Newton step does
//  not exist and this must degrade rather than fail.
//
//  SOURCE, not generated -- see Cpp_src/ikbt_types.h.
//
//  Copyright 2026 University of Washington
//
//  Developed by Blake Hannaford
//  BioRobotics Lab, University of Washington

#ifndef IKBT_LINALG_H
#define IKBT_LINALG_H

#include "ikbt_types.h"

namespace ikbt {

//  Solve A x = b for a 6x6 A.  A and b are copied, not modified.
//  false means singular (or non-finite), and x is then untouched.
inline bool solve6(const double A_in[6][6], const std::array<double, 6> &b_in,
                   std::array<double, 6> &x)
{
    const int N = 6;
    double A[6][7];

    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            if (!std::isfinite(A_in[i][j]))
                return false;
            A[i][j] = A_in[i][j];
        }
        if (!std::isfinite(b_in[i]))
            return false;
        A[i][N] = b_in[i];
    }

    for (int col = 0; col < N; ++col) {
        //  partial pivot
        int piv = col;
        double best = std::fabs(A[col][col]);
        for (int r = col + 1; r < N; ++r) {
            const double v = std::fabs(A[r][col]);
            if (v > best) { best = v; piv = r; }
        }
        if (best == 0.0)
            return false;                 // exactly singular
        if (piv != col)
            for (int j = col; j <= N; ++j)
                std::swap(A[col][j], A[piv][j]);

        const double d = A[col][col];
        for (int r = col + 1; r < N; ++r) {
            const double f = A[r][col] / d;
            if (f == 0.0)
                continue;
            for (int j = col; j <= N; ++j)
                A[r][j] -= f * A[col][j];
        }
    }

    for (int i = N - 1; i >= 0; --i) {
        double s = A[i][N];
        for (int j = i + 1; j < N; ++j)
            s -= A[i][j] * x[j];
        x[i] = s / A[i][i];
        if (!std::isfinite(x[i]))
            return false;
    }
    return true;
}

}   // namespace ikbt

#endif   // IKBT_LINALG_H
