//
//  ikbt_dls.h --  Phase II numerics:  damped least squares (Levenberg-Marquardt)
//
//      dq = J'(JJ' + lam^2 I)^-1 e
//
//  with the residual [dp ; w_rot*theta*axis] and the scalar metric
//  ||dp|| + w_rot*theta sharing ONE rotation parameterisation and ONE weight,
//  so the step and the stopping test agree about what "closer" means.  The
//  Jacobian's rotation rows are scaled by w_rot to match.
//
//  The C++ twin of output_hybrid_python.DLS_CORE, which is itself a copy of
//  ikbtbasics/numeric_ik.py.  A copy is a liability, so it is pinned:
//  scripts/cpp_closed_loop_check.py --dls runs this and the python
//  solve_numeric() on the same robot from the same seed and requires the same
//  answer, iteration for iteration.
//
//  SOURCE, not generated -- see Cpp_src/ikbt_types.h.
//
//  Copyright 2026 University of Washington
//
//  Developed by Blake Hannaford
//  BioRobotics Lab, University of Washington

#ifndef IKBT_DLS_H
#define IKBT_DLS_H

//  <algorithm> for std::min/std::max.  Named explicitly rather than picked up
//  transitively from <vector>:  libstdc++ happens to supply it, and a
//  generated artifact should not depend on which standard library the user
//  has.
#include <algorithm>
#include <functional>

#include "ikbt_types.h"
#include "ikbt_linalg.h"
#include "ikbt_pose_error.h"

namespace ikbt {

typedef std::function<Mat4(const JointVec &)>   FkFn;
typedef std::function<Matrix(const JointVec &)> JacFn;

//  What solve_numeric() reports.  The python twin returns a dict with exactly
//  these keys;  `history` is kept because its length is the iteration count
//  and its shape is how a non-converging solve is diagnosed.
struct SolveResult {
    JointVec q;
    double metric;
    int iterations;
    bool converged;
    const char *reason;
    std::vector<double> history;
};


//  J_0 = blkdiag(R_06, R_06) . J_66 -- frame 6 into the base frame.
inline Matrix jacobian_base(const Mat4 &T, const Matrix &J66)
{
    const size_t n = J66.empty() ? 0 : J66[0].size();
    Matrix J(6, std::vector<double>(n, 0.0));
    for (int blk = 0; blk < 2; ++blk)
        for (int i = 0; i < 3; ++i)
            for (size_t c = 0; c < n; ++c) {
                double s = 0.0;
                for (int k = 0; k < 3; ++k)
                    s += T[i][k] * J66[3 * blk + k][c];
                J[3 * blk + i][c] = s;
            }
    return J;
}


//  Refine seed q0 until fk(q) matches T_d.
//
//  NEVER FAILS ON A BAD STEP:  a singular solve or a non-finite FK is a
//  REJECTED step, which raises lam and tries again.
inline SolveResult solve_numeric(const FkFn &fk, const JacFn &jac,
                                 const JointVec &q0, const Mat4 &T_d,
                                 double w_rot, double tol = 1e-9,
                                 int max_iter = 100, double lam0 = 1e-3,
                                 double lam_min = 1e-12, double lam_max = 1e12)
{
    JointVec q = q0;
    double lam = lam0;
    const double w[6] = {1.0, 1.0, 1.0, w_rot, w_rot, w_rot};

    //  measure(): the python closure, with `inf` for a non-finite FK.
    std::array<double, 6> e;
    double m;
    {
        const Mat4 T = fk(q);
        if (!all_finite(T)) {
            e.fill(0.0);
            m = INF;
        } else {
            const PoseError pe = pose_error(T, T_d, w_rot);
            e = pe.residual;
            m = pe.metric;
        }
    }

    SolveResult out;
    out.history.push_back(m);
    out.reason = "max_iter";

    for (int it = 0; it < max_iter; ++it) {
        if (m <= tol) {
            out.reason = "converged";
            break;
        }

        const Mat4 T = fk(q);
        const Matrix J66 = jac(q);
        Matrix J = jacobian_base(T, J66);
        const size_t n = J.empty() ? 0 : J[0].size();
        for (int i = 0; i < 6; ++i)                 // J <- W J
            for (size_t c = 0; c < n; ++c)
                J[i][c] *= w[i];

        double A[6][6];
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j) {
                double s = 0.0;
                for (size_t c = 0; c < n; ++c)
                    s += J[i][c] * J[j][c];
                A[i][j] = s + (i == j ? lam * lam : 0.0);
            }

        std::array<double, 6> y;
        bool ok = solve6(A, e, y);

        JointVec dq(n, 0.0);
        if (ok) {
            for (size_t c = 0; c < n; ++c) {
                double s = 0.0;
                for (int i = 0; i < 6; ++i)
                    s += J[i][c] * y[i];
                dq[c] = s;
            }
            ok = all_finite(dq);
        }
        if (!ok) {
            lam = std::min(lam * 10.0, lam_max);
            if (lam >= lam_max) {
                out.reason = "lam_max";
                break;
            }
            continue;                    //  no history entry, as in python
        }

        JointVec q_try(n);
        for (size_t c = 0; c < n; ++c)
            q_try[c] = q[c] + dq[c];

        std::array<double, 6> e_try;
        double m_try;
        {
            const Mat4 Tt = fk(q_try);
            if (!all_finite(Tt)) {
                e_try.fill(0.0);
                m_try = INF;
            } else {
                const PoseError pe = pose_error(Tt, T_d, w_rot);
                e_try = pe.residual;
                m_try = pe.metric;
            }
        }

        if (m_try < m) {
            q = q_try;
            e = e_try;
            m = m_try;
            lam = std::max(lam / 3.0, lam_min);
        } else {
            lam = std::min(lam * 3.0, lam_max);
            if (lam >= lam_max) {
                out.reason = "lam_max";
                break;
            }
        }
        out.history.push_back(m);
    }

    out.q = q;
    out.metric = m;
    out.iterations = (int) out.history.size() - 1;
    out.converged = (m <= tol);
    if (out.converged)
        out.reason = "converged";
    return out;
}

}   // namespace ikbt

#endif   // IKBT_DLS_H
