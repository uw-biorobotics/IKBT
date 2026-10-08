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
//  THE 6x6 SOLVE IS LU WITH PARTIAL PIVOTING, the factorisation numpy's
//  np.linalg.solve uses (LAPACK dgesv).  JJ' + lam^2 I is symmetric positive
//  definite, so LDLT would also do -- but the iteration count is pinned
//  against python, and the same factorisation is the shortest route to the
//  same rounding.
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
//  transitively:  a generated artifact should not depend on which standard
//  library the user has.
#include <algorithm>
#include <functional>

#include "ikbt_types.h"
#include "ikbt_pose_error.h"

namespace ikbt {

using FkFn  = std::function<Mat4(const JointVec &)>;
using JacFn = std::function<Matrix(const JointVec &)>;

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
    const Mat3 R = T.topLeftCorner<3, 3>();
    Matrix J(6, J66.cols());
    J.topRows<3>() = R * J66.topRows<3>();
    J.bottomRows<3>() = R * J66.bottomRows<3>();
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
    Vec6 w;
    w << 1.0, 1.0, 1.0, w_rot, w_rot, w_rot;

    //  The python closure measure(), with `inf` for a non-finite FK.
    auto measure = [&](const JointVec &qq, Vec6 &e_out, double &m_out) {
        const Mat4 T = fk(qq);
        if (!T.allFinite()) {
            e_out.setZero();
            m_out = INF;
        } else {
            const PoseError pe = pose_error(T, T_d, w_rot);
            e_out = pe.residual;
            m_out = pe.metric;
        }
    };

    Vec6 e;
    double m;
    measure(q, e, m);

    SolveResult out;
    out.history.push_back(m);
    out.reason = "max_iter";

    for (int it = 0; it < max_iter; ++it) {
        if (m <= tol) {
            out.reason = "converged";
            break;
        }

        const Matrix J = w.asDiagonal() * jacobian_base(fk(q), jac(q));   // W J
        Eigen::Matrix<double, 6, 6> A = J * J.transpose();
        A.diagonal().array() += lam * lam;

        //  A singular or non-finite system is a rejected step, as numpy's
        //  LinAlgError is in python.  Eigen's LU does not report singularity;
        //  it divides by the zero pivot, and the finiteness test catches it.
        bool ok = A.allFinite() && e.allFinite();
        JointVec dq;
        if (ok) {
            dq = J.transpose() * A.partialPivLu().solve(e);
            ok = dq.allFinite();
        }
        if (!ok) {
            lam = std::min(lam * 10.0, lam_max);
            if (lam >= lam_max) {
                out.reason = "lam_max";
                break;
            }
            continue;                    //  no history entry, as in python
        }

        const JointVec q_try = q + dq;
        Vec6 e_try;
        double m_try;
        measure(q_try, e_try, m_try);

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
