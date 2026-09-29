//
//  ikbt_pose_error.h --  how far one pose is from another
//
//  The C++ twin of output_numeric_common.POSE_ERROR_CORE, function for
//  function.  ONE definition of "closer" across the methods: the residual
//  [dp ; w_rot*theta*axis] and the scalar metric ||dp|| + w_rot*theta share
//  one rotation parameterisation and one weight, so a hybrid refinement and a
//  one-variable search cannot disagree about which of two joint vectors is
//  nearer the goal.
//
//  SOURCE, not generated -- see Cpp_src/ikbt_types.h.
//
//  Copyright 2026 University of Washington
//
//  Developed by Blake Hannaford
//  BioRobotics Lab, University of Washington

#ifndef IKBT_POSE_ERROR_H
#define IKBT_POSE_ERROR_H

#include "ikbt_types.h"

namespace ikbt {

typedef std::array<std::array<double, 3>, 3> Mat3;

//  The 6-vector residual and the scalar metric built from it, together,
//  because they must never be computed two different ways.
struct PoseError {
    std::array<double, 6> residual;
    double metric;
};


//  (theta, axis) of a rotation matrix, theta in [0, pi].
//
//  THETA COMES FROM atan2 OF THE SKEW NORM, NOT arccos((tr-1)/2).  arccos has
//  an infinite derivative at R = I, so the O(1e-16) rounding in the trace
//  comes back out as O(1e-8) radians, and w_rot scales that to O(1e-6) in the
//  metric -- a floor no search can see past, and one that is QUANTISED, so
//  the curve near a root is a staircase rather than a V and golden section
//  cannot see past a step.  atan2 is linear in the perturbation and gives
//  exactly 0.0 at R = I.  Same reasoning, and the same fix, as the python
//  twin and as dh_analysis.py.
inline double rotation_angle_axis(const Mat3 &Rerr, std::array<double, 3> &axis)
{
    const double skew[3] = {Rerr[2][1] - Rerr[1][2],
                            Rerr[0][2] - Rerr[2][0],
                            Rerr[1][0] - Rerr[0][1]};
    const double s = std::sqrt(skew[0] * skew[0] + skew[1] * skew[1]
                               + skew[2] * skew[2]);
    const double tr = Rerr[0][0] + Rerr[1][1] + Rerr[2][2];
    const double theta = std::atan2(0.5 * s, (tr - 1.0) / 2.0);

    axis[0] = axis[1] = axis[2] = 0.0;
    if (theta < 1e-12)
        return 0.0;
    if (theta < M_PI - 1e-6) {
        for (int i = 0; i < 3; ++i)      // s = 2*sin(theta), never 0 here
            axis[i] = skew[i] / s;
        return theta;
    }

    //  theta ~ pi:  the skew part vanishes, so take the axis from R + I = 2aa'
    double Msym[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            Msym[i][j] = Rerr[i][j] + (i == j ? 1.0 : 0.0);

    int best = 0;
    double bestn = -1.0;
    for (int j = 0; j < 3; ++j) {        // largest COLUMN norm, as numpy does
        double n = 0.0;
        for (int i = 0; i < 3; ++i)
            n += Msym[i][j] * Msym[i][j];
        n = std::sqrt(n);
        if (n > bestn) { bestn = n; best = j; }
    }
    if (bestn < 1e-12)
        return theta;                    // axis stays zero
    for (int i = 0; i < 3; ++i)
        axis[i] = Msym[i][best] / bestn;
    return theta;
}


//  Residual and scalar metric between pose T and target T_d.
inline PoseError pose_error(const Mat4 &T, const Mat4 &T_d, double w_rot)
{
    PoseError out;

    double dp[3];
    for (int i = 0; i < 3; ++i)
        dp[i] = T_d[i][3] - T[i][3];

    //  Rerr = T_d.R * T.R'
    Mat3 Rerr;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            double s = 0.0;
            for (int k = 0; k < 3; ++k)
                s += T_d[i][k] * T[j][k];
            Rerr[i][j] = s;
        }

    std::array<double, 3> axis;
    const double theta = rotation_angle_axis(Rerr, axis);

    for (int i = 0; i < 3; ++i) {
        out.residual[i] = dp[i];
        out.residual[3 + i] = w_rot * theta * axis[i];
    }
    out.metric = std::sqrt(dp[0] * dp[0] + dp[1] * dp[1] + dp[2] * dp[2])
                 + w_rot * theta;
    return out;
}

}   // namespace ikbt

#endif   // IKBT_POSE_ERROR_H
