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
//  NOT Eigen::AngleAxisd.  It takes the angle from an arccosine -- see below
//  for why that is the wrong choice here -- and its axis near theta = pi is
//  chosen differently from the python twin's, which the two languages must
//  agree on.
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

//  The 6-vector residual and the scalar metric built from it, together,
//  because they must never be computed two different ways.
struct PoseError {
    Vec6 residual;
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
inline double rotation_angle_axis(const Mat3 &Rerr, Eigen::Vector3d &axis)
{
    const Eigen::Vector3d skew(Rerr(2, 1) - Rerr(1, 2),
                               Rerr(0, 2) - Rerr(2, 0),
                               Rerr(1, 0) - Rerr(0, 1));
    const double s = skew.norm();
    const double theta = std::atan2(0.5 * s, (Rerr.trace() - 1.0) / 2.0);

    axis.setZero();
    if (theta < 1e-12)
        return 0.0;
    if (theta < M_PI - 1e-6) {
        axis = skew / s;                 // s = 2*sin(theta), never 0 here
        return theta;
    }

    //  theta ~ pi:  the skew part vanishes, so take the axis from R + I = 2aa',
    //  using its largest COLUMN, as numpy does.  maxCoeff() keeps the first of
    //  equal values, as np.argmax does.
    const Mat3 Msym = Rerr + Mat3::Identity();
    Eigen::Index best;
    const double bestn = Msym.colwise().norm().maxCoeff(&best);
    if (bestn < 1e-12)
        return theta;                    // axis stays zero
    axis = Msym.col(best) / bestn;
    return theta;
}


//  Residual and scalar metric between pose T and target T_d.
inline PoseError pose_error(const Mat4 &T, const Mat4 &T_d, double w_rot)
{
    const Eigen::Vector3d dp = T_d.block<3, 1>(0, 3) - T.block<3, 1>(0, 3);
    const Mat3 Rerr = T_d.topLeftCorner<3, 3>()
                      * T.topLeftCorner<3, 3>().transpose();

    Eigen::Vector3d axis;
    const double theta = rotation_angle_axis(Rerr, axis);

    PoseError out;
    out.residual << dp, w_rot * theta * axis;
    out.metric = dp.norm() + w_rot * theta;
    return out;
}

}   // namespace ikbt

#endif   // IKBT_POSE_ERROR_H
