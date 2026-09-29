#!/usr/bin/python
#
#   output_cpp_onevar.py --  the one-variable method's C++ top level
#
#   The twin of output_onevar_python.write_onevar_top().  What a one-variable
#   solve delivers in C++, for e.g. C-Arm with th_2 assumed known:
#
#       CodeGen/Cpp/IK_onevarC-Arm.cpp       the 1-D search  <- the entry point
#       CodeGen/Cpp/IK_conditionalC-Arm.cpp  the closed form, th_2 an argument
#       CodeGen/Cpp/FK_numericC-Arm.h        this arm's FK (no Jacobian needed)
#
#   IK_conditional, NEVER IK_equations:  that name means an unconditional
#   inverse kinematics for the robot, and these equations hold only where the
#   assumed value is right.
#
#   NOTHING ABOUT THE ROBOT IS APPROXIMATED.  The DH table, the FK and the
#   equations are the true arm's;  the only edit was removing one entry from
#   the unknowns list.  That is what separates this from the hybrid path, and
#   why every file here carries the real robot's name.
#
#   THE SEARCH ITSELF IS IN Cpp_src/ikbt_search.h, not emitted here.  The
#   python twin emits ~470 lines per robot because a generated python module
#   stands on numpy alone and has no library to call;  in C++ Cpp_src/ IS that
#   library, inlined, so the algorithm is written once and this file supplies
#   only the two robot-specific callbacks it takes.
#
#   Copyright 2026 University of Washington
#
#   Developed by Blake Hannaford
#   BioRobotics Lab, University of Washington

import os

import ikbtbasics.numeric_ik as nik
from ikbtfunctions.output_cpp_common import (DIR_NAME, cpp_identifier,
                                             file_header, inline_src,
                                             name_table_cpp)
from ikbtfunctions.output_onevar_python import search_domain


CORE_HEADERS = ['ikbt_types.h', 'ikbt_pose_error.h', 'ikbt_search.h']


def write_onevar_top_cpp(M, name, known, dirname=DIR_NAME, n_samples=128,
                         max_samples=4096):
    '''Write IK_onevar<name>.cpp -- the 1-D search over the assumed variable.

       M            the TRUE arm's mechanism (nothing here is approximated)
       name         the robot the user asked about
       known        the variable the closed form assumes is known, e.g. 'th_2'
       n_samples    where the resolution ladder starts
       max_samples  where it gives up looking for more

       Returns the path written.'''

    ident = cpp_identifier(name)
    #  BOTH ROUNDED UP TO A POWER OF TWO, for the same reason as the python
    #  twin:  the ladder only re-uses the coarse grid's cached errors when
    #  lo + span*(2k)/(2n) is the same double as lo + span*k/n, which it is
    #  exactly when n is a power of two.
    n_samples = 1 << max(1, int(n_samples) - 1).bit_length()
    max_samples = max(n_samples, 1 << max(1, int(max_samples) - 1).bit_length())

    ndof = M.ndof
    jnames = [str(s) for s in nik.joint_symbols(M, ndof)]
    #  `or 1.0`:  w_rot scales the acceptance tolerance, and a degenerate DH
    #  table (every length zero) would otherwise make it 0 -- a test no root
    #  can pass, so a solve that worked would report nothing found.
    w_rot = float(nik.w_rot_for(M, ndof)) or 1.0
    lo, hi, periodic = search_domain(M, known, ndof)

    filename = 'IK_onevar%s.cpp' % name
    path = os.path.join(dirname, filename)

    with open(path, 'w') as f:
        print(file_header('ONE-VARIABLE inverse kinematics for %s' % name,
                          name, filename), file=f)
        print(inline_src(CORE_HEADERS), file=f)
        print('', file=f)
        print('#include <cstdio>', file=f)
        print('#include <cstdlib>', file=f)
        print('', file=f)
        print('//  Siblings, in the same directory -- which is what', file=f)
        print('//  #include "..." searches first.', file=f)
        print('#include "FK_numeric%s.h"' % name, file=f)
        print('', file=f)
        print('//  The conditional closed form.  Its own self-test is', file=f)
        print('//  suppressed while it is included:  this file has one, and', file=f)
        print('//  two main()s do not link.', file=f)
        print('#ifdef IKBT_MAIN', file=f)
        print('#define IKBT_ONEVAR_MAIN_WAS_SET', file=f)
        print('#undef IKBT_MAIN', file=f)
        print('#endif', file=f)
        print('#include "IK_conditional%s.cpp"' % name, file=f)
        print('#ifdef IKBT_ONEVAR_MAIN_WAS_SET', file=f)
        print('#define IKBT_MAIN', file=f)
        print('#endif', file=f)
        print('', file=f)
        print('using namespace ikbt;', file=f)
        print('', file=f)

        print('''/////////////////////////////////////////////////////////////
//
//   READ THIS FIRST.
//
//   %s could not be solved in closed form outright.  What
//   IK_conditional%s.cpp holds is a closed form for every OTHER
//   joint, given a value of %s -- exact for THIS arm, no DH parameter
//   changed, but only where that value is right.
//
//   solve_%s(T) is the entry point:  it searches %s for the
//   values that are right, and returns complete joint vectors that reach T.
//   A joint vector taken straight from the conditional closed form, at a
//   value you picked yourself, does NOT put the robot at the pose you asked
//   for.
//
/////////////////////////////////////////////////////////////
''' % (name, name, known, ident, known), file=f)

        print('const char* const ROBOT          = "%s";' % name, file=f)
        print('const char* const KNOWN_VARIABLE = "%s";' % known, file=f)
        print('//  Chain order -- what a q vector is, and what comes back.', file=f)
        print(name_table_cpp(jnames, 'ONEVAR_JOINT_NAMES'), file=f)
        print('const int ONEVAR_NDOF = %d;' % ndof, file=f)
        print('', file=f)
        print('//  One characteristic arm length per radian:  the weight that', file=f)
        print('//  puts position error and orientation error in comparable', file=f)
        print('//  units.  In the DH table\'s units.', file=f)
        print('const double ONEVAR_W_ROT = %.17g;' % w_rot, file=f)
        print('', file=f)
        if periodic:
            print('//  %s is periodic, so this is the WHOLE domain and the'
                  % known, file=f)
            print('//  search over it is complete.', file=f)
        else:
            print('//  %s is PRISMATIC and its travel is not recorded anywhere'
                  % known, file=f)
            print('//  in a DH table, so these bounds are a guess:  the sum of', file=f)
            print('//  the arm\'s own lengths, which is past anything it can', file=f)
            print('//  reach.  Narrow them to the real travel if you know it.', file=f)
        print('const double SEARCH_LO = %.17g;' % float(lo), file=f)
        print('const double SEARCH_HI = %.17g;' % float(hi), file=f)
        print('const bool   PERIODIC  = %s;' % ('true' if periodic else 'false'),
              file=f)
        print('', file=f)
        print('//  Where the resolution ladder STARTS, and where it gives up.', file=f)
        print('//  Both powers of two, which is what makes a doubling re-use', file=f)
        print('//  the coarser grid exactly.', file=f)
        print('const int N_SAMPLES   = %d;' % int(n_samples), file=f)
        print('const int MAX_SAMPLES = %d;' % int(max_samples), file=f)
        print('', file=f)
        print('//  A refined minimum is a SOLUTION only if it gets this close', file=f)
        print('//  -- a millionth of an arm length.', file=f)
        print('const double ACCEPT_TOL = %.17g;' % (1e-6 * float(w_rot)), file=f)
        print('//  Two solutions closer than this in every joint are one.', file=f)
        print('const double DEDUP_TOL  = 1e-6;', file=f)
        print('', file=f)

        print(_BODY.replace('**IDENT**', ident), file=f)
        print(_MAIN.replace('**IDENT**', ident), file=f)

    return path


_BODY = '''
/////////////////////////////////////////////////////////////
//
//   The two robot-specific pieces the search takes as callbacks.
//
/////////////////////////////////////////////////////////////

//  Every closed-form branch at one assumed value.
//
//  -> joint vectors, ONEVAR_NDOF each, in ONEVAR_JOINT_NAMES order and
//  INCLUDING the assumed variable in its own chain position.  Empty when the
//  closed form reports the pose unreachable for that value, which is
//  ordinary: a branch is defined on part of the range, not all of it.
inline std::vector<JointVec> branches_at_**IDENT**(const Mat4 &T, double value)
{
    //  No errstate equivalent is needed and none is wanted:  where the closed
    //  form divides by zero or roots a negative, IEEE gives inf or NaN and
    //  says so quietly.  That is the branch being undefined there, which is
    //  DATA, not a fault -- errors_**IDENT**() turns it into INF.
    return ikin_**IDENT**_given(T, value);
}


//  Pose error of each branch at one assumed value;  INF where undefined.
//  This is the function the search minimises, one entry per branch.
inline std::vector<double> errors_**IDENT**(const Mat4 &T, double value)
{
    const std::vector<JointVec> qs = branches_at_**IDENT**(T, value);
    std::vector<double> out;
    out.reserve(qs.size());
    for (size_t i = 0; i < qs.size(); ++i) {
        if (!all_finite(qs[i])) {
            out.push_back(INF);
            continue;
        }
        const Mat4 Tq = fk_**IDENT**(qs[i]);
        if (!all_finite(Tq)) {
            out.push_back(INF);
            continue;
        }
        out.push_back(pose_error(Tq, T, ONEVAR_W_ROT).metric);
    }
    return out;
}


inline SearchConfig search_config_**IDENT**(int n_samples = N_SAMPLES)
{
    SearchConfig cfg;
    cfg.lo = SEARCH_LO;
    cfg.hi = SEARCH_HI;
    cfg.periodic = PERIODIC;
    cfg.n_samples = n_samples;
    cfg.max_samples = MAX_SAMPLES;
    cfg.accept_tol = ACCEPT_TOL;
    cfg.dedup_tol = DEDUP_TOL;
    cfg.refine_iters = 80;
    return cfg;
}


/////////////////////////////////////////////////////////////
//
//   THE ENTRY POINT.
//
/////////////////////////////////////////////////////////////

//  Goal pose T -> every joint vector that reaches it.  Each carries the
//  resolution it settled at in its n_samples field;  reaching MAX_SAMPLES
//  means the answer was still changing, and says so.
inline std::vector<OneVarSolution> solve_**IDENT**(const Mat4 &T,
                                                   int n_samples = N_SAMPLES)
{
    return onevar_solve(T, branches_at_**IDENT**, errors_**IDENT**,
                        search_config_**IDENT**(n_samples));
}


//  The raw scan at ONE resolution, for plotting and for understanding a pose
//  that comes back unreachable.  solve_**IDENT**() does not stop at one
//  resolution -- it climbs a ladder of them until the answer repeats.
inline void sweep_**IDENT**(const Mat4 &T, int n_samples,
                            std::vector<double> &values,
                            std::vector<std::vector<double> > &curves)
{
    ErrorCache cache;
    scan_curves(T, n_samples, cache, errors_**IDENT**,
                search_config_**IDENT**(n_samples), values, curves);
}
'''


_MAIN = '''

/////////////////////////////////////////////////////////////
//
//   TEST CODE:  pick a pose the arm can reach, and go back to it.
//
//   THE POSE IS RANDOM AND THE SEED IS PRINTED.  A fixed seed exercises one
//   pose forever, and what goes wrong here is pose-dependent -- a pair of
//   roots too close for the starting grid, a branch undefined over most of
//   the range.  Pass the printed seed back to get the same pose again:
//
//       g++ -std=c++11 -O2 -DIKBT_MAIN <this file> -o ik_onevar -lm
//       ./ik_onevar <seed>
//
/////////////////////////////////////////////////////////////

#ifdef IKBT_MAIN

int main(int argc, char **argv)
{
    //  std::rand with a printed seed, not <random>:  the point is only that
    //  the pose differs run to run and can be reproduced, and srand/rand is
    //  the same two lines in any C++ a user might build this with.
    const unsigned seed = (argc > 1) ? (unsigned) std::strtoul(argv[1], 0, 10)
                                     : (unsigned) 12345u;
    std::srand(seed);

    JointVec q_true(ONEVAR_NDOF);
    for (int i = 0; i < ONEVAR_NDOF; ++i)
        q_true[i] = -1.0 + 2.0 * (std::rand() / (double) RAND_MAX);
    const Mat4 T = fk_**IDENT**(q_true);

    std::printf("%s:  searching over %s in [%.3f, %.3f]   (seed %u)\\n",
                ROBOT, KNOWN_VARIABLE, SEARCH_LO, SEARCH_HI, seed);
    std::printf("  a reachable pose, from joints:\\n   ");
    for (int i = 0; i < ONEVAR_NDOF; ++i)
        std::printf(" %s=%.4f", ONEVAR_JOINT_NAMES[i], q_true[i]);
    std::printf("\\n");

    std::vector<OneVarSolution> sols = solve_**IDENT**(T);
    std::printf("  %d solution(s) found, at %d samples:\\n",
                (int) sols.size(),
                sols.empty() ? N_SAMPLES : sols[0].n_samples);

    //  THE ERROR IS RE-MEASURED HERE, from the joint vector actually
    //  returned.  The search's own number is what it accepted;  a self-test
    //  that prints that number is testing nothing.
    double worst = 0.0;
    for (size_t i = 0; i < sols.size(); ++i) {
        const double e = pose_error(fk_**IDENT**(sols[i].q), T,
                                    ONEVAR_W_ROT).metric;
        worst = std::max(worst, e);
        std::printf("    %s = %8.4f   error %.3e\\n", KNOWN_VARIABLE,
                    sols[i].known_value, e);
        std::printf("      ");
        for (int j = 0; j < ONEVAR_NDOF; ++j)
            std::printf(" %s=%.4f", ONEVAR_JOINT_NAMES[j], sols[i].q[j]);
        std::printf("\\n");
    }

    if (sols.empty()) {
        std::printf("    none -- try a larger MAX_SAMPLES\\n");
        return 0;
    }

    std::printf("  worst round-trip error %.3e   (accept tolerance %.1e)\\n",
                worst, ACCEPT_TOL);

    //  the pose was built from q_true, so q_true had better be among them
    double nearest = INF;
    for (size_t i = 0; i < sols.size(); ++i)
        nearest = std::min(nearest, max_abs_diff(sols[i].q, q_true));
    if (nearest >= 1e-6)
        std::printf("  The random starting pose is MISSING -- the closest is "
                    "%.1e away\\n", nearest);
    return 0;
}

#endif   // IKBT_MAIN
'''
