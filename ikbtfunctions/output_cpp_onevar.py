#!/usr/bin/python
#
#   output_cpp_onevar.py --  the one-variable method's C++ top level
#
#   The twin of output_onevar_python.write_onevar_top(), which writes three
#   python modules.  In C++ a one-variable solve is three SECTIONS of the one
#   file a robot gets -- CodeGen/Cpp/C-ArmCppCode/C-Arm.cpp -- and this emits
#   the last of them, the search:
#
#       the arm's FK and Jacobian          output_cpp_common.fk_body_cpp()
#       ikin_given(T, th_2), the closed    output_cpp.output_cpp_code()
#         form with th_2 an argument
#       solve(T), the 1-D search  <- THE   this module
#         entry point
#
#   ikin_given, never ikin:  the plain name would promise an unconditional
#   inverse kinematics, and these equations hold only where the assumed value
#   of th_2 is right.  The python twin makes the same distinction with its
#   file names, IK_conditional<Robot>.py rather than IK_equations<Robot>.py.
#
#   NOTHING ABOUT THE ROBOT IS APPROXIMATED.  The DH table, the FK and the
#   equations are the true arm's;  the only change was removing one entry from
#   the unknowns list.  That is what separates this from the hybrid path, and
#   why everything here carries the real robot's name.
#
#   THE SEARCH ITSELF IS IN Cpp_src/ikbt_search.h, not emitted here.  The
#   python twin has to emit ~470 lines per robot, because a generated python
#   module has only numpy to call;  in C++ Cpp_src/ is a library the generated
#   file can #include, so the algorithm is written once and this module
#   supplies only the two robot-specific callbacks it takes.
#
#   Copyright 2026 University of Washington
#
#   Developed by Blake Hannaford
#   BioRobotics Lab, University of Washington

import ikbtbasics.numeric_ik as nik
from ikbtfunctions.output_cpp import section_banner
from ikbtfunctions.output_cpp_common import name_table_cpp
from ikbtfunctions.output_onevar_python import search_domain


CORE_HEADERS = ['ikbt_types.h', 'ikbt_pose_error.h', 'ikbt_search.h']


def write_onevar_top_cpp(M, name, known, f, n_samples=128, max_samples=4096):
    '''Emit the 1-D search section of a robot's <Robot>.cpp.

       M            the TRUE arm's mechanism (nothing here is approximated)
       name         the robot the user asked about
       known        the variable the closed form assumes is known, e.g. 'th_2'
       f            the open stream of the robot file being assembled
       n_samples    where the resolution ladder starts
       max_samples  where it gives up looking for more

       Returns the name of the entry point emitted.'''

    #  BOTH ROUNDED UP TO A POWER OF TWO, as in the python twin.  The
    #  resolution ladder re-uses the coarse grid's cached errors only when
    #  lo + span*(2k)/(2n) is the same double as lo + span*k/n, and that holds
    #  exactly when n is a power of two.
    n_samples = 1 << max(1, int(n_samples) - 1).bit_length()
    max_samples = max(n_samples, 1 << max(1, int(max_samples) - 1).bit_length())

    ndof = M.ndof
    jnames = [str(s) for s in nik.joint_symbols(M, ndof)]
    #  `or 1.0`:  w_rot scales the acceptance tolerance, and a degenerate DH
    #  table with every length zero would make it 0 -- a test no root can
    #  pass, so a working solve would report nothing found.
    w_rot = float(nik.w_rot_for(M, ndof)) or 1.0
    lo, hi, periodic = search_domain(M, known, ndof)

    print(section_banner('the 1-D search over %s   -- THE ENTRY POINT'
                         % known), file=f)
    print('''//
//   READ THIS FIRST.
//
//   %s could not be solved in closed form outright.  The section above
//   holds a closed form for every OTHER joint, given a value of %s.
//   It is exact for THIS arm -- no DH parameter was changed -- but only
//   at the values of %s that actually reach the goal pose.
//
//   solve(T) is the entry point.  It searches for those values and
//   returns complete joint vectors that reach T.  A joint vector taken
//   straight from the conditional closed form, at a value you picked
//   yourself, will NOT put the robot at the pose you asked for.
//
''' % (name, known, known), file=f)

    print('const char* const ROBOT          = "%s";' % name, file=f)
    print('const char* const KNOWN_VARIABLE = "%s";' % known, file=f)
    print('//  Chain order -- what a q vector is, and what comes back.', file=f)
    print(name_table_cpp(jnames, 'ONEVAR_JOINT_NAMES'), file=f)
    print('const int ONEVAR_NDOF = %d;' % ndof, file=f)
    print('', file=f)
    print('//  The weight that puts position error and orientation error', file=f)
    print('//  into comparable units:  one characteristic arm length per', file=f)
    print('//  radian, in whatever length unit the DH table uses.', file=f)
    print('const double ONEVAR_W_ROT = %.17g;' % w_rot, file=f)
    print('', file=f)
    if periodic:
        print('//  %s is a revolute joint, so this is its WHOLE range and'
              % known, file=f)
        print('//  a search over it misses nothing.', file=f)
    else:
        print('//  %s is PRISMATIC, and a DH table does not record how far'
              % known, file=f)
        print('//  a prismatic joint travels.  These bounds are therefore a', file=f)
        print('//  guess -- the sum of the arm\'s own lengths, which is past', file=f)
        print('//  anything it could reach.  Narrow them if you know the', file=f)
        print('//  real travel.', file=f)
    print('const double SEARCH_LO = %.17g;' % float(lo), file=f)
    print('const double SEARCH_HI = %.17g;' % float(hi), file=f)
    print('const bool   PERIODIC  = %s;' % ('true' if periodic else 'false'),
          file=f)
    print('', file=f)
    print('//  Where the resolution ladder starts, and where it gives up.', file=f)
    print('//  Both are powers of two, so that doubling the grid lands on', file=f)
    print('//  the coarser grid\'s points exactly and re-uses their errors.', file=f)
    print('const int N_SAMPLES   = %d;' % int(n_samples), file=f)
    print('const int MAX_SAMPLES = %d;' % int(max_samples), file=f)
    print('', file=f)
    print('//  A refined minimum counts as a SOLUTION only if it gets this', file=f)
    print('//  close to the goal pose:  a millionth of an arm length.', file=f)
    print('const double ACCEPT_TOL = %.17g;' % (1e-6 * float(w_rot)), file=f)
    print('//  Two solutions closer than this in every joint are one.', file=f)
    print('const double DEDUP_TOL  = 1e-6;', file=f)
    print('', file=f)

    print(_BODY, file=f)

    return 'solve'


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
//  closed form reports the pose unreachable at that value, which is ordinary:
//  a branch is typically defined over part of the range, not all of it.
inline std::vector<JointVec> branches_at(const Mat4 &T, double value)
{
    //  Nothing is guarded here.  Where the closed form divides by zero or
    //  takes the root of a negative, IEEE arithmetic gives inf or NaN
    //  quietly;  that means the branch is undefined at this value, and
    //  errors() turns it into INF.
    return ikin_given(T, value);
}


//  Pose error of each branch at one assumed value;  INF where undefined.
//  This is the function the search minimises, one entry per branch.
inline std::vector<double> errors(const Mat4 &T, double value)
{
    const std::vector<JointVec> qs = branches_at(T, value);
    std::vector<double> out;
    out.reserve(qs.size());
    for (size_t i = 0; i < qs.size(); ++i) {
        if (!all_finite(qs[i])) {
            out.push_back(INF);
            continue;
        }
        const Mat4 Tq = fk(qs[i]);
        if (!all_finite(Tq)) {
            out.push_back(INF);
            continue;
        }
        out.push_back(pose_error(Tq, T, ONEVAR_W_ROT).metric);
    }
    return out;
}


inline SearchConfig search_config(int n_samples = N_SAMPLES)
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

//  Goal pose T -> every joint vector that reaches it.  Each carries, in its
//  n_samples field, the grid resolution the search settled at.  A solution
//  reporting MAX_SAMPLES means the answer was still changing when the ladder
//  ran out.
inline std::vector<OneVarSolution> solve(const Mat4 &T,
                                                   int n_samples = N_SAMPLES)
{
    return onevar_solve(T, branches_at, errors,
                        search_config(n_samples));
}


//  The raw scan at ONE resolution:  for plotting the error curves, and for
//  understanding a pose that comes back unreachable.  solve() does not stop
//  at one resolution -- it climbs a ladder of them until the answer stops
//  changing.
inline void sweep(const Mat4 &T, int n_samples,
                            std::vector<double> &values,
                            std::vector<std::vector<double> > &curves)
{
    ErrorCache cache;
    scan_curves(T, n_samples, cache, errors,
                search_config(n_samples), values, curves);
}
'''


MAIN = '''

/////////////////////////////////////////////////////////////
//
//   TEST CODE:  pick a pose the arm can reach, then search its way back.
//
//   THE POSE IS RANDOM AND THE SEED IS PRINTED.  What goes wrong here is
//   pose-dependent -- a pair of roots too close together for the starting
//   grid, a branch undefined over most of the range -- so a fixed seed would
//   exercise one pose forever.  Pass the printed seed back to get the same
//   pose again:
//
//       g++ -std=c++11 -O2 -DIKBT_MAIN <this file> -o ik_onevar -lm
//       ./ik_onevar <seed>
//
/////////////////////////////////////////////////////////////

#ifdef IKBT_MAIN

//  main() has to be at global scope, so the self-test reaches back into the
//  robot's namespace from outside it.
using namespace ikbt;
using namespace ikbt::**IDENT**;

int main(int argc, char **argv)
{
    //  std::rand rather than <random>:  all that is wanted is a pose that
    //  differs run to run and can be reproduced, and srand/rand is the same
    //  two lines in every version of C++.
    const unsigned seed = (argc > 1) ? (unsigned) std::strtoul(argv[1], 0, 10)
                                     : (unsigned) 12345u;
    std::srand(seed);

    JointVec q_true(ONEVAR_NDOF);
    for (int i = 0; i < ONEVAR_NDOF; ++i)
        q_true[i] = -1.0 + 2.0 * (std::rand() / (double) RAND_MAX);
    const Mat4 T = fk(q_true);

    std::printf("%s:  searching over %s in [%.3f, %.3f]   (seed %u)\\n",
                ROBOT, KNOWN_VARIABLE, SEARCH_LO, SEARCH_HI, seed);
    std::printf("  a reachable pose, from joints:\\n   ");
    for (int i = 0; i < ONEVAR_NDOF; ++i)
        std::printf(" %s=%.4f", ONEVAR_JOINT_NAMES[i], q_true[i]);
    std::printf("\\n");

    std::vector<OneVarSolution> sols = solve(T);
    std::printf("  %d solution(s) found, at %d samples:\\n",
                (int) sols.size(),
                sols.empty() ? N_SAMPLES : sols[0].n_samples);

    //  THE ERROR IS RE-MEASURED HERE, from the joint vector actually
    //  returned.  Reprinting the number the search accepted would test
    //  nothing.
    double worst = 0.0;
    for (size_t i = 0; i < sols.size(); ++i) {
        const double e = pose_error(fk(sols[i].q), T,
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
