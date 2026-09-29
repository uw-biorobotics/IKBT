#!/usr/bin/python
#
#   output_cpp_hybrid.py --  the hybrid method's C++ top level
#
#   The twin of output_hybrid_python.write_hybrid_top(), entry point for entry
#   point.  What a hybrid solve delivers in C++, for a true arm X simplified
#   to a derived arm X_d:
#
#       CodeGen/Cpp/IK_hybrid_X.cpp          the two phases   <- the entry point
#       CodeGen/Cpp/FK_numericX.h            true arm, FK AND Jacobian
#       CodeGen/Cpp/IK_equationsX_d.cpp      derived arm, closed form
#       CodeGen/Cpp/FK_numericX_d.h          derived arm, FK
#
#   The same four files the python path writes, with the same names for the
#   same reasons:  the module a user calls carries the name they asked about,
#   the pieces carry the name of the arm they actually describe, so opening
#   any one file tells you which robot it is.
#
#   TWO ENTRY POINTS, BECAUSE A SEED IS A CHOICE.  Phase IIa takes a seed, not
#   an index;  Phase II is its wrapper over a seed list.  The branches are
#   different postures -- elbow up or down, wrist flipped -- and which is
#   wanted depends on obstacles, joint limits and where the arm is now.  None
#   of that is known here, and damped least squares should stay in the basin
#   of the seed it is given, so the choice of seed IS the choice of posture.
#
#   Copyright 2026 University of Washington
#
#   Developed by Blake Hannaford
#   BioRobotics Lab, University of Washington

import os

import ikbtbasics.numeric_ik as nik
from ikbtfunctions.output_cpp_common import (DIR_NAME, cpp_identifier,
                                             file_header, inline_src,
                                             name_table_cpp,
                                             write_fk_module_cpp)


#  The shared numerics, in dependency order.  ikbt_dls.h is the C++ copy of
#  ikbtbasics/numeric_ik.solve_numeric(), pinned against it by
#  `python3 -m scripts.cpp_closed_loop_check --dls`.
CORE_HEADERS = ['ikbt_types.h', 'ikbt_linalg.h', 'ikbt_pose_error.h',
                'ikbt_dls.h']


def write_hybrid_top_cpp(M_true, true_name, derived_name, edits_text,
                         cost_text, dirname=DIR_NAME):
    '''Write IK_hybrid_<true_name>.cpp -- the two-phase top level.

       M_true        the TRUE arm's mechanism, for w_rot and the joint names
       true_name     the robot the user asked about
       derived_name  the simplified arm that was actually solved in closed form
       edits_text    human-readable list of the DH changes ('d_5: 57 -> 0')
       cost_text     the task-space cost of those changes, or ''

       Returns the path written.'''

    ident = cpp_identifier(true_name)
    dident = cpp_identifier(derived_name)
    ndof = M_true.ndof
    jnames = [str(s) for s in nik.joint_symbols(M_true, ndof)]

    #  w_rot HAS NO CORRECT DEFAULT, so it is baked in per robot rather than
    #  defaulted in the numerics.  BH's metric is ||dp|| + (1 m)*theta, but
    #  nothing records a model's units and they are not consistent across the
    #  robot set -- Puma is in metres, KinovaLite in millimetres.  w_rot_for()
    #  sidesteps that with a unit-free choice: one characteristic arm length
    #  per radian, which behaves the same on both.
    w_rot = float(nik.w_rot_for(M_true, ndof))

    filename = 'IK_hybrid_%s.cpp' % true_name
    path = os.path.join(dirname, filename)

    with open(path, 'w') as f:
        print(file_header('HYBRID inverse kinematics for %s' % true_name,
                          true_name, filename), file=f)
        print(inline_src(CORE_HEADERS), file=f)
        print('', file=f)
        print('#include <cstdio>', file=f)
        print('', file=f)
        print('//  Siblings, in the same directory -- which is what', file=f)
        print('//  #include "..." searches first.  The python twin does the', file=f)
        print('//  same thing with sys.path.insert(0, dirname(__file__)).', file=f)
        print('#include "FK_numeric%s.h"    // approximate arm, FK' % derived_name,
              file=f)
        print('#include "FK_numeric%s.h"    // TRUE arm, FK and Jacobian'
              % true_name, file=f)
        print('', file=f)
        print('//  The approximate arm\'s closed form.  Its own self-test is', file=f)
        print('//  suppressed while it is included:  this file has one, and two', file=f)
        print('//  main()s do not link.', file=f)
        print('#ifdef IKBT_MAIN', file=f)
        print('#define IKBT_HYBRID_MAIN_WAS_SET', file=f)
        print('#undef IKBT_MAIN', file=f)
        print('#endif', file=f)
        print('#include "IK_equations%s.cpp"' % derived_name, file=f)
        print('#ifdef IKBT_HYBRID_MAIN_WAS_SET', file=f)
        print('#define IKBT_MAIN', file=f)
        print('#endif', file=f)
        print('', file=f)
        print('using namespace ikbt;', file=f)
        print('', file=f)

        print('''/////////////////////////////////////////////////////////////
//
//   READ THIS FIRST.
//
//   %s could not be solved in closed form.  What is solved here
//   is a SIMPLIFIED arm, %s, obtained by changing
//
//       %s%s
//
//   The closed form is EXACT for that simplified arm and only APPROXIMATE for
//   %s.  Phase II corrects it, by damped least squares against the
//   true arm's own forward kinematics.  A joint vector that has not been
//   through Phase II does not put the real robot at the pose you asked for.
//
/////////////////////////////////////////////////////////////
''' % (true_name, derived_name, edits_text,
       ('   (%s)' % cost_text) if cost_text else '', true_name), file=f)

        print('const char* const TRUE_ROBOT      = "%s";' % true_name, file=f)
        print('const char* const APPROXIMATE_ARM = "%s";' % derived_name, file=f)
        print('const char* const DH_CHANGES      = "%s";'
              % edits_text.replace('"', '\\"'), file=f)
        print(name_table_cpp(jnames, 'HYBRID_JOINT_NAMES'), file=f)
        print('const int NDOF = %d;' % ndof, file=f)
        print('', file=f)
        print('//  One characteristic arm length per radian:  the weight that', file=f)
        print('//  puts position error and orientation error in comparable', file=f)
        print('//  units.  Baked in because nothing records a model\'s units', file=f)
        print('//  and they are not consistent across the robot set.', file=f)
        print('const double W_ROT = %.17g;' % w_rot, file=f)
        print('', file=f)
        print('//  A Phase I branch is kept when its own arm reproduces the', file=f)
        print('//  goal pose this closely.  Loose enough to survive float', file=f)
        print('//  noise, tight enough that a spurious branch cannot pass.', file=f)
        print('const double APPROX_TOL = 1e-6;', file=f)
        print('', file=f)

        print(_PHASES
              .replace('**IDENT**', ident)
              .replace('**DIDENT**', dident), file=f)

        print(_MAIN.replace('**IDENT**', ident), file=f)

    return path


_PHASES = '''
/////////////////////////////////////////////////////////////
//
//   PHASE I -- every closed-form solution of the APPROXIMATE arm
//
/////////////////////////////////////////////////////////////

//  Goal pose T -> joint vectors for the APPROXIMATE arm, NDOF each, in
//  HYBRID_JOINT_NAMES order.  Empty when the closed form reports the pose
//  unreachable.
//
//  filter_spurious drops branches that do not actually reach T.  IKBT
//  enumerates combinations of each unknown's solution branches and does not
//  discard the ones that fail the original equations, so a returned branch is
//  a CANDIDATE.  Checking each against the approximate arm's own FK is cheap
//  and removes them;  pass false to see the raw list.
//
//  THESE ARE SEEDS, NOT ANSWERS.  Feed the one you want to
//  refine_seed_**IDENT**().
inline SolutionList ikin_**IDENT**_approx(const Mat4 &T,
                                          bool filter_spurious = true)
{
    SolutionList raw = ikin_**DIDENT**(T);
    if (!filter_spurious)
        return raw;

    SolutionList out;
    for (size_t i = 0; i < raw.size(); ++i) {
        const Mat4 Tq = fk_**DIDENT**(raw[i]);
        if (!all_finite(Tq))
            continue;
        double worst = 0.0;
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c)
                worst = std::max(worst, std::fabs(Tq[r][c] - T[r][c]));
        if (worst < APPROX_TOL)
            out.push_back(raw[i]);
    }
    return out;
}


/////////////////////////////////////////////////////////////
//
//   PHASE IIa -- refine ONE seed against the TRUE arm
//
//   This is the operational call.  You pick the posture you want -- normally
//   the branch nearest where the arm is now -- and refine that one seed.
//
/////////////////////////////////////////////////////////////

//  Correct one seed against the TRUE arm's FK.  Damped least squares.
//
//      T       the goal pose, in the DH table's length units
//      q_seed  NDOF joint values to start from, in HYBRID_JOINT_NAMES order --
//              normally one entry of ikin_**IDENT**_approx(T), but any joint
//              vector will do (the arm's current pose, say)
//
//  The python twin returns (q, error, iterations);  this returns the whole
//  SolveResult, which carries those three plus `converged` and `reason`.
//
//  CONVERGED MEANS metric <= tol;  TEST IT, because a large error is a real
//  answer.  The true arm may not reach T from this seed, or at all.
//
//  REFINEMENT STAYS IN THE BASIN OF ITS SEED, which is the whole reason this
//  takes a seed rather than choosing one.
inline SolveResult refine_seed_**IDENT**(const Mat4 &T, const JointVec &q_seed,
                                         double tol = 1e-9, int max_iter = 100)
{
    if ((int) q_seed.size() != NDOF) {
        SolveResult bad;
        bad.q = q_seed;
        bad.metric = INF;
        bad.iterations = 0;
        bad.converged = false;
        bad.reason = "seed has the wrong number of joints";
        return bad;
    }
    return solve_numeric(fk_**IDENT**, jacobian_**IDENT**,
                         q_seed, T, W_ROT, tol, max_iter);
}


/////////////////////////////////////////////////////////////
//
//   PHASE II -- refine EVERY seed, so you can see which postures survive
//
//   A wrapper over Phase IIa.  Diagnostic:  run it once to learn which
//   branches the true arm can actually reach, then call
//   refine_seed_**IDENT**() on the one you want from then on.
//
/////////////////////////////////////////////////////////////

struct RefineRecord {
    int index;
    JointVec q_seed;
    SolveResult result;
};

inline std::vector<RefineRecord> refine_all_**IDENT**(const Mat4 &T,
                                                      double tol = 1e-9,
                                                      int max_iter = 100)
{
    const SolutionList seeds = ikin_**IDENT**_approx(T);
    std::vector<RefineRecord> out;
    for (size_t i = 0; i < seeds.size(); ++i) {
        RefineRecord rec;
        rec.index = (int) i;
        rec.q_seed = seeds[i];
        rec.result = refine_seed_**IDENT**(T, seeds[i], tol, max_iter);
        out.push_back(rec);
    }
    return out;
}
'''


_MAIN = '''

/////////////////////////////////////////////////////////////
//
//   TEST CODE.  Build it with
//       g++ -std=c++11 -O2 -DIKBT_MAIN <this file> -o ik_hybrid -lm
//
/////////////////////////////////////////////////////////////

#ifdef IKBT_MAIN

int main(void)
{
    //  Round trip:  pick joints, build the pose they produce on the TRUE arm,
    //  and see whether the two phases recover them.
    const double q_all[6] = {0.4, -0.6, 0.7, 0.9, -0.5, 0.3};
    JointVec q_demo(q_all, q_all + NDOF);
    const Mat4 T_goal = fk_**IDENT**(q_demo);

    std::printf("\\n  %s -- hybrid IK via %s\\n", TRUE_ROBOT, APPROXIMATE_ARM);
    std::printf("  approximated by: %s\\n", DH_CHANGES);
    std::printf("  goal pose T_goal is FK(");
    for (int i = 0; i < NDOF; ++i)
        std::printf("%s%.2f", i ? " " : "", q_demo[i]);
    std::printf(") on the TRUE arm\\n\\n");

    SolutionList seeds = ikin_**IDENT**_approx(T_goal);
    std::printf("  PHASE I: %d usable branch(es) from the approximate arm\\n",
                (int) seeds.size());
    std::printf("           joints: ");
    for (int j = 0; j < NDOF; ++j)
        std::printf("%s%s", j ? "  " : "", HYBRID_JOINT_NAMES[j]);
    std::printf("\\n");
    for (size_t i = 0; i < seeds.size(); ++i) {
        std::printf("     [%d] ", (int) i);
        for (int j = 0; j < NDOF; ++j)
            std::printf(" %8.4f", seeds[i][j]);
        std::printf("\\n");
    }

    std::printf("\\n  PHASE II: every seed scored against the true arm\\n");
    std::vector<RefineRecord> all = refine_all_**IDENT**(T_goal);
    for (size_t i = 0; i < all.size(); ++i) {
        std::printf("     [%d] ", all[i].index);
        for (int j = 0; j < NDOF; ++j)
            std::printf(" %8.4f", all[i].result.q[j]);
        std::printf("   error %.2e  %2d iters  %s\\n",
                    all[i].result.metric, all[i].result.iterations,
                    all[i].result.converged ? "converged" : "NOT converged");
    }

    if (!seeds.empty()) {
        std::printf("\\n  PHASE IIa: refining seed [0] on its own -- this is the\\n");
        std::printf("             call you use in operation, once you know\\n");
        std::printf("             which posture you want\\n");
        SolveResult r = refine_seed_**IDENT**(T_goal, seeds[0]);
        std::printf("     q     ");
        for (int j = 0; j < NDOF; ++j)
            std::printf(" %8.4f", r.q[j]);
        std::printf("\\n     error  %.3e   iterations %d\\n\\n",
                    r.metric, r.iterations);
    }
    return 0;
}

#endif   // IKBT_MAIN
'''
