#!/usr/bin/python
#
#   output_cpp_hybrid.py --  the hybrid method's C++ top level
#
#   The twin of output_hybrid_python.write_hybrid_top(), entry point for entry
#   point.  The python twin writes FOUR modules;  in C++ a hybrid solve is four
#   SECTIONS of the one file the true arm gets -- CodeGen/Cpp/XCppCode/X.cpp --
#   and this emits the last of them:
#
#       fk_X_d(q)                  derived arm, FK        fk_body_cpp()
#       ikin_X_d(T)                derived arm, closed    output_cpp_code()
#                                    form
#       fk(q), jacobian(q)         TRUE arm               fk_body_cpp()
#       ikin_approx / refine_seed  the two phases  <-     this module
#         / refine_all               THE ENTRY POINT
#
#   TWO ARMS IN ONE NAMESPACE, AND THE SUFFIX SAYS WHICH.  The true arm takes
#   the plain names, since the namespace is already its name;  the derived arm
#   keeps `_X_d`, so that `fk()` and `fk_X_d()` cannot be confused for one
#   another.
#
#   TWO ENTRY POINTS, BECAUSE A SEED IS A CHOICE.  Phase IIa refines one seed;
#   Phase II is a wrapper that refines them all.  The seeds are different
#   postures -- elbow up or down, wrist flipped -- and which one is wanted
#   depends on obstacles, joint limits and where the arm is now, none of which
#   is known here.  Damped least squares stays in the basin of the seed it is
#   given, so choosing the seed is how the caller chooses the posture.
#
#   Copyright 2026 University of Washington
#
#   Developed by Blake Hannaford
#   BioRobotics Lab, University of Washington

import ikbtbasics.numeric_ik as nik
from ikbtfunctions.output_cpp import section_banner
from ikbtfunctions.output_cpp_common import cpp_identifier, name_table_cpp


#  The shared numerics, in dependency order.  ikbt_dls.h is the C++ copy of
#  ikbtbasics/numeric_ik.solve_numeric();  the two are checked against each
#  other by `python3 -m scripts.cpp_closed_loop_check --dls`.
CORE_HEADERS = ['ikbt_types.h', 'ikbt_linalg.h', 'ikbt_pose_error.h',
                'ikbt_dls.h']


def write_hybrid_top_cpp(M_true, true_name, derived_name, edits_text,
                         cost_text, f):
    '''Emit the two-phase section of the true arm's <Robot>.cpp.

       M_true        the TRUE arm's mechanism, for w_rot and the joint names
       true_name     the robot the user asked about
       derived_name  the simplified arm that was actually solved in closed form
       edits_text    human-readable list of the DH changes ('d_5: 57 -> 0')
       cost_text     the task-space cost of those changes, or ''
       f             the open stream of the robot file being assembled

       Returns the name of the operational entry point emitted.'''

    dident = cpp_identifier(derived_name)
    ndof = M_true.ndof
    jnames = [str(s) for s in nik.joint_symbols(M_true, ndof)]

    #  w_rot HAS NO CORRECT DEFAULT, so it is computed per robot and baked
    #  into the generated file.  The metric is ||dp|| + w_rot*theta, which
    #  needs a length per radian;  nothing records a model's units, and they
    #  differ across the robot set -- Puma is in metres, KinovaLite in
    #  millimetres.  w_rot_for() avoids the question by using one
    #  characteristic arm length per radian, which behaves the same either
    #  way.
    w_rot = float(nik.w_rot_for(M_true, ndof))

    print(section_banner('the two phases   -- THE ENTRY POINT   (%s)'
                         % true_name), file=f)
    print('''//
//   READ THIS FIRST.
//
//   %s could not be solved in closed form.  What IS solved, in the
//   section above, is a SIMPLIFIED arm, %s, obtained by changing
//
//       %s%s
//
//   That closed form is exact for the simplified arm and only approximate
//   for %s.  Phase II corrects it by damped least squares against the true
//   arm's own forward kinematics.  A joint vector that has not been through
//   Phase II will not put the real robot at the pose you asked for.
//
''' % (true_name, derived_name, edits_text,
       ('   (%s)' % cost_text) if cost_text else '', true_name), file=f)

    print('const char* const TRUE_ROBOT      = "%s";' % true_name, file=f)
    print('const char* const APPROXIMATE_ARM = "%s";' % derived_name, file=f)
    print('const char* const DH_CHANGES      = "%s";'
          % edits_text.replace('"', '\\"'), file=f)
    print(name_table_cpp(jnames, 'HYBRID_JOINT_NAMES'), file=f)
    #  NO `const int NDOF` HERE:  the true arm's FK section already declares
    #  it, in this same namespace and from the same M_true.ndof.
    print('', file=f)
    print('//  The weight that puts position error and orientation error', file=f)
    print('//  into comparable units:  one characteristic arm length per', file=f)
    print('//  radian, in whatever length unit the DH table uses.', file=f)
    print('const double W_ROT = %.17g;' % w_rot, file=f)
    print('', file=f)
    print('//  A Phase I branch is kept if the approximate arm reproduces', file=f)
    print('//  the goal pose at least this closely:  loose enough to', file=f)
    print('//  survive rounding, tight enough to reject a false branch.', file=f)
    print('const double APPROX_TOL = 1e-6;', file=f)
    print('', file=f)

    print(_PHASES.replace('**DIDENT**', dident), file=f)

    return 'refine_seed'


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
//  enumerates every combination of the unknowns' solution branches without
//  checking each one against the original equations, so what comes back is a
//  list of CANDIDATES.  Running each through the approximate arm's own FK is
//  cheap and weeds them out;  pass false to see the unfiltered list.
//
//  THESE ARE SEEDS, NOT ANSWERS.  Pass the one you want to refine_seed().
inline SolutionList ikin_approx(const Mat4 &T,
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
//   This is the call to use in operation:  pick the posture you want --
//   normally the branch nearest where the arm is now -- and refine that seed.
//
/////////////////////////////////////////////////////////////

//  Correct one seed against the TRUE arm's FK, by damped least squares.
//
//      T       the goal pose, in the DH table's length units
//      q_seed  NDOF joint values to start from, in HYBRID_JOINT_NAMES order.
//              Normally one entry of ikin_approx(T), but any joint vector
//              will do -- the arm's current pose, for instance.
//
//  The python twin returns (q, error, iterations);  this returns a whole
//  SolveResult, which carries those three plus `converged` and `reason`.
//
//  CHECK `converged`, which means metric <= tol.  A large error is a real
//  result:  the true arm may not reach T from this seed, or at all.
//
//  Refinement stays in the basin of the seed it is given, which is why this
//  takes a seed rather than choosing one.
inline SolveResult refine_seed(const Mat4 &T, const JointVec &q_seed,
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
    return solve_numeric(fk, jacobian,
                         q_seed, T, W_ROT, tol, max_iter);
}


/////////////////////////////////////////////////////////////
//
//   PHASE II -- refine EVERY seed, to see which postures survive
//
//   A wrapper over Phase IIa, for learning which branches the true arm can
//   actually reach.  Run it once, then call refine_seed() on the posture you
//   want from then on.
//
/////////////////////////////////////////////////////////////

struct RefineRecord {
    int index;
    JointVec q_seed;
    SolveResult result;
};

inline std::vector<RefineRecord> refine_all(const Mat4 &T,
                                                      double tol = 1e-9,
                                                      int max_iter = 100)
{
    const SolutionList seeds = ikin_approx(T);
    std::vector<RefineRecord> out;
    for (size_t i = 0; i < seeds.size(); ++i) {
        RefineRecord rec;
        rec.index = (int) i;
        rec.q_seed = seeds[i];
        rec.result = refine_seed(T, seeds[i], tol, max_iter);
        out.push_back(rec);
    }
    return out;
}
'''


MAIN = '''

/////////////////////////////////////////////////////////////
//
//   TEST CODE.  Build it with
//       g++ -std=c++11 -O2 -DIKBT_MAIN <this file> -o ik_hybrid -lm
//
/////////////////////////////////////////////////////////////

#ifdef IKBT_MAIN

//  main() has to be at global scope, so the self-test reaches back into the
//  robot's namespace from outside it.
using namespace ikbt;
using namespace ikbt::**IDENT**;

int main(void)
{
    //  A round trip:  pick joint values, build the pose they produce on the
    //  TRUE arm, and see whether the two phases recover them.
    const double q_all[6] = {0.4, -0.6, 0.7, 0.9, -0.5, 0.3};
    JointVec q_demo(q_all, q_all + NDOF);
    const Mat4 T_goal = fk(q_demo);

    std::printf("\\n  %s -- hybrid IK via %s\\n", TRUE_ROBOT, APPROXIMATE_ARM);
    std::printf("  approximated by: %s\\n", DH_CHANGES);
    std::printf("  goal pose T_goal is FK(");
    for (int i = 0; i < NDOF; ++i)
        std::printf("%s%.2f", i ? " " : "", q_demo[i]);
    std::printf(") on the TRUE arm\\n\\n");

    SolutionList seeds = ikin_approx(T_goal);
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
    std::vector<RefineRecord> all = refine_all(T_goal);
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
        SolveResult r = refine_seed(T_goal, seeds[0]);
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
