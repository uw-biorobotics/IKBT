#!/usr/bin/python
#
#   output_cpp.py --  generate C++ code for the IK solution
#
#   THE TWIN OF output_python.output_python_code(), derived from it line by
#   line.  Same walk of Robot.FinalEqnMatrix, same first-seen dedup on the LHS,
#   same joint/aux column split, same return contract, same two file names:
#
#       known=None    CodeGen/Cpp/IK_equations<Robot>.cpp,  ikin_<Robot>(T)
#                     -- an unconditional closed form
#       known='th_2'  CodeGen/Cpp/IK_conditional<Robot>.cpp,
#                     ikin_<Robot>_given(T, th_2) -- the ONE-VARIABLE branch's
#                     closed form, valid only where th_2 is right
#
#   This file was rewritten from nothing in Sept 2026.  The version it replaced
#   predated solListMatrix, pvals-in-generated-code and sp.pycode, and had
#   never been compiled by any test:  1 of the 26 .cpp files it had produced
#   compiled as shipped, and 11 of 26 still failed once parameters were
#   supplied by hand.  IKdocs/DEV_NOTES.md records the six divergences and the
#   measurement.  Keeping the old file would have meant repairing six
#   independent departures from a python generator that is already right;
#   deriving a new one from that generator is less work and asserts more.
#
#   Copyright 2017-2026 University of Washington
#
#   Developed by Dianmu Zhang and Blake Hannaford
#   BioRobotics Lab, University of Washington

import sympy as sp

import ikbtbasics.numeric_ik as nik
from ikbtfunctions.output_cpp_common import (DIR_NAME, cpp_identifier,
                                             expr_cpp, file_header,
                                             name_table_cpp, param_decls,
                                             pose_unpack_cpp, read_src)


#  Console chatter from the generator.  Off by default, for the same reason
#  output_python.py's copy is:  it is a line per version per variable.
VERBOSE = False


def _say(*args):
    if VERBOSE:
        print(*args)


def _versions(Robot, node):
    '''The DISTINCT solution equations for one variable, in first-seen order.

       Straight from output_python.py.  A variable solved early shares its
       versions between matrix rows (Puma's th_1: 2 versions, 8 rows), so
       walking the rows emits the same statement several times.'''

    eqnlist = []
    seen = set()
    col = node.unknown.solveorder - 1
    for row in range(Robot.nversions):
        e = Robot.FinalEqnMatrix[row][col]
        if str(e.LHS) in seen:
            continue
        seen.add(str(e.LHS))
        eqnlist.append(e)
    return eqnlist


#  NO DOMAIN-GUARD HELPER HERE, and none is needed.
#
#  std::acos ALREADY returns NaN when its argument leaves [-1, 1] -- quietly,
#  with no exception and no undefined behaviour -- which is exactly the
#  contract dc_rewrite() has to install by hand on the python side, where
#  math.acos raises.  So the two languages carry the SAME contract in two
#  spellings, and scripts/cpp_expr_check asserts it rather than assuming it.
#
#  The hoisted `if (fabs(argument) > 1)` this replaced is gone for the reason
#  BH gave on 2026-09-29: a check at the point of use cannot be written down
#  wrong, and a hoisted one has to re-derive the argument, which is a separate
#  problem that can be got wrong -- and was.


def output_cpp_code(Robot, solution_groups, known=None):
    '''Write the generated C++ IK for `Robot`.

       Robot            solved, with create_solution_set() already run
       solution_groups  R.solutionSet, used only as a fallback for the rows
       known            the one-variable branch's assumed variable, or None

       Returns the path written.'''

    print('\n\n\n                       Starting IK C++ Output work \n\n\n')

    orig_name = Robot.name.replace('test: ', '')
    ident = cpp_identifier(orig_name)
    funcname = 'ikin_' + ident + ('_given' if known else '')

    #
    #   THE RETURN CONTRACT.  Identical to python's, and it matters more here:
    #   C has no way to hand back a name with a value, so the ORDER IS THE
    #   CONTRACT.  Joints only, in DH CHAIN order.  The sum-of-angle variables
    #   are computed -- later solutions depend on them -- but not returned.
    #
    #   The sizes used to be hardcoded [64][6], and a 6-DOF arm with one
    #   sum-of-angles variable wrote SEVEN columns into a row of six.
    #
    jnames = [str(s) for s in nik.joint_symbols(Robot.Mech)]
    order = [nd.unknown.name for nd in Robot.solution_nodes]
    #  The assumed-known joint was never solved, so it is not in `order` -- but
    #  it IS known, by assumption, and leaving it out would return a joint
    #  vector with a hole in it.  Its value is the argument.
    joint_cols = [j for j in jnames if j in order or j == known]
    unsolved = [j for j in jnames if j not in order and j != known]
    aux_cols = [nm for nm in order if nm not in jnames]

    rows = getattr(Robot, 'solListMatrix', None)
    if not rows:
        rows = [list(g) for g in sorted(solution_groups)]

    #  max(1, ...):  the legacy entry point declares
    #  `double solution_list[IK_NBRANCHES][IK_NJOINTS]`, and a zero-length
    #  array is ill-formed in C++.  A solve with no joints or no branches
    #  should never reach here -- report_gen only ticks on a complete solve --
    #  but an ill-formed declaration would fail the whole file rather than
    #  say so.
    n_joints = max(1, len(joint_cols))
    n_branches = max(1, len(rows))

    #
    #   EVERY NAME THE BODY ASSIGNS, declared up front.
    #
    #   From FinalEqnMatrix, not from solution_groups:  the old generator took
    #   them from the groups and missed some, so DZhang, UR5 and JennyGuoSp24
    #   referred to undeclared variables.
    #
    #   INITIALISED TO NaN, which is also the value an out-of-domain
    #   arccosine produces, so the two cases arrive at the finiteness test at
    #   the bottom by the same route.  An uninitialised double would be
    #   undefined behaviour and would read as plausible garbage instead.
    #
    sol_vars = []
    for node in Robot.solution_nodes:
        for e in _versions(Robot, node):
            nm = str(e.LHS)
            if nm not in sol_vars:
                sol_vars.append(nm)

    par_text, missing = param_decls(Robot.Mech,
                                    getattr(Robot.Mech, 'params', None)
                                    or Robot.params)

    filename = ('IK_conditional' if known else 'IK_equations') + orig_name + '.cpp'
    path = DIR_NAME + '/' + filename
    f = open(path, 'w')

    what = ('CONDITIONAL inverse kinematics for %s (%s assumed known)'
            % (orig_name, known)) if known else \
           ('C++ inverse kinematic equations for %s' % orig_name)
    print(file_header(what, orig_name, filename), file=f)

    #  Cpp_src/ inlined, so this translation unit stands alone -- the same
    #  bargain output_latex.py strikes with LaTex_src/IK_preamble.tex.
    print(read_src('ikbt_types.h'), file=f)
    print('', file=f)
    print('#include <cstdio>', file=f)
    print('', file=f)
    print('using namespace ikbt;', file=f)
    print('', file=f)

    if known:
        print('//  CONDITIONAL.  %s is an INPUT, not an output:  these' % known,
              file=f)
        print('//  equations hold only where its value is right.', file=f)
        print('//  IK_onevar%s.cpp searches for the values that are, and is'
              % orig_name, file=f)
        print('//  what you should normally call.', file=f)
        #  COND_KNOWN_VARIABLE, not KNOWN_VARIABLE.  The python twin can call
        #  it the plain name because modules have namespaces;  here
        #  IK_onevar<Robot>.cpp INCLUDES this file, and the entry point a user
        #  reads should own the unqualified name.  The included dependency
        #  yields.
        print('const char* const COND_KNOWN_VARIABLE = "%s";' % known, file=f)
        print('', file=f)

    print('//\n//      Robot Parameters\n//', file=f)
    print(par_text, file=f)
    if missing:
        print('//  %d parameter(s) above have no value in this robot\'s pvals.'
              % len(missing), file=f)
        print('//  XXXXX is a DELIBERATE COMPILE STOP -- g++ names the line so a', file=f)
        print('//  missing link length cannot be silently defaulted.', file=f)
    print('', file=f)

    if known:
        print('//  EVERY branch is returned, in a FIXED position, whether or', file=f)
        print('//  not it exists at this pose -- one that does not comes back', file=f)
        print('//  with NaN in it.  IK_onevar%s.cpp indexes branches by'
              % orig_name, file=f)
        print('//  position, so row i must be the same branch at every value.', file=f)
    else:
        print('//  Only the branches that EXIST at the goal pose are returned,', file=f)
        print('//  so the count varies with the pose.  Empty means none do.', file=f)
    print('//  Joint values returned by %s(), in this order.' % funcname, file=f)
    print('//  C has no way to hand back a name with a value, so this order IS', file=f)
    print('//  the contract.', file=f)
    print(name_table_cpp(joint_cols, 'JOINT_NAMES'), file=f)
    print('//  Sum-of-angle intermediates:  computed, but NOT returned.', file=f)
    print(name_table_cpp(aux_cols, 'AUX_NAMES'), file=f)
    if unsolved:
        print('//  WARNING:  these joints were NOT solved, so they are absent', file=f)
        print('//            from every returned branch:  %s'
              % ', '.join(unsolved), file=f)
    print('const int IK_NJOINTS   = %d;' % n_joints, file=f)
    print('const int IK_NBRANCHES = %d;' % n_branches, file=f)
    print('', file=f)

    #
    #   The solver itself.
    #
    arglist = 'const Mat4 &T' + (', double %s' % known if known else '')
    print('//', file=f)
    print('//   Auto generated code to solve the unknowns.', file=f)
    print('//       T   4x4 numerical target for T06, row major', file=f)
    print('//   Returns one JointVec per solution branch, or an EMPTY list if', file=f)
    print('//   the pose is not reachable  (python returns False for that).', file=f)
    print('//', file=f)
    print('SolutionList %s(%s)' % (funcname, arglist), file=f)
    print('{', file=f)
    print('    //  define the input vars', file=f)
    print(pose_unpack_cpp(), file=f)
    print('', file=f)
    print('    //  every solved variable, NaN until its branch assigns it', file=f)
    for v in sol_vars:
        print('    double %s = std::numeric_limits<double>::quiet_NaN();' % v,
              file=f)
    print('', file=f)
    print('''    /////////////////////////////////////////////////////////////
    //
    //  REACHABILITY is decided at the END, from the answer itself.
    //
    //  std::acos and std::asin return NaN, quietly, when their argument
    //  leaves [-1, 1] -- which means the posture that branch describes does
    //  not exist at this pose.  That is DATA, not a fault:  the solver
    //  enumerates combinations of each unknown's branches without discarding
    //  the ones the arm cannot adopt.  The NaN propagates through the
    //  arithmetic that follows and is recognised at the bottom of this
    //  function.
    //
    /////////////////////////////////////////////////////////////
''', file=f)

    for node in Robot.solution_nodes:
        print('', file=f)
        print('    //  Variable: %s   (solvemethod: %s)'
              % (str(node.symbol), node.solvemethod), file=f)
        for solEqnVer in _versions(Robot, node):
            lhs = str(solEqnVer.LHS)
            rhs = solEqnVer.RHS
            _say('Cpp Output: ', lhs, ' = ', rhs)
            try:
                rhs_src = expr_cpp(rhs)
            except ValueError as e:
                print('    //  NOT EMITTED: %s' % e, file=f)
                print('output_cpp: could not emit %s -- %s' % (lhs, e))
                continue

            print('    %s = %s;' % (lhs, rhs_src), file=f)

    #
    #   Package the solutions.
    #
    print('''
    /////////////////////////////////////////////////////////////
    //
    //  Package the solutions:  one row per solution branch,
    //  columns in JOINT_NAMES order.
    //
    /////////////////////////////////////////////////////////////
''', file=f)
    print('    SolutionList solution_list;', file=f)
    print('    solution_list.reserve(%d);' % n_branches, file=f)
    for row in rows:
        #  `known` has no column in the solution matrix -- nothing solved it --
        #  so its cell is the argument's own name.
        vals = [known if j == known else row[order.index(j)] for j in joint_cols]
        print('    {', file=f)
        print('        JointVec s;', file=f)
        print('        s.reserve(%d);' % n_joints, file=f)
        for j, v in zip(joint_cols, vals):
            print('        s.push_back(%s);   // %s' % (v, j), file=f)
        print('        solution_list.push_back(s);', file=f)
        print('    }', file=f)
    print('', file=f)
    #
    #   THE TWO ENTRY POINTS DIFFER HERE, exactly as they do in python -- see
    #   output_python.output_python_code() for the reasoning, which is the
    #   same reasoning and must stay the same.
    #
    if not known:
        print('    //  Keep the postures that EXIST.  A row holding a', file=f)
        print('    //  non-finite value is one that does not exist at this', file=f)
        print('    //  pose -- an arccosine out of range, or a negative', file=f)
        print('    //  discriminant -- and the count varies with the pose.', file=f)
        print('    //  An EMPTY list means none of them do, which is what', file=f)
        print('    //  python says with False.', file=f)
        print('    SolutionList reachable;', file=f)
        print('    for (size_t i = 0; i < solution_list.size(); ++i)', file=f)
        print('        if (all_finite(solution_list[i]))', file=f)
        print('            reachable.push_back(solution_list[i]);', file=f)
        print('', file=f)
        print('    return reachable;', file=f)
    else:
        print('    //  EVERY row, in a FIXED position, NaN and all.  The 1-D', file=f)
        print('    //  search indexes branches by position and needs row i to', file=f)
        print('    //  be the same branch at every value of %s;  it reads a'
              % known, file=f)
        print('    //  non-finite row as "this branch is undefined here".', file=f)
        print('    return solution_list;', file=f)
    print('}', file=f)

    #
    #   The legacy C-array entry point.
    #
    print('''

/////////////////////////////////////////////////////////////
//
//  The original fixed-array interface, kept so that callers holding a
//  double[4][4] do not have to be rewritten.  Fills solution_list in place
//  and returns 1 for a solved pose, 0 for none.
//
/////////////////////////////////////////////////////////////
''', file=f)
    legacy_args = 'double T[4][4], double solution_list[IK_NBRANCHES][IK_NJOINTS]'
    if known:
        legacy_args += ', double %s' % known
    print('int ikin(%s)' % legacy_args, file=f)
    print('{', file=f)
    print('    SolutionList sols = %s(from_array(T)%s);'
          % (funcname, (', %s' % known) if known else ''), file=f)
    print('    if (sols.empty())', file=f)
    print('        return 0;', file=f)
    print('    for (size_t i = 0; i < sols.size() && i < (size_t) IK_NBRANCHES; ++i)',
          file=f)
    print('        for (size_t j = 0; j < sols[i].size() && j < (size_t) IK_NJOINTS; ++j)',
          file=f)
    print('            solution_list[i][j] = sols[i][j];', file=f)
    print('    return 1;', file=f)
    print('}', file=f)

    #
    #   The self-test.  The twin of python's `if __name__ == "__main__":`,
    #   and behind an #ifdef for the same reason it is behind an if:  so that
    #   this file can be linked into a program, or alongside another robot,
    #   without two main()s.  The old generator emitted an unconditional
    #   main() -- and printed `std::cout << sol_list`, which prints a pointer.
    #
    print(_MAIN_BLOCK
          .replace('**FUNC**', funcname)
          .replace('**ROBOT**', orig_name)
          .replace('**EXTRA_ARG**', ', 0.3' if known else '')
          .replace('**KNOWN_NOTE**',
                   ('\n    std::printf("    (%s assumed = 0.3)\\n");' % known)
                   if known else ''),
          file=f)

    f.close()
    print('\n\n\n                       End of C++ Output work \n\n\n')
    return path


#  Kept out of the function body so the emitted text reads as C++.
_MAIN_BLOCK = '''

/////////////////////////////////////////////////////////////
//
//   TEST CODE.  Build it with
//       g++ -std=c++11 -O2 -DIKBT_MAIN <this file> -o ik_test -lm
//
/////////////////////////////////////////////////////////////

#ifdef IKBT_MAIN

static Mat4 RotX4(double t)
{
    Mat4 R = identity4();
    R[1][1] =  std::cos(t);  R[1][2] = -std::sin(t);
    R[2][1] =  std::sin(t);  R[2][2] =  std::cos(t);
    return R;
}

static Mat4 RotY4(double t)
{
    Mat4 R = identity4();
    R[0][0] =  std::cos(t);  R[0][2] =  std::sin(t);
    R[2][0] = -std::sin(t);  R[2][2] =  std::cos(t);
    return R;
}

int main()
{
    //  The same sample pose the generated python tries.
    Mat4 T = mat_mul(RotX4(M_PI / 7.0), RotY4(2.0 * M_PI / 7.0));
    T[0][3] = 0.2;
    T[1][3] = 0.3;
    T[2][3] = 0.6;

    std::printf("**ROBOT**:  inverse kinematics of a sample pose\\n");**KNOWN_NOTE**

    SolutionList sols = **FUNC**(T**EXTRA_ARG**);
    if (sols.empty()) {
        std::printf("  no solution:  that pose is not reachable by this arm\\n");
        return 0;
    }

    std::printf("  joint order: ");
    for (int j = 0; j < IK_NJOINTS; ++j)
        std::printf("%s%s", j ? ", " : "", JOINT_NAMES[j]);
    std::printf("\\n");

    for (size_t i = 0; i < sols.size(); ++i) {
        std::printf("Solution %d:", (int) i);
        for (size_t j = 0; j < sols[i].size(); ++j)
            std::printf("  %s = %+.9f", JOINT_NAMES[j], sols[i][j]);
        std::printf("\\n");
    }
    return 0;
}

#endif   // IKBT_MAIN
'''
