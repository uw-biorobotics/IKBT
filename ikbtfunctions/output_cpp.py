#!/usr/bin/python
#
#   output_cpp.py --  the closed-form IK, in C++
#
#   The twin of output_python.output_python_code(), derived from it line by
#   line:  the same walk of Robot.FinalEqnMatrix, the same rule of keeping
#   one equation per left-hand side in the order first seen, the same
#   joint/aux column split, and the same two entry points:
#
#       known=None     ikin(T)             -- an unconditional closed form
#       known='th_2'   ikin_given(T, th_2) -- the one-variable branch's closed
#                      form, valid only where the assumed th_2 is right
#
#   Python writes a file per artifact;  C++ writes one translation unit per
#   robot, CodeGen/Cpp/<Robot>CppCode/<Robot>.cpp, so this emits a SECTION of
#   that file into an already-open stream.  output_cpp_robot.py assembles the
#   file around it and is the only caller.
#
#   Copyright 2017-2026 University of Washington
#
#   Developed by Dianmu Zhang and Blake Hannaford
#   BioRobotics Lab, University of Washington

import sympy as sp

import ikbtbasics.numeric_ik as nik
from ikbtfunctions.output_cpp_common import (cpp_identifier, expr_cpp,
                                             fk_suffix, name_table_cpp,
                                             param_decls, pose_unpack_cpp)


#  Console chatter from the generator:  one line per version per variable,
#  so off by default.  output_python.py has the same switch.
VERBOSE = False


def _say(*args):
    if VERBOSE:
        print(*args)


def _versions(Robot, node):
    '''The DISTINCT solution equations for one variable, in first-seen order.

       Straight from output_python.py.  A variable solved early shares its
       versions across matrix rows -- Puma's th_1 has 2 versions spread over
       8 rows -- so walking the rows blindly would emit each statement
       several times.'''

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
#  std::acos, std::asin and std::sqrt return NaN, quietly, when their argument
#  leaves their domain.  Python raises instead, which is why
#  output_python.dc_rewrite() swaps in acos_dc / asin_dc / sqrt_dc there.
#  scripts/cpp_expr_check compares the two languages case by case.


def output_cpp_code(Robot, solution_groups, f, known=None, owner=None):
    '''Emit the closed-form IK section of a robot's <Robot>.cpp.

       Robot            solved, with create_solution_set() already run
       solution_groups  R.solutionSet, used only as a fallback for the rows
       f                the open stream of the robot file being assembled
       known            the one-variable branch's assumed variable, or None
       owner            the robot whose namespace this lands in.  Robot's own
                        name on the symbolic and one-variable paths;  on the
                        hybrid path it is the true arm, so this closed form
                        belongs to the derived arm and keeps its suffix.

       Returns the name of the function emitted.'''

    print('\n\n\n                       Starting IK C++ Output work \n\n\n')

    orig_name = Robot.name.replace('test: ', '')
    ident = cpp_identifier(orig_name)
    #  The namespace is the qualification -- see output_cpp_common.fk_suffix().
    sfx = fk_suffix(orig_name, owner)
    funcname = 'ikin' + sfx + ('_given' if known else '')

    #
    #   WHAT COMES BACK, the same as python:  joints only, in DH CHAIN order.
    #   Sum-of-angle variables are computed, because later solutions depend
    #   on them, but are not returned.
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

    #  max(1, ...):  the fixed-array entry point declares
    #  `double solution_list[IK_NBRANCHES][IK_NJOINTS]`, and a zero-length
    #  array will not compile.  A solve with no joints or no branches should
    #  never get this far, but if one did it should not take the whole file
    #  down with it.
    n_joints = max(1, len(joint_cols))
    n_branches = max(1, len(rows))

    #
    #   EVERY NAME THE BODY ASSIGNS, declared up front, and taken from
    #   FinalEqnMatrix -- which is the same place the assignments come from,
    #   so no name can be used without being declared.
    #
    #   Each is initialised to NaN, which is also what an out-of-domain
    #   arccosine returns, so a variable never assigned and a variable with no
    #   real value reach the finiteness test at the bottom the same way.  An
    #   uninitialised double is undefined behaviour in C++ and would read as
    #   plausible garbage instead.
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

    banner = ('the CONDITIONAL closed form -- %s assumed known' % known) \
        if known else 'the closed form'
    print(section_banner('%s   (%s)' % (banner, orig_name)), file=f)

    if known:
        print('//  CONDITIONAL.  %s is an INPUT, not an output:  these' % known,
              file=f)
        print('//  equations hold only where its value is right.  solve()', file=f)
        print('//  searches for the values that are, and is what you should', file=f)
        print('//  normally call.', file=f)
        #  COND_KNOWN_VARIABLE, not KNOWN_VARIABLE:  the search section of
        #  this same file declares the plain name, and both land in one
        #  namespace.
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
        print('//  with NaN in it.  solve() indexes branches by position, so', file=f)
        print('//  row i must be the same branch at every value.', file=f)
    else:
        print('//  Only the branches that EXIST at the goal pose are returned,', file=f)
        print('//  so the count varies with the pose.  Empty means none do.', file=f)
    print('//  Joint values returned by %s(), in this order.' % funcname,
          file=f)
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
    #   The solution equations themselves.
    #
    arglist = 'const Mat4 &T' + (', double %s' % known if known else '')
    print('//', file=f)
    print('//   Solve the unknowns in closed form.', file=f)
    print('//       T   4x4 numerical target for T06, row major', file=f)
    print('//   Returns one JointVec per solution branch, or an EMPTY list', file=f)
    print('//   if the pose is not reachable.', file=f)
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
    //  leaves [-1, 1].  That means the posture this branch describes does
    //  not exist at this pose -- a normal result, since the solver
    //  enumerates every combination of the unknowns' branches without
    //  checking which ones the arm can actually adopt.  The NaN carries
    //  through the arithmetic below and is caught at the bottom of this
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
    #   A (void) sweep first.  A sum-of-angles intermediate can be computed
    #   and then read by nothing -- th_34 on Axtman13, th_34 and th_45 on
    #   JennyGuoSp24 -- which g++ reports as -Wunused-but-set-variable.
    #   Python makes the same dead assignment and says nothing.  Emitting
    #   every version of every variable keeps the body the same shape for
    #   every robot, which is worth more than the cost of silencing a
    #   warning.
    if sol_vars:
        print('    ' + '; '.join('(void) %s' % v for v in sol_vars) + ';',
              file=f)
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
        #  `known` has no column in the solution matrix, because nothing
        #  solved it, so its cell is just the argument's own name.
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
    #   THE TWO ENTRY POINTS DIFFER HERE, exactly as they do in python.
    #   ikin() returns only the branches that exist;  ikin_given() returns
    #   every row in a fixed position, because the search indexes them.
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
    #   The fixed-array entry point, for callers who have a double[4][4]
    #   rather than a Mat4.
    #
    print('''

/////////////////////////////////////////////////////////////
//
//  A plain C-array interface, for callers holding a double[4][4].  Fills
//  solution_list in place and returns 1 for a solved pose, 0 for none.
//
/////////////////////////////////////////////////////////////
''', file=f)
    array_args = 'double T[4][4], double solution_list[IK_NBRANCHES][IK_NJOINTS]'
    if known:
        array_args += ', double %s' % known
    #  ikin_array, not ikin:  one name per interface, so that ikin() means
    #  exactly one function.
    print('int %s(%s)'
          % (funcname.replace('ikin', 'ikin_array', 1), array_args), file=f)
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

    #  NO main() HERE.  A self-test belongs to the file, and a robot's file
    #  gets exactly one:  output_cpp_robot.py emits whichever of the three
    #  the robot's solution path calls for.  On the hybrid path this section
    #  is the derived arm's closed form, which is not what the file's
    #  self-test should be exercising.
    print('\n\n\n                       End of C++ Output work \n\n\n')
    return funcname


def section_banner(title):
    '''A labelled rule between the sections of a <Robot>.cpp.

       Everything about a robot is in one file, so a reader scrolls through
       it;  the sections have to announce themselves.'''

    return ('\n\n/////////////////////////////////////////////////////////////'
            '\n//\n//   %s\n//\n'
            '/////////////////////////////////////////////////////////////\n'
            % title)


#  The file's self-test, kept out of the function body so that it reads as
#  the C++ it is.
MAIN_BLOCK = '''

/////////////////////////////////////////////////////////////
//
//   TEST CODE.  Build it with
//       g++ -std=c++11 -O2 -DIKBT_MAIN <this file> -o ik_test -lm
//
/////////////////////////////////////////////////////////////

#ifdef IKBT_MAIN

//  main() has to be at global scope, so the self-test reaches back into the
//  robot's namespace from outside it.
using namespace ikbt;
using namespace ikbt::**IDENT**;

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
