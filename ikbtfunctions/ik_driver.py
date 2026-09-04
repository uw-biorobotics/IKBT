#!/usr/bin/python
#
#   ik_driver.py --  the IK solution pipeline, as importable functions
#
#   Extracted from ikSolver.py.  Previously the whole pipeline -- robot loading,
#   pickle handling, the DH check, blackboard setup, ticking, solution-set
#   generation and three codegen calls -- ran at module level, so nothing could
#   import any part of it without triggering a full solve.  Every additional
#   front end would have had to copy all of it.
#
#   Typical use:
#
#       from ikbtfunctions.ik_driver   import *
#       from ikbtfunctions.bt_assembly import build_default_bt
#
#       M, R, unknowns = load_robot('Puma')
#       bt, nodes      = build_default_bt()
#       R, unks, bb    = run_solver(R, unknowns, bt)
#       emit_outputs(R, unks)
#
#   Copyright 2017-2026 University of Washington
#
#   Developed by Dianmu Zhang and Blake Hannaford
#   BioRobotics Lab, University of Washington

import os

import b3 as b3          # behavior trees

import ikbtfunctions.output_latex  as ol
import ikbtfunctions.output_python as op
import ikbtfunctions.output_cpp    as oc
import ikbtfunctions.output_hybrid_python as ohp
import ikbtfunctions.texwidth as texwidth

from ikbtfunctions.ik_robots import robot_params
from ikbtbasics.ik_classes  import kinematics_pickle, check_the_pickle


def load_robot(name, testing=False):
    '''Fetch a robot definition and its forward kinematics.

       Returns (M, R, unknowns):  a mechanism, a Robot, and the unknown list --
       which kinematics_pickle() may EXTEND with sum-of-angles variables, so
       always use the returned list, never the one from robot_params().

       FK and the sum-of-angles scan are slow, so kinematics_pickle() caches to
       fk_eqns/<name>_pickle.p.  check_the_pickle() compares the cached DH table
       against the current one and tells you to delete the pickle if they differ;
       ANY other change to the FK or SOA code also requires deleting it by hand.'''

    [dh, vv, params, pvals, unknowns] = robot_params(name)   # see ik_robots.py
    print('Solver:  unknowns:', unknowns)

    [M, R, unknowns] = kinematics_pickle(name, dh, params, pvals, vv, unknowns, testing)
    print('GOT HERE (Fk completed): robot name: ', R.name)

    R.name   = name
    R.params = params

    ##  check the pickle in case DH params were changed
    check_the_pickle(M.DH, dh)   # check that two mechanisms have identical DH params

    return M, R, unknowns


def init_blackboard(R, unknowns):
    '''Split every scalar equation out of the 4x4 matrix equations into the
       1-unknown / 2-unknown / 3+-unknown lists, and put them plus the Robot and
       the unknowns on a fresh blackboard.  Solving is a side effect on these
       objects, so the blackboard is the whole of the solver's state.'''

    bb = b3.Blackboard()

    [L1, L2, L3p] = R.scan_for_equations(unknowns)
    bb.set('eqns_1u',  L1)    # eqns with one unknown
    bb.set('eqns_2u',  L2)    #           two unknowns
    bb.set('eqns_3pu', L3p)   #           three or more

    bb.set('Robot', R)
    bb.set('unknowns', unknowns)

    return bb


def run_solver(R, unknowns, bt, bb=None, create_solutions=True):
    '''Tick the behavior tree until it terminates, then build the solution set.

       Returns (R, unks, bb) read back OFF the blackboard -- the tree may have
       replaced them, so use the returned objects rather than the arguments.

       create_solutions=False stops after the tick, before create_solution_set().
       That is what the TEST_DATA_GENERATION path in ikSolver.py wants:  it
       pickles the raw post-solve state.

       If the tree gave up without solving anything, comp_det sets 'no_progress'
       on the blackboard;  the solution set is then not built, and callers must
       not call emit_outputs().  Check it with solved_anything(bb).'''

    if bb is None:
        bb = init_blackboard(R, unknowns)

    print("Ticking IK BT for ", R.name, " -------------------------\n\n")
    bt.tick("Test a full solver", bb)

    print('\n\n           Processing Results \n\n')

    unks = bb.get('unknowns')
    R    = bb.get('Robot')

    if bb.get('no_progress'):
        #  Nothing was solved.  create_solution_set() would leave solListMatrix
        #  empty, and the report generator then dies in make_LHS_versions() with
        #  an IndexError instead of saying it found no solution.
        return R, unks, bb

    if create_solutions:
        #  generate the solution sets (as a set of tuples (don't ask))
        R.create_solution_set()

    return R, unks, bb


def solved_anything(bb):
    '''False when the tree gave up having solved nothing.  Gate emit_outputs()
       on this -- there is no solution set to report.'''
    return not bb.get('no_progress')


def emit_outputs(R, unks):
    '''Write the LaTeX report and the generated Python and C++ code.

       Everything under LaTex/, CodeGen/Python/ and CodeGen/Cpp/ is a generated
       artifact -- these calls overwrite whatever is there.'''

    write_latex_fitted(R, unks, R.solutionSet)
    op.output_python_code(R, R.solutionSet)
    oc.output_cpp_code(R, R.solutionSet)


def write_latex_fitted(R, unks, groups, hybrid=None, R_true=None, passes=4,
                       slack_pt=6.0):
    '''Write the LaTeX report, then MEASURE it and re-write what did not fit.

       Pass 1 writes the report as usual.  pdflatex then says which equations
       are too wide for the page, in points, against the source line each one
       sits on -- and because every equation is emitted on its own line with a
       marker above it, that maps back to the equation.  The next pass re-writes
       those with their pieces named as K_i, which is what shortens them;  it
       repeats while each pass turns up equations the previous one had not seen,
       because shortening one equation can expose another.

       WHY MEASURE RATHER THAN PREDICT.  Both cheap proxies were tried and both
       failed on measured data:  a 439-character equation overflows while a
       2030-character one fits, and Brad's th_3 is 25 operations with 12 in each
       atan2 argument -- under every threshold -- while its line is 144pt too
       wide.  Width is a property of the typeset line, not of the expression.
       See ikbtfunctions/texwidth.py.

       slack_pt ignores trivial overflows.  Stanford has a box 3.7pt over, about
       one character;  restructuring an equation to win that back trades a
       readable equation for a folded one and gains nothing.

       DEGRADES TO PASS 1.  No pdflatex, a LaTeX error, a timeout:  the report
       from pass 1 is already written and already correct, just wide.  A
       formatting refinement must never cost the report itself.'''

    path = ol.output_latex_solution(R, unks, groups, hybrid=hybrid, R_true=R_true)

    force = set()
    for _ in range(max(0, passes - 1)):
        try:
            measured = texwidth.overfull_ids(path, slack_pt=slack_pt)
        except Exception as e:
            print('  LaTeX width check skipped -- %s: %s' % (type(e).__name__, e))
            break

        new_force = {i for i in measured if i not in force}
        if not new_force:
            break                    # nothing left that shortening has not seen
        force |= new_force
        print('  LaTeX: %d equation(s) too wide, shortening: %s'
              % (len(new_force), ', '.join(sorted(new_force))))
        ol.output_latex_solution(R, unks, groups, hybrid=hybrid, R_true=R_true,
                                 force_ids=force)
    return path


def emit_hybrid_outputs(R, unks, hybrid, R_true=None):
    '''Write the artifacts for a HYBRID solve.

       R        the DERIVED robot -- the simplified arm that was actually
                solved in closed form, with its solution set already built
       unks     its unknowns
       hybrid   the blackboard's hybrid_source dict
       R_true   the TRUE robot, loaded fresh;  None if it could not be loaded

       FOUR PYTHON FILES AND ONE REPORT, and which name each carries is the
       whole point:

           LaTex/ik_solution_<True>.tex            named for the TRUE robot
           CodeGen/Python/IK_hybrid_<True>.py      named for the TRUE robot
           CodeGen/Python/IK_equations<Derived>.py named for the DERIVED arm
           CodeGen/Python/FK_equations<Derived>.py named for the DERIVED arm
           CodeGen/Python/FK_equations<True>.py    named for the TRUE robot

       The two artifacts a user reaches for -- the report and the module they
       import -- carry the name they asked about, because that is the question
       they asked.  The pieces those are built from carry the name of the arm
       they actually describe, because calling a simplified arm's closed form
       `IK_equations<True>` is exactly the confusion this method has to avoid.
       A reader who opens any one file can tell which robot it is about.

       NO C++ ON THIS PATH, yet.  Emitting the derived arm's C++ under the true
       robot's name would ship precisely the misleading artifact the naming
       above exists to prevent, and there is no C++ numeric correction to pair
       it with;  a hybrid C++ target is its own piece of work.'''

    true_name = hybrid.get('true_robot') or R.name
    derived_name = hybrid.get('derived_robot') or R.name

    #  The report, named and titled for the TRUE robot.  R is still the derived
    #  arm -- output_latex_solution takes the substitution as an argument and
    #  says so in its own section, rather than being handed a robot that lies
    #  about which arm it is.
    write_latex_fitted(R, unks, R.solutionSet, hybrid=hybrid, R_true=R_true)

    #  Phase I's closed form, under the DERIVED arm's name.
    op.output_python_code(R, R.solutionSet)

    #  FK of the approximate arm (Phase I uses it to drop spurious branches)
    #  and FK + Jacobian of the true arm (Phase II refines against it).
    ohp.write_fk_module(R.Mech, derived_name, jacobian=False,
                        what='Forward kinematics for %s (the APPROXIMATE arm)'
                             % derived_name)
    if R_true is not None:
        ohp.write_fk_module(R_true.Mech, true_name, jacobian=True,
                            what='Forward kinematics and Jacobian for %s '
                                 '(the TRUE arm)' % true_name)

        edits = '; '.join('%s: %s -> %s' % (e['symbol'], e['from'], e['to'])
                          for e in (hybrid.get('edits') or []))
        cost = hybrid.get('cost')
        ohp.write_hybrid_top(R_true.Mech, true_name, derived_name, edits,
                             '' if cost is None else
                             'task-space cost %.4g' % float(cost))


def print_solved_equations(unks):
    '''Dump the equations that were actually used to solve each variable.'''

    print("equations evaluated")
    for one_unk in unks:
        print(one_unk.symbol)
        print(one_unk.eqntosolve)
        print(one_unk.secondeqn)
        print('\n')


def ensure_logdir(logdir='logs/'):
    '''BT node logging (ikbt.log_flag / ikbt.log_file) writes here.'''
    if not os.path.isdir(logdir):
        os.mkdir(logdir)
    return logdir
