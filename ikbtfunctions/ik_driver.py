#!/usr/bin/python
#
#   ik_driver.py --  the IK solution pipeline, as importable functions
#
#   Robot loading, blackboard setup, ticking, solution-set generation and the
#   codegen calls.  Importing this module does not run a solve.
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

#  Messages from the report-fitting pass ("2 equations too wide, shortening:").
#  ikSolver.py turns this on with PERFORMANCE_OUTPUT.
REPORT_PROGRESS = False

from ikbtfunctions.ik_robots import robot_params
from ikbtbasics.ik_classes  import kinematics_pickle, check_the_pickle


def load_robot(name, testing=False):
    '''Fetch a robot definition and its forward kinematics.

       Returns (M, R, unknowns):  a mechanism, a Robot, and the unknown list --
       which kinematics_pickle() may EXTEND with sum-of-angles variables, so
       always use the returned list, never the one from robot_params().

       FK and the sum-of-angles scan are slow, so kinematics_pickle() caches to
       fk_eqns/<name>_pickle.p.  A DH change heals itself, but a change to the
       FK or SOA CODE requires deleting the pickle by hand.'''

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
    '''False when the tree gave up having solved nothing;  there is then no
       solution set, so do not call emit_outputs().'''
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

       Pass 1 writes the report.  pdflatex then reports which equations are too
       wide, in points, against the source line each one sits on;  every equation
       is emitted on its own line with a marker above it, so that maps back to
       the equation.  The next pass re-writes those with their pieces named as
       K_i, which shortens them.  Repeat while each pass turns up equations the
       previous one had not seen, since shortening one can expose another.

       Width is a property of the typeset line, not of the expression, so it is
       MEASURED and not predicted.  See ikbtfunctions/texwidth.py.

       slack_pt ignores overflows of a character or two, which are not worth
       folding a readable equation to fix.

       Degrades to pass 1:  with no pdflatex, a LaTeX error, or a timeout, the
       pass 1 report is already written and correct, just wide.'''

    path = ol.output_latex_solution(R, unks, groups, hybrid=hybrid, R_true=R_true)

    force = set()
    for _ in range(max(0, passes - 1)):
        try:
            measured = texwidth.overfull_ids(path, slack_pt=slack_pt)
        except Exception as e:
            if REPORT_PROGRESS:
                print('  LaTeX width check skipped -- %s: %s'
                      % (type(e).__name__, e))
            break

        new_force = {i for i in measured if i not in force}
        if not new_force:
            break                    # nothing left that shortening has not seen
        force |= new_force
        if REPORT_PROGRESS:
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

       Four Python files and one report, each named for the arm it describes:

           LaTex/ik_solution_<True>.tex            TRUE robot
           CodeGen/Python/IK_hybrid_<True>.py      TRUE robot
           CodeGen/Python/IK_equations<Derived>.py DERIVED arm
           CodeGen/Python/FK_numeric<Derived>.py   DERIVED arm
           CodeGen/Python/FK_numeric<True>.py      TRUE robot

       The report and the module a user imports carry the name they asked
       about;  the pieces they are built from carry the name of the arm they
       actually describe, so opening any one file tells you which robot it is.

       No C++ on this path yet -- see "Still open" in CLAUDE.md.'''

    true_name = hybrid.get('true_robot') or R.name
    derived_name = hybrid.get('derived_robot') or R.name

    #  The report, named and titled for the TRUE robot.  R is still the derived
    #  arm;  output_latex_solution takes the substitution as an argument and
    #  documents it in its own section.
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
