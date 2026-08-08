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

    ol.output_latex_solution(R, unks, R.solutionSet)   # calling args could be optimized for V3
    op.output_python_code(R, R.solutionSet)
    oc.output_cpp_code(R, R.solutionSet)


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
