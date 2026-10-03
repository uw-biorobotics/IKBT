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
#       bt             = build_default_bt()
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
import ikbtfunctions.output_cpp_robot as ocr
import ikbtfunctions.output_hybrid_python as ohp
import ikbtfunctions.output_onevar_python as oop
import ikbtfunctions.output_numeric_common as onc
import ikbtfunctions.texwidth as texwidth

#  Messages from the report-fitting pass ("2 equations too wide, shortening:").
#  ikSolver.py turns this on with PERFORMANCE_OUTPUT.
REPORT_PROGRESS = False

from ikbtfunctions.ik_robots import robot_params
from ikbtbasics.ik_classes  import kinematics_pickle, check_the_pickle


def load_robot(name):
    '''Fetch a robot definition and its forward kinematics.

       Returns (M, R, unknowns):  a mechanism, a Robot, and the unknown list --
       which kinematics_pickle() may EXTEND with sum-of-angles variables, so
       always use the returned list, never the one from robot_params().

       FK and the sum-of-angles scan are slow, so kinematics_pickle() caches to
       fk_eqns/<name>_pickle.p.  A DH change heals itself, but a change to the
       FK or SOA CODE requires deleting the pickle by hand.'''

    [dh, vv, params, pvals, unknowns] = robot_params(name)   # see ik_robots.py
    print('Solver:  unknowns:', unknowns)

    [M, R, unknowns] = kinematics_pickle(name, dh, params, pvals, vv, unknowns)

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


def emit_fk_python(M, name, jacobian=True, what=None):
    '''Write FK_numeric<name>.py for one arm.

       M         a mechanism (Robot.Mech) -- T_06 and J66 come from it
       name      the arm this is;  also the file name
       jacobian  emit jacobian_<name>() too
       what      one line for the file header, describing which arm this is

       PYTHON ONLY.  The C++ forward kinematics is not a file of its own:  it
       is a SECTION of CodeGen/Cpp/<Robot>CppCode/<Robot>.cpp, which
       output_cpp_robot.py assembles together with that robot's IK.  THE TWO
       LANGUAGES SPLIT DIFFERENTLY, deliberately -- a python module is the
       unit of import, so python keeps one per artifact;  a C++ translation
       unit is what a caller compiles, so C++ gets one per robot.

       RAISES if the arm's pvals do not resolve every parameter;  the caller
       decides whether that is fatal.'''

    if what is None:
        what = ('Forward kinematics and Jacobian for %s' % name if jacobian
                else 'Forward kinematics for %s' % name)
    onc.write_fk_module(M, name, jacobian=jacobian, what=what)


def emit_outputs(R, unks):
    '''Write the LaTeX report and the generated Python and C++ code.

       Everything under LaTex/, CodeGen/Python/ and CodeGen/Cpp/ is a generated
       artifact -- these calls overwrite whatever is there.

       THE SAME FK AND JACOBIAN THE REPORT PRINTS, AS CODE.  Every report has a
       Forward Kinematics section and a Jacobian section whichever path wrote
       it, so every path emits them as code too and the three paths' outputs
       differ only where the methods genuinely differ.  A symbolic solve does
       not need them to reach its answer -- but whoever CALLS that answer does:
       round-tripping a joint vector through FK is how you check a posture, and
       the Jacobian is what anything with a velocity in it starts from.  Before
       this, the one path that solved the robot exactly was the one path whose
       caller had to write its own FK.

       THE PYTHON ONE DEGRADES TO A WARNING, on fkOnly.py's precedent:  it
       needs every parameter to have a numeric value, and a robot with
       incomplete pvals is still a perfectly good symbolic solve, so losing one
       that took minutes over a bonus artifact would be wrong.  The C++ side
       makes the same judgement one level down, in output_cpp_robot._assemble(),
       which skips the FK section and still writes the closed form.'''

    write_latex_fitted(R, unks, R.solutionSet)
    op.output_python_code(R, R.solutionSet)
    ocr.write_symbolic_cpp(R, R.solutionSet)
    try:
        emit_fk_python(R.Mech, R.name)
    except Exception as e:
        print('   no python FK module for %s -- %s: %s'
              % (R.name, type(e).__name__, e))


def write_latex_fitted(R, unks, groups, hybrid=None, R_true=None, onevar=None,
                       passes=4, slack_pt=6.0):
    '''Long latex line 'wrapping':

       Write the LaTeX report, then MEASURE it and re-write to avoid long equations overflowing the line.

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

    path = ol.output_latex_solution(R, unks, groups, hybrid=hybrid,
                                    R_true=R_true, onevar=onevar)

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
                                 onevar=onevar, force_ids=force)
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

       ... and ALL FOUR OF THEM AS SECTIONS OF ONE C++ FILE, named for the
       true robot:  CodeGen/Cpp/<True>CppCode/<True>.cpp.  They were four
       files that #included one another, which is the same translation unit
       with more ceremony -- see output_cpp_robot.py.

       The report and the module a user imports carry the name they asked
       about;  the pieces they are built from carry the name of the arm they
       actually describe, so opening any one file -- or reading any one
       function name in the C++ -- tells you which arm it is.

       NO IK_equations<True>, AND NO PLAIN ikin().  There is no closed form for
       that robot -- that is why this path was taken -- so an artifact claiming
       to be one would be a simplified arm's equations shipped under the real
       robot's name.  That is the one thing this method must never do.  Python
       says it with the file name;  C++ says it with the function name, which
       is ikin_<Derived>() and never ikin().'''

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
    what_approx = ('Forward kinematics for %s (the APPROXIMATE arm)'
                   % derived_name)
    emit_fk_python(R.Mech, derived_name, jacobian=False, what=what_approx)

    if R_true is None:
        #  NO C++ AT ALL WITHOUT THE TRUE ARM.  Everything the hybrid path
        #  emits in C++ now lands in one file, and a seed from Phase I that
        #  has not been through Phase II is not an answer -- so half of that
        #  file is not worth writing.
        print('   no C++ for %s -- the true arm did not load, so there is no '
              'Phase II' % true_name)
        return

    what_true = ('Forward kinematics and Jacobian for %s (the TRUE arm)'
                 % true_name)
    emit_fk_python(R_true.Mech, true_name, what=what_true)

    edits = '; '.join('%s: %s -> %s' % (e['symbol'], e['from'], e['to'])
                      for e in (hybrid.get('edits') or []))
    cost = hybrid.get('cost')
    cost_text = ('' if cost is None
                 else 'task-space cost %.4g' % float(cost))
    ohp.write_hybrid_top(R_true.Mech, true_name, derived_name, edits,
                         cost_text)

    #  ONE C++ FILE FOR THE WHOLE SOLVE, named for the TRUE robot and holding
    #  BOTH arms:  the derived arm's FK and closed form keep their suffix, the
    #  true arm's take the plain names, and the two phases sit on top.
    ocr.write_hybrid_cpp(R, R.solutionSet, R_true, true_name, derived_name,
                         edits, cost_text)


def emit_onevar_outputs(R, unks, onevar):
    '''Write the artifacts for a ONE-VARIABLE solve.

       R        the TRUE robot -- nothing was approximated, so there is only
                one arm here and every file carries its name
       unks     its unknowns, MINUS the one that was assumed known
       onevar   the blackboard's onevar_source dict

       One report, three python modules, and one C++ file:

           LaTex/ik_solution_<Robot>.tex        the equations, and the assumption
           CodeGen/Python/IK_onevar<Robot>.py   the 1-D search  <- the entry point
           CodeGen/Python/IK_conditional<Robot>.py  closed form, assumed value in
           CodeGen/Python/FK_numeric<Robot>.py  this arm's FK and Jacobian
           CodeGen/Cpp/<Robot>CppCode/<Robot>.cpp   all three, as sections

       NOT IK_equations<Robot>, AND NOT ikin().  Those names mean the inverse
       kinematics of this robot, and a solution conditional on an assumed value
       is not that:  python says it with the file name, C++ with the function
       name ikin_given().

       THE JACOBIAN IS EMITTED THOUGH THE SEARCH DOES NOT USE IT.  The search
       is 1-D and golden section needs no derivative.  It is there because the
       report has a Jacobian section for this arm and the caller of a complete
       joint vector has the same use for one here as on any other path -- see
       emit_outputs().'''

    known = onevar.get('known')

    write_latex_fitted(R, unks, R.solutionSet, onevar=onevar)

    #  The closed form, with the assumed variable as a function ARGUMENT.
    op.output_python_code(R, R.solutionSet, known=known)

    #  The true arm's FK -- what the search measures the closed form against --
    #  and its Jacobian, which the search does not use but the caller may.
    what_fk = 'Forward kinematics and Jacobian for %s (the TRUE arm)' % R.name
    emit_fk_python(R.Mech, R.name, what=what_fk)

    #  The search itself:  the module a user actually calls.
    oop.write_onevar_top(R.Mech, R.name, known)

    #  ... and all three of those as one C++ translation unit.
    ocr.write_onevar_cpp(R, R.solutionSet, known)


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
