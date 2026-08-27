#!/usr/bin/python
#
#   numerical_closed_loop_sol_check.py --  is the generated IK actually correct?
#
#   Run this to verify that a branch produces a correct solution set.  It is
#   deliberately built on nothing but `ikSolver.py` and the FK pickle, so it
#   runs on ANY branch -- including ones without the ik_driver / bt_assembly
#   refactor, which is where a checker is most needed.
#
#       python3 -m scripts.numerical_closed_loop_sol_check                 # default robots
#       python3 -m scripts.numerical_closed_loop_sol_check Puma Stanford    # just these
#       python3 -m scripts.numerical_closed_loop_sol_check --keep           # don't re-solve
#
#   WHAT IT PROVES, and why it needs no hand-checked answers:
#
#       pick joints q  ->  T = FK(q)  ->  generated ikin_*(T)  ->  every
#       returned branch must satisfy  FK(branch) == T
#
#   T comes from the robot's own forward kinematics, so it is reachable by
#   construction:  a failure is the solver's, never the target's.  Any valid IK
#   branch has to land back on T;  which branch is which does not matter, and
#   there is nothing to look up or trust.
#
#   WHY THIS EXISTS.  Nothing else checks that the GENERATED CODE is right.
#   scripts/check_solution_sets.py checks the same property one stage earlier,
#   on the symbolic version matrix in memory;  this one runs what a user would
#   actually import, so it also covers the code generator.
#
#   NOT EVERY BRANCH IS EXPECTED TO PASS.  The version matrix enumerates
#   combinations of each unknown's solution branches and IKBT does not filter
#   spurious ones, so a robot legitimately scores less than 100%.  What must not
#   happen is a robot in KNOWN_GOOD dropping below the count recorded here.
#
#   Copyright 2026 University of Washington
#
#   Developed by Blake Hannaford
#   BioRobotics Lab, University of Washington

import argparse
import importlib.util
import io
import contextlib
import os
import re
import subprocess
import sys

import numpy as np
import sympy as sp


#  Measured 2026-08-24, immediately after the solution/version namespace fix.
#  A drop below these is a regression in the closed form itself.
KNOWN_GOOD = {'Puma': 8, 'Pumaoffset': 8, 'Stanford': 8, 'Khat6DOF': 8,
              'Olson13': 4, 'Brad': 1, 'Wrist': 2}

DEFAULT_ROBOTS = ['Puma', 'Pumaoffset', 'Stanford', 'Khat6DOF', 'Olson13',
                  'Brad', 'Wrist']

GEN_DIR = os.path.join('CodeGen', 'Python')
TOL = 1e-7

#  The joint vector used to build the target pose.  Arbitrary, but fixed, so a
#  failure is reproducible.  Truncated to the arm's real DOF.
Q_PROBE = [0.4, -0.6, 0.7, 0.9, -0.5, 0.3]


def solve_robot(name, quiet=True):
    '''Run the solver as a subprocess.

       Subprocess, not import:  on some branches ikSolver.py does its work at
       module level, and several helpers call quit() on the unhappy path, which
       would take this checker down with them.'''

    cmd = [sys.executable, 'ikSolver.py', name]
    env = dict(os.environ, PYTHONHASHSEED='0')
    r = subprocess.run(cmd, capture_output=True, text=True, env=env)
    if r.returncode != 0:
        tail = (r.stdout or '')[-300:]
        return False, 'ikSolver.py exited %d  %s' % (r.returncode,
                                                     tail.replace('\n', ' '))
    return True, ''


def robot_fk(name):
    """Load robot `name` and return (fk, joint_names).

       fk:          q -> 4x4 numpy matrix, from the pickled T_06
       joint_names: the joints q must hold, in DH chain order

       Both jobs belong to ikbtbasics.numeric_ik and are NOT repeated here:
       resolving M.pvals (which is not uniformly numeric -- kin_cl writes
       'np.cos(al_1)' strings for a non-right-angle alpha) and naming the
       joints (a DH cell can be 'B + pi/2', so the joint is the symbol inside
       the cell, not the cell)."""

    from ikbtfunctions.ik_robots import robot_params
    from ikbtbasics.ik_classes import kinematics_pickle
    import ikbtbasics.numeric_ik as nik

    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        dh, vv, params, pvals, unks = robot_params(name)
        M, R, unks = kinematics_pickle(name, dh, params, pvals, vv, unks, False)

    jnames = [str(s) for s in nik.joint_symbols(M)]
    return nik.fk_callable(M), jnames


def returned_joint_order(mod, path):
    """Which joint each column of a returned solution row holds.

       mod:   the imported generated module
       path:  its source file, for the fallback below
       returns: list of joint names, one per column, in column order

       The generator now states this itself as JOINT_NAMES.  A module generated
       before that landed has to be read for it, so fall back to the version
       names in the emitted solution_list."""

    names = getattr(mod, 'JOINT_NAMES', None)
    if names:
        return list(names)

    src = open(path).read()
    m = re.search(r'solution_list\.append\(\s*\[(.*?)\]\s*\)', src, re.S)
    if not m:
        return None
    return [n.rsplit('v', 1)[0]
            for n in re.findall(r'[A-Za-z_]\w*v\d+', m.group(1))]


def import_generated_ik(name):
    """Load the generated IK module for robot `name` and its column order.

       name:    robot name, as CodeGen/Python/IK_equations<name>.py
       returns: (ikin function, list of joint names in column order)
       raises:  IOError if no module was generated, ValueError if it defines
                no ikin_* function or its column order cannot be determined."""

    path = os.path.join(GEN_DIR, 'IK_equations%s.py' % name)
    if not os.path.exists(path):
        raise IOError('no generated code at %s' % path)

    spec = importlib.util.spec_from_file_location('genik_' + name, path)
    mod = importlib.util.module_from_spec(spec)
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        spec.loader.exec_module(mod)

    fns = [getattr(mod, a) for a in dir(mod)
           if a.startswith('ikin_') and not a.endswith('_labeled')]
    if not fns:
        raise ValueError('%s defines no ikin_* function' % path)

    cols = returned_joint_order(mod, path)
    if not cols:
        raise ValueError('could not determine the column order of %s' % path)
    return fns[0], cols


def check(name, resolve=True, verbose=False):
    '''Round-trip one robot through its GENERATED python IK.

       name:    robot name
       resolve: re-run ikSolver.py first (False re-uses what is on disk)
       returns: (branches that reproduced the pose, branches returned, note)
       Never raises:  a robot that could not be checked comes back with 0
       branches and a note saying why.'''

    if resolve:
        ok, why = solve_robot(name)
        if not ok:
            return 0, 0, why

    try:
        fk, jnames = robot_fk(name)
    except Exception as e:
        return 0, 0, 'FK unavailable -- %s' % str(e)[:70]

    try:
        fn, cols = import_generated_ik(name)
    except Exception as e:
        return 0, 0, '%s: %s' % (type(e).__name__, str(e)[:70])

    try:
        jcol = [cols.index(j) for j in jnames]
    except ValueError as e:
        return 0, 0, 'joint %s not among returned columns %s' % (e, cols)

    q_true = [float(x) for x in Q_PROBE[:len(jnames)]]
    T = np.asarray(fk(q_true), dtype=float)

    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        sols = fn(np.matrix(T))
    if sols is False:
        return 0, 0, 'ikin_*() reports the pose unreachable (solvable_pose False)'

    good = 0
    for i, row in enumerate(sols):
        try:
            q = [float(row[k]) for k in jcol]
            err = float(np.max(np.abs(np.asarray(fk(q), dtype=float) - T)))
        except Exception as e:
            if verbose:
                print('      branch %-3d unusable (%s)' % (i, type(e).__name__))
            continue
        if err < TOL:
            good += 1
        elif verbose:
            print('      branch %-3d FK error %.2e' % (i, err))
    return good, len(sols), ''


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('robots', nargs='*', help='robots to check')
    ap.add_argument('--keep', action='store_true',
                    help='use the existing generated code, do not re-solve')
    ap.add_argument('--verbose', action='store_true',
                    help='say why each failing branch failed')
    a = ap.parse_args(argv)

    names = a.robots if a.robots else DEFAULT_ROBOTS

    print('')
    print('  Does the GENERATED IK reproduce the pose it solved for?')
    print('    q -> T = FK(q) -> ikin_*(T) -> FK(each branch) must equal T')
    print('  T comes from the robot\'s own FK, so it is reachable by')
    print('  construction and a failure is the solver\'s.')
    print('')
    print('  %-16s %10s  %s' % ('robot', 'branches', 'note'))
    print('  ' + '-' * 66)

    regressions, blanks = [], []
    for name in names:
        good, n, note = check(name, resolve=not a.keep, verbose=a.verbose)
        if n == 0:
            #  COULD NOT CHECK IS A FAILURE for a robot we know should pass.
            known = name in KNOWN_GOOD
            print('  %-16s %10s  %s%s' % (name, '-', note,
                  '  <-- REGRESSION (expected %d branches)' % KNOWN_GOOD[name]
                  if known else ''))
            (regressions if known else blanks).append(name)
            continue
        flag = ''
        want = KNOWN_GOOD.get(name)
        if want is not None and good < want:
            flag = '  <-- REGRESSION (expected %d)' % want
            regressions.append(name)
        elif good == 0:
            flag = '  <-- no branch reproduces the pose'
        print('  %-16s %10s%s' % (name, '%d/%d' % (good, n), flag))

    print('')
    if regressions:
        print('  FAILED: %s' % ', '.join(regressions))
        return 1
    if blanks and not a.robots:
        print('  Could not check: %s' % ', '.join(blanks))
    print('  PASS: every checked robot meets its expected branch count.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
