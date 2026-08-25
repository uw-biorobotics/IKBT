#!/usr/bin/python
#
#   check_solution_set.py --  is the generated IK actually correct?
#
#   Run this to verify that a branch produces a correct solution set.  It is
#   deliberately built on nothing but `ikSolver.py` and the FK pickle, so it
#   runs on ANY branch -- including ones without the ik_driver / bt_assembly
#   refactor, which is where a checker is most needed.
#
#       python3 -m scripts.check_solution_set                 # default robots
#       python3 -m scripts.check_solution_set Puma Stanford    # just these
#       python3 -m scripts.check_solution_set --keep           # don't re-solve
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
#   WHY THIS EXISTS.  Nothing else checks that the equations are RIGHT.  The
#   solve either finishes or it does not, and every generated artifact is taken
#   on faith.  That is how this survived until 2026-08-24:  Puma reported
#   "solved 7/7" while not one of its eight solution versions could even be
#   evaluated -- each referenced a `th_1s1` that nothing ever assigns, because
#   version names had been built out of solution names.  LaTeX and C++ printed
#   that symbol into documents and source that looked authoritative;  Python at
#   least failed loudly, because Python runs.
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
              'Olson13': 4, 'Brad': 1}

DEFAULT_ROBOTS = ['Puma', 'Pumaoffset', 'Stanford', 'Khat6DOF', 'Olson13',
                  'Brad']

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
    '''(fk, joint_names) built from the pickled T_06.
    '''

    from ikbtfunctions.ik_robots import robot_params
    from ikbtbasics.ik_classes import kinematics_pickle

    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        dh, vv, params, pvals, unks = robot_params(name)
        M, R, unks = kinematics_pickle(name, dh, params, pvals, vv, unks, False)

    #  parameter values as floats: forward_kinematics() writes the STRINGS
    #  'np.cos(al_1)' / 'np.sin(pi/4)' for a robot whose alpha is not a
    #  multiple of pi/2 (Craig417, Raven-II), and those symbols really do
    #  appear in T_06, so they must be resolved rather than skipped.
    pv, left = {}, {}
    for k, v in (M.pvals or {}).items():
        if isinstance(v, str):
            left[k] = v
        else:
            pv[sp.sympify(k)] = float(v)
    for _ in range(2):
        if not left:
            break
        still = {}
        for k, v in left.items():
            try:
                pv[sp.sympify(k)] = float(
                    sp.sympify(str(v).replace('np.', '')).subs(pv))
            except Exception:
                still[k] = v
        if len(still) == len(left):
            break
        left = still

    #  Joint variable of each DH row, in chain order.  vv[r]==1 -> theta
    #  (col 3), else d (col 2).  The table is always 6 rows;  a shorter arm is
    #  padded with [0,0,0,0], and a padding row has no symbol.
    jnames = []
    for r in range(6):
        e = sp.sympify(M.DH[r, 3 if M.vv[r] else 2])
        if not e.free_symbols:
            break
        jnames.append(str(e))

    T = sp.Matrix(M.T_06).subs(pv)
    extra = sorted(T.free_symbols - set(sp.Symbol(j) for j in jnames), key=str)
    if extra:
        raise ValueError('T_06 has no value for %s' % [str(x) for x in extra])

    fk = sp.lambdify([[sp.Symbol(j) for j in jnames]], T, 'numpy')
    return fk, jnames


def generated_columns(path):
    '''Return the variable names of one returned solution row, in order.

       Read out of the generated source, because the returned list is UNLABELLED
       and includes the sum-of-angle variables:  Puma returns 7 values ordered
       ['th_1','th_23','th_2','th_3','th_4','th_5','th_6'], because the
       generator sorts each row's version names alphabetically.  A caller cannot
       otherwise tell which entry is which joint.'''

    src = open(path).read()
    m = re.search(r'solution_list\.append\(\s*\[(.*?)\]\s*\)', src, re.S)
    if not m:
        return None
    names = re.findall(r'[A-Za-z_]\w*v\d+', m.group(1))
    return [n.rsplit('v', 1)[0] for n in names]


def load_generated(name):
    '''Import the generated module and return (function, column base names).'''

    path = os.path.join(GEN_DIR, 'IK_equations%s.py' % name)
    if not os.path.exists(path):
        raise IOError('no generated code at %s' % path)

    cols = generated_columns(path)
    if not cols:
        raise ValueError('could not read the solution_list columns from %s'
                         % path)

    spec = importlib.util.spec_from_file_location('genik_' + name, path)
    mod = importlib.util.module_from_spec(spec)
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        spec.loader.exec_module(mod)

    fns = [getattr(mod, a) for a in dir(mod) if a.startswith('ikin_')]
    if not fns:
        raise ValueError('%s defines no ikin_* function' % path)
    return fns[0], cols


def check(name, resolve=True, verbose=False):
    '''Returns (n_ok, n_branches, note).  Never raises.'''

    if resolve:
        ok, why = solve_robot(name)
        if not ok:
            return 0, 0, why

    try:
        fk, jnames = robot_fk(name)
    except Exception as e:
        return 0, 0, 'FK unavailable -- %s' % str(e)[:70]

    try:
        fn, cols = load_generated(name)
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
