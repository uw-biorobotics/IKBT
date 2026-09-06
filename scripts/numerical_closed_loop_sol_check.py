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
#   happen is a robot dropping below the count recorded in scripts/expected.py.
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

from scripts.expected import EXPECT, judge_counts


#  THE EXPECTATION TABLE LIVES IN scripts/expected.py.  It used to live here as
#  KNOWN_GOOD, and in two other files besides, over overlapping sets of the same
#  robots -- three hand-maintained copies with no mechanism to agree.  EXPECT
#  carries (good, total) per robot:  how many branches must reproduce the pose
#  and how many branches there should be.  See that module for why both.
#
#  KinovaLite is the hybrid entry, and it converges on ALL EIGHT branches from
#  a raw seed error of 43-57 mm down to 1e-12..1e-9 in 4 or 5 iterations.  That
#  spread is worth keeping in view:  the simplification really does displace the
#  end effector by tens of millimetres, and Phase II really does remove all of
#  it, so the answer is exact even though the seed was not.

#  With no robots named, check every robot we have an expectation for.
DEFAULT_ROBOTS = sorted(EXPECT)

GEN_DIR = os.path.join('CodeGen', 'Python')

#  Symbolic path:  an EVALUATED EXPRESSION either reproduces the pose or it
#  does not, so this is float noise and nothing else.
TOL = 1e-7

#  Hybrid path:  looser on purpose.  This answer is the output of an ITERATIVE
#  solve, not an evaluated expression -- solve_numeric() stops at
#  metric <= 1e-9, and the max-abs matrix difference that corresponds to is a
#  small multiple of that.  Holding it to 1e-7 would fail runs that converged
#  perfectly well.
HYBRID_TOL = 1e-6

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


def check_symbolic(name, verbose=False):
    '''Round-trip one robot through its GENERATED CLOSED-FORM python IK.

       name:    robot name
       returns: (branches that reproduced the pose, branches returned, note)
       Never raises:  a robot that could not be checked comes back with 0
       branches and a note saying why.

       Solving is the DISPATCHER's job, not this function's:  which checker to
       run cannot be known until the artifacts exist, so the solve has to
       happen before the choice is made and must not be repeated after it.'''

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


###############################################################################
#
#    The HYBRID path:  Phase I seeds, Phase II corrects, the TRUE arm judges
#
#  WHY THE APPROXIMATE ARM CANNOT BE THE JUDGE.  Phase I's branches solve the
#  simplified arm exactly, so checking them against the simplified arm's FK
#  would pass no matter how bad the approximation was -- it would confirm only
#  that a closed form is a closed form.  Every check below goes through the
#  TRUE arm's forward kinematics, which is the arm the user asked about and
#  the only one whose pose error means anything.
#
#  WHAT A PASS PROVES.  If a refined branch lands on T, then for that pose the
#  hybrid answer is EXACT, not approximate:  Phase II converged against the
#  real kinematics.  The approximation is in the SEED, not the answer.
#
#  NOT EVERY BRANCH IS EXPECTED TO CONVERGE, and that is not a defect:  the
#  branches are different postures and the true arm may not reach the pose in
#  some of them, and damped least squares stays in the basin of its seed --
#  which is the whole reason the caller chooses the index.
#

def import_hybrid(name):
    '''Load CodeGen/Python/IK_hybrid_<name>.py and find its two entry points.

       Returns (module, phase1, phase2).
       Raises IOError if the module was never generated, ValueError if it does
       not carry the pair of entry points this method is defined by.

       The functions are found by SHAPE, not by rebuilding their names here.
       py_identifier() may rewrite a robot name that is not a valid python
       identifier, and a checker that re-derived the name would drift from the
       generator the first time that mattered.'''

    path = os.path.join(GEN_DIR, 'IK_hybrid_%s.py' % name)
    if not os.path.exists(path):
        raise IOError('no generated hybrid code at %s' % path)

    spec = importlib.util.spec_from_file_location('hybrid_' + name, path)
    mod = importlib.util.module_from_spec(spec)
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        spec.loader.exec_module(mod)

    p1 = [getattr(mod, a) for a in dir(mod)
          if a.startswith('ikin_') and a.endswith('_approx')]
    p2 = [getattr(mod, a) for a in dir(mod)
          if a.startswith('refine_') and not a.startswith('refine_all_')]
    if not p1:
        raise ValueError('%s has no Phase I entry point' % path)
    if not p2:
        raise ValueError('%s has no Phase II entry point' % path)
    return mod, p1[0], p2[0]


def true_fk_of(mod):
    '''The TRUE arm's FK callable, out of the hybrid module's own imports.

       Taken from the module rather than rebuilt from the robot definition on
       purpose:  this check has to judge what was SHIPPED.  Rebuilding the FK
       here would let a generator that emitted the wrong arm's kinematics pass,
       because the checker would be comparing that arm against itself.'''

    tf = getattr(mod, 'true_fk', None)
    if tf is None:
        raise ValueError('hybrid module does not import a true-arm FK')
    fns = [getattr(tf, a) for a in dir(tf) if a.startswith('fk_')]
    if not fns:
        raise ValueError('true-arm FK module has no fk_* function')
    return fns[0]


def check_hybrid(name, verbose=False):
    '''Round-trip one robot through its generated HYBRID IK.

       Returns (branches that reached the pose, branches Phase I offered, note).
       Never raises:  a robot that could not be checked comes back with 0
       branches and a note saying why.

       Solving is the dispatcher's job -- see check().'''

    try:
        mod, phase1, phase2 = import_hybrid(name)
        fk_true = true_fk_of(mod)
    except Exception as e:
        return 0, 0, '%s: %s' % (type(e).__name__, str(e)[:90])

    ndof = int(getattr(mod, 'NDOF', len(Q_PROBE)))
    q_true = [float(x) for x in Q_PROBE[:ndof]]

    try:
        T = np.asarray(fk_true(q_true), dtype=float)
    except Exception as e:
        return 0, 0, 'true FK failed: %s: %s' % (type(e).__name__, str(e)[:70])

    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        try:
            seeds = phase1(T)
        except Exception as e:
            return 0, 0, ('Phase I raised %s: %s'
                          % (type(e).__name__, str(e)[:70]))

    if not seeds:
        return 0, 0, 'Phase I returned no usable branch for a reachable pose'

    good = 0
    seed_err, final_err = [], []
    for i, seed in enumerate(seeds):
        #  How wrong the RAW approximate answer is on the true arm.  This is
        #  the quantity Phase II exists to remove, so it is measured rather
        #  than assumed.
        try:
            e0 = float(np.max(np.abs(np.asarray(fk_true(seed), dtype=float) - T)))
        except Exception:
            e0 = float('inf')
        seed_err.append(e0)

        with contextlib.redirect_stdout(buf):
            try:
                r = phase2(T, i, seeds=seeds)
            except Exception as e:
                if verbose:
                    print('      branch %-2d Phase II raised %s'
                          % (i, type(e).__name__))
                continue

        if r.get('q') is None:
            continue
        try:
            e1 = float(np.max(np.abs(
                np.asarray(fk_true(r['q']), dtype=float) - T)))
        except Exception:
            continue
        final_err.append(e1)

        if e1 < HYBRID_TOL:
            good += 1
            if verbose:
                print('      branch %-2d  seed err %.2e -> %.2e   %2d iters  OK'
                      % (i, e0, e1, r.get('iterations', -1)))
        elif verbose:
            print('      branch %-2d  seed err %.2e -> %.2e   %2d iters  %s'
                  % (i, e0, e1, r.get('iterations', -1),
                     r.get('reason', '?')))

    note = ''
    if seed_err and final_err:
        note = ('seed err %.1e -> %.1e'
                % (min(seed_err), min(final_err)))
    return good, len(seeds), note


###############################################################################
#
#    Which path did this robot take, and therefore which check applies
#

def detect_path(name):
    """'symbolic', 'hybrid' or None, from the artifacts on disk.

       Returns (path, note).

       READ OFF THE ARTIFACTS, not re-derived.  The alternative -- re-solving
       and asking the blackboard -- would make the checker's answer depend on a
       second solve rather than on what was actually shipped, which is the one
       thing a checker of generated code must not do.  The naming contract is
       what makes this reliable:

           IK_hybrid_<name>.py     only the hybrid path writes this
           IK_equations<name>.py   only the symbolic path writes this
                                   UNDER THE TRUE ROBOT'S NAME

       A hybrid solve also writes IK_equations<derived>.py, but that carries the
       derived arm's name, so it can never be mistaken for this robot's closed
       form.  That separation exists exactly so questions like this one have an
       unambiguous answer."""

    hyb = os.path.join(GEN_DIR, 'IK_hybrid_%s.py' % name)
    sym = os.path.join(GEN_DIR, 'IK_equations%s.py' % name)
    has_hyb, has_sym = os.path.exists(hyb), os.path.exists(sym)

    if has_hyb and has_sym:
        #  NOT legitimate:  a robot solves one way or the other.  Almost always
        #  one of them is left over from an earlier run, before the robot
        #  changed character.  Go with the newer and SAY SO, rather than
        #  picking silently -- a checker that quietly tested the stale artifact
        #  would report a confident PASS about code nobody is using.
        newer = 'hybrid' if os.path.getmtime(hyb) > os.path.getmtime(sym) else 'symbolic'
        return newer, ('both %s and %s exist -- checking the newer (%s);  the '
                       'other is stale, delete it'
                       % (os.path.basename(hyb), os.path.basename(sym), newer))
    if has_hyb:
        return 'hybrid', ''
    if has_sym:
        return 'symbolic', ''
    return None, 'nothing generated for %s' % name


def check(name, resolve=True, verbose=False):
    '''Closed-loop check of whatever IKBT generated for `name`.

       Determines the path from the artifacts and runs the matching check:

         symbolic  q -> T = FK(q) -> ikin_*(T) -> FK(each branch) == T
         hybrid    q -> T = FK_true(q) -> Phase I -> Phase II
                                       -> FK_true(refined) == T

       name:    robot name
       resolve: re-run ikSolver.py first (False re-uses what is on disk)
       returns: (poses that reproduced the target, poses checked, note, path)

       Never raises:  a robot that could not be checked comes back with 0
       poses and a note saying why.

       WHY ONE ENTRY POINT.  The two paths ask the same question -- does the
       code IKBT just wrote put this arm at the pose it was asked for -- and a
       caller with a robot name in hand has no reason to know which branch of
       the tree answered it.  Keeping them apart meant every caller had to
       decide first, using the same artifact test that now lives here once.'''

    if resolve:
        ok, why = solve_robot(name)
        if not ok:
            return 0, 0, why, None

    path, note = detect_path(name)
    if path is None:
        return 0, 0, note, None

    if path == 'hybrid':
        good, n, hnote = check_hybrid(name, verbose=verbose)
    else:
        good, n, hnote = check_symbolic(name, verbose=verbose)

    #  detect_path's note is a WARNING about the artifacts;  the checker's is a
    #  measurement.  Both matter, so neither is dropped.
    both = '; '.join(x for x in (note, hnote) if x)
    return good, n, both, path


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('robots', nargs='*', help='robots to check')
    ap.add_argument('--keep', action='store_true',
                    help='use the existing generated code, do not re-solve')
    ap.add_argument('--verbose', action='store_true',
                    help='say why each failing pose failed')
    a = ap.parse_args(argv)

    names = a.robots if a.robots else DEFAULT_ROBOTS

    print('')
    print('  Does the GENERATED code put the arm at the pose it was asked for?')
    print('  The path is detected per robot, from the artifacts on disk:')
    print('    symbolic  q -> T = FK(q) -> ikin_*(T) -> FK(each branch) == T')
    print('    hybrid    q -> T = FK_true(q) -> Phase I -> Phase II')
    print('                                  -> FK_true(refined) == T')
    print("  T comes from the robot's own FK, so it is reachable by")
    print("  construction and a failure is the solver's, never the target's.")
    print('')
    print('  %-16s %-9s %8s  %s' % ('robot', 'path', 'poses', 'note'))
    print('  ' + '-' * 70)

    regressions, blanks = [], []
    for name in names:
        good, n, note, path = check(name, resolve=not a.keep, verbose=a.verbose)
        known = name in EXPECT

        if n == 0:
            #  COULD NOT CHECK IS A FAILURE for a robot we know should pass.
            flag = ('  <-- REGRESSION (expected %d poses)' % EXPECT[name][0]
                    if known else '')
            print('  %-16s %-9s %8s  %s%s'
                  % (name, path or '-', '-', note, flag))
            (regressions if known else blanks).append(name)
            continue

        bad = judge_counts(name, good, n)
        flag = ''
        if bad:
            flag = '  <-- REGRESSION: ' + '; '.join(bad)
            regressions.append(name)
        elif good == 0:
            flag = '  <-- no pose reproduces the target'
        print('  %-16s %-9s %8s  %s%s'
              % (name, path, '%d/%d' % (good, n), note, flag))

    print('')
    if regressions:
        print('  FAILED: %s' % ', '.join(sorted(set(regressions))))
        print('')
        return 1
    if blanks and not a.robots:
        print('  Could not check: %s' % ', '.join(blanks))
    print('  PASS: every checked robot meets its expected pose count.')
    print('')
    return 0


if __name__ == '__main__':
    sys.exit(main())
