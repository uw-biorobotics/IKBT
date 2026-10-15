#!/usr/bin/python
#
#   numerical_closed_loop_sol_check.py --  is the generated IK actually correct?
#
#   Verifies that a branch produces a correct solution set.  Built on nothing
#   but `ikSolver.py` and the FK pickle, so it runs on ANY branch.
#
#       python3 -m scripts.numerical_closed_loop_sol_check                 # default robots
#       python3 -m scripts.numerical_closed_loop_sol_check Puma Stanford    # just these
#       python3 -m scripts.numerical_closed_loop_sol_check --keep           # don't re-solve
#
#   WHAT IT PROVES, and why it needs no hand-checked answers:
#
#       pick joints q  ->  T = FK(q)  ->  generated ikin_*(T)  ->
#
#         1) SOUND     every returned joint vector satisfies  FK(vector) == T
#         2) COMPLETE  q itself is among the returned vectors (mod 2*pi on
#                      revolute joints)
#
#   T comes from the robot's own forward kinematics, so it is reachable by
#   construction:  a failure is the solver's, never the target's.  Any valid IK
#   branch has to land back on T, and q is one solution known to exist, so
#   neither check needs anything looked up or trusted.  The same two checks
#   apply on all three paths.
#
#   A robot failing either one FAILS.  There is NO per-robot expectation
#   table (scripts/expected.py was removed 2026-10-15):  a table of accepted
#   partial scores is a table of hidden flaws.  Every robot that has generated
#   IK is held to the same two rules, and generated IK that cannot be run at
#   all is a failure too.  A robot with no generated IK was not solved, which
#   is reported and is not a failure of the generated code.
#
#   This is the only check on the GENERATED CODE.  check_solution_sets.py
#   checks the same property one stage earlier, on the symbolic version matrix
#   in memory;  this runs what a user would actually import.
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

from ikbtfunctions.ik_robots import ROBOT_LIST


#  With no robots named, check every robot;  the ones with no generated IK are
#  reported as not solved.
DEFAULT_ROBOTS = sorted(ROBOT_LIST)

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

#  Joint-space tolerance for "this returned solution IS the probe".  Looser
#  than TOL because the hybrid answer is iterative (see HYBRID_TOL), and a
#  pose error of 1e-9 can be a joint error a few orders larger.
PROBE_TOL = 1e-5


###############################################################################
#
#    COMPLETENESS:  is the joint vector the pose was BUILT from among the answers?
#
#  Reproducing T is SOUNDNESS:  every returned solution is a real one.  It says
#  nothing about whether a real one is MISSING.  The probe is the one solution
#  known for certain to exist, so its absence is a defect on every path, with
#  no stored expectation needed:  a closed form whose branches exclude it has a
#  wrong expression, a 1-D search that misses it stepped over its basin, and a
#  hybrid that misses it had no seed in its basin.
#

def probe_recovered(solutions, q_true, revolute):
    '''Is q_true among `solutions` (joint vectors, DH chain order)?

       Revolute joints are compared MOD 2*pi:  th and th + 2*pi are the same
       posture, and a closed form is free to return either.'''

    for q in solutions:
        d = np.asarray(q, dtype=float) - np.asarray(q_true, dtype=float)
        d = np.where(revolute, (d + np.pi) % (2 * np.pi) - np.pi, d)
        if np.all(np.abs(d) < PROBE_TOL):
            return True
    return False


def load_mechanism(name):
    '''The robot's mechanism M, from the FK pickle cache.'''

    from ikbtfunctions.ik_robots import robot_params
    from ikbtbasics.ik_classes import kinematics_pickle

    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        dh, vv, params, pvals, unks = robot_params(name)
        M, R, unks = kinematics_pickle(name, dh, params, pvals, vv, unks)
    return M


def revolute_joints(name):
    '''One bool per joint, DH chain order:  True for a revolute joint.

       From the DH table's vv, not from joint names:  Wrist's joints are
       A, B and C.'''

    M = load_mechanism(name)
    return [M.vv[r] == 1 for r in range(M.ndof)]


def probe_note(recovered, n):
    return 'probe recovered' if recovered else f'PROBE NOT FOUND among {n}'


def solve_robot(name, quiet=True):
    '''Run the solver as a subprocess.

       Subprocess, not import:  on some branches ikSolver.py does its work at
       module level, and several helpers call quit() when something goes
       wrong, which would take this checker down with them.'''

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
       resolving M.pvals (kin_cl stores a number for ca_i/sa_i since
       2026-09-03, but still falls back to an 'np.cos(al_1)' string for an
       alpha it cannot evaluate) and naming the joints (a DH cell can be
       'B + pi/2', so the joint is the symbol inside the cell, not the
       cell)."""

    import ikbtbasics.numeric_ik as nik

    M = load_mechanism(name)
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
        return 0, 0, 'FK unavailable -- %s' % str(e)[:70], None

    try:
        fn, cols = import_generated_ik(name)
    except Exception as e:
        return 0, 0, '%s: %s' % (type(e).__name__, str(e)[:70]), None

    try:
        jcol = [cols.index(j) for j in jnames]
    except ValueError as e:
        return 0, 0, 'joint %s not among returned columns %s' % (e, cols), None

    q_true = [float(x) for x in Q_PROBE[:len(jnames)]]
    T = np.asarray(fk(q_true), dtype=float)

    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        try:
            sols = fn(np.matrix(T))
        except Exception as e:
            return 0, 0, ('ikin_*() raised %s: %s'
                          % (type(e).__name__, str(e)[:70])), None
    if sols is False:
        return 0, 0, 'ikin_*() reports the pose unreachable (solvable_pose False)', None

    good = 0
    qs = []
    for i, row in enumerate(sols):
        try:
            q = [float(row[k]) for k in jcol]
            err = float(np.max(np.abs(np.asarray(fk(q), dtype=float) - T)))
        except Exception as e:
            if verbose:
                print('      branch %-3d unusable (%s)' % (i, type(e).__name__))
            continue
        qs.append(q)
        if err < TOL:
            good += 1
        elif verbose:
            print('      branch %-3d FK error %.2e' % (i, err))

    recovered = probe_recovered(qs, q_true, revolute_joints(name))
    return good, len(sols), probe_note(recovered, len(sols)), recovered


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
#  EVERY REFINED BRANCH MUST CONVERGE, as every branch must on the other
#  paths.  A branch's posture may genuinely not exist on the true arm, and
#  damped least squares stays in the basin of its seed;  but a branch handed to
#  the caller that misses T is still a wrong answer, so it fails here and the
#  question of whether the posture exists is then asked of that robot.
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
    #  Phase IIa is the primitive:  it takes ONE seed and returns
    #  (q, error, iterations).  refine_all_* is the wrapper over a seed list.
    p2 = [getattr(mod, a) for a in dir(mod) if a.startswith('refine_seed_')]
    if not p1:
        raise ValueError('%s has no Phase I entry point' % path)
    if not p2:
        raise ValueError('%s has no Phase IIa entry point' % path)
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
        return 0, 0, '%s: %s' % (type(e).__name__, str(e)[:90]), None

    ndof = int(getattr(mod, 'NDOF', len(Q_PROBE)))
    q_true = [float(x) for x in Q_PROBE[:ndof]]

    try:
        T = np.asarray(fk_true(q_true), dtype=float)
    except Exception as e:
        return 0, 0, 'true FK failed: %s: %s' % (type(e).__name__, str(e)[:70]), None

    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        try:
            seeds = phase1(T)
        except Exception as e:
            return 0, 0, ('Phase I raised %s: %s'
                          % (type(e).__name__, str(e)[:70])), None

    if not seeds:
        return 0, 0, 'Phase I returned no usable branch for a reachable pose', None

    good = 0
    seed_err, final_err, refined = [], [], []
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
                q_ref, error, iters = phase2(T, seed)
            except Exception as e:
                if verbose:
                    print('      branch %-2d Phase IIa raised %s'
                          % (i, type(e).__name__))
                continue

        if q_ref is None:
            continue
        try:
            e1 = float(np.max(np.abs(
                np.asarray(fk_true(q_ref), dtype=float) - T)))
        except Exception:
            continue
        final_err.append(e1)
        refined.append(q_ref)

        if e1 < HYBRID_TOL:
            good += 1
            if verbose:
                print('      branch %-2d  seed err %.2e -> %.2e   %2d iters  OK'
                      % (i, e0, e1, iters))
        elif verbose:
            print('      branch %-2d  seed err %.2e -> %.2e   %2d iters  '
                  'error %.2e' % (i, e0, e1, iters, error))

    #  The REFINED postures, not the seeds:  a seed belongs to the simplified
    #  arm, and only Phase II's answer is a joint vector of the true one.
    recovered = probe_recovered(refined, q_true, revolute_joints(name))
    note = probe_note(recovered, len(seeds))
    if seed_err and final_err:
        note = ('seed err %.1e -> %.1e;  %s'
                % (min(seed_err), min(final_err), note))
    return good, len(seeds), note, recovered


###############################################################################
#
#    The ONE-VARIABLE path
#
#  The same two checks as the other paths.  Completeness matters most here:
#  a 1-D search can step over a basin narrower than its sample spacing, and
#  the only way to notice is to look for a solution you already know is there.
#

def import_onevar(name):
    """Load CodeGen/Python/IK_onevar<name>.py and find what it offers.

       -> (module, solve callable, true-arm fk callable)"""

    path = os.path.join(GEN_DIR, 'IK_onevar%s.py' % name)
    if not os.path.exists(path):
        raise IOError('no generated one-variable code at %s' % path)

    spec = importlib.util.spec_from_file_location('onevar_' + name, path)
    mod = importlib.util.module_from_spec(spec)
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        spec.loader.exec_module(mod)

    solve = [getattr(mod, a) for a in dir(mod)
             if a.startswith('solve_') and not a.endswith('_labeled')]
    if not solve:
        raise ValueError('one-variable module has no solve_* entry point')

    tf = getattr(mod, 'true_fk', None)
    fks = [getattr(tf, a) for a in dir(tf or ())] if tf else []
    fks = [f for f in fks if callable(f) and getattr(f, '__name__', '').startswith('fk_')]
    if not fks:
        raise ValueError('one-variable module did not load a true-arm FK')
    return mod, solve[0], fks[0]


def check_onevar(name, verbose=False):
    """Round-trip one robot through its generated ONE-VARIABLE search.

       Returns (solutions that reached the pose, solutions returned, note).
       Never raises."""

    try:
        mod, solve, fk_true = import_onevar(name)
    except Exception as e:
        return 0, 0, '%s: %s' % (type(e).__name__, str(e)[:90]), None

    ndof = int(getattr(mod, 'NDOF', len(Q_PROBE)))
    q_true = [float(x) for x in Q_PROBE[:ndof]]

    try:
        T = np.asarray(fk_true(q_true), dtype=float)
    except Exception as e:
        return 0, 0, 'true FK failed: %s: %s' % (type(e).__name__, str(e)[:70]), None

    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        try:
            sols = solve(T)
        except Exception as e:
            return 0, 0, 'search raised %s: %s' % (type(e).__name__, str(e)[:70]), None

    if not sols:
        return 0, 0, ('the search found nothing for a pose built from this '
                      "arm's own FK"), None

    good = 0
    for i, s in enumerate(sols):
        try:
            e = float(np.max(np.abs(np.asarray(fk_true(s['q']), dtype=float) - T)))
        except Exception:
            continue
        if e < TOL:
            good += 1
        if verbose:
            print('      %-6s = %9.4f  pose err %.2e  %s'
                  % (mod.KNOWN_VARIABLE, s['known_value'], e,
                     'OK' if e < TOL else 'FAILED'))

    #  COMPLETENESS.  The pose was built from q_true, so q_true is a solution;
    #  a search that returns four other solutions and not that one has stepped
    #  over its basin.
    recovered = probe_recovered([s['q'] for s in sols], q_true,
                                revolute_joints(name))
    return good, len(sols), probe_note(recovered, len(sols)), recovered


###############################################################################
#
#    Which path did this robot take, and therefore which check applies
#

def detect_path(name):
    """'symbolic', 'hybrid', 'onevar' or None, from the artifacts on disk.

       Returns (path, note).

       READ OFF THE ARTIFACTS, not re-derived.  The alternative -- re-solving
       and asking the blackboard -- would make the checker's answer depend on a
       second solve rather than on what was actually shipped, which is the one
       thing a checker of generated code must not do.  What makes it reliable
       is the naming:  one file per path, under the TRUE robot's name:

           IK_hybrid_<name>.py     only the hybrid path writes this
           IK_onevar<name>.py      only the one-variable path writes this
           IK_equations<name>.py   only the symbolic path writes this

       The other paths write closed forms too, but never under this name:  a
       hybrid solve writes IK_equations<derived>.py, carrying the derived arm's
       name, and a one-variable solve writes IK_conditional<name>.py, whose own
       name says the equations hold only conditionally.  That separation exists
       exactly so questions like this one have an unambiguous answer."""

    candidates = [('hybrid',   os.path.join(GEN_DIR, 'IK_hybrid_%s.py' % name)),
                  ('onevar',   os.path.join(GEN_DIR, 'IK_onevar%s.py' % name)),
                  ('symbolic', os.path.join(GEN_DIR, 'IK_equations%s.py' % name))]
    present = [(p, f) for p, f in candidates if os.path.exists(f)]

    if not present:
        return None, 'nothing generated for %s' % name
    if len(present) == 1:
        return present[0][0], ''

    #  NOT legitimate:  a robot solves one way or another.  Almost always all
    #  but one is left over from an earlier run, before the robot changed
    #  character.  Go with the newest and SAY SO, rather than picking silently
    #  -- a checker that quietly tested the stale artifact would report a
    #  confident PASS about code nobody is using.
    present.sort(key=lambda pf: os.path.getmtime(pf[1]), reverse=True)
    newer = present[0][0]
    stale = ', '.join(os.path.basename(f) for _, f in present[1:])
    return newer, ('%s also exist(s) -- checking the newest (%s);  the rest '
                   'are stale, delete them' % (stale, newer))


def check(name, resolve=True, verbose=False):
    '''Closed-loop check of whatever IKBT generated for `name`.

       Determines the path from the artifacts and runs the matching check:

         symbolic  q -> T = FK(q) -> ikin_*(T) -> FK(each branch) == T
         hybrid    q -> T = FK_true(q) -> Phase I -> Phase II
                                       -> FK_true(refined) == T
         onevar    q -> T = FK(q) -> 1-D search -> FK(each found) == T

       and on every path, was q itself among the answers.

       name:    robot name
       resolve: re-run ikSolver.py first (False re-uses what is on disk)
       returns: (good, n, note, path, recovered)
                good       returned joint vectors that reproduce the target
                n          joint vectors returned
                recovered  True/False:  is the probe joint vector among them;
                           None when nothing could be checked

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
            return 0, 0, why, None, None

    path, note = detect_path(name)
    if path is None:
        return 0, 0, note, None, None

    if path == 'hybrid':
        good, n, hnote, recovered = check_hybrid(name, verbose=verbose)
    elif path == 'onevar':
        good, n, hnote, recovered = check_onevar(name, verbose=verbose)
    else:
        good, n, hnote, recovered = check_symbolic(name, verbose=verbose)

    #  detect_path's note is a WARNING about the artifacts;  the checker's is a
    #  measurement.  Both matter, so neither is dropped.
    both = '; '.join(x for x in (note, hnote) if x)
    return good, n, both, path, recovered


def closed_loop_failures(good, n, recovered):
    '''The two requirements every checked robot must meet, on every path.

         SOUND     every returned joint vector reproduces the target T
         COMPLETE  the joint vector T was built from is among them

       Neither needs a stored expectation:  T came from the arm's own FK, so a
       returned vector that misses it is wrong, and the probe is a solution
       that is known to exist.  Returns a list of complaints, empty = pass.'''

    bad = []
    if good < n:
        bad.append(f'{n - good} of {n} returned joint vectors miss the target')
    if recovered is False:
        bad.append('the probe joint vector is not among them')
    return bad


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
    print('    onevar    q -> T = FK(q) -> 1-D search -> FK(each found) == T')
    print('  and on every path, q itself must be among the answers.')
    print("  T comes from the robot's own FK, so it is reachable by")
    print("  construction and a failure is the solver's, never the target's.")
    print('')
    print('  %-16s %-9s %8s  %s' % ('robot', 'path', 'poses', 'note'))
    print('  ' + '-' * 70)

    failures, unsolved = [], []
    for name in names:
        good, n, note, path, recovered = check(name, resolve=not a.keep,
                                               verbose=a.verbose)
        if n == 0:
            #  NO GENERATED IK means the robot was not solved:  reported, not
            #  failed.  Generated IK that cannot be run, or returns nothing for
            #  a pose built from the arm's own FK, is a failure.
            if path is None:
                flag = ''
                unsolved.append(name)
            else:
                flag = '  <-- FAIL: generated IK could not be checked'
                failures.append(name)
            print('  %-16s %-9s %8s  %s%s'
                  % (name, path or '-', '-', note, flag))
            continue

        fail = closed_loop_failures(good, n, recovered)
        flag = ('  <-- FAIL: ' + '; '.join(fail)) if fail else ''
        if fail:
            failures.append(name)
        print('  %-16s %-9s %8s  %s%s'
              % (name, path, '%d/%d' % (good, n), note, flag))

    print('')
    if unsolved:
        print('  Not solved (no generated IK): %s' % ', '.join(unsolved))
    if failures:
        print('  FAIL: %s' % ', '.join(sorted(set(failures))))
        print('')
        return 1
    print('  PASS: every returned joint vector reproduces T, and the probe')
    print('        joint vector is among them, for every checked robot.')
    print('')
    return 0


if __name__ == '__main__':
    sys.exit(main())
