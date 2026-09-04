#!/usr/bin/python
#
#   bt_path_gate.py --  does every PATH through the behavior tree deliver what
#                       that path is supposed to deliver?
#
#       python3 -m scripts.bt_path_gate            # the whole gate, exit 1 on a miss
#       python3 -m scripts.bt_path_gate Puma       # just these robots
#       python3 -m scripts.bt_path_gate --list     # what is asserted, and why
#
#   WHY THIS EXISTS.  scripts/robot_baseline.py sweeps all 32 robots and takes
#   over an hour;  that is the right instrument for "did anything move
#   anywhere", and the wrong one to run after every edit.  This gate asks a
#   narrower and more useful question of five robots in about a minute:  for
#   each distinct route through the tree, did the tree produce the ARTIFACTS
#   that route owes the user, and is the generated IK actually correct?
#
#   It deliberately does NOT try to cover solver methods.  tests/leavestest.py
#   already exercises every leaf directly, and a gate that re-solves whole
#   robots to reach a leaf is paying minutes for what a unit test buys in
#   milliseconds.  What only an end-to-end run can show is the WIRING:  which
#   branch won, whether report_gen was reached, and whether the files on disk
#   describe the robot that was asked for.
#
#   THE PATHS, and the robot that stands for each.  b3.Priority tries the
#   symbolic branch first and unconditionally;  the hybrid branch is reached
#   only when the symbolic one fails completely (symbolic_loop's
#   require_complete = True).
#
#     symbolic wins, 6 DOF          Puma            tex + py + cpp
#     symbolic wins, 5 DOF          Chair_Helper    tex + py + cpp
#     hybrid: derived arm solves    KinovaLite      tex + hybrid + fk,
#                                                   and py + fk for the
#                                                   DERIVED arm
#     hybrid: derived arm partial   ArmRobo         NOTHING (see below)
#     hybrid: derived arm fails     KawasakiRS05L   NOTHING
#
#   A sixth path -- simplified_arm finding no usable candidate, which closes
#   the hybrid branch on an arm that already satisfies Pieper's condition on
#   every triple -- is not reachable by any robot in ROBOT_LIST and is covered
#   by TestSolver018.test_hybJ instead.
#
#   WHY A HYBRID *PARTIAL* STILL PRODUCES NOTHING.  The second solver is a
#   symbolic_loop like the first, and it carries require_complete = True:  a
#   closed form for SOME of the joints is not inverse kinematics, so it must
#   not be reported as an answer.  A derived arm solved 2-of-7 therefore FAILs
#   the inner branch, the hybrid Sequence FAILs with it, and report_gen -- which
#   sits after the Priority in an outer Sequence -- never ticks.  So the hybrid
#   path reports exactly when the derived arm solved COMPLETELY, and ArmRobo and
#   KawasakiRS05L assert that boundary from the far side.
#
#   WHICH NAME EACH ARTIFACT CARRIES is the load-bearing assertion of this file.
#   The two things a user reaches for -- the report and the module they import --
#   carry the TRUE robot's name, because that is the robot they asked about.
#   The closed form underneath carries the DERIVED arm's name, because that is
#   the arm it actually describes.  An IK_equations<True>.py appearing on the
#   hybrid path would mean IKBT had just handed someone a simplified arm's
#   equations under the real robot's name, which is the one thing this method
#   must never do;  the gate fails on it explicitly.
#
#   HOW CORRECTNESS IS CHECKED.  One call, scripts/numerical_closed_loop_sol_check.check(),
#   which detects the path from the artifacts and runs the matching test:
#
#     symbolic   q -> T = FK(q) -> ikin_*(T) -> FK(each branch) == T
#     hybrid     q -> T = FK_true(q) -> Phase I branches -> Phase II refine
#                  -> FK_true(refined) == T
#
#   T is built from the robot's own forward kinematics, so it is reachable by
#   construction and a failure is the solver's, never the target's.  Not every
#   branch is expected to pass -- IKBT enumerates version combinations and does
#   not filter spurious ones -- so the gate compares against a RECORDED count
#   per robot, exactly as those scripts do.
#
#   ONE SOLVE PER ROBOT.  The child runs the tree with codegen=True and then
#   the parent checks the files the child just wrote (resolve=False), so no
#   robot is solved twice.
#
#   Copyright 2026 University of Washington
#
#   Developed by Blake Hannaford
#   BioRobotics Lab, University of Washington

import argparse
import json
import os
import subprocess
import sys
import time
import traceback


#  The child hands its result back on stdout, fenced.  Same reason as
#  robot_baseline.py:  everything in IKBT prints freely and a solve is
#  thousands of lines, so the record needs an unambiguous delimiter.
FENCE_OPEN = '#### BT-PATH-JSON-BEGIN ####'
FENCE_CLOSE = '#### BT-PATH-JSON-END ####'

#  Generous, because two of these robots run the symbolic solver TWICE (once on
#  the true arm, once on the derived one).  A robot that hits this has changed
#  character and the gate should say so rather than hang.
DEFAULT_TIMEOUT = 900


###############################################################################
#
#    What each path owes the user
#
#  'branch'     which branch produced the answer:  'symbolic' or 'hybrid'.
#               Read from the blackboard's hybrid_source, not guessed from the
#               status -- KawasakiRS05L ends 'unsolved' having gone all the way
#               through install_simplified, so solved-count and branch are two
#               independent facts.
#  'solved'     (n_solved, n_unknowns).  n_unknowns includes sum-of-angle
#               variables, which is why Puma is 7 and not 6.
#  'artifacts'  the EXACT set of artifact kinds that must appear under the TRUE
#               robot's name.  Exact, not "at least":  an unexpected file is a
#               failure too, because the characteristic hybrid defect is an
#               EXTRA artifact -- a simplified arm's closed form emitted as
#               IK_equations<True>.py.
#  'derived_artifacts'  the same, under the DERIVED arm's name.
#  'closed_loop'  branches that must reproduce the probe pose through the
#                 GENERATED python, or None when nothing is generated.  On the
#                 hybrid path this runs Phase I then Phase II.
#  'derived'    the derived robot install_simplified should have built.  Named
#               here because the arm it picks is the whole content of the
#               hybrid decision:  a change that silently starts zeroing a
#               different DH parameter is a real change of behaviour.
#
#  Measured 2026-09-02.  Puma's 8 matches KNOWN_GOOD in
#  numerical_closed_loop_sol_check.py;  Chair_Helper's 2-of-4 was measured with
#  that same script.

NOTHING = frozenset()

EXPECTED = {
    'Puma': {
        'why': 'symbolic branch wins, 6 DOF, sum-of-angles (th_23), ranking',
        'branch': 'symbolic', 'solved': (7, 7), 'derived': None,
        'artifacts': frozenset({'tex', 'py', 'cpp'}),
        'derived_artifacts': NOTHING,
        'closed_loop': 8,
    },
    'Chair_Helper': {
        'why': 'symbolic branch wins, 5 DOF, prismatic joint',
        'branch': 'symbolic', 'solved': (5, 5), 'derived': None,
        'artifacts': frozenset({'tex', 'py', 'cpp'}),
        'derived_artifacts': NOTHING,
        'closed_loop': 2,
    },
    'KinovaLite': {
        'why': 'hybrid branch, derived arm solves completely -- the only path '
               'that reports a hybrid answer',
        'branch': 'hybrid', 'solved': (7, 7),
        'derived': 'KinovaLite_d_5_0',
        #  NO 'py' and NO 'cpp' under the true name.  There is no closed form
        #  for KinovaLite -- that is why this path was taken -- so a file
        #  claiming to be one would be the defect this gate exists to catch.
        'artifacts': frozenset({'tex', 'hybrid', 'fk'}),
        'derived_artifacts': frozenset({'py', 'fk'}),
        'closed_loop': 'hybrid',
    },
    'ArmRobo': {
        'why': 'hybrid branch, derived arm solves only some unknowns, so '
               'require_complete withholds the report',
        'branch': 'hybrid', 'solved': (2, 7),
        'derived': 'ArmRobo_d_4_0',
        'artifacts': NOTHING, 'derived_artifacts': NOTHING,
        'closed_loop': None,
    },
    'KawasakiRS05L': {
        'why': 'hybrid branch, derived arm solves nothing',
        'branch': 'hybrid', 'solved': (0, 7),
        'derived': 'KawasakiRS05L_a_3_0',
        'artifacts': NOTHING, 'derived_artifacts': NOTHING,
        'closed_loop': None,
    },
}

GATE_ROBOTS = ['Puma', 'Chair_Helper', 'KinovaLite', 'ArmRobo', 'KawasakiRS05L']


def artifact_paths(name):
    """Every file IKBT can write under the name `name`.

       Five kinds, because the two paths write different sets and WHICH set
       appeared is the thing worth asserting:

           tex     the report                    both paths
           py      closed-form IK                symbolic path (and the
                                                 DERIVED arm on the hybrid one)
           cpp     closed-form IK, C++           symbolic path only
           hybrid  the two-phase top level       hybrid path, TRUE name only
           fk      FK (+ Jacobian) callables     hybrid path

       A name here is an arm, not a robot:  on the hybrid path the true robot
       and the derived arm each own some of these, and the gate checks both."""

    return {
        'tex': os.path.join('LaTex', 'ik_solution_%s.tex' % name),
        'py': os.path.join('CodeGen', 'Python', 'IK_equations%s.py' % name),
        'cpp': os.path.join('CodeGen', 'Cpp', 'IK_equations%s.cpp' % name),
        'hybrid': os.path.join('CodeGen', 'Python', 'IK_hybrid_%s.py' % name),
        'fk': os.path.join('CodeGen', 'Python', 'FK_numeric%s.py' % name),
    }


def fresh_since(paths, t0):
    """Which of `paths` this run actually wrote, as a sorted list of keys.

       FRESHNESS, not existence.  CodeGen/ and LaTex/ carry artifacts from
       earlier runs -- every robot in this gate has stale output on disk from
       some previous session -- so "the file is there" proves nothing.

       Timed against the solve's own start rather than a before/after snapshot,
       because the DERIVED arm's name is not known until install_simplified has
       run, and its files therefore cannot be snapshotted in advance.  One rule
       for both names beats two rules that could disagree."""

    got = []
    for k in sorted(paths):
        try:
            if os.path.getmtime(paths[k]) >= t0:
                got.append(k)
        except OSError:
            pass                       # not there at all: not written
    return got


###############################################################################
#
#    The child:  solve ONE robot, with codegen, and describe what came out
#

def run_one(name):
    '''Solve `name` with the tree owning codegen, and return a plain-data
       record of which branch won and what reached the disk.

       Runs in the child process.  Raises nothing:  every failure mode becomes
       a field in the returned dict, so one bad robot cannot take out the gate.

       codegen=True is the point of this file.  robot_baseline.py deliberately
       runs with codegen OFF -- it measures solving, not output -- so nothing
       else in the test suite ever exercises report_gen end to end.'''

    from ikbtfunctions.ik_driver import load_robot, run_solver
    from ikbtfunctions.bt_assembly import build_default_bt

    rec = {'name': name, 'status': 'crash', 'error': None,
           'branch': None, 'derived': None,
           'n_solved': 0, 'n_unknowns': 0,
           'written': [], 'written_derived': [], 'wall_s': 0.0}

    t0 = time.time()
    try:
        M, R, unknowns = load_robot(name)
        bt, nodes = build_default_bt(codegen=True)

        #  comp_det's read_pause is 0 by default now, but robot_baseline sets
        #  it explicitly and so does this:  a gate must not inherit a pause
        #  that exists only so a human can read scrolling output.
        nodes['compDetect'].read_pause = 0

        #  create_solutions=False:  with codegen in the tree, output_gen_full
        #  owns create_solution_set(), and it is NOT idempotent.
        R, unks, bb = run_solver(R, unknowns, bt, create_solutions=False)

        rec['n_unknowns'] = len(unks)
        rec['n_solved'] = sum(1 for u in unks if u.solved)

        hs = bb.get('hybrid_source')
        rec['branch'] = 'hybrid' if hs else 'symbolic'
        if hs:
            rec['derived'] = hs.get('derived_robot')

        rec['written'] = fresh_since(artifact_paths(name), t0)

        #  The DERIVED arm's files are recorded SEPARATELY rather than merged
        #  into one list.  Which name an artifact carries is the whole
        #  correctness question on this path -- a closed form under the true
        #  robot's name would be a simplified arm passed off as the real one --
        #  so the two name-spaces are never allowed to blur together.
        if rec['derived']:
            rec['written_derived'] = fresh_since(
                artifact_paths(rec['derived']), t0)

        rec['status'] = 'ok'

    except SystemExit as e:
        #  quit() somewhere on the unhappy path.  This is exactly why the gate
        #  forks per robot;  in-process it would end the whole run.
        rec['error'] = 'SystemExit(%s) -- a quit() on the unhappy path' % (e.code,)
        rec['traceback'] = traceback.format_exc()
    except BaseException as e:
        rec['error'] = '%s: %s' % (type(e).__name__, e)
        rec['traceback'] = traceback.format_exc()

    rec['wall_s'] = round(time.time() - t0, 2)
    return rec


def run_child(name):
    '''Child entry point:  solve `name`, print the fenced record.'''

    rec = run_one(name)
    sys.stdout.flush()
    print(FENCE_OPEN)
    print(json.dumps(rec, sort_keys=True))
    print(FENCE_CLOSE)
    sys.stdout.flush()
    return 0


def parse_child_output(text):
    '''Pull the fenced record out of a child's stdout, or None if absent.'''

    if FENCE_OPEN not in text or FENCE_CLOSE not in text:
        return None
    body = text.split(FENCE_OPEN, 1)[1].split(FENCE_CLOSE, 1)[0]
    try:
        return json.loads(body.strip())
    except ValueError:
        return None


###############################################################################
#
#    The parent:  run each robot and judge it
#

def solve_in_child(name, timeout, logdir):
    '''Run one robot in its own process and return its record.

       Subprocess for the same two reasons as robot_baseline.py:  helpers call
       quit() on the unhappy path, and a solve leaves a great deal of state
       behind (sympy caches, the b3 blackboard) that a second robot in the same
       interpreter would inherit.'''

    cmd = [sys.executable, '-m', 'scripts.bt_path_gate', '--child', name]
    env = dict(os.environ, PYTHONHASHSEED='0')

    timed_out = False
    try:
        p = subprocess.run(cmd, stdout=subprocess.PIPE,
                           stderr=subprocess.STDOUT, env=env, timeout=timeout)
        out = p.stdout.decode('utf-8', errors='replace')
    except subprocess.TimeoutExpired as e:
        out = (e.stdout or b'').decode('utf-8', errors='replace')
        timed_out = True

    if logdir:
        try:
            with open(os.path.join(logdir, '%s.log' % name), 'w') as f:
                f.write(out)
        except OSError:
            pass                       # a log we could not write is not a gate failure

    rec = parse_child_output(out)
    if rec is None:
        rec = {'name': name,
               'status': 'timeout' if timed_out else 'crash',
               'error': ('exceeded %d s' % timeout if timed_out
                         else 'child produced no record'),
               'branch': None, 'derived': None,
               'n_solved': 0, 'n_unknowns': 0,
               'written': [], 'written_derived': [], 'wall_s': float(timeout)}
    return rec


def judge(name, rec):
    '''Compare one record against EXPECTED.  Returns a list of complaints,
       empty when the path did exactly what it owes.'''

    want = EXPECTED[name]
    bad = []

    if rec['status'] != 'ok':
        return ['%s -- %s' % (rec['status'], rec.get('error'))]

    if rec['branch'] != want['branch']:
        bad.append('took the %s branch, expected %s'
                   % (rec['branch'], want['branch']))

    got = (rec['n_solved'], rec['n_unknowns'])
    if got != tuple(want['solved']):
        bad.append('solved %d/%d, expected %d/%d'
                   % (got + tuple(want['solved'])))

    if rec['derived'] != want['derived']:
        bad.append('derived arm %s, expected %s'
                   % (rec['derived'], want['derived']))

    #  EXACT set comparison, in both name-spaces.  A missing artifact and an
    #  unexpected one are both failures:  the characteristic defect of this
    #  method is an EXTRA file -- the simplified arm's closed form emitted
    #  under the true robot's name -- and an "at least these" check would wave
    #  it through.
    bad += _artifact_complaints(set(rec['written']), set(want['artifacts']),
                                'under %s' % name)
    bad += _artifact_complaints(set(rec['written_derived']),
                                set(want['derived_artifacts']),
                                'under the derived name %s'
                                % (rec['derived'] or '?'))

    return bad


def _artifact_complaints(got, want, where):
    '''What is wrong with one name-space's artifact set.'''

    out = []
    missing = sorted(want - got)
    extra = sorted(got - want)
    if missing:
        out.append('did not write %s %s' % (', '.join(missing), where))
    if extra:
        #  The loud one.  Named separately from "missing" because an
        #  unexpected artifact is not an omission, it is a claim: a file that
        #  says it is the IK of a robot IKBT could not solve.
        out.append('WROTE UNEXPECTED %s %s' % (', '.join(extra), where))
    return out


def closed_loop(name, want, verbose=False):
    """Round-trip the generated python for `name` through FK.

       want is an integer for the symbolic path (branches that must reproduce
       the pose) or the string 'hybrid', which runs both phases instead.

       Returns (line, ok).  Neither checker re-solves:  the child has just run
       this robot with codegen on, so the modules on disk are the ones we mean
       and re-solving would double the gate's cost for nothing."""

    from scripts.numerical_closed_loop_sol_check import check

    #  ONE entry point for both paths.  It detects symbolic vs hybrid from the
    #  artifacts the child just wrote, which is the same test this gate makes
    #  itself -- so the two cannot disagree about which path a robot took.
    good, n, note, path = check(name, resolve=False, verbose=verbose)
    if n == 0:
        return 'could not check -- %s' % note, False

    detail = ' (%s)' % note if note else ''
    what = ('refined poses reach the true arm' if path == 'hybrid'
            else 'branches reproduce the pose')

    #  'hybrid' as the expectation means "at least one pose must survive";  an
    #  integer means "at least this many", as recorded in KNOWN_GOOD.
    floor = 1 if want == 'hybrid' else want
    if good < floor:
        return ('%d/%d %s (expected %s)%s'
                % (good, n, what, floor, detail), False)
    return '%d/%d %s%s' % (good, n, what, detail), True


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('robots', nargs='*',
                    help='robots to gate (default: all five)')
    ap.add_argument('--child', metavar='NAME',
                    help=argparse.SUPPRESS)   # internal: solve one robot
    ap.add_argument('--list', action='store_true',
                    help='print what each robot stands for, and exit')
    ap.add_argument('--timeout', type=int, default=DEFAULT_TIMEOUT,
                    help='per-robot seconds (default %d)' % DEFAULT_TIMEOUT)
    ap.add_argument('--logdir', default=os.path.join('logs', 'bt_path_gate'),
                    help='where to put each child\'s output')
    ap.add_argument('--verbose', action='store_true',
                    help='say why each failing IK branch failed')
    a = ap.parse_args(argv)

    if a.child:
        return run_child(a.child)

    if a.list:
        print('')
        for n in GATE_ROBOTS:
            w = EXPECTED[n]
            print('  %-16s %s' % (n, w['why']))
            arts = ', '.join(sorted(w['artifacts'])) or 'NOTHING'
            darts = ', '.join(sorted(w['derived_artifacts']))
            print('  %-16s   %s branch, %d/%d unknowns, writes %s'
                  % ('', w['branch'], w['solved'][0], w['solved'][1], arts))
            if darts:
                print('  %-16s   and %s as %s' % ('', darts, w['derived']))
            if w['closed_loop'] is not None:
                print('  %-16s   closed loop: %s' % ('', w['closed_loop']))
        print('')
        return 0

    names = a.robots or GATE_ROBOTS
    unknown = [n for n in names if n not in EXPECTED]
    if unknown:
        print('  not gated robots: %s' % ', '.join(unknown))
        print('  the gate asserts a recorded expectation per robot;  add one to')
        print('  EXPECTED in scripts/bt_path_gate.py before gating a new arm.')
        return 2

    try:
        os.makedirs(a.logdir, exist_ok=True)
        logdir = a.logdir
    except OSError:
        logdir = None

    print('')
    print('  Does every path through the BT deliver what that path owes?')
    print('  Each robot is solved once, with codegen ON, in its own process.')
    print('')
    print('  %-16s %-9s %-7s %-22s %7s' %
          ('robot', 'branch', 'solved', 'artifacts', 'wall_s'))
    print('  ' + '-' * 68)

    failures = []
    t0 = time.time()
    for name in names:
        rec = solve_in_child(name, a.timeout, logdir)
        bad = judge(name, rec)
        want = EXPECTED[name]

        if rec['status'] == 'ok':
            arts = ', '.join(rec['written']) if rec['written'] else '(none, as required)'
        else:
            arts = rec['status']

        print('  %-16s %-9s %-7s %-22s %7.1f'
              % (name, rec['branch'] or '-',
                 '%d/%d' % (rec['n_solved'], rec['n_unknowns']),
                 arts, rec['wall_s']))

        #  The closed loop only runs on a path that produced python AND got
        #  that far cleanly -- checking generated code after a wiring failure
        #  reports the same defect twice and buries the first one.
        if not bad and want['closed_loop'] is not None:
            line, ok = closed_loop(name, want['closed_loop'], verbose=a.verbose)
            print('  %-16s   closed loop: %s' % ('', line))
            if not ok:
                bad.append(line)

        for b in bad:
            print('  %-16s   <-- %s' % ('', b))
        if bad:
            failures.append((name, bad))

    print('  ' + '-' * 68)
    print('  %d robots, %.1f s total' % (len(names), time.time() - t0))

    if failures:
        print('')
        print('  GATE FAILED -- %d of %d paths did not deliver:'
              % (len(failures), len(names)))
        for name, bad in failures:
            for b in bad:
                print('    %-16s %s' % (name, b))
        print('')
        print('  If this is a DELIBERATE change of contract -- most likely')
        print('  hybrid_stub being replaced, so the hybrid paths start')
        print('  producing files -- update EXPECTED in this file and say in')
        print('  the commit message what the hybrid path now owes the user.')
        print('')
        return 1

    print('')
    print('  GATE PASSED -- every path delivered what it owes.')
    print('')
    return 0


if __name__ == '__main__':
    sys.exit(main())
