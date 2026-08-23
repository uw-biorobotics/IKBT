#!/usr/bin/python
#
#   robot_baseline.py --  record what IKBT actually solves, for every robot,
#                         and diff two such records.
#
#   This is test FIXTURING, not a test.  It asserts nothing about which robots
#   ought to solve:  "does not solve" is a legitimate and expected entry.  The
#   deliverable is a record you can diff, so that a change to the behavior tree
#   can be shown to have moved exactly the robots it was supposed to move and
#   nothing else.
#
#       python3 -m scripts.robot_baseline                    # capture a record
#       python3 -m scripts.robot_baseline --diff             # capture + compare
#       python3 -m scripts.robot_baseline --robots Puma Wrist # just these two
#
#   Exit status of a --diff run is 0 when nothing moved and 1 when something
#   did, so it works as a gate in a shell script.
#
#   ------------------------------------------------------------------------
#   Two things about IKBT shape this design:
#
#   ONE SUBPROCESS PER ROBOT.  Several functions on the unhappy path terminate
#   the *process* rather than returning an error -- Num_check() (pykinsym.py),
#   get_variable_index() (ik_classes.py), robot_params() on an unknown name
#   (ik_robots.py).  In-process iteration over all robots would therefore stop
#   at the first robot that trips one.  A child that dies is recorded as a
#   'crash' row and the sweep carries on.
#
#   PYTHONHASHSEED=0 IN THE CHILD.  Parts of the solver iterate over sets, so
#   the choice between equally-good solutions -- and hence unknown.solvemethod
#   -- can vary run to run under hash randomization.  Pinning the seed is what
#   makes an empty diff mean "nothing changed" instead of "nothing changed,
#   probably".
#   ------------------------------------------------------------------------
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

from ikbtfunctions.ik_robots import ROBOT_LIST


#  Where the checked-in record lives.  It is fixturing, so it sits with the
#  tests rather than in the repo root.
DEFAULT_RECORD = os.path.join('tests', 'baselines', 'robot_baseline.json')
DEFAULT_SUMMARY = os.path.join('tests', 'baselines', 'robot_baseline.txt')
DEFAULT_LOGDIR = os.path.join('logs', 'baseline')

#  Bump when a field that diff_records() compares changes meaning, so an old
#  record is rejected loudly instead of producing a nonsense diff.
RECORD_VERSION = 1

#  A full solve is minutes for the slow robots (FK on a cold pickle plus the
#  BT itself).  This is a backstop against a hang, not a performance target.
#
#  RAISED FROM 900 s, because at 900 s it made the record NON-REPRODUCIBLE.
#  Wall time on the slow robots varies far more than the rest:  measured over
#  two back-to-back sweeps of all 32, every robot came in at 1.0-1.1x except
#  DZhang (20.7 s -> 78.9 s, 3.8x) and Issue4, which landed at 187 s, 246 s and
#  then >900 s on identical code at PYTHONHASHSEED=0.  Issue4 sat close enough
#  to the ceiling that its recorded STATUS flipped between 'partial (hybrid)'
#  and 'timeout' run to run -- and `status` is compared, so --diff reported a
#  regression that did not exist.  A flaky gate is worse than a slow one:  it
#  trains you to ignore it.
#
#  1800 s gives Issue4 (typically ~250 s) roughly a 7x margin, which covers the
#  3.8x variance actually observed.  It remains a hang backstop:  nothing in the
#  set legitimately approaches it.  Override per run with --timeout.
DEFAULT_TIMEOUT = 1800

#  The child hands its result back on stdout, fenced, because everything else
#  in IKBT prints freely and the solver output is thousands of lines.
FENCE_OPEN = '#### BASELINE-JSON-BEGIN ####'
FENCE_CLOSE = '#### BASELINE-JSON-END ####'


def robot_names(subset=None):
    '''The robots to sweep, de-duplicated, in ROBOT_LIST order.

       ROBOT_LIST has been de-duplicated at the source, but this stays
       defensive:  a duplicate costs a whole extra solve and makes the record
       ambiguous (two rows, one key).'''

    if subset:
        wanted = list(subset)
    else:
        wanted = list(ROBOT_LIST)

    seen = set()
    out = []
    for n in wanted:
        if n not in seen:
            seen.add(n)
            out.append(n)
    return out


###############################################################################
#
#    The child:  solve ONE robot and describe what happened
#

def solve_one(name):
    '''Solve one robot and return a plain-data record of the outcome.

       Runs in the child process.  Raises nothing: every failure mode becomes a
       field in the returned dict.'''

    from ikbtfunctions.ik_driver import load_robot, run_solver
    from ikbtfunctions.bt_assembly import build_default_bt

    rec = {'name': name,
           'status': 'crash',
           'error': None,
           'n_unknowns': 0,
           'n_solved': 0,
           'no_progress': False,
           'methods': {},
           'n_solutions': None,
           'solution_set_error': None,
           'comp_det_ticks': 0,
           'hybrid': None,
           'wall_s': 0.0}

    t0 = time.time()
    try:
        M, R, unknowns = load_robot(name)

        bt, nodes = build_default_bt()

        #  comp_det sleeps 2 s per tick so a human can read the status as it
        #  scrolls past.  On Puma that is most of the wall clock, and no human
        #  is reading a sweep.
        nodes['compDetect'].read_pause = 0

        #  create_solutions=False on purpose:  the solve and the solution-set
        #  construction are recorded SEPARATELY.  create_solution_set() (and
        #  make_LHS_versions() under it) can throw on a partial solve, and if
        #  that took down the whole row we would lose the solve result -- which
        #  is the thing being measured.
        R, unks, bb = run_solver(R, unknowns, bt, create_solutions=False)

        rec['no_progress'] = bool(bb.get('no_progress'))
        rec['comp_det_ticks'] = int(getattr(nodes['compDetect'], 'N_ticks_all', 0))

        #  Order matters here: the unknown list is in chain order, and
        #  kinematics_pickle() may have EXTENDED it with sum-of-angles
        #  variables (th_23 and friends), which count as unknowns like any
        #  other.
        methods = {}
        nsolved = 0
        for u in unks:
            methods[str(u.symbol)] = str(u.solvemethod) if u.solved else ''
            if u.solved:
                nsolved += 1

        rec['methods'] = methods
        rec['n_unknowns'] = len(unks)
        rec['n_solved'] = nsolved

        if not rec['no_progress']:
            try:
                R.create_solution_set()
                rec['n_solutions'] = len(R.solutionSet)
            except Exception as e:
                #  Recorded, not raised.  A robot whose solution set cannot be
                #  built is a real and interesting outcome, distinct from a
                #  robot that solved nothing.
                rec['solution_set_error'] = '%s: %s' % (type(e).__name__, e)

        #  Did the hybrid branch take over?  If so the equations describe a
        #  DERIVED arm, not the one we asked for, and recording that as plain
        #  'solved' would be the single most misleading thing this file could
        #  do -- the whole point of the record is to say what IKBT can actually
        #  deliver for a named robot.
        hs = bb.get('hybrid_source')
        if hs:
            rec['hybrid'] = {
                'derived_robot': hs.get('derived_robot'),
                'axes': list(hs.get('axes') or []),
                'kind': hs.get('kind'),
                'route': hs.get('route'),
                'edits': ['%s: %s -> %s' % (e['symbol'], e['from'], e['to'])
                          for e in hs.get('edits') or []],
            }

        suffix = ' (hybrid)' if hs else ''
        if rec['n_unknowns'] and nsolved == rec['n_unknowns']:
            rec['status'] = 'solved' + suffix
        elif nsolved:
            rec['status'] = 'partial' + suffix
        else:
            rec['status'] = 'unsolved'

    except SystemExit as e:
        #  quit() somewhere on the unhappy path.  This is exactly why the sweep
        #  forks per robot;  in-process it would have ended the whole run.
        rec['status'] = 'crash'
        rec['error'] = 'SystemExit(%s) -- a quit() on the unhappy path' % (e.code,)
        rec['traceback'] = traceback.format_exc()
    except BaseException as e:
        rec['status'] = 'crash'
        rec['error'] = '%s: %s' % (type(e).__name__, e)
        #  Diagnostic only -- 'traceback' is deliberately absent from COMPARED,
        #  so it never registers as a change.  'MiniDD crashes' is the useful
        #  baseline fact;  WHERE it crashes is what you need to go fix it.
        rec['traceback'] = traceback.format_exc()

    rec['wall_s'] = round(time.time() - t0, 2)
    return rec


def run_child(name):
    '''Child entry point:  solve `name`, print the fenced record.'''

    rec = solve_one(name)
    sys.stdout.flush()
    print(FENCE_OPEN)
    print(json.dumps(rec, sort_keys=True))
    print(FENCE_CLOSE)
    sys.stdout.flush()
    return 0


def parse_child_output(text):
    '''Pull the fenced record out of a child's stdout, or None if absent.'''

    if FENCE_OPEN not in text:
        return None
    body = text.split(FENCE_OPEN, 1)[1]
    if FENCE_CLOSE not in body:
        return None
    body = body.split(FENCE_CLOSE, 1)[0]
    try:
        return json.loads(body.strip())
    except ValueError:
        return None


###############################################################################
#
#    The parent:  sweep
#

def sweep_one(name, timeout=DEFAULT_TIMEOUT, logdir=DEFAULT_LOGDIR):
    '''Solve one robot in a child process and return its record.

       A crash or a timeout comes back as a record, never as an exception.'''

    env = dict(os.environ)
    env['PYTHONHASHSEED'] = '0'          # see the header

    cmd = [sys.executable, '-m', 'scripts.robot_baseline', '--child', name]

    t0 = time.time()
    timed_out = False
    try:
        p = subprocess.run(cmd, cwd=os.getcwd(), env=env,
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                           timeout=timeout)
        out = p.stdout.decode('utf-8', errors='replace')
        code = p.returncode
    except subprocess.TimeoutExpired as e:
        out = (e.output or b'').decode('utf-8', errors='replace')
        code = None
        timed_out = True
    wall = round(time.time() - t0, 2)

    if logdir:
        if not os.path.isdir(logdir):
            os.makedirs(logdir)
        with open(os.path.join(logdir, name + '.log'), 'w') as f:
            f.write(out)

    rec = parse_child_output(out)

    if rec is None:
        #  No fenced record: the child died before it could report, or was
        #  killed by the timeout.  Keep the tail of its output -- that is the
        #  only diagnosis available.
        tail = '\n'.join(out.strip().splitlines()[-15:])
        rec = {'name': name,
               'status': 'timeout' if timed_out else 'crash',
               'error': ('no result after %s s' % timeout) if timed_out
                        else ('child exited %s with no result' % code),
               'n_unknowns': 0, 'n_solved': 0, 'no_progress': False,
               'methods': {}, 'n_solutions': None, 'solution_set_error': None,
               'comp_det_ticks': 0, 'wall_s': wall,
               'child_tail': tail}

    rec['wall_s'] = wall                 # parent's clock includes interpreter start
    return rec


def sweep(names, timeout=DEFAULT_TIMEOUT, logdir=DEFAULT_LOGDIR, progress=True):
    '''Solve every robot in `names`, one child each.  Returns a record dict.'''

    robots = {}
    t0 = time.time()
    for i, name in enumerate(names, start=1):
        if progress:
            print('[%2d/%2d] %-18s ' % (i, len(names), name), end='', flush=True)
        rec = sweep_one(name, timeout=timeout, logdir=logdir)
        robots[name] = rec
        if progress:
            note = ''
            if rec['status'] in ('crash', 'timeout'):
                note = '   <- ' + str(rec.get('error'))
            print('%-9s %2d/%-2d vars  %7.1fs%s'
                  % (rec['status'], rec['n_solved'], rec['n_unknowns'],
                     rec['wall_s'], note), flush=True)

    return {'record_version': RECORD_VERSION,
            'n_robots': len(robots),
            'total_wall_s': round(time.time() - t0, 2),
            'robots': robots}


###############################################################################
#
#    Diff
#

#  Fields compared.  wall_s and comp_det_ticks are deliberately NOT here:
#  wall time is noise, and the comp_det tick count is a measurement of the
#  tree's shape -- restructuring the outer loop is expected to change it
#  without changing a single solution.  Both are reported, neither is a change.
COMPARED = ['status', 'n_solved', 'n_unknowns', 'methods', 'n_solutions',
            'solution_set_error', 'hybrid']

#  For deciding whether a status change is an improvement or a regression.
#  'solved (hybrid)' ranks BELOW 'solved':  it is a real improvement over
#  unsolved, so unsolved -> solved (hybrid) classifies as newly-solved, but a
#  robot that moves from an exact closed form to a simplified one has REGRESSED
#  and must not be reported as an improvement.
RANK = {'crash': 0, 'timeout': 0, 'unsolved': 1,
        'partial (hybrid)': 2, 'partial': 3,
        'solved (hybrid)': 4, 'solved': 5}


def _hybrid_str(rec):
    """'no' or 'KinovaLite_d5_0 (d_5: 57 -> 0)' for a record's hybrid field."""
    h = rec.get('hybrid')
    if not h:
        return 'no simplification'
    return '%s (%s)' % (h.get('derived_robot'), '; '.join(h.get('edits') or []))


def classify(old, new):
    '''How one robot moved between two records.  Returns (verdict, detail).'''

    if old['status'] != new['status']:
        ro, rn = RANK.get(old['status'], 0), RANK.get(new['status'], 0)
        detail = '%s -> %s' % (old['status'], new['status'])
        if rn > ro:
            return 'newly-solved', detail
        if rn < ro:
            return 'newly-unsolved', detail
        return 'changed-status', detail

    if old['n_solved'] != new['n_solved']:
        return ('changed-count',
                'solved %d/%d -> %d/%d' % (old['n_solved'], old['n_unknowns'],
                                           new['n_solved'], new['n_unknowns']))

    if old['methods'] != new['methods']:
        bits = []
        for k in sorted(set(old['methods']) | set(new['methods'])):
            a, b = old['methods'].get(k), new['methods'].get(k)
            if a != b:
                bits.append('%s: %r -> %r' % (k, a, b))
        return 'changed-method', '; '.join(bits)

    if old['n_solutions'] != new['n_solutions']:
        return ('changed-solutions',
                '%s -> %s solutions' % (old['n_solutions'], new['n_solutions']))

    if old['solution_set_error'] != new['solution_set_error']:
        return ('changed-solutions',
                'solution set error: %r -> %r' % (old['solution_set_error'],
                                                  new['solution_set_error']))

    #  A change in WHICH arm the equations describe, at an unchanged status --
    #  e.g. the same robot simplified a different way.  .get() throughout:  a
    #  record captured before this field existed simply has no 'hybrid' key, and
    #  an old baseline must stay diffable rather than raising KeyError.
    if old.get('hybrid') != new.get('hybrid'):
        return ('changed-simplification',
                '%s -> %s' % (_hybrid_str(old), _hybrid_str(new)))

    #  Backstop:  anything in COMPARED that is not hand-checked above.  COMPARED
    #  used to be documentation that only LOOKED like configuration -- adding a
    #  field to it changed nothing.  Now it is authoritative, so a future field
    #  cannot be silently uncompared.
    for f in COMPARED:
        if old.get(f) != new.get(f):
            return 'changed-other', '%s: %r -> %r' % (f, old.get(f), new.get(f))

    return 'unchanged', ''


def diff_records(old, new):
    '''Classify every robot in two records.  Returns a list of
       (verdict, name, detail), interesting verdicts first.'''

    o, n = old['robots'], new['robots']
    rows = []

    for name in sorted(set(o) | set(n)):
        if name not in o:
            rows.append(('added', name, n[name]['status']))
        elif name not in n:
            rows.append(('removed', name, o[name]['status']))
        else:
            verdict, detail = classify(o[name], n[name])
            rows.append((verdict, name, detail))

    order = ['newly-unsolved', 'newly-solved', 'changed-status',
             'changed-count', 'changed-method', 'changed-simplification',
             'changed-solutions', 'changed-other',
             'added', 'removed', 'unchanged']
    rows.sort(key=lambda r: (order.index(r[0]) if r[0] in order else 99, r[1]))
    return rows


def print_diff(rows):
    '''Print the diff.  Returns the number of robots that moved.'''

    moved = [r for r in rows if r[0] != 'unchanged']

    print('')
    print('=' * 74)
    print('  Baseline diff')
    print('=' * 74)

    if not moved:
        print('  %d robots, all unchanged.' % len(rows))
        print('=' * 74)
        return 0

    for verdict, name, detail in moved:
        print('  %-18s %-18s %s' % (verdict, name, detail))

    print('-' * 74)
    print('  %d moved, %d unchanged, %d total'
          % (len(moved), len(rows) - len(moved), len(rows)))
    print('=' * 74)
    return len(moved)


###############################################################################
#
#    Human-readable summary
#

def format_summary(record):
    '''The same data as the JSON, laid out for a person.'''

    lines = []
    robots = record['robots']

    lines.append('IKBT robot baseline')
    lines.append('%d robots, %.1f s total' % (record['n_robots'],
                                              record['total_wall_s']))
    lines.append('')
    lines.append('%-18s %-16s %7s %7s %8s %8s'
                 % ('robot', 'status', 'solved', 'solns', 'cd_ticks', 'wall_s'))
    lines.append('-' * 70)

    counts = {}
    for name in sorted(robots):
        r = robots[name]
        counts[r['status']] = counts.get(r['status'], 0) + 1
        lines.append('%-18s %-16s %3d/%-3d %7s %8d %8.1f'
                     % (name, r['status'], r['n_solved'], r['n_unknowns'],
                        '-' if r['n_solutions'] is None else r['n_solutions'],
                        r['comp_det_ticks'], r['wall_s']))

    lines.append('-' * 62)
    lines.append('  ' + ',  '.join('%s: %d' % (k, counts[k])
                                   for k in sorted(counts)))

    #  Anything that did not simply solve gets spelled out.  This is the part
    #  a person actually reads.
    lines.append('')
    lines.append('Notes')
    lines.append('-' * 62)
    quiet = True
    for name in sorted(robots):
        r = robots[name]
        if r.get('error'):
            lines.append('%-18s %s' % (name, r['error']))
            quiet = False
        if r.get('solution_set_error'):
            lines.append('%-18s solution set: %s' % (name, r['solution_set_error']))
            quiet = False
        if r.get('hybrid'):
            lines.append('%-18s SIMPLIFIED to %s -- %s (axes %s, %s)'
                         % (name, r['hybrid']['derived_robot'],
                            '; '.join(r['hybrid']['edits']),
                            r['hybrid']['axes'], r['hybrid']['kind']))
            quiet = False
        if r['status'].startswith('partial'):
            unsolved = [k for k, v in sorted(r['methods'].items()) if not v]
            lines.append('%-18s unsolved: %s' % (name, ', '.join(unsolved)))
            quiet = False
    if quiet:
        lines.append('(none)')

    #  Per-variable methods:  the field most likely to move when a solver leaf
    #  changes, so it is worth having in the readable file too.
    lines.append('')
    lines.append('Solution methods')
    lines.append('-' * 62)
    for name in sorted(robots):
        r = robots[name]
        if not r['methods']:
            continue
        lines.append(name)
        for k in sorted(r['methods']):
            lines.append('    %-8s %s' % (k, r['methods'][k] or '(unsolved)'))

    return '\n'.join(lines) + '\n'


###############################################################################
#
#    Record I/O
#

def load_record(path):
    with open(path) as f:
        rec = json.load(f)
    got = rec.get('record_version')
    if got != RECORD_VERSION:
        raise ValueError('%s is record_version %r, this script writes %r -- '
                         'recapture it rather than diffing across versions'
                         % (path, got, RECORD_VERSION))
    return rec


def save_record(record, path, summary_path=None):
    d = os.path.dirname(path)
    if d and not os.path.isdir(d):
        os.makedirs(d)
    with open(path, 'w') as f:
        json.dump(record, f, indent=1, sort_keys=True)
        f.write('\n')
    print('  wrote ' + path)

    if summary_path:
        with open(summary_path, 'w') as f:
            f.write(format_summary(record))
        print('  wrote ' + summary_path)


###############################################################################
#
#    CLI
#

def main(argv=None):
    ap = argparse.ArgumentParser(
        description='Record what IKBT solves for every robot, and diff two '
                    'such records.  Asserts nothing -- "does not solve" is a '
                    'legitimate entry.')

    ap.add_argument('--child', metavar='ROBOT',
                    help='internal: solve one robot and print its record')
    ap.add_argument('--robots', nargs='+', metavar='NAME',
                    help='sweep only these robots (default: all of ROBOT_LIST)')
    ap.add_argument('--list', action='store_true',
                    help='print the robots that would be swept, and stop')
    ap.add_argument('-o', '--out', default=DEFAULT_RECORD,
                    help='where to write the record (default: %s)' % DEFAULT_RECORD)
    ap.add_argument('--summary', default=None,
                    help='where to write the readable summary '
                         '(default: alongside --out, .txt)')
    ap.add_argument('--no-save', action='store_true',
                    help='do not write anything (use with --diff)')
    ap.add_argument('--diff', nargs='?', const=DEFAULT_RECORD, metavar='OLD',
                    help='compare against OLD (default: %s).  Exit 1 if '
                         'anything moved.' % DEFAULT_RECORD)
    ap.add_argument('--new', metavar='NEW',
                    help='with --diff: compare OLD against this existing '
                         'record instead of running a fresh sweep')
    ap.add_argument('--timeout', type=float, default=DEFAULT_TIMEOUT,
                    help='per-robot wall-clock limit, seconds (default: %d)'
                         % DEFAULT_TIMEOUT)
    ap.add_argument('--no-logs', action='store_true',
                    help='do not keep per-robot child output under %s'
                         % DEFAULT_LOGDIR)

    a = ap.parse_args(argv)

    if a.child:
        return run_child(a.child)

    names = robot_names(a.robots)

    if a.list:
        for n in names:
            print(n)
        print('(%d robots)' % len(names))
        return 0

    #  --diff --new: pure offline comparison, no solving at all.
    if a.new:
        if not a.diff:
            ap.error('--new only makes sense with --diff')
        old = load_record(a.diff)
        new = load_record(a.new)
        return 1 if print_diff(diff_records(old, new)) else 0

    #  A --diff run loads the old record BEFORE the sweep, so a bad path or a
    #  stale record_version fails in a second rather than after an hour.
    old = load_record(a.diff) if a.diff else None

    print('')
    print('Sweeping %d robots, %s s each at most, PYTHONHASHSEED=0'
          % (len(names), a.timeout))
    print('')

    record = sweep(names, timeout=a.timeout,
                   logdir=None if a.no_logs else DEFAULT_LOGDIR)

    print('')
    print(format_summary(record))

    if not a.no_save:
        summary = a.summary
        if summary is None:
            summary = (os.path.splitext(a.out)[0] + '.txt'
                       if a.out != DEFAULT_RECORD else DEFAULT_SUMMARY)
        save_record(record, a.out, summary)

    if old is not None:
        rows = diff_records(old, record)
        #  A subset sweep cannot speak for the robots it did not run.
        if a.robots:
            keep = set(names)
            rows = [r for r in rows if r[1] in keep]
            print('  (subset run: only %d robots compared)' % len(keep))
        return 1 if print_diff(rows) else 0

    return 0


if __name__ == '__main__':
    sys.exit(main())
