#!/usr/bin/python
#
#   progress.py --  tell the user what a long solve is doing
#
#   A full IK solve runs from 1 s (Wrist) to 700 s (Issue4 on the hybrid branch).
#   For most of that time the old output was silent, so the only honest question
#   a user could ask -- "is this working, or is it stuck?" -- had no answer.
#   comp_detect's `read_pause = 2` was the previous attempt at this:  sleep two
#   seconds per pass so a human can read the status wall as it scrolls by.  That
#   costs ~18 s of an interactive Puma's ~27 s and still does not say whether
#   anything is improving.  One compact line per pass replaces it.
#
#   WHAT MAKES THIS TRUSTWORTHY:  the solved count is MONOTONIC.  set_solved()
#   never un-solves a variable, so the count can only rise.  If it is rising, the
#   solver is working.  If it is flat, the solver is spinning -- but *bounded*,
#   because symbolic_loop's budget is finite, so the line can say exactly how
#   many passes remain before it gives up.  A user never has to guess whether to
#   keep waiting.
#
#   ON EXPRESSION SIZE, AND WHY THIS MEASURES CALL COUNT INSTEAD.  The first
#   design printed sp.count_ops() of each expression before handing it to sympy,
#   on the theory that size predicts runtime.  Measured over Puma, KR16 and
#   KinovaLite, it does not:
#
#       robot        wall    simplify calls   % wall in simplify   max count_ops
#       Puma         13 s        22                52 %                33
#       KR16         40 s        69                85 %                46
#       KinovaLite   49 s       746                37 %                44
#
#   EVERY expression IKBT simplifies is under 50 ops, on every robot -- while the
#   call count varies 34x.  Size does not discriminate a 0.025 s call from a
#   2.3 s one, and it never grows, so printing it before each call would be a
#   constant.  Long runtime here is thousands of small sympy calls, not a few
#   enormous ones, so the meter counts CALLS and CUMULATIVE SECONDS, which is
#   what actually tracks wall clock.  Individual slow calls are still reported
#   as they happen (see slow_call_s) -- those are the ones worth naming, and
#   their size is printed there, where it is diagnostic rather than noise.
#
#   Output is deliberately LINE-ORIENTED, never carriage-return animated.
#   scripts/robot_baseline.py captures every solve to logs/baseline/<robot>.log,
#   and the existing sum-of-angles progress bar collapses into one unreadable
#   multi-kilobyte line there -- which is what made Issue4's log useless for
#   diagnosis.  Do not add \r animation to this file.
#
#   Copyright 2026 University of Washington
#
#   Developed by Blake Hannaford
#   BioRobotics Lab, University of Washington

import time


def fmt_time(s):
    '''Seconds as something a human reads without counting digits.

       Minutes above 90 s because "2.5 min" is graspable and "150 s" is not.'''

    if s is None:
        return '?'
    if s < 90:
        return '%.0fs' % s
    return '%.1f min' % (s / 60.0)


def _say(msg):
    """print(), but FLUSHED.

       Python buffers stdout whenever it is not a tty, so every line in this
       file would otherwise be withheld until the process exits -- which is
       exactly backwards for a progress reporter.  It matters in three real
       cases:  scripts/robot_baseline.py captures each solve to
       logs/baseline/<robot>.log, `python3 ikSolver.py X | tee log` is the
       obvious way to keep a record of a long run, and an editor/IDE terminal
       is often not a tty either.  Measured:  without the flush, an Issue4 solve
       printed NOTHING for 12 minutes and then everything at once.
    """

    print(msg, flush=True)


#####################################################################
#
#   The sympy meter.
#
#   sp.simplify() is 37-85 % of a solve's wall clock (table above), and the
#   calls are scattered over a dozen leaves.  Counting them at the call sites
#   would mean touching every one and would rot the moment a new leaf is added,
#   so the meter wraps sp.simplify and sp.trigsimp once, here.
#
#   It is OPT-IN and reversible.  Rebinding a name inside a third-party module
#   is not something to do implicitly:  enable_sympy_meter() is called by ikSolver.py, where a
#   human is waiting and wants the feedback.  Nothing else turns it on.
#

_meter = {'calls': 0, 'seconds': 0.0, 'on': False, 'orig': None,
          'slow': [], 'slow_call_s': 2.0, 'depth': 0}


def enable_sympy_meter(slow_call_s=2.0):
    '''Start counting expensive sympy calls and the time spent in them.

       slow_call_s -- a single call taking longer than this is reported on its
       own line as it happens.  That line is the live heartbeat during a long
       pass:  nothing else can print from inside a blocking sympy call, so
       without it the solver goes silent for however long the call takes.

       WHAT IT PATCHES, AND WHY IT IS NOT Basic.simplify (BH, 2026-09-04).
       This used to wrap the METHOD, Basic.simplify -- and the solver leaves
       almost never call it that way:

           sp.simplify(...)     26 call sites        <- the function
           sp.trigsimp(...)     12
           .simplify(...)        6 call sites        <- the method

       so the meter counted a fraction of the work and, worse, the heartbeat
       stayed silent through the long calls.  Puma would print its pass-1 line
       and then say nothing for minutes while pinned at 100% CPU, which is
       indistinguishable from being hung -- and was reported as exactly that.

       Basic.simplify does `from sympy.simplify.simplify import simplify` and
       calls it, so patching the FUNCTION catches the method too;  the method
       itself is deliberately left alone to avoid counting a call twice.

       The name has to be replaced in BOTH places it is reachable from:
       `sympy.simplify` (what `sp.simplify(...)` resolves) and
       `sympy.simplify.simplify.simplify` (what the method imports).  They are
       the same function object, but they are two separate bindings.'''

    if _meter['on']:
        return

    import importlib
    import sympy as sp

    _meter['slow_call_s'] = slow_call_s
    _meter['orig'] = []

    #  (module, attribute) pairs to rebind, per metered function.
    targets = []
    for name, modpath in (('simplify', 'sympy.simplify.simplify'),
                          ('trigsimp', 'sympy.simplify.trigsimp')):
        try:
            sub = importlib.import_module(modpath)
        except ImportError:
            continue
        targets.append((name, [(sp, name), (sub, name)]))

    for name, places in targets:
        orig = getattr(sp, name, None)
        if orig is None:
            continue

        def make(orig, name):
            def metered(expr, *args, **kwargs):
                #  ONLY THE OUTERMOST CALL IS COUNTED.  simplify() calls
                #  trigsimp() internally, and both are metered, so a naive
                #  counter charges the same seconds twice and reports a call
                #  count that has nothing to do with the call sites.  Measured:
                #  three calls at the top level came back as seven.
                if _meter['depth']:
                    return orig(expr, *args, **kwargs)

                _meter['depth'] += 1
                t0 = time.time()
                try:
                    return orig(expr, *args, **kwargs)
                finally:
                    _meter['depth'] -= 1
                    dt = time.time() - t0
                    _meter['calls'] += 1
                    _meter['seconds'] += dt
                    if dt >= _meter['slow_call_s']:
                        #  count_ops only here:  it is itself a tree walk, so it
                        #  is not free enough to call on every one of
                        #  KinovaLite's hundreds of calls.
                        try:
                            n = int(sp.count_ops(expr))
                        except Exception:
                            n = -1
                        _meter['slow'].append((dt, n))
                        _say('        ... slow %s: %.1fs on a %d-operation '
                             'expression (still working)' % (name, dt, n))
            return metered

        m = make(orig, name)
        for mod, attr in places:
            _meter['orig'].append((mod, attr, getattr(mod, attr, None)))
            setattr(mod, attr, m)

    _meter['on'] = True


def disable_sympy_meter():
    '''Put every patched name back.  Tests that assert on timing want this.'''

    if not _meter['on']:
        return
    for mod, attr, orig in reversed(_meter['orig'] or []):
        if orig is not None:
            setattr(mod, attr, orig)
    _meter['orig'] = []
    _meter['on'] = False


def meter_stats():
    '''(calls, seconds) since the meter was enabled.'''
    return _meter['calls'], _meter['seconds']


#####################################################################


class SolveProgress(object):
    '''One of these per symbolic solve.  symbolic_loop owns it.

       The contract is one banner at the start and one (or two) lines per pass.
       Nothing here decides anything -- it only reports -- so it is always safe
       to leave on, and `enabled = False` silences it completely for tests.'''

    def __init__(self, robot_name, max_passes, n_unknowns, enabled=True,
                 tag=''):
        self.robot = robot_name or '?'
        self.max_passes = max_passes
        self.n = n_unknowns
        self.enabled = enabled
        self.tag = tag                  # ' (hybrid)' for the second solver

        self.t_start = time.time()
        self.t_last = self.t_start
        #  The meter is process-global and cumulative, but each SolveProgress
        #  covers ONE solve.  On the hybrid branch there are two solves in one
        #  process, so reporting the raw cumulative totals against this solve's
        #  elapsed time mixes scopes and can print "100% of wall" or worse.
        #  Snapshot here and report deltas.
        self._meter0 = meter_stats()
        self.last_solved = 0
        #  NAMES, not just a count.  The newly-solved variable is not
        #  necessarily last in the unknown list -- the assigner round-robins, so
        #  it can be solved in any order.  Slicing a filtered list by the old
        #  count therefore names the wrong variable (it reported th_23 for five
        #  consecutive Puma passes).  A set difference is the only correct way.
        self.solved_names = set()
        self.last_pools = None
        self.flat_passes = 0            # consecutive passes with no change

    def _meter_delta(self):
        '''(calls, seconds) attributable to THIS solve.'''
        c, sec = meter_stats()
        return c - self._meter0[0], sec - self._meter0[1]

    #  ---------------------------------------------------------------- banner

    def banner(self):
        '''Say what is about to happen, how it is bounded, and to be patient.

           The bound is the reassuring part and it is a fact, not a hope:
           symbolic_loop runs at most max_passes passes, so this cannot spin
           forever no matter what the robot does.'''

        if not self.enabled or self.n == 0:
            return

        _say('')
        _say('  Solving %s%s:  %d unknowns, at most %d passes.'
              % (self.robot, self.tag, self.n, self.max_passes))
        _say('  This is symbolic algebra and it is SLOW -- typically a few')
        _say('  seconds to a few minutes, occasionally 10+ minutes.  Please be')
        _say('  patient.  A status line appears after every pass, and the')
        _say('  count of solved variables can only go UP, so you can always')
        _say('  tell whether it is making progress.')
        _say('')

    #  ------------------------------------------------------------- per pass

    def pass_done(self, passno, unknowns, pools=None):
        '''One line (sometimes two) summarising the pass that just finished.

           unknowns -- the live unknown list;  pools -- (n1u, n2u, n3pu) or None.

           Wrapped so that a reporting bug can never abort a solve.  This runs
           on the hot path of every pass of every robot, and a solve that took
           700 s must not be thrown away because a status line could not be
           formatted.  Observability is not allowed to be a failure mode.'''

        try:
            self._pass_done(passno, unknowns, pools)
        except Exception as e:
            _say('  (progress reporting failed -- %s: %s;  the solve continues)'
                 % (type(e).__name__, e))

    def _pass_done(self, passno, unknowns, pools=None):
        if not self.enabled or self.n == 0:
            return

        now = time.time()
        dt_pass = now - self.t_last
        dt_tot = now - self.t_start
        self.t_last = now

        #  getattr, not u.name:  this is REPORTING code on the hot path of every
        #  solve, and it must not be able to break one.  kin_cl.unknown always
        #  has .name, but test doubles and any future unknown-like object may
        #  not, and a progress line is never worth an AttributeError mid-solve.
        names = set(getattr(u, 'name', str(u))
                    for u in unknowns if getattr(u, 'solved', False))
        ns = len(names)
        gained = ns - self.last_solved
        newly = sorted(names - self.solved_names)

        pools_changed = (pools is not None and self.last_pools is not None
                         and pools != self.last_pools)

        #  The headline.
        line = '  pass %d/%d  solved %d/%d' % (passno, self.max_passes, ns, self.n)
        if newly:
            line += ' (+%d: %s)' % (len(newly), ', '.join(newly))
        if pools is not None:
            line += '  eqns 1u/2u/3pu %d/%d/%d' % pools
        line += '  %s this pass  %s total' % (fmt_time(dt_pass), fmt_time(dt_tot))
        _say(line)

        #  The interpretation -- this is the part that answers "is it stuck?".
        if newly:
            self.flat_passes = 0
            left_vars = self.n - ns
            #  NO TIME ESTIMATE (BH, 2026-09-04).  There used to be one here --
            #  "about 5s more, at most 16s" -- extrapolated from the cost per
            #  solved variable so far.  It was misleading to the point of being
            #  worse than silence, because the passes are nowhere near uniform:
            #  measured on Puma, passes 1 and 5-8 take about a second each while
            #  2-4 together take 2.7 minutes, so an estimate formed after pass 1
            #  said "about 6s more" for a solve that ran 2.8 minutes.  A count of
            #  what is left is a fact;  a projection from it is a guess dressed
            #  as one.
            if left_vars > 0:
                _say('            making progress -- %d variable%s left'
                     % (left_vars, '' if left_vars == 1 else 's'))
            else:
                _say('            making progress -- all variables solved')
        elif pools_changed:
            self.flat_passes = 0
            _say('            no new variable this pass, but the equation set '
                  'changed -- still working')
        else:
            self.flat_passes += 1
            left = self.max_passes - passno
            if left <= 0:
                _say('            nothing changed and the pass budget is spent '
                     '-- stopping')
            elif self.flat_passes >= 2:
                #  The strongest statement this reporter can make, and the one a
                #  waiting user most needs.  comp_det stops on the SECOND
                #  identical pass, so reaching two consecutive flat passes means
                #  the solve is genuinely stuck rather than merely slow -- and
                #  before the comp_det fix, this was the state Issue4 sat in for
                #  nine passes and 636 s while printing nothing alarming.
                _say('            %d passes in a row changed NOTHING -- this '
                     'solve is stuck, not slow;  it should stop now'
                     % self.flat_passes)
            else:
                #  "at most", because the budget is the OUTER bound and not
                #  usually what stops the solve:  comp_det ends it as soon as a
                #  pass changes nothing, which on KawasakiRS05L is pass 2 of 10.
                #  Promising 9 more passes there would be simply wrong.
                _say('            nothing changed this pass -- at most %d more '
                     'pass%s (comp_det usually stops sooner)'
                     % (left, '' if left == 1 else 'es'))

        #  Where the time is actually going, when the meter is on.
        calls, secs = self._meter_delta()
        if calls:
            _say('            sympy: %d simplify calls, %s in them (%.0f%% of '
                  'wall)' % (calls, fmt_time(secs), 100.0 * secs / max(dt_tot, 1e-9)))

        self.last_solved = ns
        self.solved_names = names
        self.last_pools = pools

    #  -------------------------------------------------------------- summary

    def finished(self, unknowns, exhausted):
        '''Closing line:  what was achieved and how long it took.

           Guarded for the same reason as pass_done -- see there.'''

        try:
            self._finished(unknowns, exhausted)
        except Exception as e:
            _say('  (progress summary failed -- %s: %s)'
                 % (type(e).__name__, e))

    def _finished(self, unknowns, exhausted):
        if not self.enabled or self.n == 0:
            return

        ns = len([u for u in unknowns if getattr(u, 'solved', False)])
        dt = time.time() - self.t_start

        if ns == self.n:
            verdict = 'COMPLETE -- all %d variables solved' % self.n
        elif ns > 0:
            verdict = 'PARTIAL -- %d of %d variables solved' % (ns, self.n)
        else:
            verdict = 'NO SOLUTION -- nothing was solved'

        _say('')
        _say('  %s%s: %s in %s.' % (self.robot, self.tag, verdict, fmt_time(dt)))
        if exhausted:
            _say('  (the %d-pass budget ran out;  comp_det did not stop it)'
                  % self.max_passes)
        calls, secs = self._meter_delta()
        if calls:
            _say('  sympy did %d simplify calls, %s of the %s total.'
                  % (calls, fmt_time(secs), fmt_time(dt)))
        _say('')


#####################################################################
#
#   Test code
#

import unittest


class test_unk(object):
    '''Minimal stand-in for ikbtbasics.kin_cl.unknown.'''

    def __init__(self, name, solved=False):
        self.name = name
        self.solved = solved


class TestSolver021(unittest.TestCase):
    '''Progress reporting.  These assert on the arithmetic and the monotonic
       bound, not on the wording.'''

    def setUp(self):
        _say('\n\n===============  Test progress reporting  =====================')
        return

    def runTest(self):
        '''tests/leavestest.py adds this class as `TestSolver021()`, which
           unittest resolves to `runTest` -- so every check has to be listed
           here as well as being discoverable standalone.  Repo convention;  see
           clear_state.TestSolver020.'''

        self.test_fmt_time()
        self.test_progress_line_states_what_is_left()
        self.test_flat_pass_counting()
        self.test_solved_count_is_read_from_the_unknowns()
        self.test_newly_solved_names_are_the_ones_that_changed()
        self.test_meter_is_reversible()
        self.test_meter_counts_and_is_off_by_default()

    #  ---- shared helper -------------------------------------------------

    def capture(self, fn, *args, **kwargs):
        '''Run fn(*args) with stdout captured, and return what it printed.

           _say() flushes to sys.stdout, which respects the reassignment, so
           the reporter is testable.

           Takes the arguments rather than a zero-argument thunk so callers read
           as `self.capture(p.pass_done, 1, unks, pools)` -- the call being
           captured stays visible instead of being wrapped in a lambda.'''

        import io
        import sys
        buf, real = io.StringIO(), sys.stdout
        sys.stdout = buf
        try:
            fn(*args, **kwargs)
        finally:
            sys.stdout = real
        return buf.getvalue()

    def test_fmt_time(self):
        fs = 'progress.fmt_time: '
        self.assertEqual(fmt_time(0), '0s', fs + 'zero')
        self.assertEqual(fmt_time(45), '45s', fs + 'seconds below the cutover')
        self.assertEqual(fmt_time(89), '89s', fs + 'just below 90')
        self.assertEqual(fmt_time(120), '2.0 min', fs + 'minutes above it')
        self.assertEqual(fmt_time(705), '11.8 min', fs + "Issue4's real time")
        self.assertEqual(fmt_time(None), '?', fs + 'unknown')

    def test_progress_line_states_what_is_left(self):
        '''The progress line reports a COUNT, and never a time estimate.

           There used to be an ETA here -- "about 5s more, at most 16s" -- and
           three tests pinning its arithmetic.  It was removed because the
           passes are nowhere near uniform:  on Puma, passes 1 and 5-8 run about
           a second each while 2-4 take 2.7 minutes together, so an estimate
           formed after pass 1 announced "about 6s more" for a solve that ran
           2.8 minutes.  What is tested now is that no such projection comes
           back -- a wrong estimate is worse than no estimate, because a reader
           acts on it.'''

        import io as _io
        import contextlib as _cl

        fs = ' progress line FAIL'
        p = SolveProgress('TestBot', n_unknowns=4, max_passes=10)
        p.enabled = True

        class U(object):
            def __init__(self, name, solved):
                self.name = name
                self.symbol = name
                self.solved = solved

        unks = [U('a', True), U('b', False), U('c', False), U('d', False)]

        buf = _io.StringIO()
        with _cl.redirect_stdout(buf):
            p.pass_done(1, unks, pools=(1, 2, 3))
        out = buf.getvalue()

        self.assertIn('3 variables left', out,
                      fs + ' (should say how many are outstanding)')
        for banned in ('at most', 'more,', 'about '):
            self.assertNotIn(banned, out,
                             fs + ' (a time estimate came back: %r)' % banned)

    def test_flat_pass_counting(self):
        '''Consecutive no-change passes are what "stuck but bounded" means, so
           the counter has to reset the moment anything moves.'''

        fs = 'progress.pass_done: '
        p = SolveProgress('R', 10, 4, enabled=True)
        a, b = test_unk('th_1'), test_unk('th_2')
        unks = [a, b, test_unk('th_3'), test_unk('th_4')]

        #  Two passes where nothing at all changes:  same solved set, same pools.
        out = self.capture(p.pass_done, 1, unks, (0, 2, 9))
        self.assertEqual(p.flat_passes, 1, fs + 'first flat pass counted')
        self.assertIn('at most 9 more passes', out,
                      fs + 'should state the remaining bound, not a promise')

        out = self.capture(p.pass_done, 2, unks, (0, 2, 9))
        self.assertEqual(p.flat_passes, 2, fs + 'second flat pass counted')
        #  Two flat passes in a row is comp_det's own stop condition, so the
        #  reporter must escalate from "nothing changed" to "stuck".
        self.assertIn('stuck, not slow', out,
                      fs + 'two flat passes should escalate the wording')

        #  A pass that changes the equation pools but solves nothing is NOT
        #  stuck -- transforms are progress, and the counter must reset.
        out = self.capture(p.pass_done, 3, unks, (3, 5, 4))
        self.assertEqual(p.flat_passes, 0, fs + 'pool change resets the counter')
        self.assertIn('still working', out, fs + 'pool change reads as working')

        #  A pass that solves a variable likewise resets it.
        self.capture(p.pass_done, 4, unks, (3, 5, 4))   # flat again -> 1
        self.assertEqual(p.flat_passes, 1, fs + 'flat after the transform')
        a.solved = True
        out = self.capture(p.pass_done, 5, unks, (3, 5, 4))
        self.assertEqual(p.flat_passes, 0, fs + 'a solve resets the counter')
        self.assertIn('making progress', out, fs + 'a solve reads as progress')

    def test_solved_count_is_read_from_the_unknowns(self):
        '''The count must come from the unknown objects, never from a local
           tally -- set_solved() mutates them in place and the whole
           monotonicity argument rests on reading the live objects.'''

        p = SolveProgress('R', 10, 3, enabled=True)
        unks = [test_unk('a', True), test_unk('b'), test_unk('c')]
        import io
        import sys
        buf, real = io.StringIO(), sys.stdout
        sys.stdout = buf
        try:
            p.pass_done(1, unks, (0, 1, 2))
        finally:
            sys.stdout = real
        out = buf.getvalue()
        self.assertIn('solved 1/3', out,
                      'progress.pass_done: count read from the unknowns')

    def test_newly_solved_names_are_the_ones_that_changed(self):
        '''Regression.  The first version sliced the filtered solved-list by the
           previous count, which assumes variables are solved in list order.
           The assigner round-robins, so they are not:  Puma reported "+1: th_23"
           for five consecutive passes while actually solving th_2, th_4, th_5
           and th_6.  Solve out of order and check the right name is named.'''

        import io
        import sys

        p = SolveProgress('R', 10, 4, enabled=True)
        a, b, c, d = (test_unk('th_1'), test_unk('th_2'),
                      test_unk('th_23'), test_unk('th_6'))
        unks = [a, b, c, d]

        #  Solve the THIRD one first -- the out-of-order case.
        c.solved = True
        out = self.capture(p.pass_done, 1, unks, (1, 1, 1))
        self.assertIn('+1: th_23', out,
                      'progress: should name the variable actually solved')

        #  Now solve the LAST one.  The buggy version would say th_2 here,
        #  because th_2 sits at index 1 of the filtered list.
        d.solved = True
        out = self.capture(p.pass_done, 2, unks, (1, 1, 1))
        self.assertIn('+1: th_6', out,
                      'progress: must name th_6, not whatever is next in order')
        self.assertNotIn('th_2,', out,
                         'progress: must not name an unsolved variable')

        #  Two at once.
        a.solved = True
        b.solved = True
        out = self.capture(p.pass_done, 3, unks, (1, 1, 1))
        self.assertIn('+2: th_1, th_2', out,
                      'progress: both newly solved variables named')

    def test_meter_is_reversible(self):
        """enable/disable must leave every patched name exactly as found.

           THE NAME THAT MATTERS IS THE FUNCTION, not Basic.simplify.  This test
           used to assert the METHOD was wrapped, which pinned the very defect
           it was meant to guard:  the leaves call sp.simplify() 26 times and
           the method 6, so metering the method missed most of the work and the
           slow-call heartbeat never fired.  Basic.simplify delegates to the
           function, so patching the function catches both paths -- and the
           method is deliberately left alone, or every call would be counted
           twice.

           Both bindings have to be restored:  sympy.simplify, and the same
           function inside sympy.simplify.simplify, which is what the method
           imports.  The meter patches a third-party module, so a leak would
           follow the process into every later test."""

        import importlib
        import sympy as sp
        from sympy.core.basic import Basic

        sub = importlib.import_module('sympy.simplify.simplify')
        before_fn, before_sub = sp.simplify, sub.simplify
        before_method = Basic.simplify

        enable_sympy_meter()
        self.assertIsNot(sp.simplify, before_fn,
                         'progress: meter should have wrapped sp.simplify')
        self.assertIsNot(sub.simplify, before_sub,
                         'progress: meter must also wrap the name the method '
                         'imports, or Basic.simplify goes uncounted')
        self.assertIs(Basic.simplify, before_method,
                      'progress: the METHOD must be left alone -- it delegates '
                      'to the function, so wrapping both double-counts')

        disable_sympy_meter()
        self.assertIs(sp.simplify, before_fn,
                      'progress: meter must restore sp.simplify')
        self.assertIs(sub.simplify, before_sub,
                      'progress: meter must restore the submodule binding')

    def test_meter_counts_and_is_off_by_default(self):
        import sympy as sp
        from sympy.core.basic import Basic
        before = Basic.simplify
        try:
            c0, _ = meter_stats()
            enable_sympy_meter()
            sp.var('q')
            (sp.sin(q) ** 2 + sp.cos(q) ** 2).simplify()
            c1, _ = meter_stats()
            self.assertGreater(c1, c0, 'progress: meter should count a call')
        finally:
            disable_sympy_meter()
            self.assertIs(Basic.simplify, before, 'progress: restored')


def run_test():
    _say('\n\n===============  Test progress reporting  =====================')
    unittest.main(module=__name__, exit=False, argv=['progress'])


if __name__ == '__main__':
    run_test()
