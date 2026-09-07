#!/usr/bin/python
#
#   symbolic_loop.py --  the outer solve loop, as a leaf that owns its own loop
#
#
#   Copyright 2026 University of Washington
#
#   Developed by Blake Hannaford
#   BioRobotics Lab, University of Washington

import b3 as b3          # behavior trees

from ikbtfunctions.progress import SolveProgress


class symbolic_loop(b3.Decorator):
    '''Tick the solve routine up to max_loop times;  SUCCEED only when every
       unknown is solved (require_complete, the default).

           SUCCESS  every unknown is solved
           FAILURE  one or more unknowns are still unsolved -- there is no
                    complete closed form to report

       Blackboard, set every tick for whoever wants to measure the loop:

           symbolic_passes     how many times the child was ticked
           symbolic_exhausted  True if the budget ran out with the child still
                               failing (as opposed to the child terminating on
                               its own, which is what comp_det does)

       require_complete is ON by default:  a closed form for some of the joints
       is not inverse kinematics, so a partial solve is a failure of this node
       even though the variables it did solve keep their solutions.  Setting it
       False restores the older "SUCCESS if anything was solved" behaviour, and
       is the hook for the future work that makes deliberate use of a partial
       result (solve what closes analytically, finish the rest numerically at
       lower dimension).'''

    def __init__(self, child=None, max_loop=10):
        super(symbolic_loop, self).__init__(child)

        #  Name is set AFTER super():  BaseNode.__init__ assigns '--unnamed--'.
        self.Name = 'Symbolic Solver Loop'
        self.BHdebug = False

        #  Named max_loop to match b3's loop decorators -- the BT linter checks
        #  any node carrying that attribute for a finite, non-zero budget.
        self.max_loop = max_loop

        self.require_complete = True
        self.progress = True

    def tick(self, tick):
        if not self.child:
            return b3.ERROR

        bb = tick.blackboard

        #  Reported per pass, so a user watching a 700 s solve can see whether
        #  anything is improving.  Built here rather than in __init__ because
        #  the robot and the unknown count are blackboard state, and on the
        #  hybrid branch this node is ticked against a DERIVED arm.
        R0 = bb.get('Robot')
        unks0 = bb.get('unknowns') or []
        prog = SolveProgress(getattr(R0, 'name', None), self.max_loop,
                             len(unks0), enabled=self.progress,
                             tag=' (hybrid)' if 'hybrid' in self.Name else '')
        prog.banner()

        def pools():
            '''Equation-pool sizes.  A pass that solves nothing but changes
               these is transforming equations, which is progress of a kind and
               reads very differently from a pass that changed nothing.'''
            return (len(bb.get('eqns_1u') or []),
                    len(bb.get('eqns_2u') or []),
                    len(bb.get('eqns_3pu') or []))

        #  Identical to RepeatUntilSuccess.tick(): re-_execute() the child while
        #  it FAILs.  The child is a Sequence, which _open()s and _close()s on
        #  every _execute(), so repeated calls are exactly repeated passes.
        passes = 0
        status = b3.FAILURE
        while passes < self.max_loop:
            status = self.child._execute(tick)
            passes += 1
            prog.pass_done(passes, bb.get('unknowns') or [], pools())
            if status != b3.FAILURE:
                break

        exhausted = (status == b3.FAILURE)
        prog.finished(bb.get('unknowns') or [], exhausted)
        bb.set('symbolic_passes', passes)
        bb.set('symbolic_exhausted', exhausted)

        unks = bb.get('unknowns') or []
        n = len(unks)
        ns = len([u for u in unks if u.solved])

        if self.BHdebug:
            print('\n', self.Name, ': ', passes, ' passes, ', ns, ' of ', n,
                  ' unknowns solved', '  (budget exhausted)' if exhausted else '')

        if n == 0:
            #  No unknowns on the blackboard at all:  nothing was ever set up.
            #  Not a solve, so not a success.
            return b3.FAILURE

        if self.require_complete:
            return b3.SUCCESS if ns == n else b3.FAILURE

        return b3.SUCCESS if ns > 0 else b3.FAILURE


#####################################################################
#
#   Test code
#

import unittest


class test_fake_pass(b3.Action):
    '''A stand-in for the solve routine.  Returns FAILURE for the first
       `fails` ticks and then SUCCESS, marking `solves` unknowns solved along
       the way -- so a test can build any (passes, solved) combination without
       running the real solver.'''

    def __init__(self, fails=0, solves=0, never_succeeds=False):
        super(test_fake_pass, self).__init__()
        self.Name = 'Fake Solve Pass'
        self.fails = fails
        self.solves = solves
        self.never_succeeds = never_succeeds
        self.ticks = 0

    def tick(self, tick):
        self.ticks += 1
        unks = tick.blackboard.get('unknowns') or []
        for u in unks[:self.solves]:
            u.solved = True
        if self.never_succeeds or self.ticks <= self.fails:
            return b3.FAILURE
        return b3.SUCCESS


class fake_unk(object):
    '''The attributes symbolic_loop reads.  `name` is here for the progress
       reporter -- which tolerates its absence deliberately, but a double that
       carries it produces readable test output instead of "<fake_unk object
       at 0x...>".'''

    _n = [0]

    def __init__(self):
        self.solved = False
        fake_unk._n[0] += 1
        self.name = 'fake_%d' % fake_unk._n[0]


class TestSolver016(unittest.TestCase):
    '''The outer solve loop:  loop budget, and the exit status it reports.'''

    def setUp(self):
        print('\n\n===============  Test symbolic solver loop  =====================')
        return

    def runTest(self):
        self.test_loopA_stops_when_child_succeeds()
        self.test_loopB_honors_the_budget()
        self.test_loopC_partial_solve_is_failure()
        self.test_loopD_nothing_solved_is_failure()
        self.test_loopE_require_complete_can_be_relaxed()
        self.test_loopF_no_unknowns_is_failure()

    def run_loop(self, child, nunk=3, max_loop=10, require_complete=None):
        #  require_complete=None means "leave the node's own default alone".
        #  It used to default to False and assign unconditionally, which
        #  silently overrode the default in EVERY test -- so no test could
        #  detect a change to it.
        node = symbolic_loop(child, max_loop=max_loop)
        if require_complete is not None:
            node.require_complete = require_complete
        node.progress = False
        bb = b3.Blackboard()
        bb.set('unknowns', [fake_unk() for _ in range(nunk)])
        t = b3.BehaviorTree()
        t.root = node
        status = t.tick('testing symbolic_loop', bb)
        return status, bb, node

    def test_loopA_stops_when_child_succeeds(self):
        '''The loop must not keep ticking after the child stops failing --
           comp_det returning SUCCESS is how the solve terminates.'''
        fs = ' symbolic_loop FAIL'
        child = test_fake_pass(fails=2, solves=3)
        st, bb, _ = self.run_loop(child)
        self.assertEqual(st, b3.SUCCESS, fs + ' (complete solve)')
        self.assertEqual(bb.get('symbolic_passes'), 3, fs + ' (pass count)')
        self.assertFalse(bb.get('symbolic_exhausted'), fs + ' (budget was not used up)')

    def test_loopB_honors_the_budget(self):
        '''A child that never succeeds gets exactly max_loop ticks.  An
           unbounded loop here would hang on any robot that stops making
           progress.'''
        fs = ' symbolic_loop budget FAIL'
        child = test_fake_pass(never_succeeds=True, solves=1)
        st, bb, _ = self.run_loop(child, max_loop=4)
        self.assertEqual(child.ticks, 4, fs + ' (wrong number of passes)')
        self.assertEqual(bb.get('symbolic_passes'), 4, fs)
        self.assertTrue(bb.get('symbolic_exhausted'), fs + ' (should flag exhaustion)')

    def test_loopC_partial_solve_is_failure(self):
        '''A partial solve is a FAILURE:  a closed form for some of the joints
           is not inverse kinematics, so the tree must not treat it as an
           answer and write a report for it (BH, 2026-08-23).

           The solved variables are NOT discarded -- set_solved() already
           recorded them and they stay on the blackboard.  Only the node's
           verdict changes.'''
        fs = ' symbolic_loop partial FAIL'
        child = test_fake_pass(never_succeeds=True, solves=1)
        st, bb, _ = self.run_loop(child, nunk=3, max_loop=3)
        self.assertTrue(bb.get('symbolic_exhausted'), fs + ' (should be exhausted)')
        self.assertEqual(st, b3.FAILURE,
                         fs + ' (1 of 3 solved is not a solution)')
        #  ... but the partial result survives for whoever wants it.
        self.assertEqual(len([u for u in bb.get('unknowns') if u.solved]), 1,
                         fs + ' (the solved variable must not be lost)')

    def test_loopD_nothing_solved_is_failure(self):
        '''Nothing solved is the one case that must FAIL -- that FAILURE is
           what hands control to a fallback strategy.'''
        fs = ' symbolic_loop no-solve FAIL'
        child = test_fake_pass(never_succeeds=True, solves=0)
        st, bb, _ = self.run_loop(child, max_loop=3)
        self.assertEqual(st, b3.FAILURE, fs)

    def test_loopE_require_complete_can_be_relaxed(self):
        '''require_complete=False restores the older "SUCCESS if anything was
           solved" behaviour.  That is the hook for the future work which makes
           deliberate use of a partial result, so it has to keep working.'''
        fs = ' symbolic_loop require_complete FAIL'
        child = test_fake_pass(never_succeeds=True, solves=1)
        st, _, _ = self.run_loop(child, nunk=3, max_loop=2,
                                 require_complete=False)
        self.assertEqual(st, b3.SUCCESS, fs + ' (relaxed: partial succeeds)')

        #  And the default stays strict.
        child = test_fake_pass(never_succeeds=True, solves=1)
        st, _, _ = self.run_loop(child, nunk=3, max_loop=2)
        self.assertEqual(st, b3.FAILURE, fs + ' (default must be strict)')

        child = test_fake_pass(fails=0, solves=3)
        st, _, _ = self.run_loop(child, nunk=3)
        self.assertEqual(st, b3.SUCCESS, fs + ' (complete should succeed)')

    def test_loopF_no_unknowns_is_failure(self):
        '''An empty unknown list means the blackboard was never set up.  Do not
           report that as a solve.'''
        fs = ' symbolic_loop empty FAIL'
        child = test_fake_pass(fails=0, solves=0)
        st, _, _ = self.run_loop(child, nunk=0)
        self.assertEqual(st, b3.FAILURE, fs)


def run_test():
    suite = unittest.TestLoader().loadTestsFromTestCase(TestSolver016)
    unittest.TextTestRunner(verbosity=2).run(suite)


##   Self Tests
if __name__ == "__main__":
    run_test()
