#!/usr/bin/python
#
#   clear_state.py --  drop leftover solver state so a solve starts clean
#
#   The IKBT tree now contains the symbolic solver TWICE:  once on the symbolic
#   branch, and once on the hybrid branch applied to a simplified arm.  Two
#   separate node instance sets, so b3's own per-node memory (is_open, loop
#   counters) is keyed by fresh node ids and is clean automatically.  What is NOT
#   automatic is the unscoped application state on the blackboard -- comp_det's
#   verdict from the FAILED first solve would still be sitting there when the
#   second solve starts.
#
#   So each solver begins with this leaf.  On the first solve there is nothing to
#   drop;  on the second it is what makes "we already failed, wipe and retry"
#   actually true.
#
#   WIPE BY DEFAULT, KEEP BY EXCEPTION.  The keep-list is the whole design
#   decision.  An enumerated *clear*-list rots silently:  add a blackboard key
#   next year, forget to list it, and stale state leaks into the second solve
#   with no symptom until some robot solves wrongly.  With a keep-list the
#   failure mode inverts -- a new key that should have survived gets dropped
#   instead, which shows up immediately as a missing value rather than a subtly
#   wrong answer.  Cheap to diagnose.
#
#   Copyright 2026 University of Washington
#
#   Developed by Blake Hannaford
#   BioRobotics Lab, University of Washington

import b3 as b3          # behavior trees


class clear_state(b3.Action):
    '''Drop every unscoped blackboard key except the ones a solve needs to
       inherit.  Always SUCCEEDs -- it is hygiene, not a decision.

       KEEP holds three kinds of thing:

         the problem itself   Robot, unknowns, eqns_1u/2u/3pu -- set up by
                              init_blackboard() or, on the hybrid branch, by
                              install_simplified() immediately before this leaf
         findings about the TRUE robot   pieper_* and simplification_*, which
                              describe the arm we started from and must survive
                              into the report even though the Robot has been
                              swapped underneath
         b3 bookkeeping       TotalCost, which BaseNode._tick increments and
                              Blackboard.__init__ requires to exist

       Everything else goes:  no_progress and comp_det_signature (the previous
       solve's give-up verdict), curr_unk (the assigner's cursor into an unknown
       list that no longer exists), invariants_done, symbolic_passes /
       symbolic_exhausted, and anything added later.'''

    KEEP = ('Robot', 'unknowns', 'eqns_1u', 'eqns_2u', 'eqns_3pu',
            'pieper_triples', 'pieper_ok', 'pieper_latex',
            'simplification_candidates', 'simplification_choice',
            'hybrid_source',
            'TotalCost')

    def __init__(self):
        super(clear_state, self).__init__()
        self.Name = 'Clear Solver State'
        self.BHdebug = False

    def tick(self, tick):
        bb = tick.blackboard

        #  Reaching into _base_memory:  b3 here is a local fork and Blackboard
        #  exposes no "list my keys".  Guarded so a future b3 that renames it
        #  degrades to a no-op with a warning rather than an AttributeError in
        #  the middle of a solve.
        base = getattr(bb, '_base_memory', None)
        if base is None:
            print(self.Name, ': blackboard has no _base_memory -- nothing '
                  'cleared.  Check the b3 version.')
            return b3.SUCCESS

        dropped = [k for k in list(base.keys()) if k not in self.KEEP]
        for k in dropped:
            del base[k]

        if self.BHdebug and dropped:
            print('\n', self.Name, ': dropped', sorted(dropped))

        return b3.SUCCESS


#####################################################################
#
#   Test code
#

import unittest


class TestSolver020(unittest.TestCase):
    '''The state-clearing leaf that heads each solver.'''

    def setUp(self):
        print('\n\n===============  Test clear_state  =====================')
        return

    def runTest(self):
        self.test_clrA_drops_the_previous_verdict()
        self.test_clrB_keeps_the_problem_and_the_findings()
        self.test_clrC_unknown_keys_are_dropped_not_kept()
        self.test_clrD_survives_a_blackboard_with_no_base_memory()

    def tick(self, bb, node=None):
        node = node or clear_state()
        t = b3.BehaviorTree()
        t.root = node
        return t.tick('testing clear_state', bb)

    def test_clrA_drops_the_previous_verdict(self):
        '''The keys that would corrupt a second solve must go.

           no_progress is the dangerous one: left set, solved_anything() reports
           the second solve as a failure even after it succeeded.  Next is
           comp_det_signature -- a stale signature that happens to match makes
           comp_det give up on the very first pass.'''
        fs = ' clear_state FAIL'
        bb = b3.Blackboard()
        for k in ('no_progress', 'comp_det_signature', 'curr_unk',
                  'invariants_done', 'symbolic_passes', 'symbolic_exhausted'):
            bb.set(k, 'stale')

        self.assertEqual(self.tick(bb), b3.SUCCESS, fs)
        for k in ('no_progress', 'comp_det_signature', 'curr_unk',
                  'invariants_done', 'symbolic_passes', 'symbolic_exhausted'):
            self.assertIsNone(bb.get(k), fs + ' (%s survived)' % k)

    def test_clrB_keeps_the_problem_and_the_findings(self):
        '''The problem being solved, and the findings about the TRUE robot, must
           survive -- the Robot gets swapped for a derived one, but the report
           still has to describe the arm we started from.'''
        fs = ' clear_state keep FAIL'
        bb = b3.Blackboard()
        keep = {'Robot': 'R', 'unknowns': ['u'],
                'eqns_1u': [1], 'eqns_2u': [2], 'eqns_3pu': [3],
                'pieper_triples': ['t'], 'pieper_ok': True,
                'pieper_latex': r'\section{Geometry}',
                'simplification_candidates': ['c'],
                'simplification_choice': 'c',
                'hybrid_source': {'robot': 'X_d5_0'}}
        for k, v in keep.items():
            bb.set(k, v)

        self.tick(bb)
        for k, v in keep.items():
            self.assertEqual(bb.get(k), v, fs + ' (%s was dropped)' % k)
        #  TotalCost is required to exist by Blackboard.inc()
        self.assertIsNotNone(bb.get('TotalCost'), fs + ' (TotalCost dropped)')

    def test_clrC_unknown_keys_are_dropped_not_kept(self):
        '''Wipe by default:  a key nobody listed goes.

           This is the point of the design.  A clear-list would leak an
           unlisted key silently;  a keep-list drops it, which surfaces as a
           missing value the first time anyone looks.'''
        fs = ' clear_state wipe-by-default FAIL'
        bb = b3.Blackboard()
        bb.set('some_future_key_nobody_thought_about', 42)
        self.tick(bb)
        self.assertIsNone(bb.get('some_future_key_nobody_thought_about'),
                          fs + ' (an unlisted key survived -- the leaf is '
                          'keeping by default, which is backwards)')

    def test_clrD_survives_a_blackboard_with_no_base_memory(self):
        '''If b3 ever renames _base_memory this must degrade to a no-op, not
           raise in the middle of a solve.'''
        fs = ' clear_state guard FAIL'

        class odd(object):
            def get(self, k, *a): return None
            def set(self, k, v, *a): pass

        class faketick(object):
            pass

        node = clear_state()
        t = faketick()
        t.blackboard = odd()
        self.assertEqual(node.tick(t), b3.SUCCESS, fs)


def run_test():
    suite = unittest.TestLoader().loadTestsFromTestCase(TestSolver020)
    unittest.TextTestRunner(verbosity=2).run(suite)


##   Self Tests
if __name__ == "__main__":
    run_test()
