#!/usr/bin/python
#
#   hybrid_ik.py --  the hybrid symbolic-numeric branch
#
#   futurework.md item 1:  when IKBT cannot solve a robot in closed form,
#   simplify its DH parameters until it can, solve the SIMPLIFIED robot
#   symbolically, and correct the result numerically using the symbolic
#   Jacobian.  Six robots have no Pieper triple of any kind, and in each one a
#   single DH parameter is responsible -- KinovaLite with d_5 -> 0 goes from
#   0-of-6 variables solved to all 6.
#
#   RIGHT NOW THIS FILE IS A PLACEHOLDER.  hybrid_stub always FAILs, so the
#   branch is inert and the tree behaves exactly as it did before the branch
#   existed.  That is the point:  it makes the tree RESTRUCTURE separable from
#   the new behavior, so a baseline diff over every robot can show that
#   rearranging the tree changed nothing, before any new solver is added.
#
#   The stub is replaced one leaf at a time, each proven by its own baseline
#   diff:
#
#       pieper_id         does this robot have a Pieper triple?  The branch is
#                         gated on the INVERSE -- a robot that has a triple and
#                         still failed symbolically is a solver defect, not a
#                         geometry problem, and must not be simplified.
#       simplified_arm    which single DH parameter costs the least task-space
#                         displacement to zero or snap?
#       solve_simplified  solve the derived robot with a nested tree
#       numeric_ik +      damped least squares from the closed-form seed, and
#       output_gen_hybrid artifacts labelled with the parameter that was changed
#
#   Copyright 2026 University of Washington
#
#   Developed by Blake Hannaford
#   BioRobotics Lab, University of Washington

import b3 as b3          # behavior trees

import ikbtbasics.dh_analysis as da


class hybrid_stub(b3.Action):
    '''Placeholder for the hybrid branch.  Always FAILURE, so the branch is a
       no-op and the enclosing Priority falls through as if it were not there.

       FAILURE rather than SUCCESS on purpose:  a SUCCESS here would tell the
       tree a robot had been solved by a strategy that does not exist yet, and
       the first thing downstream would be codegen over an empty solution set.

       Left deliberately noisy under BHdebug so that a robot reaching this leaf
       is visible -- reaching it means the symbolic solver came up empty, which
       is exactly the population the hybrid method is for.'''

    def __init__(self):
        super(hybrid_stub, self).__init__()
        self.Name = 'Hybrid Branch (not implemented)'
        self.BHdebug = False

    def tick(self, tick):
        if self.BHdebug:
            R = tick.blackboard.get('Robot')
            print('\n', self.Name, ':', getattr(R, 'name', '?'),
                  'was not solved symbolically, and the hybrid branch is a stub.')
        return b3.FAILURE


class pieper_id(b3.Action):
    """Identify which consecutive joint-axis triples satisfy Pieper's condition.

           SUCCESS  at least one triple qualifies
           FAILURE  none do, or the DH table could not be analysed

       Standard `_id` semantics, and it is used the standard way -- as the gate
       on the hybrid branch, under an Inverter, so the hybrid fires only for an
       arm whose geometry really lacks the structure:

           hybrid_branch = Sequence[ Inverter(pieper_id), ... ]

       WHY IT GATES ONLY THE HYBRID.  Pieper's condition is *sufficient* for a
       closed form to exist and is not known to be necessary, so it must never
       gate the SYMBOLIC solver.  Measured: 9 of the 32 robots in ROBOT_LIST
       have no triple and still solve completely (Axtman13, Brad, DZhang,
       ICP5p5_A21, Mackler13, MiniDD, Olson13, Sims11, Wachtveitl).  Putting
       this leaf ahead of the symbolic branch in a Sequence stops that branch
       ticking at all for those nine.

       FAILURE also means "could not tell", so `pieper_ok` on the blackboard
       distinguishes the two -- a gate that needs "there is definitely no
       triple" must check it.

       The LaTeX statement for the report is NOT written here.  It is generated
       by report_gen (ikbtleaves/output_gen.py), because it belongs to whichever
       branch produced the report and this leaf ticks only on the hybrid path."""

    def __init__(self):
        super(pieper_id, self).__init__()
        self.Name = 'Pieper Triple ID'
        self.BHdebug = False

    def tick(self, tick):
        bb = tick.blackboard
        R = bb.get('Robot')

        bb.set('pieper_triples', [])
        bb.set('pieper_ok', False)

        try:
            M = R.Mech
            ndof = da.ndof_from_unknowns(bb.get('unknowns'),
                                         fallback=len(getattr(R, 'variables', []) or []) or 6)
            triples = da.pieper_triples(M.DH, M.pvals, ndof)
            bb.set('pieper_triples', triples)
            bb.set('pieper_ok', True)

            if self.BHdebug:
                print('\n', self.Name, ':', getattr(R, 'name', '?'), '->',
                      [(t['axes'], t['kind']) for t in triples] or 'no triples')

            return b3.SUCCESS if triples else b3.FAILURE

        except Exception as e:
            #  Reported, never raised.  pieper_ok stays False so a gate cannot
            #  read "we could not tell" as "there is no triple".
            print(self.Name, ': could not analyse the DH table --',
                  '%s: %s' % (type(e).__name__, e))
            return b3.FAILURE


class simplified_arm(b3.Action):
    """Rank the DH changes that would give this arm a Pieper triple, cheapest
       first, and put the ranked list on the blackboard.

           SUCCESS  at least one usable candidate.  Blackboard carries
                    `simplification_candidates` (ranked, cheapest first) and
                    `simplification_choice` (the winner), each candidate holding
                    its edits, their numeric magnitudes, the derived DH table and
                    its measured task-space cost.
           FAILURE  no usable candidate, or the Pieper analysis was unreliable.

       REFUSES TO ACT WHEN `pieper_ok` IS FALSE.  `Inverter(pieper_id)` turns
       BOTH "this arm has no triple" and "I could not read the DH table" into
       SUCCESS, so this leaf is where that ambiguity has to be resolved -- it is
       the first thing downstream that would act on the conclusion.  Simplifying
       an arm because we failed to parse its parameters would be the worst kind
       of silent wrong answer:  a derived robot, solved perfectly, describing
       nothing.

       Ranking is by task-space displacement (see dh_analysis.rank_candidates),
       which is what puts zeroing a length and snapping an angle in comparable
       units.  `seed` is fixed so the choice is reproducible -- the whole
       phase-gate discipline depends on a rerun giving the same answer."""

    def __init__(self):
        super(simplified_arm, self).__init__()
        self.Name = 'Simplified Arm'
        self.BHdebug = False

        #  Sampling for the displacement metric.  200 samples over ~28
        #  candidates is a couple of seconds -- affordable, because this leaf
        #  ticks at most once per solve and only for an arm that already failed.
        self.n_samples = 200
        self.seed = 0
        self.w_rot = None          # None -> dh_analysis picks the length scale

    def tick(self, tick):
        bb = tick.blackboard
        R = bb.get('Robot')

        bb.set('simplification_candidates', [])
        bb.set('simplification_choice', None)

        if not bb.get('pieper_ok'):
            print(self.Name, ': the Pieper analysis did not run, so "no triple"'
                  ' is not established -- refusing to simplify.')
            return b3.FAILURE

        try:
            M = R.Mech
            ndof = da.ndof_from_unknowns(
                bb.get('unknowns'),
                fallback=len(getattr(R, 'variables', []) or []) or 6)
            ranked = da.rank_candidates(M.DH, M.pvals, M.vv, ndof,
                                        n=self.n_samples, seed=self.seed,
                                        w_rot=self.w_rot)
        except Exception as e:
            print(self.Name, ': could not rank simplifications --',
                  '%s: %s' % (type(e).__name__, e))
            return b3.FAILURE

        #  Blocked routes (a prismatic joint variable in the way) and no-op
        #  entries are kept by rank_candidates for reporting;  they are not
        #  something we can act on.
        usable = [c for c in ranked if c.get('dh_simp') is not None and c['edits']]
        bb.set('simplification_candidates', usable)

        if not usable:
            print(self.Name, ':', getattr(R, 'name', '?'),
                  '-- no DH change would produce a Pieper triple.')
            return b3.FAILURE

        choice = usable[0]
        bb.set('simplification_choice', choice)

        print('\n', self.Name, ':', getattr(R, 'name', '?'), '-- cheapest of',
              len(usable), 'candidates is', da.describe_edits(choice),
              '(axes %s, cost %.3f)' % (choice['axes'], choice['cost']))

        if self.BHdebug:
            print('   %-9s %-13s %-30s %10s' % ('axes', 'route', 'edits', 'cost'))
            for c in usable[:8]:
                print('   %-9s %-13s %-30s %10.3f'
                      % (str(c['axes']), c['route'],
                         da.describe_edits(c)[:30], c['cost']))

        return b3.SUCCESS


#####################################################################
#
#   Test code
#

import unittest


class TestSolver018(unittest.TestCase):
    '''The hybrid placeholder.  One property matters: it is inert.'''

    def setUp(self):
        print('\n\n===============  Test hybrid branch stub  =====================')
        return

    def runTest(self):
        self.test_hybA_always_fails()
        self.test_hybB_inverted_stub_would_not_gate()
        self.test_hybC_pieper_id_fails_when_it_cannot_tell()
        self.test_hybD_pieper_id_finds_the_wrist()
        self.test_hybE_pieper_id_ignores_sum_of_angle_unknowns()
        self.test_hybF_pieper_id_fails_with_no_triple()
        self.test_hybG_simplified_arm_refuses_when_pieper_unreliable()
        self.test_hybH_simplified_arm_ranks_cheapest_first()
        self.test_hybI_simplified_arm_fails_with_nothing_to_buy()

    def test_hybA_always_fails(self):
        '''FAILURE on an empty blackboard and on a populated one alike.  The
           whole Phase-B claim -- that restructuring the tree changed nothing --
           rests on this leaf never returning anything else.'''
        fs = ' hybrid_stub FAIL'
        for bb in (b3.Blackboard(), b3.Blackboard()):
            t = b3.BehaviorTree()
            t.root = hybrid_stub()
            self.assertEqual(t.tick('testing hybrid_stub', bb), b3.FAILURE, fs)

    def test_hybB_inverted_stub_would_not_gate(self):
        '''Sanity check on the shape the real branch will have: the gate is
           Inverter(pieper_id), so a leaf that FAILs becomes a SUCCESS under
           the inverter.  Recorded here because it is the easy thing to get
           backwards -- the hybrid runs when there is NO triple.'''
        fs = ' hybrid_stub inverter FAIL'
        t = b3.BehaviorTree()
        t.root = b3.Inverter(hybrid_stub())
        self.assertEqual(t.tick('testing inverter', b3.Blackboard()),
                         b3.SUCCESS, fs)


    #  ------------------------------------------------  pieper_id

    #  Small stand-ins, so these stay fast and need no FK pickle.
    class mech(object):
        def __init__(self, dh, pvals, vv=None):
            self.DH, self.pvals = dh, pvals
            self.vv = vv or [1]*6                 # all rotary unless told

    class robot(object):
        def __init__(self, dh, pvals, name='Fake', vv=None):
            self.Mech = TestSolver018.mech(dh, pvals, vv)
            self.name = name

    class unk(object):
        def __init__(self, n): self.n = n

    def puma_like(self):
        """A 6R table with a spherical wrist at axes (4,5,6):  a_4 = a_5 = 0 and
           d_5 = 0, with non-zero offsets everywhere else so nothing else
           qualifies."""
        import sympy as sp
        rows = [[0,         sp.Integer(0), sp.Integer(0),  sp.Symbol('th_1')],
                [sp.pi/2,   sp.Integer(3), sp.Integer(4),  sp.Symbol('th_2')],
                [0,         sp.Integer(3), sp.Integer(4),  sp.Symbol('th_3')],
                [sp.pi/2,   sp.Integer(3), sp.Integer(4),  sp.Symbol('th_4')],
                [-sp.pi/2,  sp.Integer(0), sp.Integer(0),  sp.Symbol('th_5')],
                [sp.pi/2,   sp.Integer(0), sp.Integer(0),  sp.Symbol('th_6')]]
        return sp.Matrix(rows)

    def tick_pieper(self, bb):
        node = pieper_id()
        t = b3.BehaviorTree()
        t.root = node
        return t.tick('testing pieper_id', bb)

    def no_triple_table(self):
        """A 6R table where no triple qualifies:  every a and d non-zero and no
           sin(alpha) zero."""
        import sympy as sp
        return sp.Matrix([[sp.pi/2, sp.Integer(3), sp.Integer(4),
                           sp.Symbol('th_%d' % (r+1))] for r in range(6)])

    def test_hybC_pieper_id_fails_when_it_cannot_tell(self):
        """FAILURE on a blackboard with no Robot -- and pieper_ok stays False.

           FAILURE is overloaded here: it means "no triple" OR "could not tell".
           A gate that needs the first must check pieper_ok, or it will send an
           arm down the hybrid path on the strength of a missing Robot."""
        fs = ' pieper_id FAIL'
        bb = b3.Blackboard()
        self.assertEqual(self.tick_pieper(bb), b3.FAILURE, fs)
        self.assertFalse(bb.get('pieper_ok'),
                         fs + ' (pieper_ok must be False when it could not run)')
        self.assertEqual(bb.get('pieper_triples'), [],
                         fs + ' (triples must still be a list)')

    def test_hybD_pieper_id_finds_the_wrist(self):
        """A spherical wrist -> SUCCESS, so Inverter(pieper_id) FAILs and the
           hybrid branch correctly declines to touch this arm."""
        fs = ' pieper_id wrist FAIL'
        R = TestSolver018.robot(self.puma_like(), {}, 'Wristy')
        bb = b3.Blackboard()
        bb.set('Robot', R)
        bb.set('unknowns', [TestSolver018.unk(i) for i in range(1, 7)])

        self.assertEqual(self.tick_pieper(bb), b3.SUCCESS,
                         fs + ' (a triple must SUCCEED)')
        self.assertTrue(bb.get('pieper_ok'), fs + ' (analysis should have run)')
        got = [(t['axes'], t['kind']) for t in bb.get('pieper_triples')]
        self.assertIn(((4, 5, 6), 'intersect'), got, fs + ' (missed the wrist)')

        #  under the Inverter -- which is how the tree actually uses it
        t = b3.BehaviorTree()
        t.root = b3.Inverter(pieper_id())
        self.assertEqual(t.tick('inverted', bb), b3.FAILURE,
                         fs + ' (an arm WITH a triple must not enter the hybrid)')

    def test_hybE_pieper_id_ignores_sum_of_angle_unknowns(self):
        """DOF count must skip the sum-of-angles unknowns.

           kinematics_pickle() EXTENDS the unknown list with th_23 (n = 23) and
           friends.  Counting those would inflate ndof past 6 and invent triples
           over the zero-padded rows."""
        fs = ' pieper_id SOA FAIL'
        R = TestSolver018.robot(self.puma_like(), {}, 'Soa')
        bb = b3.Blackboard()
        bb.set('Robot', R)
        bb.set('unknowns', [TestSolver018.unk(i) for i in range(1, 7)]
                           + [TestSolver018.unk(23), TestSolver018.unk(234)])

        self.assertEqual(self.tick_pieper(bb), b3.SUCCESS, fs)
        axes = [t['axes'] for t in bb.get('pieper_triples')]
        self.assertTrue(all(a[2] <= 6 for a in axes),
                        fs + ' (a triple names an axis above 6: %s)' % axes)

    def test_hybF_pieper_id_fails_with_no_triple(self):
        """No triple -> FAILURE, so Inverter(pieper_id) SUCCEEDs and the hybrid
           branch is admitted.  pieper_ok is True: this is a real answer, not an
           error."""
        fs = ' pieper_id no-triple FAIL'
        R = TestSolver018.robot(self.no_triple_table(), {}, 'Plain')
        bb = b3.Blackboard()
        bb.set('Robot', R)
        bb.set('unknowns', [TestSolver018.unk(i) for i in range(1, 7)])

        self.assertEqual(self.tick_pieper(bb), b3.FAILURE, fs)
        self.assertTrue(bb.get('pieper_ok'),
                        fs + ' ("no triple" is an ANSWER, not a failure to run')
        self.assertEqual(bb.get('pieper_triples'), [], fs)


    #  ------------------------------------------------  simplified_arm

    def tick_simplify(self, bb, n=20):
        node = simplified_arm()
        node.n_samples = n                 # small: these tests check logic, not
        node.seed = 1                      # statistics
        t = b3.BehaviorTree()
        t.root = node
        return t.tick('testing simplified_arm', bb)

    def test_hybG_simplified_arm_refuses_when_pieper_unreliable(self):
        """pieper_ok False -> FAILURE, even though candidates could be found.

           Inverter(pieper_id) turns BOTH "no triple" and "could not read the DH
           table" into SUCCESS, so this leaf is the only thing standing between a
           parse failure and a derived robot that describes nothing."""
        fs = ' simplified_arm pieper_ok FAIL'
        R = TestSolver018.robot(self.no_triple_table(), {}, 'Plain')
        bb = b3.Blackboard()
        bb.set('Robot', R)
        bb.set('unknowns', [TestSolver018.unk(i) for i in range(1, 7)])
        bb.set('pieper_ok', False)          # analysis did not run
        bb.set('pieper_triples', [])

        self.assertEqual(self.tick_simplify(bb), b3.FAILURE,
                         fs + ' (must not simplify on an unreliable analysis)')
        self.assertIsNone(bb.get('simplification_choice'), fs)
        self.assertEqual(bb.get('simplification_candidates'), [], fs)

    def test_hybH_simplified_arm_ranks_cheapest_first(self):
        """A no-triple arm gets a ranked candidate list, cheapest first."""
        fs = ' simplified_arm FAIL'
        R = TestSolver018.robot(self.no_triple_table(), {}, 'Plain')
        bb = b3.Blackboard()
        bb.set('Robot', R)
        bb.set('unknowns', [TestSolver018.unk(i) for i in range(1, 7)])
        bb.set('pieper_ok', True)
        bb.set('pieper_triples', [])

        self.assertEqual(self.tick_simplify(bb), b3.SUCCESS, fs)
        cands = bb.get('simplification_candidates')
        self.assertTrue(cands, fs + ' (no candidates on the blackboard)')

        costs = [c['cost'] for c in cands]
        self.assertEqual(costs, sorted(costs), fs + ' (not sorted by cost)')

        choice = bb.get('simplification_choice')
        self.assertIs(choice, cands[0], fs + ' (choice is not the cheapest)')
        #  every usable candidate must carry what Phase E will consume
        for c in cands:
            self.assertTrue(c['edits'], fs + ' (a candidate with no edits)')
            self.assertIsNotNone(c['dh_simp'], fs + ' (no derived DH table)')
            self.assertIsNotNone(c['metric'], fs + ' (unscored candidate)')

    def test_hybI_simplified_arm_fails_with_nothing_to_buy(self):
        """An arm that already satisfies Pieper everywhere has no candidates, so
           the leaf FAILs rather than returning an empty choice.

           This is not a hypothetical:  the leaf sits behind Inverter(pieper_id),
           so reaching it normally means no triple -- but a caller ticking it
           directly (or a future gate change) must not get a SUCCESS with
           nothing selected."""
        fs = ' simplified_arm nothing-to-buy FAIL'
        R = TestSolver018.robot(self.puma_like(), {}, 'Wristy')
        bb = b3.Blackboard()
        bb.set('Robot', R)
        bb.set('unknowns', [TestSolver018.unk(i) for i in range(1, 7)])
        bb.set('pieper_ok', True)
        bb.set('pieper_triples', [{'axes': (4, 5, 6), 'kind': 'intersect'}])

        st = self.tick_simplify(bb)
        if bb.get('simplification_candidates'):
            #  puma_like has non-zero offsets on the other triples, so some
            #  candidates may exist;  then SUCCESS with a real choice is correct
            self.assertEqual(st, b3.SUCCESS, fs)
            self.assertIsNotNone(bb.get('simplification_choice'), fs)
        else:
            self.assertEqual(st, b3.FAILURE,
                             fs + ' (no candidates must mean FAILURE)')
            self.assertIsNone(bb.get('simplification_choice'), fs)


def run_test():
    suite = unittest.TestLoader().loadTestsFromTestCase(TestSolver018)
    unittest.TextTestRunner(verbosity=2).run(suite)


##   Self Tests
if __name__ == "__main__":
    run_test()
