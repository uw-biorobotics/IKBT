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
#       no_pieper_id      does this robot LACK a Pieper triple?  The branch is
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
from ikbtbasics.ik_classes  import kinematics_pickle
from ikbtfunctions.ik_robots import robot_params


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


class no_pieper_id(b3.Action):
    """Identify the ABSENCE of a Pieper triple -- the condition that makes an
       arm a candidate for the hybrid branch.

           SUCCESS  the DH table was analysed and NO triple qualifies
           FAILURE  a triple qualifies, or the table could not be analysed

       Either way it publishes `pieper_triples`, `pieper_ok` and `pieper_latex`,
       which downstream leaves and the report read regardless of the verdict.

       WHY THE POLARITY IS THIS WAY ROUND (BH, 2026-08-23).  This was
       `pieper_id` -- SUCCESS when a triple EXISTS -- used as
       `Sequence[Inverter(pieper_id), ...]`.  That double negative was the only
       Inverter in the whole tree and it made the branch hard to read:  you had
       to hold "succeeds when it finds one, and we want the ones where it does
       not" in your head at the point where the tree should just say what it
       gates on.  Nothing anywhere needed the other polarity -- `pieper_id` had
       exactly one instance and exactly one use -- so the node now answers the
       question the tree actually asks:

           hybrid_branch = Sequence[ no_pieper_id, ... ]

       It keeps the `_id` suffix because that is what it does:  it identifies a
       condition and stashes what it learned.  The condition is just a negative
       one, and the name says so rather than an operator elsewhere implying it.

       FAILING on "could not analyse" is the second thing the swap buys, and it
       is a real correctness gain, not just tidiness.  `Inverter` could not tell
       "no triple" from "I could not read the table" -- both became SUCCESS --
       so the branch opened on a parse failure and `simplified_arm` had to catch
       it one node later via `pieper_ok`.  Now "I could not tell" is a FAILURE
       here, at the gate, where it belongs.  simplified_arm still checks
       `pieper_ok`;  that check is now defence in depth rather than the only
       thing standing between a parse failure and a derived robot describing
       nothing.

       WHY IT GATES ONLY THE HYBRID.  Pieper's condition is *sufficient* for a
       closed form to exist and is not known to be necessary, so its absence
       must never gate the SYMBOLIC solver.  Measured: 9 of the 32 robots in ROBOT_LIST
       have no triple and still solve completely (Axtman13, Brad, DZhang,
       ICP5p5_A21, Mackler13, MiniDD, Olson13, Sims11, Wachtveitl).  Putting
       this leaf ahead of the symbolic branch in a Sequence stops that branch
       ticking at all for those nine.

       FAILURE means EITHER "there is a triple" or "could not tell", so
       `pieper_ok` on the blackboard still distinguishes the two for anything
       that needs to know which.

       It ALSO stashes the report's geometry statement on the blackboard as
       `pieper_latex`, and that is not redundant with report_gen generating its
       own.  On the hybrid path `install_simplified` replaces the blackboard's
       Robot with a DERIVED arm, so by the time report_gen runs, describing
       "the robot" would describe the simplified one -- the wrong answer, and a
       quietly misleading report.  The statement made here describes the arm we
       actually started from, and report_gen prefers it when present.
       clear_state keeps the key for exactly this reason."""

    def __init__(self):
        super(no_pieper_id, self).__init__()
        self.Name = 'No Pieper Triple ID'
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

            #  Snapshot the report section for the TRUE robot, before anything
            #  downstream swaps the Robot for a derived one.
            bb.set('pieper_latex',
                   da.pieper_latex(M.DH, M.pvals, ndof, getattr(R, 'name', None)))

            if self.BHdebug:
                print('\n', self.Name, ':', getattr(R, 'name', '?'), '->',
                      [(t['axes'], t['kind']) for t in triples] or 'no triples')

            #  Inverted relative to the old pieper_id:  SUCCESS is "no
            #  triple", which is what the hybrid branch gates on.
            return b3.FAILURE if triples else b3.SUCCESS

        except Exception as e:
            #  Reported, never raised.  FAILURE, because "I could not read the
            #  table" is not "there is no triple" -- the old Inverter could not
            #  make that distinction and let the branch open on a parse failure.
            #  pieper_ok stays False for anything that wants to know which.
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

REFUSES TO ACT WHEN `pieper_ok` IS FALSE.  Defence in depth:  `no_pieper_id`
       now FAILs on a table it could not analyse, so the branch should never
       open in that case at all.  The check stays because the cost of being
       wrong here is a derived robot that describes nothing, and this leaf is
       the first thing that would act on the conclusion.  Simplifying
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


class install_simplified(b3.Action):
    """Build the derived robot from simplification_choice and install it on the
       blackboard, so the solver that follows solves the SIMPLIFIED arm.

           SUCCESS  blackboard now carries the derived Robot, a fresh unknown
                    list, freshly scanned equation lists, and `hybrid_source`
                    describing what was changed
           FAILURE  no choice on the blackboard, or the derived FK could not be
                    built

       Two things must be fresh, and neither is the node set:

       FRESH `unknown` OBJECTS.  `set_solved()` mutates them in place and
       `create_solution_set()` appends to `LHSversionNames`, so the outer
       solve's objects cannot be reused even though that solve failed --
       tan/sincos leaves may have recorded candidate solutions on them.  We call
       robot_params() again for a clean set, through number_unknowns().

       ITS OWN PICKLE NAME, encoding the change (e.g. `KinovaLite_d5_0`).
       kinematics_pickle() takes the name independently of the DH table, so a
       derived arm gets its own cache entry;  sharing the true robot's name would
       either serve the wrong FK or throw the cache away on every run.  The name
       is also what a human reads in fk_eqns/.

       `hybrid_source` is left on the blackboard for report_gen and for
       scripts/robot_baseline.py:  a solve of a derived arm must not be recorded,
       or reported, as though it solved the real one."""

    def __init__(self):
        super(install_simplified, self).__init__()
        self.Name = 'Install Simplified Arm'
        self.BHdebug = False

    def suffix(self, choice):
        """A pickle-name suffix naming what changed:  'd_5_0', 'al_4_snap'."""
        bits = []
        for e in choice['edits']:
            nm = str(e['symbol'])
            bits.append(nm + ('_0' if e['kind'] == 'zero' else '_snap'))
        return '_'.join(bits) or 'simplified'

    def tick(self, tick):
        bb = tick.blackboard
        choice = bb.get('simplification_choice')
        R_true = bb.get('Robot')

        if not choice or choice.get('dh_simp') is None:
            print(self.Name, ': no simplification chosen -- nothing to install.')
            return b3.FAILURE

        true_name = getattr(R_true, 'name', None)
        if not true_name:
            print(self.Name, ': the blackboard Robot has no name;  cannot derive'
                  ' a pickle name from it.')
            return b3.FAILURE

        dname = '%s_%s' % (true_name, self.suffix(choice))

        try:
            #  Fresh unknown objects -- see the class docstring.  robot_params()
            #  numbers them via number_unknowns() on the way out.
            dh, vv, params, pvals, unks = robot_params(true_name)
            dh = choice['dh_simp']

            M2, R2, unks = kinematics_pickle(dname, dh, params, pvals, vv,
                                             unks, False)
            R2.name = dname
            R2.params = params

            L1, L2, L3p = R2.scan_for_equations(unks)
        except Exception as e:
            print(self.Name, ': could not build the derived robot --',
                  '%s: %s' % (type(e).__name__, e))
            return b3.FAILURE

        bb.set('Robot', R2)
        bb.set('unknowns', unks)
        bb.set('eqns_1u', L1)
        bb.set('eqns_2u', L2)
        bb.set('eqns_3pu', L3p)

        #  What the report and the baseline record need in order to be honest
        #  about which arm the equations describe.
        bb.set('hybrid_source', {
            'true_robot': true_name,
            'derived_robot': dname,
            'axes': choice['axes'],
            'kind': choice['kind'],
            'route': choice['route'],
            'edits': [{'symbol': str(e['symbol']),
                       'from': str(e['from']),
                       'to': str(e['to']),
                       'delta': e['delta']} for e in choice['edits']],
            'cost': choice.get('cost'),
        })

        print('\n', self.Name, ': solving', dname, 'instead --',
              da.describe_edits(choice))

        if self.BHdebug:
            print('   eqns 1u/2u/3pu = %d/%d/%d   unknowns = %d'
                  % (len(L1), len(L2), len(L3p), len(unks)))

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
        self.test_hybC_no_pieper_id_fails_when_it_cannot_tell()
        self.test_hybD_no_pieper_id_fails_on_the_wrist()
        self.test_hybE_no_pieper_id_ignores_sum_of_angle_unknowns()
        self.test_hybF_no_pieper_id_succeeds_with_no_triple()
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
           the tree, so a leaf that FAILs becomes a SUCCESS under
           the inverter.  Recorded here because it is the easy thing to get
           backwards -- the hybrid runs when there is NO triple.'''
        fs = ' hybrid_stub inverter FAIL'
        t = b3.BehaviorTree()
        t.root = b3.Inverter(hybrid_stub())
        self.assertEqual(t.tick('testing inverter', b3.Blackboard()),
                         b3.SUCCESS, fs)


    #  ------------------------------------------------  no_pieper_id

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
        node = no_pieper_id()
        t = b3.BehaviorTree()
        t.root = node
        return t.tick('testing no_pieper_id', bb)

    def no_triple_table(self):
        """A 6R table where no triple qualifies:  every a and d non-zero and no
           sin(alpha) zero."""
        import sympy as sp
        return sp.Matrix([[sp.pi/2, sp.Integer(3), sp.Integer(4),
                           sp.Symbol('th_%d' % (r+1))] for r in range(6)])

    def test_hybC_no_pieper_id_fails_when_it_cannot_tell(self):
        """FAILURE on a blackboard with no Robot -- and pieper_ok stays False.

           This is the case the old Inverter(pieper_id) could not express.  It
           turned "could not read the table" into SUCCESS, opening the hybrid
           branch on a parse failure.  Now the gate itself FAILs, so an
           unreadable table and a genuine triple both keep the branch shut --
           and pieper_ok is what tells them apart for anything that cares."""
        fs = ' no_pieper_id FAIL'
        bb = b3.Blackboard()
        self.assertEqual(self.tick_pieper(bb), b3.FAILURE,
                         fs + ' (could-not-tell must NOT open the hybrid)')
        self.assertFalse(bb.get('pieper_ok'),
                         fs + ' (pieper_ok must be False when it could not run)')
        self.assertEqual(bb.get('pieper_triples'), [],
                         fs + ' (triples must still be a list)')

    def test_hybD_no_pieper_id_fails_on_the_wrist(self):
        """A spherical wrist HAS a triple -> FAILURE, so the hybrid branch
           correctly declines to touch this arm.  No Inverter involved."""
        fs = ' no_pieper_id wrist FAIL'
        R = TestSolver018.robot(self.puma_like(), {}, 'Wristy')
        bb = b3.Blackboard()
        bb.set('Robot', R)
        bb.set('unknowns', [TestSolver018.unk(i) for i in range(1, 7)])

        self.assertEqual(self.tick_pieper(bb), b3.FAILURE,
                         fs + ' (an arm WITH a triple must not enter the hybrid)')
        self.assertTrue(bb.get('pieper_ok'), fs + ' (analysis should have run)')
        got = [(t['axes'], t['kind']) for t in bb.get('pieper_triples')]
        self.assertIn(((4, 5, 6), 'intersect'), got, fs + ' (missed the wrist)')

    def test_hybE_no_pieper_id_ignores_sum_of_angle_unknowns(self):
        """DOF count must skip the sum-of-angles unknowns.

           kinematics_pickle() EXTENDS the unknown list with th_23 (n = 23) and
           friends.  Counting those would inflate ndof past 6 and invent triples
           over the zero-padded rows."""
        fs = ' no_pieper_id SOA FAIL'
        R = TestSolver018.robot(self.puma_like(), {}, 'Soa')
        bb = b3.Blackboard()
        bb.set('Robot', R)
        bb.set('unknowns', [TestSolver018.unk(i) for i in range(1, 7)]
                           + [TestSolver018.unk(23), TestSolver018.unk(234)])

        #  This table HAS a wrist, so the gate FAILs;  what is under test is
        #  that no triple names an axis above 6.
        self.assertEqual(self.tick_pieper(bb), b3.FAILURE, fs)
        axes = [t['axes'] for t in bb.get('pieper_triples')]
        self.assertTrue(all(a[2] <= 6 for a in axes),
                        fs + ' (a triple names an axis above 6: %s)' % axes)

    def test_hybF_no_pieper_id_succeeds_with_no_triple(self):
        """No triple -> SUCCESS, and the hybrid branch is admitted.  pieper_ok
           is True: this is a real answer, not a failure to run."""
        fs = ' no_pieper_id no-triple FAIL'
        R = TestSolver018.robot(self.no_triple_table(), {}, 'Plain')
        bb = b3.Blackboard()
        bb.set('Robot', R)
        bb.set('unknowns', [TestSolver018.unk(i) for i in range(1, 7)])

        self.assertEqual(self.tick_pieper(bb), b3.SUCCESS,
                         fs + ' (no triple is exactly what the hybrid wants)')
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

           no_pieper_id FAILs on "could not read the DH table", but the DH
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

           This is not a hypothetical:  the leaf sits behind no_pieper_id,
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
