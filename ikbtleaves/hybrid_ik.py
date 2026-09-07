#!/usr/bin/python
#
#   hybrid_ik.py --  the hybrid symbolic-numeric branch
#
#   When IKBT cannot solve a robot in closed form:  simplify its DH parameters
#   until it can, solve the SIMPLIFIED robot symbolically, and correct the
#   result numerically using the symbolic Jacobian.  11 of the 32 robots in
#   ROBOT_LIST have no Pieper triple of any kind, and where that is what blocks
#   the solve a single DH parameter is usually responsible -- KinovaLite with
#   d_5 -> 0 goes from 0 of 7 variables solved to all 7.
#
#   The pieces, in the order the branch ticks them:
#
#       pieper_geom_report  does this robot have a Pieper triple?  REPORTING
#                         ONLY -- always SUCCESS;  the branch does not depend
#                         on the answer.  See its docstring.
#       simplified_arm    which single DH parameter costs the least task-space
#                         displacement to zero or snap?  THIS leaf decides
#                         whether the branch proceeds.
#       install_simplified build the derived robot and swap it in
#       (the symbolic solver again, over its own leaf set)
#       then report_gen -- ikbtfunctions/output_hybrid_python.py -- emits the
#       closed form for the derived arm and the damped-least-squares correction
#       against the true one.  Every artifact is named for the arm it describes;
#       emitting the derived arm's closed form as though it were the real robot
#       is the one thing this method must never do.
#
#   Copyright 2026 University of Washington
#
#   Developed by Blake Hannaford
#   BioRobotics Lab, University of Washington

import b3 as b3          # behavior trees

import ikbtbasics.dh_analysis as da
from ikbtbasics.ik_classes  import kinematics_pickle
from ikbtfunctions.ik_robots import robot_params


class pieper_geom_report(b3.Action):
    """Analyse the arm's joint-axis geometry and publish it.  ALWAYS SUCCESS.

       Publishes, whatever it finds:

           pieper_triples   the triples of consecutive joint axes that
                            intersect or are parallel (Pieper's condition), [] if none
           pieper_ok        True iff the DH table was actually analysed --
                            "no triple" is an ANSWER, "could not read the
                            table" is not, and only this flag separates them
           pieper_latex     the report's geometry statement, for the TRUE robot

       WHY IT REPORTS AND DOES NOT DECIDE (BH, 2026-08-31).  This was
       `no_pieper_id`, SUCCESS iff the arm had NO triple, sitting bare at the
       head of `hybrid_branch` so that a `b3.Sequence` abort turned it into a
       gate:  admit the hybrid branch only for an arm with no triple, because an
       arm that HAS one and still failed symbolically is an IKBT defect rather
       than a geometry problem, and simplifying it would hand back an
       approximate arm for a robot whose exact arm is solvable in principle --
       masking our own bug as an answer.

       That reasoning fails on the same asymmetry that keeps this leaf away from
       the symbolic branch.  Pieper's condition is SUFFICIENT for a closed form
       and is not known to be necessary;  so its absence proves nothing, AND its
       presence proves nothing either.  Measured over the unsolved set, ArmRobo
       (triples (2,3,4) and (4,5,6)), Panda ((1,2,3)) and Raven-II
       ((1,2,3),(2,3,4),(3,4,5)) all satisfy the condition and all solve 0 of
       their unknowns -- so for the three arms the gate turned away, its answer
       was "this is our bug, so you get nothing", which is strictly worse for
       the user than a numerically corrected hybrid answer that names the
       parameter it changed.

       So the leaf returns SUCCESS unconditionally and the branch is gated on an
       OUTCOME instead:  simplified_arm finding a usable candidate.  That is not
       a weaker gate.  candidate_simplifications() skips triples that already
       qualify, so an arm satisfying the condition on EVERY triple yields nothing
       usable and the branch closes there by itself -- see
       TestSolver018.test_hybJ.  An unconditional SUCCESS is also why there is no
       `Priority[..., Succeeder]` wrapper around this node:  nothing has to
       swallow a verdict that is never adverse.

       WHY IT MUST STILL NOT GATE THE SYMBOLIC BRANCH.  Same asymmetry, other
       direction, and this one is measured:  9 of the 32 robots in ROBOT_LIST
       have no triple and still solve completely (Axtman13, Brad, DZhang,
       ICP5p5_A21, Mackler13, MiniDD, Olson13, Sims11, Wachtveitl).  A leaf
       whose FAILURE could abort the symbolic branch would stop that branch
       ticking at all for those nine.  This one cannot FAIL, so it could not do
       that even if it were moved -- which is a small safety property worth
       having, not an invitation to move it.

       AND THERE IS NO INVERTER.  The node began as `pieper_id` (SUCCESS when a
       triple EXISTS) under `b3.Inverter` -- the only Inverter in the tree.  The
       polarity was folded into the node, then the verdict dropped entirely;
       tests/bt_assembly_test.py asserts the tree still contains no Inverter.
       What the Inverter could never express survives as `pieper_ok`:  it turned
       "could not read the table" into SUCCESS, indistinguishable from "no
       triple".  With no gate left, that flag is the WHOLE of the protection --
       simplified_arm reads it and refuses -- so it must stay False for
       "could not run" and never be set optimistically.

       `pieper_latex` IS WHY THE LEAF STILL TICKS FIRST IN THE BRANCH.  On the
       hybrid path `install_simplified` replaces the blackboard's Robot with a
       DERIVED arm, so by the time report_gen runs, describing "the robot" would
       describe the simplified one -- the wrong answer, and a quietly misleading
       report.  The statement made here describes the arm we actually started
       from, and report_gen prefers it when present.  clear_state keeps the key
       for exactly this reason.  "Had a triple, failed symbolically, and the
       simplified arm solved" is also the report line that identifies a solver
       gap and points at it, which the gate used to throw away."""

    def __init__(self):
        super(pieper_geom_report, self).__init__()
        self.Name = 'Pieper Geometry Report'
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

        except Exception as e:
            #  Reported, never raised, and STILL SUCCESS:  this leaf reports, it
            #  does not decide.  pieper_ok is left False, which is what stops
            #  simplified_arm from deriving a robot out of a table nobody read.
            print(self.Name, ': could not analyse the DH table --',
                  '%s: %s' % (type(e).__name__, e))

        #  Unconditional.  Having a triple does not mean IKBT can solve the arm,
        #  so nothing here is grounds for refusing the hybrid branch.
        return b3.SUCCESS


class simplified_arm(b3.Action):
    """Rank the DH changes that would give this arm a Pieper triple, cheapest
       first, and put the ranked list on the blackboard.

           SUCCESS  at least one usable candidate.  Blackboard carries
                    `simplification_candidates` (ranked, cheapest first) and
                    `simplification_choice` (the winner), each candidate holding
                    its edits, their numeric magnitudes, the derived DH table and
                    its measured task-space cost.
           FAILURE  no usable candidate, or the Pieper analysis was unreliable.

       THIS LEAF IS THE HYBRID BRANCH'S GATE (BH, 2026-08-31).
       pieper_geom_report ahead of it reports and never FAILs, so admission
       rests entirely on the two refusals here, and both are load-bearing:

       REFUSES TO ACT WHEN `pieper_ok` IS FALSE.  Not defence in depth any more
       -- it is the only defence.  Simplifying an arm because we failed to parse
       its parameters would be the worst kind of silent wrong answer:  a derived
       robot, solved perfectly, describing nothing.

       REFUSES WHEN THERE IS NOTHING TO BUY, which is what closes the branch on
       an arm that already satisfies Pieper's condition everywhere --
       candidate_simplifications() skips triples that already qualify, so such an
       arm yields no usable candidate.  That is the property that made dropping
       the Pieper gate safe;  test_hybJ pins it.

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

       ITS OWN PICKLE NAME, encoding the change (e.g. `KinovaLite_d_5_0`).
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

        print('\n\n        HYBRID Solution Started:  now solving simplified arm\n\n')

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

        #  What the report needs in order to say which arm the equations
        #  describe.
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
    '''The hybrid branch:  geometry reporting, and the gate that admits it.'''

    def setUp(self):
        print('\n\n===============  Test hybrid branch  =====================')
        return

    def runTest(self):
        self.test_hybC_pieper_geom_report_succeeds_when_it_cannot_tell()
        self.test_hybD_pieper_geom_report_names_the_wrist()
        self.test_hybE_pieper_geom_report_ignores_sum_of_angle_unknowns()
        self.test_hybF_pieper_geom_report_with_no_triple()
        self.test_hybG_simplified_arm_refuses_when_pieper_unreliable()
        self.test_hybH_simplified_arm_ranks_cheapest_first()
        self.test_hybI_simplified_arm_fails_with_nothing_to_buy()
        self.test_hybJ_every_triple_qualifies_closes_the_branch()

    #  ------------------------------------------------  pieper_geom_report

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
        node = pieper_geom_report()
        t = b3.BehaviorTree()
        t.root = node
        return t.tick('testing pieper_geom_report', bb)

    def no_triple_table(self):
        """A 6R table where no triple qualifies:  every a and d non-zero and no
           sin(alpha) zero."""
        import sympy as sp
        return sp.Matrix([[sp.pi/2, sp.Integer(3), sp.Integer(4),
                           sp.Symbol('th_%d' % (r+1))] for r in range(6)])

    def every_triple_table(self):
        """A 6R table where EVERY triple qualifies:  all a and all d zero, so
           every set of three consecutive axes is concurrent.

           This is the arm the dropped Pieper gate used to turn away, and the
           one that proves simplified_arm closes the branch on its own."""
        import sympy as sp
        return sp.Matrix([[sp.pi/2, sp.Integer(0), sp.Integer(0),
                           sp.Symbol('th_%d' % (r+1))] for r in range(6)])

    def test_hybC_pieper_geom_report_succeeds_when_it_cannot_tell(self):
        """A blackboard with no Robot:  STILL SUCCESS, and pieper_ok stays False.

           The leaf reports, it does not decide, so an unreadable table is not
           grounds for aborting the branch -- and it must not raise either.
           `pieper_ok` carries the whole of the protection that the verdict used
           to: simplified_arm reads it and refuses, which is the only thing
           between a table nobody parsed and a derived robot describing nothing.
           So False here is load-bearing, and never confusable with "no triple"
           (which is a real answer, with pieper_ok True -- see hybF)."""
        fs = ' pieper_geom_report FAIL'
        bb = b3.Blackboard()
        self.assertEqual(self.tick_pieper(bb), b3.SUCCESS,
                         fs + ' (a reporting leaf must never FAIL)')
        self.assertFalse(bb.get('pieper_ok'),
                         fs + ' (pieper_ok must be False when it could not run)')
        self.assertEqual(bb.get('pieper_triples'), [],
                         fs + ' (triples must still be a list)')

    def test_hybD_pieper_geom_report_names_the_wrist(self):
        """A spherical wrist HAS a triple:  found, named, and STILL SUCCESS.

           This is the arm the dropped gate used to turn away.  Having a triple
           is no longer grounds for refusing the hybrid branch (see hybJ for the
           node that actually closes it), but the triple must still be named,
           because the report says so."""
        fs = ' pieper_geom_report wrist FAIL'
        R = TestSolver018.robot(self.puma_like(), {}, 'Wristy')
        bb = b3.Blackboard()
        bb.set('Robot', R)
        bb.set('unknowns', [TestSolver018.unk(i) for i in range(1, 7)])

        self.assertEqual(self.tick_pieper(bb), b3.SUCCESS,
                         fs + ' (a triple must not make this leaf FAIL -- that'
                         ' was the gate)')
        self.assertTrue(bb.get('pieper_ok'), fs + ' (analysis should have run)')
        got = [(t['axes'], t['kind']) for t in bb.get('pieper_triples')]
        self.assertIn(((4, 5, 6), 'intersect'), got, fs + ' (missed the wrist)')

    def test_hybE_pieper_geom_report_ignores_sum_of_angle_unknowns(self):
        """DOF count must skip the sum-of-angles unknowns.

           kinematics_pickle() EXTENDS the unknown list with th_23 (n = 23) and
           friends.  Counting those would inflate ndof past 6 and invent triples
           over the zero-padded rows."""
        fs = ' pieper_geom_report SOA FAIL'
        R = TestSolver018.robot(self.puma_like(), {}, 'Soa')
        bb = b3.Blackboard()
        bb.set('Robot', R)
        bb.set('unknowns', [TestSolver018.unk(i) for i in range(1, 7)]
                           + [TestSolver018.unk(23), TestSolver018.unk(234)])

        #  This table HAS a wrist;  what is under test is that no triple
        #  names an axis above 6.
        self.assertEqual(self.tick_pieper(bb), b3.SUCCESS, fs)
        axes = [t['axes'] for t in bb.get('pieper_triples')]
        self.assertTrue(all(a[2] <= 6 for a in axes),
                        fs + ' (a triple names an axis above 6: %s)' % axes)

    def test_hybF_pieper_geom_report_with_no_triple(self):
        """No triple:  an empty triple list, and pieper_ok TRUE.

           The pair (SUCCESS, pieper_ok=True, triples=[]) is what "this arm has
           no triple" looks like now that the verdict is constant.  Contrast
           hybC, where the SAME verdict comes with pieper_ok False -- the flag is
           the only thing distinguishing a real answer from a failure to run."""
        fs = ' pieper_geom_report no-triple FAIL'
        R = TestSolver018.robot(self.no_triple_table(), {}, 'Plain')
        bb = b3.Blackboard()
        bb.set('Robot', R)
        bb.set('unknowns', [TestSolver018.unk(i) for i in range(1, 7)])

        self.assertEqual(self.tick_pieper(bb), b3.SUCCESS, fs)
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

           pieper_geom_report cannot FAIL on "could not read the DH table"
           any more, so this refusal is the only thing standing between a
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
        #  every usable candidate must carry what install_simplified consumes
        for c in cands:
            self.assertTrue(c['edits'], fs + ' (a candidate with no edits)')
            self.assertIsNotNone(c['dh_simp'], fs + ' (no derived DH table)')
            self.assertIsNotNone(c['metric'], fs + ' (unscored candidate)')

    def test_hybI_simplified_arm_fails_with_nothing_to_buy(self):
        """An arm that already satisfies Pieper everywhere has no candidates, so
           the leaf FAILs rather than returning an empty choice.

           THIS IS THE HYBRID BRANCH'S GATE, as of 2026-08-31.
           pieper_geom_report no longer decides admission -- it cannot FAIL --
           so every arm that failed symbolically
           reaches this leaf -- including arms that have a qualifying triple.
           candidate_simplifications() skips triples that already qualify, so an
           arm satisfying the condition on every triple yields nothing usable
           and the branch closes here, which is what makes dropping the Pieper
           gate safe.  Returning SUCCESS with nothing selected would hand
           install_simplified a None choice."""
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

    def test_hybJ_every_triple_qualifies_closes_the_branch(self):
        """THE PROPERTY THAT MAKES DROPPING THE PIEPER GATE SAFE.

           An arm whose every triple already satisfies Pieper's condition is
           exactly the population the old `Sequence[no_pieper_id, ...]` gate
           (now pieper_geom_report, verdict dropped)
           existed to keep out of the hybrid branch.  With the gate gone, the
           branch must still close on that arm -- and it does, one node later,
           because candidate_simplifications() skips triples that already
           qualify, leaving nothing to buy.  Unconditional, unlike hybI: this
           table has NO triple with a candidate, so FAILURE is the only correct
           answer and the test does not have to hedge."""
        fs = ' simplified_arm every-triple FAIL'
        R = TestSolver018.robot(self.every_triple_table(), {}, 'AllWrist')
        bb = b3.Blackboard()
        bb.set('Robot', R)
        bb.set('unknowns', [TestSolver018.unk(i) for i in range(1, 7)])

        #  the arm HAS triples, and the leaf reports them without refusing
        #  anything
        self.assertEqual(self.tick_pieper(bb), b3.SUCCESS, fs)
        self.assertTrue(bb.get('pieper_triples'),
                        fs + ' (fixture reports no triple at all)')
        self.assertTrue(bb.get('pieper_ok'), fs + ' (analysis did not run)')

        self.assertEqual(self.tick_simplify(bb), b3.FAILURE,
                         fs + ' (nothing to simplify must close the branch)')
        self.assertEqual(bb.get('simplification_candidates'), [], fs)
        self.assertIsNone(bb.get('simplification_choice'), fs)


def run_test():
    suite = unittest.TestLoader().loadTestsFromTestCase(TestSolver018)
    unittest.TextTestRunner(verbosity=2).run(suite)


##   Self Tests
if __name__ == "__main__":
    run_test()
