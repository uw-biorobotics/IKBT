#!/usr/bin/python
#
#   onevar_ik.py --  the one-variable branch
#
#   When IKBT cannot solve a robot in closed form, ONE of its unknowns is often
#   all that stands in the way:  declare that variable known and the rest of the
#   arm falls out symbolically.  The solution is then closed form in six
#   variables and one parameter, and a 1-D numerical search over that parameter
#   finds the poses where it is actually a solution of the TRUE arm.
#
#   Friedman, D.C.W., Kowalewski, T., Jovanovic, R., Rosen, J. and Hannaford, B.
#   "Freeing the serial mechanism designer from inverse kinematic solvability
#   constraints", Applied Bionics and Biomechanics 7(3), 2010, 209-216.
#   (IKdocs/ holds the PDF;  C-Arm in ik_robots.py is that paper's arm.)
#
#   The pieces, in the order the branch ticks them:
#
#       onevar_rank    which unknown, declared known, restocks the most
#                      one-unknown equations?  THIS leaf decides whether the
#                      branch proceeds.
#       install_known  declare the next candidate known and install the reduced
#                      problem, so the solver that follows solves 5 unknowns
#                      instead of 6
#       (the symbolic solver again, over its own leaf set)
#
#   NOTHING ABOUT THE ROBOT CHANGES.  The DH table, the FK and the equations are
#   the true arm's throughout -- the only edit is to the list of unknowns, and
#   `count_unknowns()` reads that list, so a symbol missing from it is already a
#   constant everywhere downstream.  That is the whole mechanism, and it is why
#   this branch needs no derived robot and no second FK (contrast
#   hybrid_ik.install_simplified, which must build both).
#
#   WHAT COMES OUT IS A CONDITIONAL CLOSED FORM, exact only on the 1-D variety
#   where the assumed value is right.  It must never be written out under the
#   name of an unconditional solution -- the same rule the hybrid branch's
#   artifact naming exists to enforce.
#
#   Copyright 2026 University of Washington
#
#   Developed by Blake Hannaford
#   BioRobotics Lab, University of Washington

import sympy as sp

import b3 as b3          # behavior trees

from ikbtbasics.ik_classes   import kinematics_pickle
from ikbtfunctions.ik_robots import robot_params


def fresh_problem(name):
    '''A pristine (Robot, unknowns) for `name`, straight from the FK cache.

       A solve MUTATES both:  `set_solved()` writes the unknowns, and the Robot
       accumulates solveN, solution_nodes and graph edges.  A failed attempt
       leaves all of it behind, so every attempt here starts from a reload
       rather than from whatever the last one left on the blackboard.

       Cheap enough to do per attempt:  kinematics_pickle() serves
       fk_eqns/<name>_pickle.p, measured at 20-30 ms against the minutes a
       symbolic attempt costs.'''

    dh, vv, params, pvals, unks = robot_params(name)
    M, R, unks = kinematics_pickle(name, dh, params, pvals, vv, unks, False)
    R.name   = name
    R.params = params
    return R, unks


def rank_by_restock(counts, min_new=1, limit=None):
    '''Sort candidate dicts best-first and drop the ones not worth a solve.

       Key:  most new 1-unknown equations, then most 2-unknown, then the
       SIMPLEST of those equations (count_ops), then chain position.

       n1 leads because a solver leaf can only start from an equation in one
       unknown:  a robot that fails symbolically usually has an empty L1 (C-Arm
       has 0 of 63), and declaring the right variable known is precisely the act
       that restocks it.  n2 breaks the common tie -- those are next pass's L1.
       count_ops then prefers short equations over long ones, which is the same
       preference erank() applies inside every solver leaf.  u.n last, so the
       order is reproducible rather than dict-ordered.

       min_new=1 drops a candidate that restocks NOTHING.  It is not a
       prediction that the others will solve;  it is a refusal to spend a full
       symbolic attempt proving that an unchanged L1 still cannot be started.'''

    usable = [c for c in counts if c['n1'] >= min_new]
    usable.sort(key=lambda c: (-c['n1'], -c['n2'], c['ops'], c['index']))
    return usable if limit is None else usable[:limit]


class onevar_rank(b3.Action):
    '''Rank the unknowns by what declaring each one known would restock, and
       put the ranked list on the blackboard.

           SUCCESS  at least one candidate worth attempting.  Blackboard carries
                    `onevar_candidates` (ranked, best first) and `onevar_cursor`.
           FAILURE  no robot to reload, or no candidate restocks anything.

       THIS LEAF IS THE BRANCH'S GATE, and the measurement is nearly free:  one
       scan_for_equations() per unknown, no sympy solving, milliseconds each.
       For C-Arm, which has no 1-unknown equation at all and so cannot even
       start:

           freeze th_2    L1/L2/L3p = 4/25/34        <- ranked first
           freeze th_6                4/20/39
           freeze th_34               4/24/36
           freeze th_5                3/23/37
           freeze th_3                2/9/53
           freeze d_1                 1/9/53
           freeze th_4                0/10/54        <- dropped

       SUM-OF-ANGLES VARIABLES RIDE ALONG as ordinary candidates (th_34 above).
       Assuming th_3 + th_4 known is as legitimate a one-parameter family as
       assuming a joint known, and it restocks as well;  what it costs is that
       th_3 and th_4 must then still come out individually, which the solver
       either manages or does not.  Nothing here has to decide that in advance.

       Ranked on a FRESH reload, not on the blackboard's Robot:  the attempt
       that follows starts from a reload too (install_known), so this measures
       the problem the solver will actually be handed, and it leaves the live
       Robot untouched -- scan_for_equations() writes R.l1/l2/l3p in place.'''

    def __init__(self):
        super(onevar_rank, self).__init__()
        self.Name = 'One Variable Rank'
        self.BHdebug = False

        #  How many candidates the branch may attempt.  The ranking predicts
        #  solvability, it does not guarantee it, so there is a fallback -- but
        #  each attempt is a full symbolic solve, and an arm with seven unknowns
        #  would otherwise cost seven of them.
        self.max_candidates = 3
        self.min_new_eqns = 1

    def tick(self, tick):
        bb = tick.blackboard
        R = bb.get('Robot')

        bb.set('onevar_candidates', [])
        bb.set('onevar_cursor', 0)

        name = getattr(R, 'name', None)
        if not name:
            print(self.Name, ': the blackboard Robot has no name, so there is'
                  ' nothing to reload.')
            return b3.FAILURE

        try:
            #  SystemExit too:  robot_params() calls quit() on a name that is
            #  not in ROBOT_LIST, and that must not take the whole run down.
            R0, unks = fresh_problem(name)
        except (Exception, SystemExit) as e:
            print(self.Name, f': could not reload {name} -- '
                  f'{type(e).__name__}: {e}')
            return b3.FAILURE

        counts = []
        for u in unks:
            reduced = [v for v in unks if v is not u]
            L1, L2, L3p = R0.scan_for_equations(reduced)
            counts.append({
                'robot':  name,
                'symbol': str(u.symbol),
                'index':  u.n,
                'n1':  len(L1),
                'n2':  len(L2),
                'n3p': len(L3p),
                'ops': sum(int(sp.count_ops(e.LHS)) + int(sp.count_ops(e.RHS))
                           for e in L1),
            })

        ranked = rank_by_restock(counts, min_new=self.min_new_eqns,
                                 limit=self.max_candidates)
        bb.set('onevar_candidates', ranked)

        if not ranked:
            print(self.Name, f': {name} -- declaring any one unknown known'
                  ' restocks no solvable equation.')
            return b3.FAILURE

        print('\n', self.Name, f': {name} -- best of {len(counts)} candidates'
              f" is {ranked[0]['symbol']} ({ranked[0]['n1']} new 1-unknown"
              f" equations, {ranked[0]['n2']} with two)")

        if self.BHdebug:
            print('   %-8s %5s %5s %6s %6s' % ('known', 'L1', 'L2', 'L3p', 'ops'))
            for c in ranked:
                print('   %-8s %5d %5d %6d %6d'
                      % (c['symbol'], c['n1'], c['n2'], c['n3p'], c['ops']))

        return b3.SUCCESS


class install_known(b3.Action):
    '''Declare the next ranked candidate known and install the reduced problem,
       so the solver that follows solves one unknown fewer.

           SUCCESS  blackboard carries a freshly reloaded Robot, an unknown list
                    with the candidate REMOVED, freshly scanned equation lists,
                    and `onevar_source` describing what was assumed
           FAILURE  no candidates, or the ranked list is exhausted

       The exhaustion FAILURE is what ends the retry loop:  the branch ticks
       this leaf once per attempt, and when the cursor runs off the end there is
       nothing left to try and the branch closes.

       Removing the unknown from the list is the entire edit.  Everything that
       asks "is this an unknown?" -- count_unknowns(), the solver leaves, and
       comp_det's completion test -- reads that list, so the symbol becomes a
       constant everywhere at once and "solved" comes to mean "all the others
       are solved".

       `onevar_source` is left on the blackboard for report_gen, and it is what
       stops a conditional solution being written out as an unconditional one.'''

    def __init__(self):
        super(install_known, self).__init__()
        self.Name = 'Install Known Variable'
        self.BHdebug = False

    def tick(self, tick):
        bb = tick.blackboard
        cands = bb.get('onevar_candidates') or []
        cursor = bb.get('onevar_cursor') or 0

        if cursor >= len(cands):
            if cands:
                print(self.Name, f': all {len(cands)} candidates have been'
                      ' tried.')
            else:
                print(self.Name, ': no candidates on the blackboard.')
            return b3.FAILURE

        c = cands[cursor]
        bb.set('onevar_cursor', cursor + 1)

        try:
            R, unks = fresh_problem(c['robot'])
        except (Exception, SystemExit) as e:
            print(self.Name, f": could not reload {c['robot']} -- "
                  f'{type(e).__name__}: {e}')
            return b3.FAILURE

        keep = [u for u in unks if str(u.symbol) != c['symbol']]
        if len(keep) == len(unks):
            print(self.Name, f": {c['symbol']} is not an unknown of"
                  f" {c['robot']} -- the ranking is stale.")
            return b3.FAILURE

        L1, L2, L3p = R.scan_for_equations(keep)

        bb.set('Robot', R)
        bb.set('unknowns', keep)
        bb.set('eqns_1u',  L1)
        bb.set('eqns_2u',  L2)
        bb.set('eqns_3pu', L3p)

        bb.set('onevar_source', {
            'robot':   c['robot'],
            'known':   c['symbol'],      # sp.Symbol(known) reconstructs it
            'attempt': cursor + 1,
            'of':      len(cands),
            'counts':  {k: c[k] for k in ('n1', 'n2', 'n3p', 'ops')},
        })

        print('\n\n        ONE VARIABLE Solution started:  assuming',
              c['symbol'], f"is known  (attempt {cursor + 1} of {len(cands)})\n")

        if self.BHdebug:
            print('   eqns 1u/2u/3pu = %d/%d/%d   unknowns = %d'
                  % (len(L1), len(L2), len(L3p), len(keep)))

        return b3.SUCCESS


#####################################################################
#
#   Test code
#

import unittest


class TestSolver028(unittest.TestCase):
    '''The one-variable branch:  the ranking, and the leaf that installs it.'''

    def setUp(self):
        print('\n\n===========  Test one-variable branch  ===================')
        return

    def runTest(self):
        self.test_ovA_ranking_prefers_the_biggest_restock()
        self.test_ovB_ranking_drops_a_candidate_that_restocks_nothing()
        self.test_ovC_rank_leaf_measures_a_real_robot()
        self.test_ovD_install_known_removes_one_unknown()
        self.test_ovE_install_known_walks_the_list_then_fails()
        self.test_ovF_each_attempt_starts_from_fresh_unknowns()

    #  Wrist throughout:  a real robot (the ranking needs real equations) and
    #  the cheapest one there is.  Its unknowns are A, B, C.
    ROBOT = 'Wrist'

    def synthetic(self, *rows):
        '''Candidate dicts from (symbol, n1, n2, ops, index) tuples.'''
        return [{'robot': 'Fake', 'symbol': s, 'n1': n1, 'n2': n2,
                 'n3p': 0, 'ops': ops, 'index': i} for s, n1, n2, ops, i in rows]

    def tick(self, node, bb):
        t = b3.BehaviorTree()
        t.root = node
        return t.tick('testing the one-variable branch', bb)

    def loaded_bb(self):
        '''A blackboard carrying a real Robot, as init_blackboard() leaves it.'''
        R, unks = fresh_problem(self.ROBOT)
        bb = b3.Blackboard()
        L1, L2, L3p = R.scan_for_equations(unks)
        bb.set('Robot', R)
        bb.set('unknowns', unks)
        bb.set('eqns_1u', L1)
        bb.set('eqns_2u', L2)
        bb.set('eqns_3pu', L3p)
        return bb

    def test_ovA_ranking_prefers_the_biggest_restock(self):
        '''n1 first, then n2, then simplicity, then chain order.'''
        fs = ' onevar ranking FAIL'

        ranked = rank_by_restock(self.synthetic(
            ('th_1', 2, 30, 10, 1),      # most 2-unknown, but fewer 1-unknown
            ('th_2', 4,  9, 50, 2),
            ('th_3', 4, 25, 50, 3),      # same n1 as th_2, more n2 -> ahead
            ('th_4', 4, 25, 20, 4),      # same n1 and n2, simpler -> ahead of both
        ))
        self.assertEqual([c['symbol'] for c in ranked],
                         ['th_4', 'th_3', 'th_2', 'th_1'], fs)

        #  a complete tie falls back to chain position, so reruns agree
        tied = rank_by_restock(self.synthetic(('th_5', 3, 3, 3, 5),
                                              ('th_2', 3, 3, 3, 2)))
        self.assertEqual([c['symbol'] for c in tied], ['th_2', 'th_5'],
                         fs + ' (tie is not broken by chain position)')

    def test_ovB_ranking_drops_a_candidate_that_restocks_nothing(self):
        '''n1 == 0 leaves L1 exactly as it was:  a solver leaf has nothing new
           to start from, so the attempt is a certain waste of a full solve.'''
        fs = ' onevar min_new FAIL'

        ranked = rank_by_restock(self.synthetic(('th_4', 0, 10, 0, 4),
                                                ('th_2', 1,  9, 5, 2)))
        self.assertEqual([c['symbol'] for c in ranked], ['th_2'], fs)

        self.assertEqual(rank_by_restock(self.synthetic(('th_4', 0, 10, 0, 4))),
                         [], fs + ' (a branch with nothing to try must close)')

        #  and the cap is honoured
        self.assertEqual(len(rank_by_restock(self.synthetic(
            ('th_1', 5, 1, 1, 1), ('th_2', 4, 1, 1, 2), ('th_3', 3, 1, 1, 3)),
            limit=2)), 2, fs + ' (limit ignored)')

    def test_ovC_rank_leaf_measures_a_real_robot(self):
        '''End to end on Wrist:  every unknown measured, best first, and the
           blackboard Robot left exactly as it was found.'''
        fs = ' onevar rank leaf FAIL'

        bb = self.loaded_bb()
        R = bb.get('Robot')
        before = (len(R.l1), len(R.l2), len(R.l3p))

        node = onevar_rank()
        node.max_candidates = 9
        self.assertEqual(self.tick(node, bb), b3.SUCCESS, fs)

        ranked = bb.get('onevar_candidates')
        self.assertEqual([c['symbol'] for c in ranked], ['B', 'A', 'C'],
                         fs + ' (measured restock: B 14, A 12, C 8)')
        self.assertEqual(bb.get('onevar_cursor'), 0, fs + ' (cursor not reset)')
        for c in ranked:
            self.assertEqual(c['robot'], self.ROBOT, fs)

        self.assertIs(bb.get('Robot'), R, fs + ' (the leaf swapped the Robot)')
        self.assertEqual((len(R.l1), len(R.l2), len(R.l3p)), before,
                         fs + ' (ranking rescanned the LIVE Robot, so its'
                         ' equation lists now describe one candidate)')

    def test_ovD_install_known_removes_one_unknown(self):
        '''The installed problem is the true arm minus one unknown.'''
        fs = ' onevar install FAIL'

        bb = self.loaded_bb()
        R_before = bb.get('Robot')
        n_before = len(bb.get('unknowns'))

        self.assertEqual(self.tick(onevar_rank(), bb), b3.SUCCESS, fs)
        self.assertEqual(self.tick(install_known(), bb), b3.SUCCESS, fs)

        unks = bb.get('unknowns')
        self.assertEqual(len(unks), n_before - 1, fs)
        self.assertNotIn('B', [str(u.symbol) for u in unks],
                         fs + ' (the assumed-known variable is still unknown)')

        src = bb.get('onevar_source')
        self.assertEqual(src['known'], 'B', fs)
        self.assertEqual((src['robot'], src['attempt'], src['of']),
                         (self.ROBOT, 1, 3), fs)

        #  same arm, reloaded:  a DIFFERENT object (a solve mutates it) over the
        #  SAME DH table (nothing about the robot is approximated here)
        R = bb.get('Robot')
        self.assertIsNot(R, R_before, fs + ' (installed the live Robot)')
        self.assertEqual(R.name, self.ROBOT, fs + ' (the arm was renamed)')
        self.assertEqual(R.Mech.DH, R_before.Mech.DH,
                         fs + ' (the DH table changed -- this branch must not'
                         ' modify the robot)')
        self.assertTrue(bb.get('eqns_1u'), fs + ' (no 1-unknown equations)')

    def test_ovE_install_known_walks_the_list_then_fails(self):
        '''One candidate per tick, in rank order, then FAILURE -- which is what
           ends the retry loop.'''
        fs = ' onevar cursor FAIL'

        bb = self.loaded_bb()
        self.assertEqual(self.tick(onevar_rank(), bb), b3.SUCCESS, fs)

        node = install_known()
        got = []
        for _ in range(3):
            self.assertEqual(self.tick(node, bb), b3.SUCCESS, fs)
            got.append(bb.get('onevar_source')['known'])
        self.assertEqual(got, ['B', 'A', 'C'], fs + ' (not in rank order)')

        self.assertEqual(self.tick(node, bb), b3.FAILURE,
                         fs + ' (an exhausted list must close the branch)')

        #  with nothing ranked at all it fails on the first tick
        self.assertEqual(self.tick(install_known(), b3.Blackboard()), b3.FAILURE,
                         fs + ' (no candidates)')

    def test_ovF_each_attempt_starts_from_fresh_unknowns(self):
        '''A failed attempt leaves `solved` flags and solution lists on the
           unknowns and a solve counter on the Robot.  Reusing either would make
           the second attempt start mid-solve, so each one reloads.'''
        fs = ' onevar fresh state FAIL'

        bb = self.loaded_bb()
        self.assertEqual(self.tick(onevar_rank(), bb), b3.SUCCESS, fs)

        node = install_known()
        self.assertEqual(self.tick(node, bb), b3.SUCCESS, fs)

        #  dirty the first attempt's state, the way a partial solve would
        u = bb.get('unknowns')[0]
        u.solved = True
        u.solutions = ['junk']
        bb.get('Robot').solveN = 7

        self.assertEqual(self.tick(node, bb), b3.SUCCESS, fs)
        for u in bb.get('unknowns'):
            self.assertFalse(u.solved, fs + ' (carried a solved flag over)')
            self.assertEqual(u.solutions, [], fs + ' (carried solutions over)')
        self.assertEqual(bb.get('Robot').solveN, 0,
                         fs + ' (carried the solve counter over)')


def run_test():
    suite = unittest.TestLoader().loadTestsFromTestCase(TestSolver028)
    unittest.TextTestRunner(verbosity=2).run(suite)


##   Self Tests
if __name__ == "__main__":
    run_test()
