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
    """Identify which consecutive joint-axis triples satisfy Pieper's condition,
       and write a LaTeX statement of the finding for the report.

       ALWAYS RETURNS SUCCESS, and that is deliberate -- it is not the repo's
       usual `_id` gate.  This leaf ticks at the FRONT of the tree, ahead of the
       Priority that chooses between the symbolic and hybrid branches, because
       the report has to carry the geometry statement whichever branch ends up
       producing the solution.  At that position a FAILURE would abort the
       enclosing Sequence and kill the entire solve, which is an absurd outcome
       for a leaf whose job is to describe the robot.  So the answer goes on the
       blackboard and nothing is gated on the status:

           pieper_triples   list of satisfied triples (possibly empty)
           pieper_ok        True if the analysis actually ran;  when False the
                            list means "unknown", NOT "none"
           Robot.pieper_latex   the report section (also stashed on the Robot,
                            which is what output_latex_solution() is handed)

       This replaces the `Inverter(pieper_id)` gate the plan originally called
       for.  The hybrid branch's gate now reads `pieper_triples` off the
       blackboard and says what it means -- "fire when there is no triple" --
       instead of expressing it as the inverse of a leaf that has to succeed in
       order to mean failure.  A gate must also refuse to fire when
       `pieper_ok` is False:  "we could not tell" is not "there is no triple"."""

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
            ndof = len([u for u in (bb.get('unknowns') or []) if u.n and u.n <= 6])
            if not ndof:
                ndof = len(R.variables) if getattr(R, 'variables', None) else 6

            triples = da.pieper_triples(M.DH, M.pvals, ndof)
            bb.set('pieper_triples', triples)
            bb.set('pieper_ok', True)

            R.pieper_latex = da.pieper_latex(M.DH, M.pvals, ndof, R.name)

            if self.BHdebug:
                print('\n', self.Name, ':', R.name, '->',
                      [(t['axes'], t['kind']) for t in triples] or 'no triples')

        except Exception as e:
            #  Reported, never raised, and never fatal:  this leaf describes the
            #  robot, it does not solve it.  pieper_ok stays False so no gate
            #  downstream mistakes "unknown" for "none".
            print(self.Name, ': could not analyse the DH table --',
                  '%s: %s' % (type(e).__name__, e))

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
        self.test_hybC_pieper_id_always_succeeds()
        self.test_hybD_pieper_id_finds_the_wrist()
        self.test_hybE_pieper_id_ignores_sum_of_angle_unknowns()

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
        def __init__(self, dh, pvals): self.DH, self.pvals = dh, pvals

    class robot(object):
        def __init__(self, dh, pvals, name='Fake'):
            self.Mech = TestSolver018.mech(dh, pvals)
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

    def test_hybC_pieper_id_always_succeeds(self):
        """SUCCESS even with nothing on the blackboard.

           It ticks at the FRONT of the tree, ahead of the branch split, so a
           FAILURE would abort the enclosing Sequence and kill the whole solve.
           A leaf that only describes the robot must never be able to do that.
           On the error path pieper_ok stays False, so a downstream gate cannot
           mistake "we could not tell" for "there is no triple"."""
        fs = ' pieper_id FAIL'
        bb = b3.Blackboard()
        self.assertEqual(self.tick_pieper(bb), b3.SUCCESS,
                         fs + ' (must succeed with no Robot)')
        self.assertFalse(bb.get('pieper_ok'),
                         fs + ' (pieper_ok must be False when it could not run)')
        self.assertEqual(bb.get('pieper_triples'), [],
                         fs + ' (triples must still be a list)')

    def test_hybD_pieper_id_finds_the_wrist(self):
        """A spherical wrist is reported, and the LaTeX statement is written to
           the Robot -- which is the object output_latex_solution() is handed."""
        fs = ' pieper_id wrist FAIL'
        dh = self.puma_like()
        R = TestSolver018.robot(dh, {}, 'Wristy')
        bb = b3.Blackboard()
        bb.set('Robot', R)
        bb.set('unknowns', [TestSolver018.unk(i) for i in range(1, 7)])

        self.assertEqual(self.tick_pieper(bb), b3.SUCCESS, fs)
        self.assertTrue(bb.get('pieper_ok'), fs + ' (analysis should have run)')
        got = [(t['axes'], t['kind']) for t in bb.get('pieper_triples')]
        self.assertIn(((4, 5, 6), 'intersect'), got, fs + ' (missed the wrist)')

        tex = getattr(R, 'pieper_latex', None)
        self.assertTrue(tex, fs + ' (no LaTeX statement written to the Robot)')
        self.assertIn(r'\section{Joint Axis Geometry', tex, fs + ' (no section)')
        self.assertIn('(4, 5, 6)', tex, fs + ' (statement omits the triple)')
        #  the logic has to be stated correctly -- sufficient, NOT necessary
        self.assertIn('sufficient', tex, fs + ' (must say sufficient)')
        self.assertIn('not known to be necessary', tex,
                      fs + ' (must NOT claim Pieper is necessary)')

    def test_hybE_pieper_id_ignores_sum_of_angle_unknowns(self):
        """DOF count must skip the sum-of-angles unknowns.

           kinematics_pickle() EXTENDS the unknown list with th_23 (n = 23) and
           friends.  Counting those would inflate ndof past 6 and invent triples
           over the zero-padded rows."""
        fs = ' pieper_id SOA FAIL'
        dh = self.puma_like()
        R = TestSolver018.robot(dh, {}, 'Soa')
        bb = b3.Blackboard()
        bb.set('Robot', R)
        bb.set('unknowns', [TestSolver018.unk(i) for i in range(1, 7)]
                           + [TestSolver018.unk(23), TestSolver018.unk(234)])

        self.assertEqual(self.tick_pieper(bb), b3.SUCCESS, fs)
        axes = [t['axes'] for t in bb.get('pieper_triples')]
        self.assertTrue(all(a[2] <= 6 for a in axes),
                        fs + ' (a triple names an axis above 6: %s)' % axes)


def run_test():
    suite = unittest.TestLoader().loadTestsFromTestCase(TestSolver018)
    unittest.TextTestRunner(verbosity=2).run(suite)


##   Self Tests
if __name__ == "__main__":
    run_test()
