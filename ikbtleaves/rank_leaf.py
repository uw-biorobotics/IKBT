# major refactor 2: control logic change, move the looping logic outside of the solver nodes
# 21-Jun-2017 DZ
# ranking node compares the solutions from sin/cos and tan solvers
# and chooses the better one as final solution

# Copyright 2017 University of Washington

# Developed by Dianmu Zhang and Blake Hannaford
# BioRobotics Lab, University of Washington

# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
import sympy as sp  
import numpy as np
from sys import exit

from ikbtfunctions.helperfunctions import *
from ikbtbasics.kin_cl import *
from ikbtbasics.ik_classes import *     # special classes for Inverse kinematics in sympy

import b3 as b3          # behavior trees

# helper function: count veriables (regardless of solved status)
def count_variables(unknowns, expr):
    n = 0
    for unk in unknowns:
        if expr.has(unk.symbol):
            n += 1
    return n

class rank(b3.Action):
    def tick(self, tick):
        u = tick.blackboard.get("curr_unk")
        unknowns = tick.blackboard.get("unknowns")
        R = tick.blackboard.get("Robot")
        choosen = None

        #  Decide on the solutions that were actually PRODUCED, not on the
        #  solvable_* flags.  An ID node sets solvable_* and its solver can then
        #  decline -- sincos_solve returns FAILURE when it cannot decompose the
        #  equation -- which leaves the flag True and the solution list empty.
        #  The old code keyed off the flags and indexed the lists unguarded, so
        #  that combination reached u.sincos_solutions[0] on an empty list and
        #  raised IndexError, aborting the whole solve.  Both of the guards it
        #  did have (len(...) > 0) sat on the two branches that could not fail;
        #  the bare else, which could, had none.
        n_sc  = len(u.sincos_solutions)
        n_tan = len(u.tan_solutions)

        # only invokes comparisions when BOTH solvers actually produced something
        if n_sc > 0 and n_tan > 0:
            # less number of solutions is better
            if n_sc < n_tan:
                choosen = "sincos"
            elif n_sc > n_tan:
                choosen = "tan"
            else:
                # if the number of solutions are the same, countinue
                # less dependency
                sol_sin = u.sincos_solutions[0]
                sol_tan = u.tan_solutions[0]
                if count_variables(unknowns, sol_sin) < count_variables(unknowns, sol_tan):
                    choosen = "sincos"
                else:
                    choosen = "tan"
            # note that we used a ranking
            u.solvemethod += ', best ranked, '

            # reset solutions
            #   (both solvers appended into u.solutions, so it currently holds
            #    the union; replace it with just the winner's)
            u.solutions = None
            #u.solveorder = u.solveorder - 1
            if choosen == "sincos":
                u.solutions = u.sincos_solutions
                u.nsolutions = len(u.sincos_solutions)
                u.eqntosolve = u.sincos_eqnlist[0]
                u.solvemethod += "sin or cos"
            elif choosen == "tan":
                u.solutions = u.tan_solutions
                u.nsolutions = len(u.tan_solutions)
                u.eqntosolve = u.tan_eqnlist[0]
                u.secondeqn = u.tan_eqnlist[1]
                u.solvemethod += "atan2(y,x)"

            u.set_solved(R, unknowns)

        elif n_sc > 0 or n_tan > 0:
            #  exactly one solver produced an answer; u.solutions already holds
            #  it, so there is nothing to choose between
            u.set_solved(R, unknowns)

        #  else: an ID fired but no solver produced anything.  Do NOT call
        #  set_solved() -- the variable is still unsolved and must stay
        #  available to the other leaves in the Priority.

        tick.blackboard.set("curr_unk", u)
        tick.blackboard.set("unknowns", unknowns)
        tick.blackboard.set("Robot", R)
        return b3.SUCCESS


#######################################################################
#  Test code:
import unittest
from ikbtleaves.sincos_solver import sincos_id, sincos_solve
from ikbtleaves.tan_solver import tan_id, tan_solve


class TestSolver011(unittest.TestCase):
    '''The ranking leaf.

       rank exists because more than one leaf can often solve the same unknown
       and the answers are not equally good -- an atan2(y,x) form is preferred
       over a +/-acos(...) pair.  That does not fit the BT's first-success-wins
       model, so assigner + OrNode + rank is a deliberate workaround: OrNode
       runs BOTH tanSol and scSol (unlike Priority, it does not stop at the
       first success) precisely so rank can compare afterwards.

       So rank is the only node in the tree that DISCARDS a correct solution,
       and what it discards on is a design decision.  These tests pin the
       preference, not just the mechanics.'''

    def setUp(self):
        self.DB = False
        print('\n\n===============  Test rank leaf  =========================')
        return

    def runTest(self):
        self.test_rankA_arccos_plus_tan_does_not_crash()
        self.test_rankB_prefers_fewer_solutions()
        self.test_rankC_id_without_solution_does_not_crash()
        self.test_rankD_single_solver_passes_through()

    def build(self, eqns_1u, eqns_2u, target, others=()):
        '''Tick the composite ikSolver.py actually builds:
               Sequence([ OrNode([tanSol, scSol]), rank ])
           Returns (status, unknown).'''
        u = unknown(target)
        unknowns = [u] + list(others)

        tid = tan_id();        tid.Name = 'tid'; tid.BHdebug = self.DB
        tsl = tan_solve();     tsl.Name = 'tsl'; tsl.BHdebug = self.DB
        sid = sincos_id();     sid.Name = 'sid'; sid.BHdebug = self.DB
        ssl = sincos_solve();  ssl.Name = 'ssl'; ssl.BHdebug = self.DB
        rk  = rank();          rk.Name  = 'rank'

        t = b3.BehaviorTree()
        t.root = b3.Sequence([b3.OrNode([b3.Sequence([tid, tsl]),
                                         b3.Sequence([sid, ssl])]), rk])
        bb = b3.Blackboard()
        bb.set('curr_unk', u)
        bb.set('unknowns', unknowns)
        bb.set('eqns_1u', eqns_1u)
        bb.set('eqns_2u', eqns_2u)
        bb.set('eqns_3pu', [])
        bb.set('Robot', Robot())
        bb.set('Tm', None)
        status = t.tick('rank test', bb)
        return status, bb.get('curr_unk')

    def test_rankA_arccos_plus_tan_does_not_crash(self):
        '''Regression: variable solvable by BOTH arccos and tan, with rank
           preferring sincos, used to raise IndexError and abort the solve.

           sincos_solve appended to u.sincos_eqnlist in its arcsin branch but
           NOT in its arccos branch, and rank does
               u.eqntosolve = u.sincos_eqnlist[0]
           when it picks sincos.  Fixed by appending in both branches.

           The fixture: a 1-unknown cos equation gives arccos (2 solutions,
           no variables in them); a sin/cos pair sharing the unsolved factor
           sin(th_5) gives tan's two-branch path (2 solutions, mentioning the
           solved th_2).  Equal counts -> the dependency tie-break -> sincos.'''
        sp.var('th_1 th_2 th_5 l_1 r_11 r_12 r_13')
        fs = ' rank arccos+tan FAIL'

        u2 = unknown(th_2); u2.solved = True; u2.nsolutions = 1
        u2.solutions = [th_2]

        try:
            status, u = self.build(
                [kequation(sp.S.Zero, l_1*sp.cos(th_1) - r_11)],
                [kequation(sp.S.Zero, sp.sin(th_5)*sp.sin(th_1) - r_12*sp.cos(th_2)),
                 kequation(sp.S.Zero, sp.sin(th_5)*sp.cos(th_1) - r_13*sp.cos(th_2))],
                th_1, others=[u2, unknown(th_5)])
        except IndexError as e:
            self.fail(fs + ' (IndexError -- sincos_eqnlist not populated by the'
                           ' arccos branch: %s)' % e)

        self.assertEqual(status, b3.SUCCESS, fs)
        self.assertTrue(u.solved, fs + ' (not marked solved)')
        #  it really did rank, and really did pick sincos
        self.assertIn('best ranked', u.solvemethod, fs + ' (no ranking happened)')
        self.assertIn('sin or cos', u.solvemethod, fs + ' (expected sincos to win)')
        self.assertEqual(u.solutions, u.sincos_solutions,
                         fs + ' (winner not installed into u.solutions)')
        self.assertIsNotNone(u.eqntosolve, fs + ' (eqntosolve not set)')

    def test_rankB_prefers_fewer_solutions(self):
        '''The core preference: tan gives one atan2() where sincos gives a
           two-branch +/-acos, so tan must win.  This is the whole reason the
           leaf exists.'''
        sp.var('th_1 l_1 l_2 r_11 r_12')
        fs = ' rank fewer-solutions FAIL'

        status, u = self.build(
            [kequation(sp.S.Zero, l_1*sp.cos(th_1) - r_11),
             kequation(sp.S.Zero, l_2*sp.sin(th_1) - r_12)],
            [], th_1)

        self.assertEqual(status, b3.SUCCESS, fs)
        self.assertTrue(u.solved, fs)
        self.assertEqual(len(u.tan_solutions), 1, fs + ' (expected tan 1-sol path)')
        self.assertEqual(len(u.sincos_solutions), 2, fs + ' (expected sincos pair)')
        self.assertIn('atan2', u.solvemethod, fs + ' (tan should have won)')
        self.assertEqual(u.nsolutions, 1, fs + ' (kept the wrong branch count)')
        self.assertEqual(u.solutions, u.tan_solutions, fs)

    def test_rankC_id_without_solution_does_not_crash(self):
        '''An ID can fire while its solver declines -- sincos_solve returns
           FAILURE when it cannot decompose.  That leaves solvable_sincos True
           with sincos_solutions EMPTY.  rank keyed off the flags and then
           indexed the empty list, so this raised IndexError.

           cos(th_1)**2 is such a shape: sincos_id claims it (it has cos of the
           unknown), sincos_solve cannot match it.  tan solves normally.'''
        sp.var('th_1 l_1 l_2 r_11 r_12')
        fs = ' rank ID-without-solution FAIL'

        try:
            status, u = self.build(
                [kequation(sp.S.Zero, sp.cos(th_1)**2 - l_1),   # sincos claims, then declines
                 kequation(sp.S.Zero, l_2*sp.sin(th_1) - r_12),
                 kequation(sp.S.Zero, l_1*sp.cos(th_1) - r_11)],
                [], th_1)
        except IndexError as e:
            self.fail(fs + ' (IndexError on an empty solution list: %s)' % e)

        #  tan still solves it; the point is simply that rank survived
        self.assertEqual(status, b3.SUCCESS, fs)
        for s in u.solutions or []:
            self.assertFalse(s.has(th_1), fs + ' (solution contains its own unknown)')

    def test_rankD_single_solver_passes_through(self):
        '''Only one solver fires -> no ranking, no discarding, and the solution
           that solver produced is kept as-is.'''
        sp.var('th_1 l_1 r_11')
        fs = ' rank single-solver FAIL'

        status, u = self.build(
            [kequation(sp.S.Zero, l_1*sp.sin(th_1) - r_11)], [], th_1)

        self.assertEqual(status, b3.SUCCESS, fs)
        self.assertTrue(u.solved, fs + ' (not marked solved)')
        self.assertNotIn('best ranked', u.solvemethod,
                         fs + ' (ranked when only one solver fired)')
        self.assertEqual(len(u.solutions), 2, fs + ' (expected the asin pair)')


def run_test():
    print('\n\n===============  Test rank_leaf.py =====================')
    testsuite = unittest.TestLoader().loadTestsFromTestCase(TestSolver011)
    unittest.TextTestRunner(verbosity=2).run(testsuite)


if __name__ == "__main__":
    run_test()
