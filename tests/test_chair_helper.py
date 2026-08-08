#!/usr/bin/python
#
#   Full-solve regression test for the 'Chair_Helper' robot.
#
#   These assertions used to sit at the bottom of ikSolver.py, so they only ran
#   when somebody happened to solve Chair_Helper by hand.  They are a real
#   regression test, so they live here now.
#
#   This runs a COMPLETE IK solve, which is far too slow for the leaf suite.  It
#   is deliberately NOT registered in tests/leavestest.py -- run it on its own:
#
#       python3 -m tests.test_chair_helper        (from the repo root)
#
#   Note it does NOT call emit_outputs(), so it leaves LaTex/ and CodeGen/ alone.
#
#   Copyright 2017-2026 University of Washington
#
#   Developed by Dianmu Zhang and Blake Hannaford
#   BioRobotics Lab, University of Washington

import unittest

import sympy as sp

from ikbtfunctions.ik_driver   import load_robot, run_solver
from ikbtfunctions.bt_assembly import build_default_bt

#  symbols that appear in the expected solutions
sp.var('th_1 th_2 th_3 th_4 th_5 th_6')
sp.var('d_1 d_2 d_3 d_4 d_5 d_6')
sp.var('l_1 l_2 l_3 l_4')
sp.var('r_11 r_12 r_13 r_21 r_22 r_23 r_31 r_32 r_33 Px Py Pz')


class TestChairHelper(unittest.TestCase):
    '''Solve Chair_Helper end to end and check the two closed-form results that
       have been stable since 2017.'''

    #  solved once for the whole class -- a full solve is expensive
    _unks = None

    @classmethod
    def setUpClass(cls):
        print('\n\n=========  Full IK solve:  Chair_Helper  =========\n')
        M, R, unknowns = load_robot('Chair_Helper')
        bt, nodes = build_default_bt()
        R, unks, bb = run_solver(R, unknowns, bt)
        cls._unks = unks

    def unk(self, symbol):
        '''Fetch the solved unknown carrying `symbol`, failing loudly if the
           solver never produced one -- that is a regression in itself.'''
        for u in self._unks:
            if u.symbol == symbol:
                return u
        self.fail('Chair_Helper FAIL: no unknown for ' + str(symbol))

    def test_chair_d1(self):
        '''d_1 is a single-solution prismatic joint.'''
        fs = 'Chair_Helper   FAIL'
        u = self.unk(d_1)
        self.assertEqual(u.nsolutions, 1, fs + ' n(d_1)')
        self.assertEqual(u.solutions[0], Pz - l_4*r_33, fs + '  [d_1]')

    def test_chair_th2(self):
        '''th_2 comes out of an arcsin, so it has the two standard branches.'''
        fs = 'Chair_Helper   FAIL'
        u = self.unk(th_2)
        self.assertEqual(u.nsolutions, 2, fs + ' n(th_2)')
        arg = (Px - l_1 - l_4*r_13)/l_2
        self.assertEqual(u.solutions[0],  sp.asin(arg),          fs + ' [th_2a]')
        self.assertEqual(u.solutions[1], -sp.asin(arg) + sp.pi,  fs + ' [th_2b]')

    def test_chair_all_solved(self):
        '''Every unknown the front end handed us must end up solved.'''
        fs = 'Chair_Helper   FAIL'
        unsolved = [str(u.symbol) for u in self._unks if not u.solved]
        self.assertEqual(unsolved, [], fs + ' (unsolved: ' + str(unsolved) + ')')


def run_test():
    suite = unittest.TestLoader().loadTestsFromTestCase(TestChairHelper)
    unittest.TextTestRunner(verbosity=2).run(suite)


if __name__ == "__main__":
    run_test()
