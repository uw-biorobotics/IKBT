#!/usr/bin/python
#
#     BT Nodes for specific symbolic steps
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
import time       

       
#   Detect when all unknowns are solved
#   
 
class comp_det(b3.Action):
    
    def __init__(self):
        super(b3.Action, self).__init__()
        self.FailAllDone = False   # we can set up to succeed when all are done or succeed when more to do. 
        self.Name = '*completion_detect*'
        #  Deliberate pause so a human can read the status as it scrolls by.
        #  It is NOT free:  comp_det ticks ~9 times in a Puma solve, so this is
        #  ~18s of Puma's ~27s wall clock.  Set to 0 for batch runs and tests.
        self.read_pause = 2
        
    def tick(self,tick):
        unks = tick.blackboard.get('unknowns')
        Tm = tick.blackboard.get('Tm')
        R  = tick.blackboard.get('Robot')
        
        SOA_DEBUG = False  # just turn this on for debugging
        if SOA_DEBUG:
            L1 = tick.blackboard.get('eqns_1u')
            L2 = tick.blackboard.get('eqns_2u')
            L3 = tick.blackboard.get('eqns_3pu')
            print('\n')
            print('L1: ', L1)   ##  for debugging sum of angles
            print('L2: ', L2)
            print('L3: ', L3)
            
        n = 0
        ns = 0
        print('\n\n')
        for u in unks:
            n += 1
            status = 'Unsolved'
            if(u.solved):
                ns += 1
                status = 'SOLVED'
            sname = str(u.symbol)
            print(f'{sname:10}: {status}')

        print('           Completion Detector: ', n, ' variables, ', ns, ' are solved.')
        print('             solved: ',)
        for u in unks:
            if(u.solved):
                outstr = '{} ({});  '.format(u.symbol, u.solvemethod)
                print(outstr,                )
        print('\n\n\n')
        time.sleep(self.read_pause)  # for easier reading/ stopping
            
        #
        #   Look for sum-of-angle equations which can now be solved
        #      for example: th2 = th23-th3 (where th23 and th3 are known)
        #     
        #   this should be algebric solver's work, 
        #   also the set solve here causes problem
        
                
        if(self.FailAllDone):               # we can set up to succeed when all are done or succeed when more to do.
            DONEComplete   = b3.FAILURE
            DONEIncomplete = b3.SUCCESS
        else:
            DONEComplete   = b3.SUCCESS
            DONEIncomplete = b3.FAILURE
        if(n == ns):
            print("")
            print(" Solution Complete!!")
            print("")
            return DONEComplete  # we have solved all vars

        #
        #   Nothing solved and nothing changed:  give up cleanly.
        #
        #   The whole BT bootstraps from eqns_1u.  Several robots (ArmRobo,
        #   Issue4, KawasakiRS05L, KinovaLite, Raven-II) start with an EMPTY
        #   L1 -- every equation has 2+ unknowns -- so no ID node can fire on
        #   anything and not one variable is ever solved.  Without this check
        #   the tree grinds all 10 outer passes and then hands an empty
        #   solListMatrix to the report generator, which dies with
        #   'IndexError: list index out of range' in make_LHS_versions().
        #   That one crash was masking five different robots' real diagnosis.
        #
        #   Why this waits for a SECOND identical pass rather than stopping
        #   after the first:  a transform (x2y2, sub_transform, the invariant
        #   generator) can legitimately solve nothing on a pass while still
        #   ADDING equations that let the next pass solve something.  Bailing
        #   on "ns == 0" alone would cut those robots off.  So we stop only
        #   when a whole pass changed nothing at all -- no variable solved and
        #   no equation added -- which no amount of further ticking can undo.
        #
        L1 = tick.blackboard.get('eqns_1u') or []
        L2 = tick.blackboard.get('eqns_2u') or []
        L3 = tick.blackboard.get('eqns_3pu') or []
        signature = (ns, len(L1), len(L2), len(L3), len(R.kequation_aux_list))
        previous  = tick.blackboard.get('comp_det_signature')
        tick.blackboard.set('comp_det_signature', signature)

        if ns == 0 and previous is not None and signature == previous:
            print('')
            print('   No solution found.')
            print('   Nothing was solved and the last pass changed nothing:')
            print('     one-unknown equations available: ', len(L1))
            if len(L1) == 0:
                print('     -- eqns_1u is EMPTY, so no ID node can fire.  Every')
                print('        solver leaf needs an equation in one unknown to start.')
            print('   Stopping here rather than generating an empty report.')
            print('')
            tick.blackboard.set('no_progress', True)
            return DONEComplete   # break out of RepeatUntilSuccess

        return DONEIncomplete # we still have unsolved vars
        
        
#####################################################################
#
#   Test code
#
import unittest


class TestSolver014(unittest.TestCase):
    '''Completion detector, including the give-up-cleanly path.'''

    def setUp(self):
        print('\n\n===============  Test completion detector  =====================')
        return

    def runTest(self):
        self.test_compA_reports_incomplete()
        self.test_compB_reports_complete()
        self.test_compC_gives_up_when_nothing_can_change()
        self.test_compD_keeps_going_while_equations_are_still_appearing()

    def make_bb(self, nsolved, nunk=3, nL1=0, naux=0):
        '''A blackboard with `nsolved` of `nunk` unknowns solved.'''
        sp.var('th_1 th_2 th_3')
        syms = [th_1, th_2, th_3][:nunk]
        unks = []
        for i, s in enumerate(syms):
            u = unknown(s)
            if i < nsolved:
                u.solved = True
                u.solutions = [s]
                u.nsolutions = 1
            unks.append(u)
        R = Robot()
        R.kequation_aux_list = [kequation(0, 0)] * naux
        bb = b3.Blackboard()
        bb.set('unknowns', unks)
        bb.set('Robot', R)
        bb.set('eqns_1u', [kequation(0, 0)] * nL1)
        bb.set('eqns_2u', [])
        bb.set('eqns_3pu', [])
        return bb

    def tick(self, bb, node=None):
        if node is None:
            node = comp_det()
            node.read_pause = 0        # no human is reading this one
        t = b3.BehaviorTree()
        t.root = node
        return t.tick('testing comp_det', bb), node

    def test_compA_reports_incomplete(self):
        fs = ' comp_det FAIL'
        bb = self.make_bb(nsolved=1, nunk=3, nL1=2)
        st, _ = self.tick(bb)
        self.assertEqual(st, b3.FAILURE, fs + ' (should report more work to do)')
        self.assertFalse(bb.get('no_progress'), fs)

    def test_compB_reports_complete(self):
        fs = ' comp_det FAIL'
        bb = self.make_bb(nsolved=3, nunk=3)
        st, _ = self.tick(bb)
        self.assertEqual(st, b3.SUCCESS, fs + ' (should report complete)')
        self.assertFalse(bb.get('no_progress'), fs + ' (complete is not "no progress")')

    def test_compC_gives_up_when_nothing_can_change(self):
        '''Nothing solved and a whole pass changed nothing -> stop.

           Five robots (ArmRobo, Issue4, KawasakiRS05L, KinovaLite, Raven-II)
           start with an EMPTY eqns_1u, so no ID node can ever fire.  Without
           this the tree ground through all 10 outer passes and then handed an
           empty solListMatrix to the report generator, which died with
           IndexError in make_LHS_versions() -- one crash masking five
           different robots' actual diagnosis.'''
        fs = ' comp_det give-up FAIL'
        bb = self.make_bb(nsolved=0, nunk=3, nL1=0)

        st, node = self.tick(bb)
        self.assertEqual(st, b3.FAILURE, fs + ' (bailed on the very first pass)')
        self.assertFalse(bb.get('no_progress'), fs + ' (too eager)')

        st, node = self.tick(bb, node)      # identical second pass
        self.assertEqual(st, b3.SUCCESS, fs + ' (did not stop the outer loop)')
        self.assertTrue(bb.get('no_progress'), fs + ' (did not flag no-progress)')

    def test_compD_keeps_going_while_equations_are_still_appearing(self):
        '''A pass that solves nothing but ADDS equations is real progress -- a
           transform (x2y2, sub_transform, invariant_gen) can set up the next
           pass.  Bailing on "nothing solved" alone would cut those robots off,
           which is why the check needs two IDENTICAL passes.'''
        fs = ' comp_det premature give-up FAIL'
        bb = self.make_bb(nsolved=0, nunk=3, nL1=0)
        st, node = self.tick(bb)
        self.assertEqual(st, b3.FAILURE, fs)

        #  a transform fired: one new equation showed up
        bb.set('eqns_1u', [kequation(0, 0)])
        st, node = self.tick(bb, node)
        self.assertEqual(st, b3.FAILURE, fs + ' (gave up while progress was happening)')
        self.assertFalse(bb.get('no_progress'), fs + ' (flagged no-progress too early)')


def run_test():
    suite = unittest.TestLoader().loadTestsFromTestCase(TestSolver014)
    unittest.TextTestRunner(verbosity=2).run(suite)


##   Self Tests
if __name__ == "__main__":
    run_test()
      
