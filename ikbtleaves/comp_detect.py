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

       
def _pool_signature(eqns):
    '''A comparable fingerprint of an equation pool's CONTENTS.

       Used by comp_det to decide whether a whole pass changed anything.  It
       must be content-sensitive:  comparing len() alone treats "swapped one
       equation for another" as "nothing happened", which stopped ICP5p5_A21
       and Parkman13 mid-solve when the stall check was opened up to partial
       solves.

       str() rather than sp.expand() or sp.simplify():  this runs on every
       comp_det tick and only needs to detect CHANGE, not to canonicalise.  Two
       mathematically equal but differently written equations reading as
       "changed" is the safe direction to err -- it keeps the solver running.
       Sorted, so a reordered pool is not mistaken for progress.'''

    out = []
    for e in eqns:
        try:
            out.append('%s|%s' % (e.LHS, e.RHS))
        except AttributeError:
            out.append(repr(e))
    return tuple(sorted(out))


#   Detect when all unknowns are solved
#

class comp_det(b3.Action):
    
    def __init__(self):
        super(b3.Action, self).__init__()
        self.FailAllDone = False   # we can set up to succeed when all are done or succeed when more to do. 
        self.Name = '*completion_detect*'
        #  Was 2 seconds:  a deliberate pause so a human could read the status
        #  wall as it scrolled by.  Now 0 -- not to reclaim the time, but
        #  because what it compensated for is gone:  symbolic_loop prints ONE
        #  compact line per pass (ikbtfunctions/progress.py), so nothing
        #  scrolls past unread and a pause that no longer buys legibility is
        #  just a pause.  scripts/robot_baseline.py already forced it to 0, so
        #  the recorded baseline is unaffected.  Raise it to get the old
        #  scroll-and-pause behaviour back.
        self.read_pause = 0
        
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
        
                
        if(self.FailAllDone):
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
        #   TWO DECISIONS, NOT ONE.  This block used to make the stop
        #   conditional on `ns == 0`, which quietly meant a PARTIAL solve that
        #   stalled could never stop.  The reason to fix that is NOT speed --
        #   these are hard problems and a long solve is entirely legitimate --
        #   it is that the node was CONTINUING PAST ITS OWN PROOF.  Once a pass
        #   has left the solved set and every equation pool exactly as it found
        #   them, re-running the identical pass cannot produce a different
        #   result;  comp_det had already established there was nothing to do,
        #   carried on anyway, and then reported "the budget ran out" as though
        #   the budget were the reason it stopped.  That is a wrong answer about
        #   WHY the solve ended, and it buries the real diagnosis.
        #
        #   Observed on Issue4's derived arm:  th_1 solved in pass 1, then nine
        #   further passes each leaving the signature at (1, 0, 13, 60, ...).
        #   Ending at the repeat reports what actually happened.  It is also
        #   faster, which is welcome but is a side effect, not the point.
        #
        #   The `ns == 0` was not simply redundant, though -- it was load-bearing
        #   for a DIFFERENT question, which is why removing it alone would be a
        #   regression.  `no_progress` does not mean "stop ticking";  it means
        #   "there is nothing to report".  ik_driver.run_solver() skips
        #   create_solution_set() when it is set, and solved_anything() gates
        #   emit_outputs() on it -- so setting it for a partial solve would
        #   DISCARD the partial closed form, and IKBT has always reported
        #   partial solves.
        #
        #   So the two are now separated:
        #
        #       stalled      -> stop ticking            (signature repeated)
        #       ns == 0      -> nothing to report       (no_progress)
        #
        L1 = tick.blackboard.get('eqns_1u') or []
        L2 = tick.blackboard.get('eqns_2u') or []
        L3 = tick.blackboard.get('eqns_3pu') or []

        #   CONTENTS, not counts.  The signature used to be
        #
        #       (ns, len(L1), len(L2), len(L3), len(aux))
        #
        #   which is not a sound "nothing changed" test:  a pass that REPLACES
        #   an equation with a different one of the same count looks identical.
        #   That unsoundness was harmless only because the stop was also gated
        #   on `ns == 0` -- a solve that had never solved anything.  Opening the
        #   stop to partial solves exposed it immediately:  measured over all 32
        #   robots, ICP5p5_A21 and Parkman13 went `solved -> partial`, because
        #   both complete via a pass whose pool counts happen to match the
        #   previous pass while its contents move on.
        #
        #   Comparing the equations themselves costs a str() per equation --
        #   about 75 of them on Issue4, a few milliseconds against passes that
        #   run 80 s.  Sorted, because a reordered pool is not progress either.
        signature = (tuple(sorted(u.name for u in unks if u.solved)),
                     _pool_signature(L1),
                     _pool_signature(L2),
                     _pool_signature(L3),
                     _pool_signature(R.kequation_aux_list))
        previous  = tick.blackboard.get('comp_det_signature')
        tick.blackboard.set('comp_det_signature', signature)

        repeated = (previous is not None and signature == previous)

        #   A REPEATED SIGNATURE IS NOT PROOF OF BEING STUCK, and that is the
        #   subtle part.  assigner_leaf round-robins `curr_unk` over the unsolved
        #   variables, so a pass can change nothing simply because it was offered
        #   a variable it cannot currently solve -- and the NEXT pass, offered a
        #   different one, succeeds.  `curr_unk` is solver state the signature
        #   cannot see.  Traced on ICP5p5_A21:
        #
        #       pass 3  solved th_1,th_3,th_4  L1=6  curr_unk=th_4  changed
        #       pass 4  solved th_1,th_3,th_4  L1=6  curr_unk=th_1  NO CHANGE
        #       pass 5+ ... goes on to solve the 4th variable
        #
        #   Stopping on the repeat alone took ICP5p5_A21 and Parkman13 from
        #   `solved` to `partial`.
        #
        #   What IS sound is an EMPTY eqns_1u:  every solver leaf needs an
        #   equation in one unknown to start, so with L1 empty nothing can fire
        #   for ANY variable and the assigner's cursor stops mattering.  Combined
        #   with unchanged pool contents -- meaning no transform produced
        #   anything either -- that is a real dead end.
        #
        #   The ns == 0 path keeps its ORIGINAL condition (repeat alone), so
        #   every previously-stopping robot stops exactly as before.  Requiring
        #   an empty L1 there too would risk letting a 0-solved robot run to
        #   budget without `no_progress`, and create_solution_set() would then
        #   crash on an empty solution set -- the bug that guard exists for.
        stalled = repeated and (ns == 0 or len(L1) == 0)

        if stalled:
            print('')
            if ns == 0:
                #  Nothing solved at all:  there is no solution to report, and
                #  create_solution_set() must not be called on an empty set.
                print('   No solution found.')
                print('   Nothing was solved and the last pass changed nothing:')
                print('     one-unknown equations available: ', len(L1))
                if len(L1) == 0:
                    print('     -- eqns_1u is EMPTY, so no ID node can fire.  Every')
                    print('        solver leaf needs an equation in one unknown to start.')
                print('   Stopping here rather than generating an empty report.')
                tick.blackboard.set('no_progress', True)
            else:
                #  A real partial result.  Stop ticking, but KEEP it -- the
                #  solution set for the solved variables is still valid.
                print('   Partial solution:  %d of %d variables solved.' % (ns, n))
                print('   The last pass changed nothing at all -- same variables,')
                print('     same equation counts (1u/2u/3pu = %d/%d/%d).'
                      % (len(L1), len(L2), len(L3)))
                if len(L1) == 0:
                    print('     -- eqns_1u is EMPTY, so no ID node can fire on the')
                    print('        remaining variables.')
                print('   Stopping here rather than re-deriving the same state')
                print('     for the rest of the pass budget.')
            print('')
            return DONEComplete   # break out of the outer solve loop

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
        self.test_compE_stalled_partial_stops_but_keeps_its_result()
        self.test_compF_partial_still_progressing_is_not_stopped()
        self.test_compG_stalled_partial_with_equations_left_keeps_going()

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


    def test_compE_stalled_partial_stops_but_keeps_its_result(self):
        """A PARTIAL solve that stalls must stop -- but keep what it solved.

           This is the Issue4 defect.  The stop used to be conditional on
           `ns == 0`, so a solve that got one variable and then stalled could
           never trigger it, and the node went on re-deriving a state it had
           already proven static -- then blamed the pass budget for the stop.
           Issue4's derived arm solved th_1 in pass 1 and then repeated an
           identical signature for nine further passes.

           But `no_progress` must NOT be set here.  It does not mean "stop
           ticking";  ik_driver.run_solver() skips create_solution_set() when it
           is set and solved_anything() gates emit_outputs() on it, so setting
           it for a partial solve would DISCARD the partial closed form.  Two
           separate decisions, and this test pins both."""

        fs = ' comp_det stalled-partial FAIL'
        #  nL1=0 matters:  an EMPTY eqns_1u is what makes the stall sound.
        bb = self.make_bb(nsolved=1, nunk=3, nL1=0)

        st, node = self.tick(bb)
        self.assertEqual(st, b3.FAILURE, fs + ' (bailed on the first pass)')

        #  An identical second pass:  same solved count, same equation counts.
        st, node = self.tick(bb, node)
        self.assertEqual(st, b3.SUCCESS,
                         fs + ' (a stalled partial must stop the outer loop)')
        self.assertFalse(bb.get('no_progress'),
                         fs + ' (must NOT discard a real partial result)')

    def test_compF_partial_still_progressing_is_not_stopped(self):
        """A partial solve that is still solving must not be stopped."""

        fs = ' comp_det premature partial stop FAIL'
        bb = self.make_bb(nsolved=1, nunk=3, nL1=1)
        st, node = self.tick(bb)
        self.assertEqual(st, b3.FAILURE, fs)

        #  Another variable falls:  the signature changes, so keep going.
        bb2 = self.make_bb(nsolved=2, nunk=3, nL1=1)
        bb2.set('comp_det_signature', bb.get('comp_det_signature'))
        st, node = self.tick(bb2, node)
        self.assertEqual(st, b3.FAILURE,
                         fs + ' (stopped while variables were still falling)')
        self.assertFalse(bb2.get('no_progress'), fs)

    def test_compG_stalled_partial_with_equations_left_keeps_going(self):
        """THE REGRESSION TEST.  A partial solve whose pass changed nothing but
           which still HAS one-unknown equations must NOT be stopped.

           assigner_leaf round-robins curr_unk, so an unchanged pass often just
           means "offered a variable it could not solve this time";  the next
           pass, offered a different one, succeeds.  Traced on ICP5p5_A21:
           pass 4 changed nothing with L1=6, and the solve went on to finish.
           Stopping on the repeat alone took ICP5p5_A21 and Parkman13 from
           `solved` to `partial` across the 32-robot sweep."""

        fs = ' comp_det stalled-partial-with-L1 FAIL'
        bb = self.make_bb(nsolved=1, nunk=3, nL1=6)

        st, node = self.tick(bb)
        self.assertEqual(st, b3.FAILURE, fs + ' (bailed on the first pass)')

        #  An IDENTICAL second pass -- but eqns_1u is not empty, so an ID node
        #  can still fire once the assigner offers a different variable.
        st, node = self.tick(bb, node)
        self.assertEqual(st, b3.FAILURE,
                         fs + ' (stopped a partial that could still progress)')
        self.assertFalse(bb.get('no_progress'), fs)


def run_test():
    suite = unittest.TestLoader().loadTestsFromTestCase(TestSolver014)
    unittest.TextTestRunner(verbosity=2).run(suite)


##   Self Tests
if __name__ == "__main__":
    run_test()
      
