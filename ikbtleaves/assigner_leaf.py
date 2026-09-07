# major refactor 2: control logic change, move the looping logic outside of the solver nodes
# 21-Jun-2017 DZ
# Assigner node is an action node that put one variable onto blackboard
# Other solver nodes will read the assigned unknown variable from the board
# and try to solve it

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


#####################################################################
#
#   Sum-of-angles variables must be offered as soon as they are DETERMINED
#
#   The plain round-robin below walks `unknowns` in list order with a
#   persistent counter.  A sum-of-angles (SOA) variable such as th_23 is
#   APPENDED to that list by the SOA scan, so it is always offered LAST, after
#   every real joint.
#
#   That is wrong, not merely suboptimal.  `th_23 = th_2 + th_3` is ARITHMETIC:
#   the instant th_2 and th_3 are solved, th_23 has one value and no branch.
#   Leaving it unsolved is not a neutral delay, because count_unknowns() counts
#   an unsolved SOA variable like any other -- so EVERY equation mentioning
#   th_23 is classified into L2/L3p instead of L1, and the ID nodes, which scan
#   eqns_1u only, cannot see it.  Delaying a free variable hides equations from
#   the solvers that need them, and the spurious branches that follow inflate
#   the solution matrix.  (Worked example: IKdocs/DEV_NOTES.md.)
#
#   THE RULE, precisely.  An unsolved unknown is DETERMINED when
#   Robot.kequation_aux_list holds a *definition* of it:  an equation whose LHS
#   is that unknown's own symbol and whose RHS is free of transcendental
#   functions and mentions only joint variables that are ALREADY SOLVED.  Such
#   an RHS evaluates to a number as it stands, so the unknown has one value and
#   no branch.  A determined unknown is offered ahead of the round-robin cursor.
#
#   THE RULE IS DELIBERATELY NARROW.  kequation_aux_list is not only the SOA
#   definitions -- invariant_gen and parallel_triple mine invariants into it,
#   and x2y2_transform appends its product -- so a broader rule of "any aux
#   equation with one unsolved unknown left" also fires on large nonlinear
#   equations where "one unknown remains" says nothing about determinacy.
#
#   A STARVATION GUARD IS MANDATORY -- this is a trap, not a refinement.
#   Being determined is a property of the solved SET, so it does not change
#   merely because the variable was offered:  if the leaf that should finish it
#   does not, the rule fires again next tick with the same answer, forever, and
#   the assigner stops offering anything else.  So each (variable, solved-state)
#   pair is offered at most ONCE.  If the preemption does not take, the memo
#   refuses the repeat and the round-robin resumes untouched -- its counter is
#   never advanced by a preemption, so no ordinary variable is skipped.  The
#   memo lives on the blackboard so clear_state drops it, giving the hybrid
#   branch's second solver a fresh one.
#
PROMOTED_KEY = 'assigner_promoted'


def determined_unknown(R, unknowns):
    '''The first unsolved unknown that kequation_aux_list already DEFINES in
       terms of solved joint variables alone, or None.

       See the block comment above for why this test and not a broader one.
       Returns the definition too, for the debug line and for the test.'''
    aux = getattr(R, 'kequation_aux_list', None) or []
    if not aux:
        return None, None
    all_syms = set(u.symbol for u in unknowns)
    solved_syms = set(u.symbol for u in unknowns if u.solved)

    for u in unknowns:
        if u.solved:
            continue
        for e in aux:
            #  a DEFINITION:  LHS is exactly this unknown, nothing else
            if e.LHS != u.symbol:
                continue
            rhs = e.RHS
            try:
                free = rhs.free_symbols
                #  no sin/cos/atan2/...:  a transcendental RHS is an equation
                #  to be solved, not an arithmetic value to be evaluated
                if rhs.atoms(sp.Function):
                    continue
            except AttributeError:
                continue            # a plain number defines nothing useful
            if not free:
                continue
            #  every symbol on the RHS must be a joint variable, and solved
            if not free.issubset(all_syms):
                continue
            if not free.issubset(solved_syms):
                continue
            return u, e
    return None, None


class assigner(b3.Action):
    def tick(self, tick):
        unknowns = tick.blackboard.get("unknowns")

        #  Offer a determined variable ahead of the cursor -- once per
        #  solved-state, see the starvation trap above.
        R = tick.blackboard.get("Robot")
        u, e = determined_unknown(R, unknowns)
        if u is not None:
            memo = tick.blackboard.get(PROMOTED_KEY)
            if memo is None:
                memo = set()
                tick.blackboard.set(PROMOTED_KEY, memo)
            state = frozenset(str(v.symbol) for v in unknowns if v.solved)
            key = (str(u.symbol), state)
            if key not in memo:
                memo.add(key)
                if getattr(self, 'BHdebug', False):
                    print('\n\nAssigner: determined variable promoted: %s  (%s = %s)'
                          % (u.symbol, e.LHS, e.RHS))
                tick.blackboard.set("curr_unk", u)
                return b3.SUCCESS

        counter = tick.blackboard.get("counter")
        if counter is None:
            counter = 0
            
        while counter < len(unknowns):
            curr = unknowns[counter]
            counter = counter + 1
            if not curr.solved:
                if getattr(self, 'BHdebug', False):
                    #  Was unconditional, and it is one line per tick:  on
                    #  Puma that is eleven lines of "Assigner:" in a normal
                    #  solve, which buries the per-pass progress it sits
                    #  between.  Every other leaf gates its chatter on
                    #  BHdebug;  this one had simply never been converted.
                    print("\n\nAssigner: variable on blackboard: %s"%curr.symbol)
                tick.blackboard.set("counter", counter)
                tick.blackboard.set("curr_unk", curr)
                return b3.SUCCESS
                
        if counter >= len(unknowns):
            counter = 0
        tick.blackboard.set("counter", counter)
        tick.blackboard.set("curr_unk", unknowns[counter])
        return b3.SUCCESS


#####################################################################
#
#   Test code
#

import unittest


class TestSolver027(unittest.TestCase):
    '''Promotion of a DETERMINED unknown ahead of the round-robin cursor,
       and the starvation guard that keeps that promotion from deadlocking.

       Two independent properties, one test class:  remove the promotion and
       test_asgA fails;  remove the memo and test_asgB fails.'''

    def setUp(self):
        print('\n\n===============  Test assigner promotion  ==============')
        return

    def runTest(self):
        self.test_asgA_soa_definition_is_promoted_ahead_of_the_cursor()
        self.test_asgB_starvation_guard_refuses_a_repeat_in_one_state()
        self.test_asgC_the_memo_is_keyed_on_the_SOLVED_STATE()
        self.test_asgD_an_unsolved_constituent_blocks_promotion()
        self.test_asgE_only_a_definition_promotes_not_any_aux_equation()

    #  -------------------------------------------------- fixtures

    class fakerobot(object):
        def __init__(self, aux):
            self.name = 'test_assigner_arm'
            self.kequation_aux_list = list(aux)

    class faketick(object):
        def __init__(self, bb):
            self.blackboard = bb

    def board(self, aux, solved=()):
        '''unknowns in the order the SOA scan leaves them:  the real joints
           first and th_23 APPENDED LAST, which is the whole reason the
           round-robin never reaches it in time.'''
        us = [unknown(sp.Symbol('th_1')), unknown(sp.Symbol('th_2')),
              unknown(sp.Symbol('th_3')), unknown(sp.Symbol('th_23'))]
        for u in us:
            u.solved = str(u.symbol) in solved
        bb = b3.Blackboard()
        bb.set('unknowns', us)
        bb.set('Robot', self.fakerobot(aux))
        bb.set('counter', 0)
        return bb, us

    def offer(self, node, bb):
        node.tick(self.faketick(bb))
        return str(bb.get('curr_unk').symbol)

    def soa(self):
        return [kequation(sp.Symbol('th_23'),
                          sp.Symbol('th_2') + sp.Symbol('th_3'))]

    #  -------------------------------------------------- tests

    def test_asgA_soa_definition_is_promoted_ahead_of_the_cursor(self):
        '''th_2 and th_3 are solved, so th_23 = th_2 + th_3 is ARITHMETIC and
           th_23 is determined.  It sits last in `unknowns` and the cursor is at
           0, so the plain round-robin would offer th_1.  It must offer th_23.

           Without the promotion this asserts th_1 and fails.'''
        fs = ' assigner promotion FAIL'
        bb, us = self.board(self.soa(), solved=('th_2', 'th_3'))
        self.assertEqual(self.offer(assigner(), bb), 'th_23',
                         fs + ' (a determined SOA variable was not promoted -- '
                         'it will be offered last, and every equation '
                         'mentioning it stays out of eqns_1u until then)')

    def test_asgB_starvation_guard_refuses_a_repeat_in_one_state(self):
        '''THE TRAP.  Being determined is a property of the solved SET, so it
           does not stop being true just because the variable was offered.  If
           nothing solves it, an unguarded rule re-offers it on every tick and
           the assigner never offers anything else again -- measured on
           KinovaLite as 102 consecutive preemptions and a solve stuck at 2/7.

           Nothing is marked solved between the two ticks here, so the second
           tick must fall through to the round-robin (th_1, cursor unmoved).

           Without the memo this asserts th_23 twice and fails.'''
        fs = ' assigner starvation-guard FAIL'
        node = assigner()
        bb, us = self.board(self.soa(), solved=('th_2', 'th_3'))

        self.assertEqual(self.offer(node, bb), 'th_23', fs + ' (setup)')
        self.assertEqual(self.offer(node, bb), 'th_1',
                         fs + ' (the same variable was promoted twice in one '
                         'solved-state -- this starves every other variable)')
        #  and the promotion must not have consumed a round-robin slot either
        self.assertEqual(self.offer(node, bb), 'th_23',
                         fs + ' (round-robin cursor is wrong after a promotion)')

    def test_asgC_the_memo_is_keyed_on_the_SOLVED_STATE(self):
        '''A one-shot flag would be too blunt:  after the solved set changes the
           variable deserves another offer.  The memo key carries the solved
           set for exactly that reason.'''
        fs = ' assigner memo-key FAIL'
        node = assigner()
        bb, us = self.board(self.soa(), solved=('th_2', 'th_3'))

        self.assertEqual(self.offer(node, bb), 'th_23', fs + ' (setup)')
        self.assertEqual(self.offer(node, bb), 'th_1', fs + ' (setup)')
        #  the solved set moves on -- th_23 is a fresh candidate again
        us[0].solved = True                      # th_1 got solved
        self.assertEqual(self.offer(node, bb), 'th_23',
                         fs + ' (memo is a permanent one-shot rather than '
                         'per-solved-state)')

    def test_asgD_an_unsolved_constituent_blocks_promotion(self):
        '''th_3 unsolved => th_23 is NOT determined => plain round-robin.'''
        fs = ' assigner premature-promotion FAIL'
        bb, us = self.board(self.soa(), solved=('th_2',))
        self.assertEqual(self.offer(assigner(), bb), 'th_1',
                         fs + ' (promoted a variable whose constituents are '
                         'not all solved)')

    def test_asgE_only_a_definition_promotes_not_any_aux_equation(self):
        '''kequation_aux_list is NOT only the SOA definitions:  invariant_gen,
           parallel_triple and x2y2_transform all append to it, and on
           KinovaLite it grows from 1 entry to 9 during the solve.  An
           invariant with one unsolved unknown left is not a determined
           variable -- th_5 inside a cos() is worth two branches and no leaf is
           guaranteed to crack it.  Only an arithmetic definition promotes.'''
        fs = ' assigner aux-equation discrimination FAIL'
        Px, l_2 = sp.symbols('Px l_2')
        t1, t2, t3, t23 = sp.symbols('th_1 th_2 th_3 th_23')

        #  (a) an invariant:  LHS is not the unknown at all
        bb, us = self.board([kequation(Px**2, l_2 * sp.cos(t23))],
                            solved=('th_2', 'th_3'))
        self.assertEqual(self.offer(assigner(), bb), 'th_1',
                         fs + ' (an invariant was read as a definition)')

        #  (b) LHS is the unknown, but the RHS is transcendental -- an equation
        #      to be solved, not a value to be evaluated
        bb, us = self.board([kequation(t23, sp.acos(l_2 * sp.sin(t2)))],
                            solved=('th_2', 'th_3'))
        self.assertEqual(self.offer(assigner(), bb), 'th_1',
                         fs + ' (a transcendental RHS was read as arithmetic)')

        #  (c) RHS mentions a symbol that is not a joint variable, so it is not
        #      determined by the solved set
        bb, us = self.board([kequation(t23, t2 + l_2)], solved=('th_2', 'th_3'))
        self.assertEqual(self.offer(assigner(), bb), 'th_1',
                         fs + ' (promoted on an RHS carrying a non-joint '
                         'symbol)')

        #  ... and the genuine definition still promotes when mixed in with all
        #  of the above, which is the real blackboard's situation
        bb, us = self.board([kequation(Px**2, l_2 * sp.cos(t23)),
                             kequation(t23, t2 + t3)],
                            solved=('th_2', 'th_3'))
        self.assertEqual(self.offer(assigner(), bb), 'th_23',
                         fs + ' (the definition was missed in a mixed aux list)')


def run_test():
    suite = unittest.TestLoader().loadTestsFromTestCase(TestSolver027)
    unittest.TextTestRunner(verbosity=2).run(suite)


##   Self Tests
if __name__ == "__main__":
    run_test()
