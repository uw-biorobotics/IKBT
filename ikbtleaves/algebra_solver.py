#!/usr/bin/python
#
#   ID/Solve simple algebra e.g.
#
#      e23 = d1 + e23*l_4  (where d1 is unk)
#
# moved loop outside the ID node, 06/2017
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

#all leaves
from ikbtfunctions.helperfunctions import *
from ikbtbasics.kin_cl import *
from ikbtbasics.ik_classes import *     # special classes for Inverse kinematics in sympy

import b3 as b3          # behavior trees

# custom
from ikbtleaves.assigner_leaf  import *
import ikbtleaves.comp_detect as cpd


 
class test_algebra_id(b3.Action):    # tester for your ID    
    def tick(self, tick):
        
        #test_number = tick.blackboard.get('test_number') # if present
        R = tick.blackboard.get('Robot')
        
        # set up bb data for testing  
        Td = ik_lhs()      # basic LHS template for TEST
        Ts = sp.zeros(4)

        
        Ts[0,1] =  th_2 + l_1*l_2
        Ts[0,2] =  d_1*l_3 + l_1
        #Ts[0,3] = sp.sin(th_1)*l2 + l_1
        
        Ts[1,1] =  th_5 + th_2*l_1
        Ts[1,2] =  0


        Ts[2,1] =  th_2 * th_3 + l_1
        Ts[2,2] =  sp.sin(th_3)

        Ts[3,1] = sp.sin(th_1 + th_2)
        Ts[3,2] = sp.sin(th_1 + th_2 + th_3)
        

        testm = matrix_equation(Td,Ts)
        sp.var('th_12 th_123 B')  # B is "known"

        ud1  = unknown(d_1)
        u12 = unknown(th_12)
        u123 = unknown(th_123)
        uth1 = unknown(th_1)
        uth2 = unknown(th_2)
        uth3 = unknown(th_3)
        uth4 = unknown(th_4)
        uth5 = unknown(th_5)
        variables = [ud1,  uth1, uth2, uth3, uth4, uth5,u123]

        [L1, L2, L3p] = R.scan_Mequation(testm, variables)  # lists of 1unk, 2unk, and 3+unk equations

        L3p.append(kequation(th_123, th_1+th_2+B))  # 3-unknown SOA equation

        tick.blackboard.set('eqns_1u', L1)
        tick.blackboard.set('eqns_2u', L2)
        tick.blackboard.set('eqns_3pu', L3p)
        tick.blackboard.set('unknowns',variables)
        tick.blackboard.set('Robot',R)
        return b3.SUCCESS
    
def spZconv(term):   # if term == int(0), make it a sympy zero
    if term==int(0):
        return sp.S.Zero
    else:
        return term


def linear_match(expr, sym):
    '''Decompose expr as A*sym + B with A, B free of sym, i.e. confirm that
       expr is LINEAR in sym -- which is the only thing this leaf can solve.

       Returns (A, B), or None if expr is not linear in sym or if A == 0.

       The exclusion is the whole point.  An unconstrained sympy Wild makes
       'Aw*sym + Bw' match any expression at all, via Aw = (expr - Bw)/sym, and
       sympy really does return that reading -- so without exclude= this
       function would accept sym**2, sin(th_1+th_2), and a plain numeric
       addend on a negative coefficient.  Same failure mode already fixed in
       tan_solver and sincos_solver.

       A == 0 is rejected separately: sym can cancel during expand()
       (e.g. (d_1+l_1)-(d_1+r_11) -> l_1-r_11), and (LHS-B)/A would then be
       zoo rather than raising.'''
    Aw = sp.Wild('Aw', exclude=[sym])
    Bw = sp.Wild('Bw', exclude=[sym])
    d = expr.match(Aw*sym + Bw)
    if d is None:
        return None
    A = d[Aw]
    B = d[Bw] if d[Bw] is not None else sp.S.Zero
    if A == 0:
        return None
    return (A, B)

class algebra_id(b3.Action):    # action leaf for  
    
    def tick(self, tick):
        Tm = tick.blackboard.get('Tm')   # the current matrix equation 
        unknowns = tick.blackboard.get('unknowns')   # the current list of unknowns
        
        one_unk = tick.blackboard.get('eqns_1u')
        if(len(one_unk)==0):   # algebra-solver can only work on one-unknown equations
            if self.BHdebug:
                print('algebra_id: there are no one-unknown equations')
            return b3.FAILURE
        
        two_unk = tick.blackboard.get('eqns_2u')
        R = tick.blackboard.get('Robot')
            
        if(self.BHdebug):
            print("\nrunning: ", self.Name)
            print(' eqns w/ one unk: ', len(one_unk))
            print(' eqns w/ two unk: ', len(two_unk))
            
    
        u = tick.blackboard.get('curr_unk')
      # identify unknowns in T where one equation can be solved by
      #           basic algebra
        found = False
        if (not u.solved):  # only if not already solved!
                for e in one_unk:  # eqns containing one unknown
                    if(self.BHdebug):
                        print('algebra ID: Looking for unknown: ', u.symbol, ' in equation: ')
                        print(e )
                        print("         ", count_unknowns(unknowns, e.RHS), " unknowns in RHS\n")
                    e.RHS = spZconv(e.RHS)
                    e.LHS = spZconv(e.LHS)
                    if (e.RHS.has(sp.sin(u.symbol)) or e.RHS.has(sp.cos(u.symbol)) or\
                        e.LHS.has(sp.sin(u.symbol)) or e.LHS.has(sp.cos(u.symbol))):
                        continue   # this shouldbe caught by another ID

                    # since we're not solving the equation here, simply count the unknowns will suffice for the identification
                    if(e.RHS.has(u.symbol) or e.LHS.has(u.symbol)):
                        tmp = e.RHS - e.LHS
                        tmp = tmp.expand()

                        tmp = tmp.collect(u.symbol)

                        #  "contains u" is not the same as "is linear in u", and
                        #  this leaf can only do linear.  Confirm the shape here
                        #  rather than letting algebra_solve divide by whatever
                        #  the Wilds happen to return.  Excluding u.symbol from
                        #  both Wilds is what gives the match teeth -- otherwise
                        #  'Aw*u + Bw' matches ANYTHING via Aw = (expr-Bw)/u,
                        #  the same degeneracy already fixed in tan_solver and
                        #  sincos_solver.  Rejected shapes and what they used to
                        #  produce:
                        #     d_1**2 - l_1            ->  d_1 = l_1/d_1
                        #     sin(th_1+th_2) - r_11   ->  th_1 = r_11*th_1/sin(th_1+th_2)
                        #     -d_1*l_3 + 5            ->  d_1 = d_1**2*l_3/5
                        #  i.e. "solutions" that are functions of the very
                        #  variable being solved -- and set_solved() was called
                        #  on all of them.
                        d = linear_match(tmp, u.symbol)
                        if d is None:
                            if(self.BHdebug):
                                print('algebra ID: not linear in ', u.symbol, ': ', tmp)
                            continue   # keep looking -- another eqn may be usable

                        u.readytosolve = True
                        u.eqntosolve = kequation(0, tmp)
                        u.solvemethod += "algebra"
                        found = True
                        break

        # not sure these "sets" are needed???? (or could be nested "if found")
        tick.blackboard.set('curr_unk', u)
        tick.blackboard.set('unknowns',unknowns)   # the current list of unknowns
        if found:
            return b3.SUCCESS
        else:
            return b3.FAILURE
       


class algebra_solve(b3.Action):    # Solve asincos equation pairs
    def tick(self, tick):
       Tm = tick.blackboard.get('Tm')   # the current matrix equation  
       unknowns = tick.blackboard.get('unknowns')
       R = tick.blackboard.get('Robot')
       
       u = tick.blackboard.get('curr_unk')
          
       if u.readytosolve:
           if(self.BHdebug):
                        print("I'm trying to solve: ", u.symbol)
                        print("  Using: ", )
                        print(u.eqntosolve  )
           if 'algebra' in u.solvemethod:
               #  algebra_id has already confirmed this shape; re-checking here
               #  is defence in depth, and gives a graceful FAILURE (letting the
               #  Priority try another leaf) instead of a TypeError on d[Aw]
               #  when d is None, or a zoo solution when A is 0.
               d = linear_match(u.eqntosolve.RHS, u.symbol)
               if d is None:
                   print('algebra_solve: ', u.symbol, ' is not linear in ',
                         u.eqntosolve.RHS, ' -- declining')
                   tick.blackboard.set('curr_unk', u)
                   tick.blackboard.set('unknowns', unknowns)
                   return b3.FAILURE
               A, B = d
               sol = (u.eqntosolve.LHS-B)/A
               #  a solution that is still a function of its own unknown is not
               #  a solution.  linear_match() should make this unreachable; the
               #  guard is kept because this is the invariant that matters and
               #  it is cheap.  Same check as sincos_solve.
               if sol.has(u.symbol):
                   print('algebra_solve: solution for ', u.symbol,
                         ' contains itself: ', sol, ' -- declining')
                   tick.blackboard.set('curr_unk', u)
                   tick.blackboard.set('unknowns', unknowns)
                   return b3.FAILURE
               u.solutions.append( sol )       # one solution
               u.nsolutions = 1   # or 1
               u.set_solved(R,unknowns)  # flag that this is solved
       tick.blackboard.set('curr_unk', u)
       tick.blackboard.set('unknowns', unknowns)
       return b3.SUCCESS
   
       
#######################################################################
#  Test code:
class TestSolver002(unittest.TestCase):
    def setUp(self):
        self.DB = False  # debug flag
        print('\n\n===============  Test algebra Solver  =====================')
        return

    def runTest(self):
        self.test_algebra()
        self.test_algB_roundtrip_numeric()
        self.test_algB_negative_coefficient_with_numeric_term()
        self.test_algB_rejects_nonlinear()
        self.test_algB_rejects_embedded_unknown()
        self.test_algB_rejects_cancelled_unknown()
        self.test_algB_scans_past_an_unusable_equation()
        self.test_algB_solution_never_contains_its_own_unknown()

    #####################################################################
    #  Shape screening and numeric round-trip tests.
    #
    #  test_algebra() below compares against exact sympy expressions captured
    #  from a run, over three linear cases with positive coefficients.  That
    #  pins the FORM of the answer, not its correctness, and says nothing about
    #  what the leaf does when handed a shape it cannot solve.
    #
    #  It could not: algebra_id's only screen was "does the equation mention u
    #  and not sin(u)/cos(u)", so anything nonlinear in u was claimed, and
    #  algebra_solve then divided by whatever an unconstrained Wild returned.
    #  All three of these were emitted AND passed to set_solved():
    #
    #      0 = d_1**2 - l_1           ->  d_1 = l_1/d_1
    #      r_11 = sin(th_1 + th_2)    ->  th_1 = r_11*th_1/sin(th_1 + th_2)
    #      0 = -d_1*l_3 + 5           ->  d_1 = d_1**2*l_3/5
    #
    #  i.e. "solutions" that are functions of the variable being solved.

    def run_alg(self, exprs, sym, others=()):
        '''Drive algebra_id + algebra_solve over `exprs` (each meaning
           "expr == 0"), trying to solve `sym`.  Returns (status, unknown).'''
        u = unknown(sym)
        aid = algebra_id();    aid.Name = 'Algebra ID';     aid.BHdebug = self.DB
        asl = algebra_solve(); asl.Name = 'Algebra Solver'; asl.BHdebug = self.DB

        t = b3.BehaviorTree()
        t.root = b3.Sequence([aid, asl])

        bb = b3.Blackboard()
        bb.set('curr_unk', u)
        bb.set('unknowns', [u] + [unknown(s) for s in others])
        bb.set('eqns_1u', [kequation(sp.S.Zero, e) for e in exprs])
        bb.set('eqns_2u', [])
        bb.set('eqns_3pu', [])
        bb.set('Robot', Robot())
        bb.set('Tm', None)
        status = t.tick('algebra shape test', bb)
        return status, bb.get('curr_unk')

    def assert_declined(self, status, u, fs):
        '''The leaf must not claim the variable, and must leave it unsolved so
           the Priority can offer it to another leaf.'''
        self.assertEqual(status, b3.FAILURE, fs + ' (leaf did not decline)')
        self.assertFalse(u.solved, fs + ' (marked solved)')
        self.assertEqual(len(u.solutions), 0, fs + ' (emitted a solution anyway)')

    def test_algB_roundtrip_numeric(self):
        '''The ordinary linear case still solves, and the answer really
           satisfies the equation -- checked by substituting numbers rather
           than by comparing expression form.'''
        sp.var('d_1 l_1 l_3 r_13')
        fs = ' algebra numeric roundtrip FAIL'
        expr = d_1*l_3 + l_1 - r_13

        status, u = self.run_alg([expr], d_1)

        self.assertEqual(status, b3.SUCCESS, fs + ' (leaf did not fire)')
        self.assertTrue(u.solved, fs)
        self.assertEqual(u.nsolutions, 1, fs)
        vals = {l_1: 0.7, l_3: -2.5, r_13: 1.3}
        got = complex(sp.N(u.solutions[0].subs(vals)))
        self.assertAlmostEqual(got.imag, 0.0, places=9, msg=fs + ' (complex)')
        resid = complex(sp.N(expr.subs(vals).subs(d_1, got.real)))
        self.assertAlmostEqual(abs(resid), 0.0, places=9,
                               msg=fs + ' (solution does not satisfy the equation)')

    def test_algB_negative_coefficient_with_numeric_term(self):
        '''Regression: a negative coefficient plus a loose numeric term is the
           exact trigger for the degenerate Wild match.  Both signs must give
           the same, correct answer.

              -d_1*l_3 + 5  ->  {Aw: 5/d_1, Bw: -d_1*l_3}   (unconstrained)
           so (LHS-B)/A came out as d_1**2*l_3/5.'''
        sp.var('d_1 l_3')
        fs = ' algebra negative-coefficient FAIL'

        for sign in (1, -1):
            expr = sign*d_1*l_3 + sign*5     # d_1 = -5/l_3 either way
            status, u = self.run_alg([expr], d_1)
            self.assertEqual(status, b3.SUCCESS,
                             fs + ' (coeff sign %+d did not solve)' % sign)
            self.assertFalse(u.solutions[0].has(d_1),
                             fs + ' (coeff sign %+d: solution contains its own'
                                  ' unknown: %s)' % (sign, u.solutions[0]))
            self.assertEqual(sp.simplify(u.solutions[0] - (-5/l_3)), 0,
                             fs + ' (coeff sign %+d: wrong answer %s)'
                                  % (sign, u.solutions[0]))

    def test_algB_rejects_nonlinear(self):
        '''u**2 is not linear in u.  Used to give d_1 = l_1/d_1, solved.'''
        sp.var('d_1 l_1')
        status, u = self.run_alg([d_1**2 - l_1], d_1)
        self.assert_declined(status, u, ' algebra nonlinear-reject FAIL')

    def test_algB_rejects_embedded_unknown(self):
        '''u inside a function of a SUM -- an un-substituted sum-of-angles term
           -- mentions u but is not linear in it.  has(sin(th_1)) is False for
           sin(th_1+th_2), so the sin/cos screen does not catch this one.
           Used to give th_1 = r_11*th_1/sin(th_1 + th_2), solved.'''
        sp.var('th_1 th_2 r_11')
        status, u = self.run_alg([sp.sin(th_1 + th_2) - r_11], th_1,
                                 others=[th_2])
        self.assert_declined(status, u, ' algebra embedded-unknown reject FAIL')

    def test_algB_rejects_cancelled_unknown(self):
        '''u can vanish during expand(), leaving A == 0.  (LHS-B)/A is then zoo
           rather than an exception, so nothing downstream notices.'''
        sp.var('d_1 l_1 r_11')
        expr = (d_1 + l_1) - (d_1 + r_11)          # expands to l_1 - r_11
        status, u = self.run_alg([expr], d_1)
        self.assert_declined(status, u, ' algebra zero-coefficient reject FAIL')

    def test_algB_scans_past_an_unusable_equation(self):
        '''The ID loop now `continue`s past a shape it cannot use instead of
           breaking, so an unusable equation early in the list no longer hides
           a usable one behind it.'''
        sp.var('d_1 l_1 l_3 r_13')
        fs = ' algebra scan-past FAIL'
        status, u = self.run_alg([d_1**2 - l_1,            # unusable
                                  d_1*l_3 + l_1 - r_13],   # usable
                                 d_1)
        self.assertEqual(status, b3.SUCCESS, fs + ' (gave up at the first eqn)')
        self.assertTrue(u.solved, fs)
        self.assertEqual(sp.simplify(u.solutions[0] - (r_13-l_1)/l_3), 0, fs)

    def test_algB_solution_never_contains_its_own_unknown(self):
        '''The invariant, swept over every shape above.  Cheap, and it catches
           degenerate matches whatever their cause.'''
        sp.var('d_1 th_1 th_2 l_1 l_3 r_11 r_13')
        fs = ' algebra self-reference FAIL'
        cases = [ (d_1*l_3 + l_1 - r_13,     d_1,  (),      True ),
                  (-d_1*l_3 - 5,             d_1,  (),      True ),
                  (d_1**2 - l_1,             d_1,  (),      False),
                  (sp.sin(th_1+th_2) - r_11, th_1, (th_2,), False),
                  (sp.cos(d_1) - l_1,        d_1,  (),      False) ]
        n = 0
        for expr, sym, others, solvable in cases:
            status, u = self.run_alg([expr], sym, others=others)
            self.assertEqual(status == b3.SUCCESS, solvable,
                             fs + ' (%s: expected solvable=%s)' % (expr, solvable))
            for s in u.solutions:
                self.assertFalse(s.has(sym), fs + ' (%s -> %s)' % (expr, s))
            n += 1
        self.assertEqual(n, len(cases), fs + ' (assert count)')

    def test_algebra(self):
        algebra_tester = b3.BehaviorTree()
        bb = b3.Blackboard()  
        bb.set('Robot', Robot())
        setup = test_algebra_id()  # see top of this file
        aid   = algebra_id()
        aid.Name = 'Algebra ID'
        aid.BHdebug = self.DB
        ais   = algebra_solve()
        ais.Name = 'Algebra Solver'
        ais.BHdebug = self.DB
        
        compdet = cpd.comp_det()
        compdet.Name = 'Completion Checker'
        compdet.BHdebug = self.DB
        
        asgn = assigner()
        subtree = b3.Sequence([asgn, aid, ais,compdet])
        
        test = b3.Sequence([setup, b3.Repeater(subtree, max_loop = 5)])          
        algebra_tester.root = test
        
        # Run the testing BT 
        algebra_tester.tick("Test the algebra ID/Solver", bb)
        
        # check the results
        Tm = bb.get('Tm')
        unk = bb.get('unknowns')        
        fs = ' algebra solver FAIL'
        sp.var(' r_13 r_12')  # elements of the rotation matrix portion of Td
        print('\n\n              Results: \n\n')
        ntests = 0
        for u in unk:
            if(u.symbol == d_1):
                ntests += 1
                self.assertTrue(u.solved, fs)
                self.assertTrue(u.nsolutions == 1, fs)
                print('Soln: ', u.solutions[0])
                self.assertTrue(u.solutions[0] == (r_13-l_1)/(l_3))
            if(u.symbol == th_2):
                ntests += 1
                self.assertTrue(u.solved, fs)
                self.assertTrue(u.nsolutions == 1, fs)
                self.assertTrue(u.solutions[0] == r_12-l_1*l_2, fs)
            if(u.symbol == th_3):
                ntests += 1
                self.assertFalse(u.solved, fs)

        self.assertTrue(ntests == 3, ' Algebra solver:  assertion count error --- FAIL')
        print('Algebra solver PASSED ', ntests, ' assertions.')
##  write tester code which runs if this file is run directly instead
##    of "imported".


#
#    Can run your test from command line by invoking this file
#
#      - or - call your TestSolverTEMPLATE()  from elsewhere
#

def run_test():
    print('\n\n===============  Test algebra solver =====================')
    testsuite = unittest.TestLoader().loadTestsFromTestCase(TestSolver002)  # replace TEMPLATE 
    unittest.TextTestRunner(verbosity=2).run(testsuite)

if __name__ == "__main__":
    
    print('\n\n===============  Test algebra solver =====================')
    testsuite = unittest.TestLoader().loadTestsFromTestCase(TestSolver002)  # replace TEMPLATE 
    unittest.TextTestRunner(verbosity=2).run(testsuite)
   

