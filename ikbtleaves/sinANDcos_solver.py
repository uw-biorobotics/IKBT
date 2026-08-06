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
import b3 as b3          # behavior trees
from ikbtfunctions.helperfunctions import *
import ikbtbasics.kin_cl as kc
from ikbtbasics.ik_classes import *     # special classes for Inverse kinematics in sympy
from ikbtleaves.assigner_leaf import *


#
#   This file identifies and solves for the case:
#
#   x = Acos(th) + Bsin(th)
#
#     whose solution is
#
#   th = { arcsin(x/r) - atan2(A,B) , pi - arcsin(x/r) - atan2(A,B) }
#
#   where r = sqrt(A**2 + B**2)
#                                  (BH text sec 4.3.5)
#   updated:
#   th = atan2(a, b) + atan2(sqrt(t), c)
#   th = atan2(a, b) + atan2(-sqrt(t), c)
#   t = sqrt(a**2 + b**2 - c**2)


class test_sinandcos_id(b3.Action):    # tester for sincos ID    
   def tick(self, tick):
      R = Robot()
      # set up bb data for testing sincos_id
      Td = sp.zeros(5)
      Ts = sp.zeros(5)

      Td[1,1] = l_1
      Ts[1,1] = sp.sin(th_2)

      Td[1,2] = l_2
      Ts[1,2] = sp.cos(th_3)
      
      Td[2,0] = l_6
      Ts[2,0] = l_1*sp.sin(th_1) +  l_2*sp.cos(th_1)

      Td[2,1] = l_2+5
      Ts[2,1] = l_3*sp.cos(th_3) + l_3*sp.sin(th_3) + l_4  # match!!
      
      Td[2,2] = l_2
      Ts[2,2] = l_3*sp.cos(th_6) + l_1*sp.sin(th_6) + l_4*sp.sin(th_6)

      Td[3,3] = l_2 + l_1
      Ts[3,3] = sp.sin(th_3)*sp.sin(th_4) + l_1*sp.cos(th_4)  # should only match if test repeats and th_3 becomes known
      
  
      testm = matrix_equation(Td,Ts)      
      R.mequation_list = [testm]

      uth1 = unknown(th_1)
      uth2 = unknown(th_2)
      uth3 = unknown(th_3)
      uth4 = unknown(th_4)
      uth5 = unknown(th_5)
      uth6 = unknown(th_6)

      variables = [uth1, uth2, uth3, uth4, uth5, uth6]
      
      [L1, L2, L3p] = R.scan_Mequation(testm, variables)  # lists of 1unk, 2unk, and 3+unk equations
      tick.blackboard.set('eqns_1u', L1)
      tick.blackboard.set('eqns_2u', L2)
      tick.blackboard.set('eqns_3pu', L3p)
      tick.blackboard.set('unknowns',variables)
      tick.blackboard.set('Robot',R)    
      return b3.SUCCESS
      

class sinandcos_id(b3.Action):    # action leaf for sincosid     
    def tick(self, tick):
        unknowns = tick.blackboard.get('unknowns')   # the current list of unknowns
        R = tick.blackboard.get('Robot')
    
        one_unk = tick.blackboard.get('eqns_1u')
        two_unk = tick.blackboard.get('eqns_2u')
    
        u = tick.blackboard.get('curr_unk')
        # identify unknowns in T where one equation can be solved by
        #        arcsin() or arccos()

        Aw = sp.Wild("Aw")
        Bw = sp.Wild("Bw")
        Cw = sp.Wild('Cw')
        Dw = sp.Wild('Dw')
        found = False      

        if (not u.solved):  # only if not already solved!
          for e in one_unk:  # only look at the eqns with one unknowns
              #print "Looking for unknown: ", u.symbol, " in equation: ", 
              print(e)
              
              tmp = e.RHS-e.LHS
              lhs = l_1 - l_1
              if (tmp.has(sp.sin(u.symbol)) and tmp.has(sp.cos(u.symbol))):

                  es = tmp.expand()
                  # collect terms in sin(x) and cos(x)
                  es = es.collect(sp.sin(u.symbol))
                  es = es.collect(sp.cos(u.symbol))


                  d ={}
                  d[Aw] = es.coeff(sp.sin(u.symbol))
                  d[Bw] = es.coeff(sp.cos(u.symbol))
                  d[Cw] = es - d[Aw]*sp.sin(u.symbol) - d[Bw]*sp.cos(u.symbol)

                  if(self.BHdebug):
                      print('Sin AND Cos identifying: ', es)
                      print('Aw: ', d[Aw], ' Bw: ', d[Bw], ' Cw: ', d[Cw])
                  
                  if not d[Cw].has(u.symbol):
                      u.readytosolve = True
                      u.eqntosolve   = kc.kequation(lhs, es)
                      u.solvemethod += 'sinANDcos'
                      found = True
                      break
            
        tick.blackboard.set('curr_unk', u)
        tick.blackboard.set('Robot', R)
        tick.blackboard.set('unknowns',unknowns)   # the current list of unknowns
        
        if found:
            return b3.SUCCESS
        else:
            return b3.FAILURE
    
 

      
# solve eqns in T where one equation can be solved by
#        arcsin() or arccos()
class sinandcos_solve(b3.Action):    # Solve asincos equation pairs
    def tick(self, tick):
       unknowns = tick.blackboard.get('unknowns')
       R = tick.blackboard.get('Robot')
       u = tick.blackboard.get('curr_unk')
       

       if(not u.solved):                
            if (u.readytosolve) and ('sinANDcos' in u.solvemethod):
                if(self.BHdebug): 
                  print("\nsinANDcos solver: I'm working on: ", u.symbol)
                  print("  Using: ", )
                  print(u.eqntosolve)

                l1  = u.eqntosolve.LHS
                rhs = u.eqntosolve.RHS
                A = rhs.coeff(sp.sin(u.symbol))
                B = rhs.coeff(sp.cos(u.symbol))
                C = A*sp.sin(u.symbol) + B*sp.cos(u.symbol) - rhs

                if self.BHdebug:
                  print("\n find the A, B ,C")
                  print(A)
                  print(B)
                  print(C)

                # lhs = l1- C


                # if (B is None):
                #     B = 1
                assert(A*A+B*B != 0), 'Somethings Wrong: divide by zero'
                #assert(not lhs.has(u.symbol)), 'Somethings wrong: solution contains itself! ' + str(d[Cw])
                r = sp.sqrt(A*A+B*B)
                if (A==B):
                    r=sp.sqrt(2)*A
                
                    
                    #  generate the solutions
                    
                # targument = C/r
                # u.argument = targument
                if not C == 0:
                  t = sp.sqrt(A*A + B*B - C*C)
                  u.solutions.append(sp.atan2(A, B) + sp.atan2(t, C))
                  u.solutions.append(sp.atan2(A, B) + sp.atan2(-t, C))
                else:
                  u.solutions.append(sp.atan2(-B, A))
                  u.solutions.append(sp.atan2(-B, A) + sp.pi)
        
                #u.solutions.append(sp.asin(targument)-sp.atan2(A,B))
                #u.solutions.append(sp.pi - sp.asin(targument)-sp.atan2(A,B))
                u.nsolutions = 2
                u.set_solved(R,unknowns)
                if(self.BHdebug):
                    print('I think I solved ', u.symbol)
                    sp.pprint(u.solutions)
                    print('')
#
       tick.blackboard.set('curr_unk', u)
       tick.blackboard.set('Robot', R)
       tick.blackboard.set('unknowns', unknowns)
       return b3.SUCCESS    
       
       
###################################
#   Unit test code
#       
class TestSolver003(unittest.TestCase):
    def setUp(self):
        self.DB = False  # debug flag
        print('\n\n===============  Test sinANDcos Solver  ==================')
        return
    
    def runTest(self):
        self.test_sinANDcos()
        self.test_scB_roundtrip_numeric()
        self.test_scB_C_equals_zero_branch()
        self.test_scB_tangency_duplicate_solutions()
        self.test_scB_unreachable_gives_complex()

    #####################################################################
    #  Numeric round-trip tests.
    #
    #  The assertions in test_sinANDcos() above compare against exact sympy
    #  expressions, which were evidently captured from a run.  That pins the
    #  current FORM, so it cannot distinguish "correct" from "unchanged" -- a
    #  sign error present when they were captured would be enshrined, not
    #  caught.  These tests instead substitute numbers into whatever the solver
    #  returns and check it actually satisfies the original equation.

    def run_sc(self, expr, sym):
        '''Drive sinandcos_id + sinandcos_solve on the equation 0 == expr.
           Returns the unknown for `sym`.'''
        u = kc.unknown(sym)
        sc_id = sinandcos_id()
        sc_id.BHdebug = self.DB
        sc_sl = sinandcos_solve()
        sc_sl.BHdebug = self.DB

        ik_tester = b3.BehaviorTree()
        ik_tester.root = b3.Sequence([sc_id, sc_sl])

        bb = b3.Blackboard()
        bb.set('curr_unk', u)
        bb.set('unknowns', [u])
        bb.set('eqns_1u', [kc.kequation(0, expr)])
        bb.set('eqns_2u', [])
        bb.set('Robot', Robot())
        ik_tester.tick("sinANDcos numeric roundtrip", bb)
        return bb.get('curr_unk')

    def check_real_and_satisfies(self, sol, expr, sym, fs):
        '''A solution must be REAL and must satisfy the equation.
           The real check is deliberate: complex results are how this solver
           currently reports an unreachable pose (see
           test_scB_unreachable_gives_complex), so a reachable case that goes
           complex is a genuine failure, not a near miss.'''
        val = complex(sp.N(sol))
        self.assertAlmostEqual(val.imag, 0.0, places=9,
                               msg=fs + ' (solution is complex for a reachable pose)')
        resid = complex(sp.N(expr.subs(sym, sp.re(sol))))
        self.assertAlmostEqual(resid.real, 0.0, places=9,
                               msg=fs + ' (solution does not satisfy the equation)')
        self.assertAlmostEqual(resid.imag, 0.0, places=9, msg=fs)
        return val.real

    def test_scB_roundtrip_numeric(self):
        '''A*sin(th) + B*cos(th) = C, generic reachable case.  BOTH returned
           solutions must satisfy the equation.'''
        sp.var('th_1')
        fs = ' sinANDcos numeric roundtrip FAIL'
        A, B, C = 3, 4, 2                      # A^2+B^2 = 25 > C^2 = 4  -> reachable
        expr = A*sp.sin(th_1) + B*sp.cos(th_1) - C

        u = self.run_sc(expr, th_1)

        self.assertTrue(u.solved, fs + ' (leaf did not fire)')
        self.assertEqual(len(u.solutions), 2, fs + ' (expected 2 branches)')
        roots = [self.check_real_and_satisfies(s, expr, th_1, fs) for s in u.solutions]
        # the two branches must be genuinely different roots here
        self.assertGreater(abs(roots[0] - roots[1]), 1e-6,
                           fs + ' (the two branches collapsed to one root)')

    def test_scB_C_equals_zero_branch(self):
        '''C == 0 takes a separate code path: atan2(-B,A) and that + pi.'''
        sp.var('th_1')
        fs = ' sinANDcos C==0 branch FAIL'
        A, B = 3, 4
        expr = A*sp.sin(th_1) + B*sp.cos(th_1)          # C == 0

        u = self.run_sc(expr, th_1)

        self.assertTrue(u.solved, fs + ' (leaf did not fire)')
        self.assertEqual(len(u.solutions), 2, fs)
        roots = [self.check_real_and_satisfies(s, expr, th_1, fs) for s in u.solutions]
        # the two roots must differ by pi
        self.assertAlmostEqual(abs(abs(roots[0] - roots[1]) - float(sp.pi)), 0.0, places=9,
                               msg=fs + ' (branches are not pi apart)')

    def test_scB_tangency_duplicate_solutions(self):
        '''A^2+B^2 == C^2 is the tangency case: t == 0, so the two branches
           coincide.  The root is correct, but nsolutions is still 2 with two
           identical entries, which feeds duplicate rows into the version
           machinery.  Characterization test -- update it if that is fixed.'''
        sp.var('th_1')
        fs = ' sinANDcos tangency FAIL'
        A, B, C = 3, 4, 5                       # 9 + 16 == 25
        expr = A*sp.sin(th_1) + B*sp.cos(th_1) - C

        u = self.run_sc(expr, th_1)

        self.assertTrue(u.solved, fs)
        roots = [self.check_real_and_satisfies(s, expr, th_1, fs) for s in u.solutions]
        self.assertAlmostEqual(roots[0], roots[1], places=9,
                               msg=fs + ' (expected the tangency double root)')
        # documents the duplicate: nsolutions counts 2 identical branches
        self.assertEqual(u.nsolutions, 2, fs + ' (duplicate-branch behavior changed)')

    def test_scB_unreachable_gives_complex(self):
        '''C^2 > A^2+B^2 is unreachable: t = sqrt(A^2+B^2-C^2) is imaginary and
           the solver returns complex solutions rather than reporting failure.

           This may be intentional -- the real part is the closest reachable
           pose -- but it is not tracked consistently across leaves.  See the
           note in ImplementationThoughts.md.  Characterization test: it pins
           the behavior so a future change is a deliberate decision.'''
        sp.var('th_1')
        fs = ' sinANDcos unreachable-case FAIL'
        A, B, C = 3, 4, 10                      # 25 < 100  -> unreachable
        expr = A*sp.sin(th_1) + B*sp.cos(th_1) - C

        u = self.run_sc(expr, th_1)

        self.assertTrue(u.solved, fs + ' (leaf did not fire)')
        imags = [abs(complex(sp.N(s)).imag) for s in u.solutions]
        self.assertGreater(max(imags), 1e-9,
                           fs + ' (unreachable pose no longer yields complex -'
                                ' if this was fixed deliberately, update this test)')


    def test_sinANDcos(self):
        ik_tester = b3.BehaviorTree()
        bb = b3.Blackboard()        
        sc_setup = test_sinandcos_id()
        sc_setup.BHdebug = self.DB
        sc_id = sinandcos_id()
        sc_id.BHdebug = self.DB
        sc_sl = sinandcos_solve()
        sc_sl.BHdebug = self.DB
        
        asg = assigner()

        subtree = b3.Repeater(b3.Sequence([asg, sc_id, sc_sl]), 6)

        test = b3.Sequence([sc_setup, subtree])

        #test = b3.Sequence([sc_setup, sc_id, sc_sl])
        ik_tester.root = test
        
        # run the test BT
        ik_tester.tick("test sin and cos solver", bb)
        
        unkns = bb.get("unknowns")        
        fs = ' sin AND cos solver FAIL'
        ntests = 0
        for u in unkns:
            print(u.symbol)
            print(u.solutions)
            if(u.symbol == th_1):
                ntests += 1
                self.assertTrue(u.solved, fs)
                self.assertTrue(u.solutions[0] ==  sp.atan2(l_1, l_2) + sp.atan2(sp.sqrt(l_1**2 + l_2**2 - l_6**2), l_6), fs)
                self.assertTrue(u.solutions[1] == sp.atan2(l_1, l_2) + sp.atan2(-sp.sqrt(l_1**2 + l_2**2 - l_6**2), l_6), fs)
                

            if(u.symbol == th_3):
                ntests += 1
                self.assertTrue(u.solved, fs)
                self.assertTrue(u.solutions[0] ==  sp.atan2(l_3, l_3) + sp.atan2(sp.sqrt(2*l_3**2 - (l_2 - l_4 + 5)**2), l_2 - l_4 + 5), fs)
                self.assertTrue(u.solutions[1] == sp.atan2(l_3, l_3) + sp.atan2(-sp.sqrt(2*l_3**2 - (l_2 - l_4 + 5)**2), l_2 - l_4 + 5), fs)
                
                
            if (u.symbol == th_4):
                ntests += 1
                self.assertTrue(not u.solved, fs + ' [th_4]' )
                
            if (u.symbol == th_6):
                ntests  += 1 
                self.assertTrue(u.solutions[0] == sp.atan2(l_1 + l_4, l_3) + sp.atan2(sp.sqrt(-l_2**2 + l_3**2 + (l_1 + l_4)**2), l_2), fs + ' [th_6]')
                self.assertTrue(u.solutions[1] == sp.atan2(l_1 + l_4, l_3) + sp.atan2(-sp.sqrt(-l_2**2 + l_3**2 + (l_1 + l_4)**2), l_2), fs + ' [th_6a]')

        self.assertTrue(ntests == 4, 'sinANDcos_solver:    Assert count fail       FAIL')
 
def run_test():
    print('\n\n===============  Test sinANDcos_solver.py =====================')
    testsuite = unittest.TestLoader().loadTestsFromTestCase(TestSolver003)  # replace TEMPLATE 
    unittest.TextTestRunner(verbosity=2).run(testsuite)
        
if __name__ == "__main__":
    
    suite = unittest.TestLoader().loadTestsFromTestCase(TestSolver003)
    unittest.TextTestRunner(verbosity=2).run(suite)
   

