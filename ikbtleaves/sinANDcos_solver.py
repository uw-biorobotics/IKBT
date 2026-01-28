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
   

