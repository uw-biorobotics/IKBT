#!/usr/bin/python
#
#    ID and Solve equations based on sin(x) / x = arcin  etc.
#         BH New test system 22-may-17
#    loop logic moved outside of the node 21-jun-18 DZ
#    tests work for the current version

# Copyright 2017 University of Washington

# Developed by Dianmu Zhang and Blake Hannaford
# BioRobotics Lab, University of Washington

# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
import unittest

import sympy as sp  
import numpy as np
from sys import exit

# from ..ikbtfunctions.helperfunctions import *
# from ..ikbtbasics.kin_cl import *
# from ..ikbtbasics.ik_classes import *     # special classes for Inverse kinematics in sympy

from ikbtfunctions import helperfunctions
#from helperfunctions import *

from ikbtbasics import kin_cl
#from kin_cl import *

from ikbtbasics import ik_classes
#from ik_classes import *

import b3 as b3          # behavior trees
from assigner_leaf  import *

 
class test_sincos_id(b3.Action):    # tester for sincos ID    
   def tick(self, tick):
       # set up bb data for testing sincos_id
      Td = sp.zeros(5)
      Ts = sp.zeros(5)

      Td[0,1] = l_1
      Ts[0,1] = sp.cos(th_1)*l_2 + (l_3+l_4)*sp.cos(th_1) + l_5

      Td[1,1] = l_1
      Ts[1,1] = sp.sin(th_3)

      Td[1,2] = l_2
      Ts[1,2] = sp.cos(th_2)

      Td[2,1] = l_2+5
      Ts[2,1] = l_1*sp.cos(th_4)
      
      Td[2,2] = l_2
      Ts[2,2] = sp.sqrt(th_5)

      Td[2,3] = l_2
      Ts[2,3] = sp.cos(th_5)*sp.sin(th_2)
      
      Td[3,1] = l_2 + l_1
      Ts[3,1] = th_2+sp.cos(th_4)*l_3

      Td[3,2] = 17
      Ts[3,2] = l_1*sp.sin(th_3) + 2*l_2*sp.cos(th_3)

      testm = matrix_equation(Td,Ts)        
      R = Robot()
      R.mequation_list = [testm]
      
      uth1 = unknown(th_1)
      uth2 = unknown(th_2)
      uth3 = unknown(th_3)
      uth4 = unknown(th_4)
      uth5 = unknown(th_5)
      variables = [uth1, uth2, uth3, uth4, uth5]
      
      [L1, L2] = R.scan_Mequation(testm, variables)  # lists of 1unk and 2unk equations
      R.generate_solution_nodes(variables)
      R.sum_of_angles_transform(variables)
      
      tick.blackboard.set('eqns_1u', L1)
      tick.blackboard.set('eqns_2u', L2)
      tick.blackboard.set('unknowns',variables)
      tick.blackboard.set('Robot',R)    
      return b3.SUCCESS
      

class sincos_id(b3.Action):    # action leaf for sincosid 
    
    def tick(self, tick):
        unknowns = tick.blackboard.get('unknowns')   # the current list of unknowns
        #self.BHdebug = True
        
        u = tick.blackboard.get("curr_unk")
        R = tick.blackboard.get('Robot')

        one_unk = tick.blackboard.get('eqns_1u')
        two_unk = tick.blackboard.get('eqns_2u')

        if(self.BHdebug):
           print "sincosID, running: ", self.Name
           print 'working with ',len(one_unk), '1-unk equations and'
           print '             ',len(two_unk), '2-unk equations.'
           
        # identify equations where one equation can be solved by
        #        arcsin() or arccos()
        found = False
        
        # only run ID on variables that are (not solved or) identified as solvable_sincos
        if (not u.solvable_sincos) and (not u.solved):
            for e in one_unk:  
                if(self.BHdebug):
                    print "Looking for unknown: ", u.symbol, " in equation: ", 
                    e.prt()
                    print "  which has one unknown(s)"
                if (e.RHS.has(sp.sin(u.symbol)) or e.LHS.has(sp.sin(u.symbol)) ) \
                    and (e.RHS.has(sp.cos(u.symbol)) or e.LHS.has(sp.cos(u.symbol))):
                    continue   # this shouldbe caught by another ID
                if e.RHS.has(sp.sin(u.symbol)) or e.LHS.has(sp.sin(u.symbol)):   # we found  X = Asin(x)
                    if(self.BHdebug):
                        print 'I found an sin() equation to ID: ', e
                    u.eqntosolve = e
                    u.solvemethod = "arcsin"
                    found = True
                    break
                    
                if e.RHS.has(sp.cos(u.symbol)) or e.LHS.has(sp.cos(u.symbol)):   # we found X = Acos(x)
                    if(self.BHdebug):
                        print 'I found an cos() equation to ID: ', e
                    u.eqntosolve = e
                    u.solvemethod = "arccos"
                    found = True
                    break
                
        
        if found:
            #u.readytosolve = True
            u.solvable_sincos = True
            tick.blackboard.set("curr_unk", u)
            tick.blackboard.set("unknowns", unknowns)
            return b3.SUCCESS
        else:
            return b3.FAILURE

      
# solve eqns in T where one equation can be solved by
#        arcsin() or arccos()
class sincos_solve(b3.Action):    # Solve asincos equation pairs
    def tick(self, tick):
        R = tick.blackboard.get('Robot')
        solvedanything = False
        # get current unknown
        u = tick.blackboard.get("curr_unk")
        unknowns = tick.blackboard.get("unknowns")
        #for u in unknowns:
        print 'sincos: checking ', u.symbol
        #self.BHdebug = True
        if u.solvable_sincos:
            if(self.BHdebug):
                print "Trying to solve: ", u.symbol
                print "  Using the ", u.solvemethod, " on:"
                print u.eqntosolve

            # parse the equation RHS 
            terms = [sp.sin(u.symbol), sp.cos(u.symbol)]
            rhs = sp.collect(u.eqntosolve.RHS, terms)
            Aw  = sp.Wild("Aw")
            Bw  = sp.Wild("Bw")                        
            if u.solvemethod == "arcsin": 
                d   = rhs.match(Aw*sp.sin(u.symbol)+Bw)  
                assert(d is not None),  "sincos_solve (arcsin branch): Somethings Wrong!"
                A = d[Aw]
                if(d[Bw] is not None):
                    B = d[Bw]
                else:
                    B = 0
                    
                targument = (u.eqntosolve.LHS-B)/A 
                sol1 = sp.asin( targument  )
                sol2 = sp.pi - sp.asin( targument  )  
                u.argument = targument
                u.solutions.append(sol1)
                u.solutions.append(sol2)
                u.sincos_solutions.append(sol1)
                u.sincos_solutions.append(sol2)
                u.sincos_eqnlist.append(u.eqntosolve)
                u.nsolutions = 2 
                if(self.BHdebug):
                    print 'I think I solved ', u.symbol
                    sp.pprint(u.solutions)
                    print ''
                #u.set_solved(R,unknowns)
                solvedanything = True
                
            elif u.solvemethod == "arccos":
                d   = rhs.match(Aw*sp.cos(u.symbol)+Bw)  
                if(d is None):
                    print "sincos_solve (arccos branch):  Somethings Wrong!"
                    return b3.FAILURE                    
                else:        
                    A = d[Aw]
                    if(d[Bw] is not None):
                        B = d[Bw]
                    else:
                        B = 0
                            
                    targument = (u.eqntosolve.LHS-B)/A 
                    sol1 =   sp.acos( targument  )
                    sol2 = - sp.acos( targument  )  
                    u.argument = targument
                     
                    u.solutions.append(sol1)
                    u.solutions.append(sol2)
                    u.sincos_solutions.append(sol1)
                    u.sincos_solutions.append(sol2)
                    
                    u.nsolutions = 2
                    if(self.BHdebug):
                        print 'I think I solved ', u.symbol
                        sp.pprint(u.solutions)
                        print ''
                    #u.set_solved(R,unknowns)
                    solvedanything = True
        if(solvedanything):
            tick.blackboard.set('Robot', R)
            #tick.blackboard.set('unknowns', unknowns)
            tick.blackboard.set('curr_unk', u)
            return b3.SUCCESS
        else:
            return b3.FAILURE
   
#################
#   Test code
       
class TestSolver001(unittest.TestCase):
    # TODO: make tests work for the new version (use assigner)
    # Done!
    def setUp(self):
        self.DB = False  # debug flag
        print '\n\n===============  Test sincos Solver  ====================='
        return
    
    def runTest(self):
        self.test_sincos()
            
    def test_sincos(self):
        Rob = Robot()
        #  test the sincos ID and solver
        ik_tester = b3.BehaviorTree()
        ik_tester.log_flag = False  # log this one
        #ik_tester.log_file = open('BT_nodeSUCCESS_log.txt', 'w')
        st1 = test_sincos_id()
        st1.Name = 'sincos test setup'
        sid = sincos_id()
        sid.BHdebug = self.DB
        sid.Name = "sincos ID"
        sis = sincos_solve()
        sis.Name = 'sincos Solver'
        sis.BHdebug = self.DB
        
        asgn = assigner()
        subtree = b3.Sequence([asgn, sid, sis])
        repeats = b3.Repeater(subtree, max_loop = 5)
        
        test_sincos = b3.Sequence([st1,repeats])
        
        
        
        test_sincos.Name = 'overall sincos test'
        bb = b3.Blackboard()   
        bb.set('Robot',Rob)
        ik_tester.root = test_sincos

        #  Off we go: tick the BT        ik_tester.root = test_sincos
        ik_tester.tick("Test the sincos solver", bb)
            
        # check the results 
        unks = bb.get('unknowns') 
        fs =  'acos() solution fail'
        ntests = 0
        for u in unks:
            if (u.symbol == th_1):
                ntests += 1
                self.assertTrue(u.nsolutions == 2, fs + '[th_1 n]')
                self.assertTrue(u.solutions[0] ==  sp.acos( (l_1-l_5)/(l_2+l_3+l_4)), fs +'[th_1]')
                self.assertTrue(u.solutions[1] == -sp.acos( (l_1-l_5)/(l_2+l_3+l_4)), fs +'[th_1]')
            if (u.symbol == th_2):
                ntests += 1
                self.assertTrue(u.solutions[0] ==  sp.acos(l_2), 'acos() solution fail')
                self.assertTrue(u.solutions[1] == -sp.acos(l_2), 'acos() solution fail')
            if (u.symbol == th_3):
                ntests += 1
                self.assertTrue(u.solutions[0] ==  sp.asin(l_1), 'asin() solution fail')
                self.assertTrue(u.solutions[1] == -sp.asin(l_1)+sp.pi, 'asin() solution fail')
            if (u.symbol == th_4):
                ntests += 1
                self.assertTrue(u.solutions[0] ==  sp.acos((l_2+5)/l_1), 'acos((a+b)/c) solution fail')
                self.assertTrue(u.solutions[1] == -sp.acos((l_2+5)/l_1), 'acos((a+b)/c) solution fail')
                
                
        self.assertTrue(ntests == 4, ' sincos_solver.py:  Assertion count FAIL ')
        
        #test equation lists
        L1 = bb.get('eqns_1u')
        L2 = bb.get('eqns_2u')
        fs = 'sincos: Equation Counts FAIL'
        self.assertTrue(len(L1) == 5, fs)
        self.assertTrue(len(L2) == 1, fs)

def run_test():
    suite2 = unittest.TestLoader().loadTestsFromTestCase(TestSolver001)
    unittest.TextTestRunner(verbosity=2).run(suite2)	


if __name__ == "__main__":
    
    suite2 = unittest.TestLoader().loadTestsFromTestCase(TestSolver001)
    unittest.TextTestRunner(verbosity=2).run(suite2)
    #unittest.main()
   
