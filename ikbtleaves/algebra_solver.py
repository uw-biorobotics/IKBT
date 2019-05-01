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

from ikbtfunctions.helperfunctions import *
from ikbtbasics.kin_cl import *
from ikbtbasics.ik_classes import *     # special classes for Inverse kinematics in sympy

import b3 as b3          # behavior trees
from assigner_leaf  import *
from comp_detect import *


 
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

        [L1, L2] = R.scan_Mequation(testm, variables)  # lists of 1unk and 2unk equations
        
        L1.append(kequation(th_123, th_1+th_2+B))
        
        # fix!!
        R.generate_solution_nodes(variables) # generate solution nodes
        
        tick.blackboard.set('eqns_1u', L1)
        tick.blackboard.set('eqns_2u', L2)
        tick.blackboard.set('unknowns',variables)
        tick.blackboard.set('Robot',R)    
        return b3.SUCCESS
    

class algebra_id(b3.Action):    # action leaf for  
    
    def tick(self, tick):
        Tm = tick.blackboard.get('Tm')   # the current matrix equation 
        unknowns = tick.blackboard.get('unknowns')   # the current list of unknowns
        
        one_unk = tick.blackboard.get('eqns_1u')
        two_unk = tick.blackboard.get('eqns_2u')
        R = tick.blackboard.get('Robot')
            
        if(self.BHdebug):
            print "\nrunning: ", self.Name
            print ' eqns w/ one unk: ', len(one_unk)
            print ' eqns w/ two unk: ', len(two_unk)
            
    
        u = tick.blackboard.get('curr_unk')
      # identify unknowns in T where one equation can be solved by
      #           basic algebra
        found = False      
        if (not u.solved):  # only if not already solved!
                for e in one_unk:
                    if(self.BHdebug):
                        print 'algebra ID: Looking for unknown: ', u.symbol, ' in equation: ', 
                        print e ,
                        print "  - ", count_unknowns(unknowns, e.RHS), " unknown in RHS"
                    if (e.RHS.has(sp.sin(u.symbol)) or e.RHS.has(sp.cos(u.symbol)) or\
                        e.LHS.has(sp.sin(u.symbol)) or e.LHS.has(sp.cos(u.symbol))):
                        continue   # this shouldbe caught by another ID
                    
                    # since we're not solving the equation here, simply count the unknowns will suffice for the identification
                    if(e.RHS.has(u.symbol) or e.LHS.has(u.symbol)):
                        u.readytosolve = True
                        tmp = e.RHS - e.LHS
                        tmp = tmp.expand()

                        tmp = tmp.collect(u.symbol)
                        u.eqntosolve = kequation(0, tmp)
                        u.solvemethod = "algebra"
                        found = True
                        break
                        
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
                        print "I'm trying to solve: ", u.symbol
                        print "  Using: ", 
                        print u.eqntosolve  
           if u.solvemethod == "algebra":

               Aw = sp.Wild("Aw")
               Bw = sp.Wild("Bw")
               d = u.eqntosolve.RHS.match(Aw*u.symbol+Bw)
               A = d[Aw]
               B = d[Bw]
               u.solutions.append( (u.eqntosolve.LHS-B)/A  )       # one soluntion 
               u.nsolutions = 1   # or 1
               u.set_solved(R,unknowns)  # flag that this is solved 
       tick.blackboard.set('curr_unk', u)
       tick.blackboard.set('unknowns', unknowns)
       return b3.SUCCESS
   
       
#######################################################################
#  Test code:
class TestSolver002(unittest.TestCase):
    def setUp(self):
        self.DB = True  # debug flag
        print '\n\n===============  Test algebra Solver  ====================='
        return
    
    def runTest(self):
        self.test_algebra()
            
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
        
        compdet = comp_det()
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
        print '\n\n              Results: \n\n'
        ntests = 0
        for u in unk:
            if(u.symbol == d_1):
                ntests += 1
                self.assertTrue(u.solved, fs)
                self.assertTrue(u.nsolutions == 1, fs)
                print 'Soln: ', u.solutions[0]
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
        print 'Algebra solver PASSED ', ntests, ' assertions.'
##  write tester code which runs if this file is run directly instead
##    of "imported".


#
#    Can run your test from command line by invoking this file
#
#      - or - call your TestSolverTEMPLATE()  from elsewhere
#

def run_test():
    print '\n\n===============  Test NAME_HERE nodes====================='
    testsuite = unittest.TestLoader().loadTestsFromTestCase(TestSolver002)  # replace TEMPLATE 
    unittest.TextTestRunner(verbosity=2).run(testsuite)

if __name__ == "__main__":
    
    print '\n\n===============  Test NAME_HERE nodes====================='
    testsuite = unittest.TestLoader().loadTestsFromTestCase(TestSolver002)  # replace TEMPLATE 
    unittest.TextTestRunner(verbosity=2).run(testsuite)
   

