#!/usr/bin/python
#
#   TEMPLATE
#     BT Nodes for Testing, ID, Solving

#    'X' is added to .py extension to avoid it being 
#       processed by the module system
#

#  Replace TEMPLATE below with your solution method
#     and change extension .pyX --> .py
#
# Copyright 2017 University of Washington

# Developed by Dianmu Zhang <dianmuz at uw.edu> and 
# Blake Hannaford <blake at uw.edu>
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
 
class test_TEMPLATE_id(b3.Action):    # tester for your ID    
   def tick(self, tick):
       
       # set up bb data for testing  
      Td = ik_lhs()      # basic LHS template for TEST
      Ts = sp.zeros(4)
 
      
      Td[2,2] =  
      Ts[2,2] =  

      Td[3,1] =  
      Ts[3,1] =  

      Td[3,2] =  
      Ts[3,2] =  


      testm = matrix_equation(Td,Ts)        
      R = Robot()
      R.mequation_list = [testm]

      ud1  = unknown(d_1)
      uth2 = unknown(th_2)
      uth3 = unknown(th_3)
      uth4 = unknown(th_4)
      uth5 = unknown(th_5)
      variables = [uth1, uth2, uth3, uth4, uth5]
      
      [L1, L2, L3p] = R.scan_for_equations(variables)  # lists of 1unk and 2unk equations
      
      tick.blackboard.set('eqns_1u', L1)
      tick.blackboard.set('eqns_2u', L2)
      tick.blackboard.set('eqns_3pu', L3p)
      tick.blackboard.set('unknowns',variables)
      tick.blackboard.set('Robot',R)    
      return b3.SUCCESS
      

class TEMPLATE_id(b3.Action):    # action leaf for  
    
    def tick(self, tick):
        unknowns = tick.blackboard.get('unknowns')   # the current list of unknowns
        R = tick.blackboard.get('Robot')
        
        one_unk = tick.blackboard.get('eqns_1u')
        two_unk = tick.blackboard.get('eqns_2u')
        
        if(self.BHdebug):
            print "running: ", self.Name," , input RHS:"
            sp.pprint(Tm.Ts)
            print "unknowns:"
            for u in unknowns:
                print u.symbol, ' : ',u.eqnlist
            print ''
        # identify unknowns in T where one equation can be solved by
        #           <YOUR METHOD HERE>
        
        found = False      
        for u in unknowns: 
            if (not u.solved):  # only if not already solved!
                for e in one_unk:     # CHANGE to two_unk depending on solve method
                    if(self.BHdebug):
                        print 'TEMPLATE < name of node>'
                        print "Looking for unknown: ", u.symbol, " in equation: ", 
                        print e
                        print "  which has ", count_unknowns(unknowns, e.RHS), " unknown"
                    if (e.RHS.has(sp.sin(u.symbol)) or e.RHS.has(sp.cos(u.symbol))):  # example exception 
                        continue   # this shouldbe caught by another ID
                    if ( --- Your ID condition 1 is met ---):   # we found  METHOD ID
                        u.readytosolve = True
                        u.eqntosolve = e
                        u.solvemethod = "**METHOD 1 NAME**"
                        found = True
                    if ( --- Your ID condition 2 is met ---):   # we found X = Acos(x)
                        u.readytosolve = True
                        u.eqntosolve = e
                        u.solvemethod = "**METHOD 2 NAME**"
                        found = True
        tick.blackboard.set('unknowns',unknowns)   # the current list of unknowns
        if found:
            return b3.SUCCESS
        else:
            return b3.FAILURE
    


class TEMPLATE_solve(b3.Action):    # Solve asincos equation pairs
    def tick(self, tick):
    unknowns = tick.blackboard.get('unknowns')
    R = tick.blackboard.get('Robot')

    solved = False
    for u in unknowns:           
        if u.readytosolve:
            if(self.BHdebug):
                print "I'm trying to solve: ", u.symbol
                print "  Using: ", 
                print u.eqntosolve 
            if u.solvemethod == "**METHOD 1 NAME**":
                ****  Your Solution Method here *****
                u.solutions.append(  )       # two solutions 
                u.solutions.append(***  use this if there are two solns ***)
                u.nsolutions = 2   # or 1
                u.set_solved(R,unknowns)    # indicate that the variable/unknown is now solved
                solved = True
            elif u.solvemethod == "**METHOD 2 NAME**":
                ****  Your Solution Method here *****
                u.solutions.append(  )       # two solutions 
                u.solutions.append(***  use this if there are two solns ***)
                u.nsolutions = 2  # or 1
                u.set_solved(R,unknowns)    # indicate that the variable/unknown is now solved
                solved = True
    tick.blackboard.set('unknowns', unknowns)
    return b3.SUCCESS

       
       
#####################################################################################
# Test code below.  See sincos_solver.py for example
#       
class TestSolverTEMPLATE(unittest.TestCase):    # change TEMPLATE to unique name (2 places)
    def setUp(self):
        self.DB = False  # debug flag
        print '===============  Test TEMPLATE NAME  ====================='
        return
    
    def runTest(self):
        self.test_TEMPLATE()
            
    def test_TEMPLATE(self):
        
        #
        #
        #
        ##    test code here for your solver
        ##
        #
        #
        

#
#    Can run your test from command line by invoking this file
#
#      - or - call your TestSolverTEMPLATE()  from elsewhere
#

if __name__ == "__main__":
    
    print '\n\n===============  Test NAME_HERE nodes====================='
    testsuite = unittest.TestLoader().loadTestsFromTestCase(TestSolverTEMPLATE)  # replace TEMPLATE 
    unittest.TextTestRunner(verbosity=2).run(testsuite)
   

