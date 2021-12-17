#!/usr/bin/python
#
#   x2z2
#     BT Nodes for Testing, ID, Solving
#

#   Solve using the x^2 + y^2 method Craig uses
#      for puma joint 2 (eqn 4.65 p 118)
#
#    BH    2/2/17
#
#    BH : Dec-21:  SIMPLIFY!    After squaring and summing,
#       if a one-unk equation is identified, just add
#       it to the list of one-unk equations (in a persistent way).
#

# Copyright 2021 University of Washington

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
from ikbtfunctions.ik_robots import *


 
sp.var('th_23 Px Py Pz')

class test_x2z2(b3.Action):    # tester for your ID    
   def tick(self, tick):
       
    test_number = tick.blackboard.get('test_number')
    assert(test_number in [1, 2]), ' BAD TEST NUMBER'
    
    if(test_number == 1):
        #  Simplistic test in which we
        # just set up bb data for testing  (not really a test!)
        Td = ik_lhs()      # basic LHS template for TEST
        Ts = sp.zeros(4)
        
        
        Td[0,3] =  sp.cos(th_1)*Px + sp.sin(th_1)*Py
        Ts[0,3] =  a_3*sp.cos(th_23) - d_4*sp.sin(th_23) + a_2 * sp.cos(th_2)

        Td[2,3] =  -Pz
        Ts[2,3] =  a_3*sp.sin(th_23) + d_4*sp.cos(th_23) + a_2 * sp.sin(th_2)
    
        testm = matrix_equation(Td,Ts)        
        R = Robot()
        R.mequation_list = [testm]
        
        ud1  = unknown(th_1)
        uth2 = unknown(th_2)
        uth3 = unknown(th_3)
        uth23 = unknown(th_23)
        uth4 = unknown(th_4)
        uth5 = unknown(th_5)
        
        variables = [ud1, uth2, uth23, uth3, uth4, uth5]
        
        R.generate_solution_nodes(variables)        #for the solution graph
        ud1.solutions.append(a_3) # placeholder
        ud1.nsolutions = 1
        ud1.set_solved(R,variables)  # needed for this test
        #uth23.set_solved(R,variables)  # needed for this test
        
        R.sum_of_angles_transform(variables)        # should add th_23=th_2+th_3 to list
 
        [L1, L2, L3p] = R.scan_for_equations(variables)  # lists of 1unk and 2unk equations
        
        tick.blackboard.set('eqns_1u', L1)
        tick.blackboard.set('eqns_2u', L2)
        tick.blackboard.set('eqns_3pu', L3p)
              
        tick.blackboard.set('unknowns',variables)
        tick.blackboard.set('Robot',R)
        return b3.SUCCESS
    
    if(test_number == 2):
        # tests in the context of the full Puma solution
        #
        #
        #   The famous Puma 560  (solved in Craig)
        # 
        robot = 'Puma'
        [dh_Puma560, vv_Puma,  params_Puma, pvals_Puma, unk_Puma] = robot_params('Puma') 
             
        dh = dh_Puma560
        vv = vv_Puma
        variables = unk_Puma  # variables aka unknowns
        params = params_Puma
        pvals = pvals_Puma

        ################## (all robots) ######################
        ##  make sure each unknown knows its position (index)
        i = 0
        for u in variables :
            u.n = i
            i+=1    

        print('Testing x2z2solver with Puma Kinematics')
        testflag = False # deprecated but needed(!)
        # read kinematic model from pickle file / or compute it from scratch
        [M, R, variables ] = kinematics_pickle(robot, dh, params, pvals, vv, variables, testflag)
        #def kinematics_pickle(rname, dh, constants, pvals, vv, unks, test):

        ##   check the pickle in case DH params were changed in robot_params making the 
        #       pickle obsolete.
        check_the_pickle(dh_Puma560, M.DH) 
        
        R.name = 'Puma x2z2 Test Robot' 
        # set th_1 to solved 
        variables [0].solutions.append(a_3)
        variables [0].nsolutions = 1
        variables [0].set_solved(R,variables )  # needed for this test
        
        print('x2z2 setup for Test 2: generate SOA equations:\n')
        R.sum_of_angles_transform(variables )        # should add th_23=th_2+th_3 to list
        [L1, L2, L3p] = R.scan_for_equations(variables )  # lists of 1unk and 2unk equations
        
        tick.blackboard.set('eqns_1u', L1)
        tick.blackboard.set('eqns_2u', L2)
        tick.blackboard.set('eqns_3pu', L3p)
              
        tick.blackboard.set('unknowns', variables )
        tick.blackboard.set('Robot', R)
    
        return b3.SUCCESS


class x2z2_id_solve(b3.Action):    #  This time we combine into a single ID/Solve leaf
    # but *really* x2z2 is a transform which only generates a 1-unk equation
    # for *other* leaves to solve. 
    def __init__(self):
        super().__init__()             
        self.SolvedOneFlag = False      # turn off this expensive leaf after it has worked once

    def tick(self, tick):
        #if self.SolvedOneFlag:           #  we will only get lucky with this method once (HACK!)
            #print(' we already used x2z2 method')
            #return b3.FAILURE
        unknowns = tick.blackboard.get('unknowns')   # the current list of unknowns
        R = tick.blackboard.get('Robot')
        one_unk = tick.blackboard.get('eqns_1u')
        two_unk = tick.blackboard.get('eqns_2u') 
        more_unk = tick.blackboard.get('eqns_3pu') 

        u = tick.blackboard.get('curr_unk')
        
        if(self.BHdebug):
            print("x2z2, running: ", self.Name)
            print('len(3p): ', len(more_unk))
            print('len(2): ', len(two_unk))
            print('len(1): ', len(one_unk))
            print("currently looking at: ", u.symbol)
            #sp.pprint(Tm.Ts) 
        
        solved = False    
        
        f1 = False
        f2 = False

        eqn_ls = []
        

        # note: x2y2 is very costly, and less likely to be successful
        #  This is a hack exploiting it seems to be needed only for
        #   Theta_3 on the Puma robot
        if not u.symbol == th_3 or u.symbol == th_2 :
            return b3.FAILURE

            
        for e in (two_unk): # only two-unk list is enough
            tmp = e.RHS + e.LHS
            if (tmp.has(Py) or tmp.has(Px) or tmp.has(Pz)):
                eqn_ls.append(e)

        found = False

            
        if (self.BHdebug):
            print("found potential eqn list: ", len(eqn_ls))
            print(eqn_ls)
            
        
        # find any two equations and add their squares of each side
        #   ( we can't count on just [0,3],[2,3])
        #
        for i in range(len(eqn_ls)):  
            eqn1 = eqn_ls[i]
            r1 = eqn1.RHS
            l1 = eqn1.LHS
            for j in range(i+1, len(eqn_ls)):
                eqn2 = eqn_ls[j]
                r2 = eqn2.RHS
                l2 = eqn2.LHS
                
                if (self.BHdebug):
                    print("currently evaluating: ")
                    print(eqn1)
                    print(eqn2)
                    print("\n")
                    
                temp_l = l1*l1 + l2*l2
                temp_l = temp_l.simplify()
                
                if count_unknowns(unknowns, temp_l) == 0:
                    
                    temp_r = r1*r1 + r2*r2

                    temp_r = temp_r.simplify()
                    
                    temp_r = temp_r.subs(soa_expansions)
                    
                    temp_r = temp_r.simplify()
                

                    if count_unknowns(unknowns, temp_r) == 1:
                        print("X2Z2 found a useful eqn!")
                        found = True
                if found:
                    break
            if found:
                break

        if not found:
            print("x2y2 did not find suitable eqns")
            return b3.FAILURE
        
        # find the current unknown
        for u in unknowns:
            if temp_r.has(u.symbol):
                unknown = u
                unk = u.symbol
                if self.BHdebug: print('x2y2: The unknown variable is: ', unk)
            
        if not unknown.solved:
            ######################################### NEW ###############
            ##  NEW  instead of solving it here, we just put it in the list
            # of one-unknown equations so that some other leaf can solve it
            unknown.solvemethod += 'x2z2_transform and ' # only part of soln.
            R.SOA_eqns.auxeqns.append(kc.kequation(temp_l,temp_r))
            #############################################################
        tick.blackboard.set('Robot', R)
        tick.blackboard.set('unknowns',unknowns)   # the current list of unknowns
        self.SolvedOneFlag = True
         # we have a new 1-unk equation for other leafs to solve
         #   but nothing new is solved yet.
        return b3.SUCCESS  
       
#######################################################################
#  Test code:
class TestSolver010(unittest.TestCase):
    def setUp(self):
        self.DB = True  # debug flag
        print('\n\n===============  Test x2z2 Solver  =====================')
        return
    
    def runTest(self):
        self.test_x2z2()
            
    def test_x2z2(self): 
        ik_tester = b3.BehaviorTree()  # two leaves to 1) setup test 2) carry out test
        bb = b3.Blackboard()
        
        ###   A BT node to set everything up for tests
        x2z2_setup = test_x2z2()     # fcn can set up test 2 ways
        #x2z2_setup.BHdebug = True
        x2z2_setup.Name = "Setup"
        
        ###   BT node for the actual tests
        x2z2_work = x2z2_id_solve()   #  same node on two different setups
        x2z2_work.Name = "x2z2 ID/Solver"
        #x2z2_work.BHdebug = True 
        
        test = b3.Sequence([x2z2_setup, x2z2_work])
        ik_tester.root = test
        
        print('')
        print('              = = =    Test 1   = = = ') 
        print('')
        bb.set('test_number', 1)
        bb.set('curr_unk', unknown(th_3)) 
        #  this was set by test 1 and needs to be cleared
        x2z2_work.SolvedOneFlag = False   # reset the SolvedOneFlag
        ik_tester.tick("test x2z2 solver (1)", bb)
        
        unkns = bb.get("unknowns")
        
        fs = 'x2z2 id/solver Test 1 FAIL'
        ntests = 0
        for u in unkns: 
            if(u.symbol == th_3):    # the special unknown for this test
               # all we are testing is that x2y2 claims that it has added
               #  a new (but simpler) equation to the list unsolved equations
               assert 'x2z2_transform' in u.solvemethod, fs 
        
        print('      x2z2 PASSED test 1')
        print('')
        print('              = = =   Test X2Z2 solver (Puma)  = = = ')
        print('')
        
        bb = b3.Blackboard()         # clear the previous bb
        bb.set('test_number', 2)     # set up Puma kinematics this time
        bb.set('curr_unk', unknown(th_3))
        ik_tester.tick("test x2z2 solver (2)", bb)
            
        unkns = bb.get("unknowns")
        
        fs = 'x2z2 id/solver Test 2 (Puma)   FAIL'
        ntests = 0
        for u in unkns: 
            if(u.symbol == th_3):
                assert 'x2z2_transform' in u.solvemethod, fs 
                
                
        print('\n\n              X2Z2 solver PASSED all tests\n\n') 
         
#
#    Can run your test from command line by invoking this file
#
#      - or - call your TestSolverTEMPLATE()  from elsewhere
#

def run_test():
    print('\n\n===============  Test X2Y2 solver=====================')
    testsuite = unittest.TestLoader().loadTestsFromTestCase(TestSolver010)  # replace TEMPLATE 
    unittest.TextTestRunner(verbosity=2).run(testsuite)

if __name__ == "__main__":
    
    print('\n\n===============  Test X2Y2 solver=====================')
    testsuite = unittest.TestLoader().loadTestsFromTestCase(TestSolver010)  # replace TEMPLATE 
    unittest.TextTestRunner(verbosity=2).run(testsuite)
   


