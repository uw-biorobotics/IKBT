#!/usr/bin/python
#
#   Implement a transform in which we identify 
#       RHS elements which can be substituted into 
#       another RHS to elminiate unknowns.
#
#   This is a new approach rather than making it a SOLVER
#    it is just a transform which allows other solvers to work. 
#
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

 
class test_sub_transform(b3.Action):    # tester for your ID    
    def tick(self, tick):
    
        #test_number = tick.blackboard.get('test_number') # if present
        R = tick.blackboard.get('Robot')
        
        sp.var('a b c d')
        # set up bb data for testing  
        Td = ik_lhs() 
        Ts = sp.zeros(4)

        Ts[1,1] =  sp.sin(th_1)*sp.cos(th_2)+sp.sin(th_5)
        Ts[1,2] =  sp.sin(th_1)*sp.cos(th_2)
        Ts[2,1] =  a+b+c+d
        Ts[2,2] =  a+b+c
        Ts[2,3] =  a*b+c
        Ts[2,0] =  a

        testm = matrix_equation(Td,Ts)

        ua = unknown(a)
        ub = unknown(b)
        uc = unknown(c)
        ud = unknown(d)
        uth2 = unknown(th_2)
        uth3 = unknown(th_3)
        uth4 = unknown(th_4)
        uth5 = unknown(th_5)
        variables = [ua,ub,uc,ud,uth2, uth3, uth4, uth5]

        R.mequation_list = [testm]
        [L1, L2] = R.scan_Mequation(testm, variables)  # lists of 1unk and 2unk equations
        
        print ' INITIAL  Ts:'
        Tm = R.mequation_list[0]   # for a single test as above
        sp.pprint(Tm.Ts)
        print ''
        
        
        
        tick.blackboard.set('eqns_1u', L1)
        tick.blackboard.set('eqns_2u', L2)
        tick.blackboard.set('unknowns',variables)
        tick.blackboard.set('Robot',R)    
        return b3.SUCCESS
    
    
    #
    #           ***********************    ERK!    Need to keep this at the mequation_list level (i.e. 4x4  eqns)
    #                                                so that equations can be modified for future use in other lists. 
    #

class sub_transform(b3.Action):    # action leaf for  
    
    def tick(self, tick):
        unknowns = tick.blackboard.get('unknowns')   # the current list of unknowns
        R = tick.blackboard.get('Robot')   # the current robot instance
        if(self.BHdebug):
            print "running: ", self.Name
            print 'number of matrix equations: ', len(R.mequation_list)
            print 'first matrix equation: ', R.mequation_list[0]
            print 'number of input equations: ', len(R.mequation_list[0].get_kequation_list())
            print "unknowns:"
            for u in unknowns:
                print u.symbol, ', solved: ',u.solved
            print ''
            
        #   We're going to look at the first N equations in the mequation_list
        N = 1   #  were only looking at first 2 matrix equations (at least for now)          
        assert (N <= len(R.mequation_list)), 'sub_transform test wants too many meqns '
        
        # identify elements of eqns where another elemnet can be substituted in
        #    to eliminate unknowns
        #    
        found = False 
        
        sp.var('a z')
        z = a-a    #  (define symbolic zero!)
        
        cols = [0,1,2,3]
        rows = [0,1,2]     # we don't care about row 4 ([0,0,0,1])!
        
        for m in range(0,N):
            for i in rows:
                for j in cols:
                    e2 = R.mequation_list[m].Ts[i,j]
                    for k in rows:
                        for l in cols:
                            e1 = R.mequation_list[m].Ts[k,l]
                            # substitute with e1 or -e1      ####################################3    *******    adapt ".has" to both LHS and RHS??
                            if((e1 != e2) and e2 != z and e2.has(e1)):  # we found a substitution
                                if(self.BHdebug):
                                    print ''
                                    print self.Name, ' found a sub transform (+)'
                                    print e1, ' / ',  e2
                                    print 'new: ', e2, ' = ',  e2.subs(e1, e2) 
                                nold = count_unknowns(unknowns, e2)
                                new = e2.subs(e1, R.mequation_list[m].Td[k,l])   # substitute
                                nnew = count_unknowns(unknowns, new)
                                if(self.BHdebug):
                                    print 'Unknowns: old/new:', nold, '/', nnew
                                    print 'Prop Sub: ', e2, '/', new
                                if(nnew < nold):
                                    R.mequation_list[m].Ts[i,j] = new
                                    found = True
                                        
                            elif((e1 != e2) and e2 != z and e2.has(-e1)):  # we found a substitution -e1
                                if(self.BHdebug):
                                    print self.Name, ' found a (-) sub transform'
                                    print e1, '/',  e2
                                nold = count_unknowns(unknowns, e2)
                                new = e2.subs(-e1, -R.mequation_list[m].Td[k,l])   # substitute with -e1
                                nnew = count_unknowns(unknowns, new)
                                if(self.BHdebug):
                                    print 'Unknowns: old/new:', nold, '/', nnew
                                    print 'Prop Sub: ', e2, '/', new
                                if(nnew < nold):
                                    R.mequation_list[m].Ts[i,j] = new
                                    found = True
                                
                                        
        if found:
            #  put the tmp_eqns list back into R !!!!  ******************************
            [L1, L2, L3p] = R.scan_for_equations(unknowns)
            tick.blackboard.set('eqns_1u', L1)
            tick.blackboard.set('eqns_2u', L2)
            tick.blackboard.set('eqns_3pu', L3p)
            tick.blackboard.set('Robot', R)
            
        return b3.SUCCESS
        #else:
            #return b3.FAILURE
            
       
#class test_sincos_solve(b3.Action):    # tester for sincos solver
    #def tick(self, tick):
      ## set up bb data for testing sincos_solve
      


       
#####################################################################################
# Test code below.  See sincos_solver.py for example
#       
class TestSolver006(unittest.TestCase):    # change TEMPLATE to unique name (2 places)
    def setUp(self):
        self.DB = False  # debug flag
        print '===============  Test sub_transform.py  ====================='
        return
    
    def runTest(self):
        self.test_subber()
            
    def test_subber(self):        
        sub_tester = b3.BehaviorTree()
        bb = b3.Blackboard()  
        bb.set('Robot', Robot())
        setup = test_sub_transform()
        trans = sub_transform()
        trans.Name = 'Substitution Transf'
        trans.BHdebug = True
        test = b3.Sequence([setup, trans])
        
        sub_tester.root = test
        sub_tester.tick("Test the substitution test tree", bb)
        
        # now examine results        
        R = bb.get('Robot')
        Tm = R.mequation_list[0]   # for a single test as above
        
        sp.var('a b c d r_23 r_31 r_33 r_43')
        fs = " sub_transform FAIL"
        self.assertTrue(Tm.Ts[1,1]== r_23+sp.sin(th_5), fs)
        self.assertTrue(Tm.Ts[1,2]== sp.sin(th_1)*sp.cos(th_2), fs)
        self.assertTrue(Tm.Ts[2,1]== d+r_33, fs)
        self.assertTrue(Tm.Ts[2,3]== b*r_31+c, fs)
        self.assertTrue(Tm.Ts[2,0]==a, fs)


#
#    Can run your test from command line by invoking this file
#
#      - or - call your TestSolverTEMPLATE()  from elsewhere
#
def run_test():
    print '\n\n===============  Test sub_transform nodes====================='
    testsuite = unittest.TestLoader().loadTestsFromTestCase(TestSolver006)  # replace TEMPLATE 
    unittest.TextTestRunner(verbosity=2).run(testsuite)
    
if __name__ == "__main__":
    
    print '\n\n===============  Test sub_transform nodes====================='
    testsuite = unittest.TestLoader().loadTestsFromTestCase(TestSolver006)  # replace TEMPLATE 
    unittest.TextTestRunner(verbosity=2).run(testsuite)
           
