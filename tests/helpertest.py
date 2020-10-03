#!/usr/bin/python
#
#     Test the helper functions 
#
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
import ikbtbasics.kin_cl as kc 
from ikbtfunctions.helperfunctions import *

#####################################################################################
# Test code for the helperfunctions
#       
class TestSolver009(unittest.TestCase):    # change TEMPLATE to unique name (2 places)
    def setUp(self):
        sp.var('d_1 th_2 th_3 th_4 th_5')
        self.ud1  = kc.unknown(d_1)
        self.uth2 = kc.unknown(th_2)
        self.uth3 = kc.unknown(th_3)
        self.uth4 = kc.unknown(th_4)
        self.uth5 = kc.unknown(th_5)
        self.vars = [self.ud1, self.uth2, self.uth3, self.uth4, self.uth5]
        self.expression01 = self.uth2.symbol + sp.sin(self.uth4.symbol)
        
        self.DB = False  # debug flag
        #print '===============  Test helperfunctions  ====================='
        return
    
    def runTest(self):
        
        self.test_lhs()
        self.test_findobj()
        self.test_get_vars()
        self.test_get_unknowns()
        return
            
    def test_lhs(self):
        x = ik_lhs()
        fs = 'ik_lhs()  FAIL'
        self.assertTrue(x[0,0] == sp.var('r_11'), fs)
        self.assertTrue(x[0,1] == sp.var('r_12'), fs)
        self.assertTrue(x[1,0] == sp.var('r_21'), fs)
        self.assertTrue(x[0,3] == sp.var('Px'), fs)
        self.assertTrue(x[2,3] == sp.var('Pz'), fs)
        t = 0
        for j in [0,1,2,3]:
            t = t + x[3,j]
        self.assertTrue(t == 1, fs)
        return
        
    def test_findobj(self):
        fs = 'find_obj()  FAIL'
        self.assertTrue(find_obj(self.uth3.symbol, self.vars) == self.vars[2], fs)
        self.assertTrue(find_obj(kc.unknown('random_name'), self.vars) == None, fs)
        return
    
    def test_get_vars(self):
        fs = 'get_variables()  FAIL'
        vlist = get_variables(self.vars, self.expression01)
        self.assertTrue(vlist[0] == self.uth2, fs)
        self.assertTrue(vlist[1] == self.uth4, fs)
        return
        
    
    def test_get_unknowns(self):
        fs = 'get_unknowns()   FAIL'
        self.uth2.solved = True
        unks = get_unknowns(self.vars, self.expression01)
        self.assertTrue(len(unks) == 1, fs)
        self.assertTrue(unks[0] == self.uth4)
        return
        

#
#    Can run your test from command line by invoking this file
#
#      - or - call your TestSolverTEMPLATE()  from elsewhere
#

if __name__ == "__main__":
    
    print('\n\n===============  Test helperfunctions =====================')
    testsuite = unittest.TestLoader().loadTestsFromTestCase(TestSolver009)  # replace TEMPLATE 
    unittest.TextTestRunner(verbosity=2).run(testsuite)
   
   
