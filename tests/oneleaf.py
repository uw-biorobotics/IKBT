#!/usr/bin/python
#
# updated testing system for classes and BT leaves
#
# BH May 2017


# Copyright 2017 University of Washington

# Developed by Dianmu Zhang and Blake Hannaford
# BioRobotics Lab, University of Washington

# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from io import StringIO   # new way to import for Python recent vintage

import unittest
import sys
from tests import HTMLTestRunner
import os as os

import sympy as sp
import numpy as np
from sys import exit

sys.path.append('../')
sys.path.append('../ikbtfunctions')   # allow this test to go 'up' to project main dir.

from ikbtfunctions.helperfunctions import *
from tests.helpertest import *    # had to separate tests form helperfunctions b/c of circular imports

from ikbtbasics.kin_cl import *
from ikbtbasics.ik_classes import *     # special classes for Inverse kinematics in sympy

from ikbtleaves.algebra_solver import *
from ikbtleaves.sinANDcos_solver import *
from ikbtleaves.sincos_solver import *
from ikbtleaves.tan_solver import *
from ikbtleaves.sub_transform import *
from ikbtleaves.updateL import *
from ikbtleaves.x2y2_transform import *

from tests.leavestest import TestIkClass


import b3 as b3          # behavior trees

os.chdir('../')            # change to project dir.


((th_1, th_2, th_3, th_4, th_5, th_6)) = sp.symbols(('th_1', 'th_2', 'th_3', 'th_4', 'th_5', 'th_6'))
((d_1, d_2, d_3, d_4, d_5, d_6)) = sp.symbols(('d_1', 'd_2', 'd_3', 'd_4', 'd_5', 'd_6'))

possible_unkns = set([th_1, th_2, th_3, th_4, th_5, th_6, d_1, d_2, d_3, d_4, d_5, d_6]) #didn't count th_XY

#def testsuiteIK():
    #suite = unittest.TestSuite()
    #suite.addTest('test_atansubs')
    #suite.addTest('test_lhsgen')
    #return suite
 

#class TestSolvers(unittest.TestCase):
class TestSolvers(unittest.TestCase):
    def setUp(self):
        self.DB = False  # debug flag
        return


def run_test_IK_class():
    suite1 = unittest.TestLoader().loadTestsFromTestCase(TestIkClass)
    print('\n\n>>>>>>>>>>>>>>>>>>>>  Test ik_classes >>>>>>>>>>>>>>>>>>>>\n')
    unittest.TextTestRunner(verbosity=2).run(suite1)

if __name__ == '__main__':
    HTML = False
    if (len(sys.argv) > 1):
        if(str(sys.argv[1]) == 'html'):
            HTML = True
            sys.argv = sys.argv[:-1] # delete html arg (but rest go to HTMLTestRunner)
        else:
            print('invalid arguments: ', sys.argv, file=sys.stderr)
            quit()

    #####################################################################
    # set up the test suites
    #
    #   Tests of basic classes
    suite1 = unittest.TestLoader().loadTestsFromTestCase(TestIkClass)
    #suite1.addTest(TestSolver008())   # kin_cl.py   # basic kinematics classes
    #suite1.addTest(TestSolver009())   # helperfunctions.py

    # test the leaves (id/solvers)
    suite2 = unittest.TestLoader().loadTestsFromTestCase(TestSolver001)  # sincos_solver.py
    suite2.addTest(TestSolver002())  # algebra_solver.py
    suite2.addTest(TestSolver003())  # sinANDcos_solver.py
    suite2.addTest(TestSolver004())  # tan_solver.py
    
    # test the transforms and misc. tests
    suite3 = unittest.TestLoader().loadTestsFromTestCase(TestSolver006)  # sub_transform.py
    suite2.addTest(TestSolver010())   # x2y2_transform.py
    suite3.addTest(TestSolver007())   # updateL.py  # updating matrix equation lists
    
    if(not HTML):
        print('\n\n>>>>>>>>>>>>>>>>>>>>  Test ik_classes >>>>>>>>>>>>>>>>>>>>\n')
        unittest.TextTestRunner(verbosity=2).run(suite1)
        #print('\n\n>>>>>>>>>>>>>>>>>>>>  Test ID/Solver nodes>>>>>>>>>>>>>>>>>>>>\n')
        #unittest.TextTestRunner(verbosity=2).run(suite2)
        #print('\n\n>>>>>>>>>>>>>>>>>>>>  Test transforms and misc nodes >>>>>>>>>>>>>>>>>>>>\n')
        #print(' (please be patient - slower tests)')
        #unittest.TextTestRunner(verbosity=2).run(suite3)
    else:
        #HTMLTestRunner.main()
       # output to a file
        projdir = 'IKBT/'
        testfname =  projdir + 'IKBT_testreport.html'
        try:
            fp = open(testfname, 'w')
        except:
            print('leavestest.py: Trouble opening file ',testfname)
            quit()
            
        buffer =  StringIO()

        #print '\n     Opening for html output: '+testfname+'\n'
        print('\n     Opening StringIO() \n')
        html_runner = HTMLTestRunner.HTMLTestRunner(buffer,
                    title='IK-BT unit tests',
                    description='IK-BT Unit test results.'
                    )

        print('\n\n>>>>>>>>>>>>>>>>>>>>  Test ik_classes >>>>>>>>>>>>>>>>>>>>\n')
        html_runner.title = 'IK-BT Kinematics Classes'
        html_runner.run(suite1)
        print('\n\n>>>>>>>>>>>>>>>>>>>>  Test ID/Solver nodes>>>>>>>>>>>>>>>>>>>>\n')
        html_runner.title = 'IK-BT Solvers (leaves)'
        html_runner.run(suite2)
        print('\n\n>>>>>>>>>>>>>>>>>>>>  Test transforms and misc nodes >>>>>>>>>>>>>>>>>>>>\n')
        print(' (please be patient - slower tests)')
        html_runner.title = 'IK-BT Transforms'
        html_runner.run(suite3)
        print(' ****************************', file=fp)
        print(buffer.getvalue(), file=fp)
        fp.close()



