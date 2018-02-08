#!/usr/bin/python
#
#   TEMPLATE
#     BT Nodes for Testing, ID, Solving
#

#  Replace TEMPLATE below with your solution method
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
from ikbtfunctions.ik_robots import *

import b3 as b3          # behavior trees
import pickle     # for storing pre-computed FK eqns

class updateL(b3.Action):    # Set up the equation lists
    def tick(self, tick):
        R = tick.blackboard.get('Robot')   # the current matrix equation
        variables = tick.blackboard.get('unknowns')   # the current list of unknowns

        R.sum_of_angles_transform(variables)
        [L1, L2, L3p] = R.scan_for_equations(variables)   # get the equation lists

        tick.blackboard.set('eqns_1u', L1)  # eqns w/ 1 unknown
        tick.blackboard.set('eqns_2u', L2)  # eqns w/ 2 unknowns
        tick.blackboard.set('eqns_3pu', L3p)  # eqns w/ 3 unknowns
        tick.blackboard.set('Robot', R)
        return b3.SUCCESS


#####################################################################################
# Test code below.  See sincos_solver.py for example
#
class TestSolver007(unittest.TestCase):    # change TEMPLATE to unique name (2 places)
    # def setUp(self):
        # self.DB = False  # debug flag
        # print '===============  Test updateL.py  ====================='
        # return

    def runTest(self):
        self.test_updateL()

    def test_updateL(self):
        #
        #     Set up robot equations for further solution by BT
        #
        #   Check for a pickle file of pre-computed Mech object. If the pickle
        #       file is not there, compute the kinematic equations
        ####  Using PUMA 560 also tests scan_for_equations() and sum_of_angles_transform()  in ik_classes.py
        #
        #   The famous Puma 560  (solved in Craig)
        #
        import os as os
        print '\n------------'
        print 'Current dir: ', os.getcwd()
        pickname = 'fk_eqns/Puma_pickle.p'
        if(os.path.isfile(pickname)):
            print 'a pickle file will be used to speed up'
        else: 
            print 'There was no pickle file'
        print '------------'

        #return [dh, vv, params, pvals, variables]
        robot = 'Puma'
        [dh, vv, params, pvals, unknowns] = robot_params(robot)  # see ik_robots.py
        #def kinematics_pickle(rname, dh, constants, pvals, vv, unks, test):
        Test = True
        [M, R, unk_Puma] = kinematics_pickle(robot, dh, params, pvals, vv, unknowns, Test)
        print 'GOT HERE: updateL robot name: ', R.name

        R.name = 'test: '+ robot # ??? TODO: get rid of this (but fix report)

        ##   check the pickle in case DH params were changed
        check_the_pickle(M.DH, dh)   # check that two mechanisms have identical DH params

        testerbt = b3.BehaviorTree()
        setup = updateL()
        setup.BHdebug = True
        bb = b3.Blackboard()
        testerbt.root= b3.Sequence([setup])  # this just runs updateL - not real solver
        bb.set('Robot',R)
        bb.set('unknowns', unk_Puma)

        testerbt.tick('test', bb)
        L1 = bb.get('eqns_1u')
        L2 = bb.get('eqns_2u')
        # print them all out(!)
        sp.var('Px Py Pz')
        fs = 'updateL: equation list building   FAIL'
        #  these self.assertTrues are not conditional - no self.assertTrueion counting needed
        self.assertTrue(L1[0].RHS == d_3, fs)
        self.assertTrue(L1[0].LHS == -Px*sp.sin(th_1)+Py*sp.cos(th_1), fs)
        self.assertTrue(L2[0].RHS == -a_2*sp.sin(th_2)-a_3*sp.sin(th_23)-d_4*(sp.cos(th_23)), fs)
        self.assertTrue(L2[0].LHS == Pz, fs)

        #########################################
        # test R.set_solved

        u = unk_Puma[2]  #  here's what should happen when we set up two solutions
        sp.var('Y X B')
        u.solutions.append(sp.atan2(Y,X)) #  # make up some equations
        u.solutions.append(sp.atan2(-Y, X)) #
        #   assumptions are used when a common denominator is factored out
        u.assumption.append(sp.Q.positive(B))  # right way to say "non-zero"?
        u.assumption.append(sp.Q.negative(B))
        u.nsolutions = 2
        u.set_solved(R, unk_Puma)  #  test the set solved function
        fs = 'updateL: testing R.set_solved   FAIL '
        self.assertTrue(not u.readytosolve, fs)
        self.assertTrue(    u.solved      , fs)
        self.assertTrue(R.solveN == 1, fs)  # when initialized solveN=0 set_solved should increment it

        # solutiontreenodes no longer used
        #self.assertTrue(len(R.solutiontreenodes) == 3, fs)  # we should now have three nodes (root + two solns)

        #print '\n\n\n               updateL    PASSES ALL TESTS  \n\n'


#
#    Can run your test from command line by invoking this file
#
#      - or - call your TestSolverTEMPLATE()  from elsewhere
#

def run_test():
    print '\n\n===============  Test updateL.py ====================='
    testsuite = unittest.TestLoader().loadTestsFromTestCase(TestSolver007)  # replace TEMPLATE
    unittest.TextTestRunner(verbosity=2).run(testsuite)

if __name__ == "__main__":

    print '\n\n===============  Test updateL.py ====================='
    testsuite = unittest.TestLoader().loadTestsFromTestCase(TestSolver007)  # replace TEMPLATE
    unittest.TextTestRunner(verbosity=2).run(testsuite)
    #unittest.main()




