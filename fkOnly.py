#!/usr/bin/python
#
#     Test for full IK solutions of know robot(s)

# Copyright 2017 University of Washington

# Developed by Dianmu Zhang and Blake Hannaford
# BioRobotics Lab, University of Washington

# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import sympy as sp
from sys import exit, argv
import pickle     # for storing pre-computed FK eqns
import unittest

# modified by BH, local version in current dir
import b3 as b3          # behavior trees

# local modules
import ikbtfunctions.helperfunctions as hf
import ikbtfunctions.output_python as op
import ikbtfunctions.output_cpp as oc
from ikbtfunctions.ik_robots import *   # a bunch of robot models: probs to solve

from ikbtbasics import *
from ikbtleaves.assigner_leaf import assigner
from ikbtleaves.rank_leaf import rank
from ikbtleaves.algebra_solver import *
from ikbtleaves.tan_solver import *
from ikbtleaves.sincos_solver import *
from ikbtleaves.sinANDcos_solver import *
from ikbtleaves.x2y2_solver import *
from ikbtleaves.sub_transform import *
#from ikbtleaves.sum_transform import *  # replaced by sum_id() + Algebra node.
from ikbtleaves.sum_id import *      # detect and sub sum-of-angles
from ikbtleaves.two_eqn_m7 import *

TEST_DATA_GENERATION = False

sp.init_printing()

if not TEST_DATA_GENERATION:
    print("")
    print("          Generating Forward Kinematics & Jacobian Only ")
    print("")
    print("")
else:
    print('-'*50)
    print("")
    print("          Generating IKBT TEST DATA only ")
    print("")
    print("          (for production: line 32: TEST_DATA_GENERATION = False)")
    print("")
    print('-'*50)

# generic variables for any maniplator
((th_1, th_2, th_3, th_4, th_5, th_6)) = sp.symbols(('th_1', 'th_2', 'th_3', 'th_4', 'th_5', 'th_6'))
((d_1, d_2, d_3, d_4, d_5, d_6)) = sp.symbols(('d_1', 'd_2', 'd_3', 'd_4', 'd_5', 'd_6'))
((h,l_0, l_1, l_2, l_3, l_4)) = sp.symbols(('h','l_0', 'l_1', 'l_2', 'l_3', 'l_4'))
((a_2, a_3)) = sp.symbols(('a_2', 'a_3'))
sp.var('Px Py Pz')


########################################################
#
#     Robot Parameters


########################################################    NEW Style robot param setups

#  Very basic Test

Rs = ['C-Arm', 'Gomez', 'Puma', 'Chair_Helper', 'Khat6DOF', 'Wrist', 'MiniDD', 'RavenII']

if len(argv) == 1:  # no argument - use default
    #robot = 'Gomez'
    #robot = 'Puma'
    #robot = 'Chair_Helper'
    #robot = 'Khat6DOF'
    robot = 'Wrist'

elif len(argv) == 2:
    robot = str(argv[1])

print('')
print('')
print('             Working on '+robot)
print('')
print('')

#   Get the robot model
[dh, vv, params, pvals, unknowns] = robot_params(robot)  # see ik_robots.py

#
#     Set up robot equations for further solution by BT
#
#   Check for a pickle file of pre-computed Mech object. If the pickle
#       file is not there, compute the kinematic equations

testing = False
[M, R, unknowns] = kinematics_pickle(robot, dh, params, pvals, vv, unknowns, testing)
print('GOT HERE: robot name: ', R.name)

R.name = robot
R.params = params

##   check the pickle in case DH params were changed
dhp = M.DH
check_the_pickle(dhp, dh)   # check that two mechanisms have identical DH params


##################################   The FK and Jacobian are already done here!



####################################################################################
##
#                              No BT needed
#
#
if TEST_DATA_GENERATION:
    # Now we're going to save some results for use in tests.
    print(' Storing results for test use')
    test_pickle_dir = 'Test_pickles/'
    name = test_pickle_dir + R.name + 'test_pickle.p'
    with open(name,'wb') as pf:
        pickle.dump( [R, unks], pf)
    quit()

output_latex_solution(R,unks, final_groups)


#################################################
# print out all eqnuations that used to solve variables
# uncomment for debugging

print("equations evaluated")
for one_unk in unks:
    print(one_unk.symbol)
    print(one_unk.eqntosolve)
    print(one_unk.secondeqn)
    print('\n')


#
#
#
################################################################################

# define symbols that appear in solutions
sp.var('r_11 r_12 r_13 r_21 r_22 r_23 r_31 r_32 r_33 Px Py Pz')


assertion_count = 0
ntests = 1

if(robot == 'Chair_Helper'):
    fs = 'Chair_Helper   FAIL'
    for u in unks:
        print('\n Asserting: ', u.symbol, ' = '),
        if(u.symbol == d_1):
            ntests += 1
            assert(u.nsolutions == 1), fs+' n(d_1)'
            assertion_count += 1
            print(str(u.solutions[0]))
            assert(u.solutions[0] == Pz - l_4*r_33), fs + '  [d_1]'
            assertion_count += 1
        if(u.symbol == th_2):
            ntests += 1
            assert(u.nsolutions == 2), fs+' n(th_2)'
            assertion_count += 1
            print(str(u.solutions[0]) + ', ' + str(u.solutions[1]))
            assert(u.solutions[0] ==  sp.asin((Px-l_1-l_4*r_13)/l_2) ), fs + ' [th_2a]'
            assertion_count += 1
            assert(u.solutions[1] == -sp.asin((Px-l_1-l_4*r_13)/l_2)+sp.pi ), fs + ' [th_2b]'
            assertion_count += 1

if(assertion_count == 0):
    print('\n         Warning: \n   No Assertions yet for ' + robot)
else:
    string = 'test robot '+robot
    print('\n\n\n                            ',string,'  PASSES ', assertion_count, 'assertions!')
    print('                                  passed ',ntests,' tests \n\n\n')

print('End of solution job')


