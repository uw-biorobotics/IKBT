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
from ikbtfunctions.ik_robots import * 
from ikbtbasics import *
from ikbtleaves.assigner_leaf import assigner
from ikbtleaves.rank_leaf import rank
from ikbtleaves.algebra_solver import *
from ikbtleaves.tan_solver import *
from ikbtleaves.sincos_solver import *
from ikbtleaves.sinANDcos_solver import *
from ikbtleaves.x2y2_solver import *
from ikbtleaves.sub_transform import *
from ikbtleaves.sum_transform import *
from ikbtleaves.two_eqn_m7 import *

TEST_DATA_GENERATION = False

sp.init_printing()

if not TEST_DATA_GENERATION:
    print ""
    print "          Running IK solution "
    print ""
    print ""
else:
    print '-'*50
    print ""
    print "          Generating IKBT TEST DATA only "
    print ""
    print "          (for production: line 32: TEST_DATA_GENERATION = False)"
    print ""
    print '-'*50

# generic variables for any maniplator
((th_1, th_2, th_3, th_4, th_5, th_6)) = sp.symbols(('th_1', 'th_2', 'th_3', 'th_4', 'th_5', 'th_6'))
((d_1, d_2, d_3, d_4, d_5, d_6)) = sp.symbols(('d_1', 'd_2', 'd_3', 'd_4', 'd_5', 'd_6'))
((h,l_0, l_1, l_2, l_3, l_4)) = sp.symbols(('h','l_0', 'l_1', 'l_2', 'l_3', 'l_4'))
((a_2, a_3)) = sp.symbols(('a_2', 'a_3'))
sp.var('Px Py Pz')

# move definition of Sum-of-Angles variables to ik-classes.py so they are available everywhere

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
        
print ''
print ''
print '             Working on '+robot
print ''
print ''

#   Get the robot model 
[dh, vv, params, pvals, unknowns] = robot_params(robot)  # see ik_robots.py 

#
#     Set up robot equations for further solution by BT
#
#   Check for a pickle file of pre-computed Mech object. If the pickle
#       file is not there, compute the kinematic equations

testing = False
[M, R, unknowns] = kinematics_pickle(robot, dh, params, pvals, vv, unknowns, testing)
print 'GOT HERE: robot name: ', R.name

R.name = robot
R.params = params

##   check the pickle in case DH params were changed
dhp = M.DH
check_the_pickle(dhp, dh)   # check that two mechanisms have identical DH params
 
####################################################################################
##
#                                   Set up the BT Leaves
# 
#
ikbt = b3.BehaviorTree()

LeafDebug = False
SolverDebug = False

###add in new nodes:assigner and rank node#############
asgn = assigner()
asgn.Name = "Assigner"
rk = rank()
rk.Name = "Rank Node"
#######################################################
tanID = tan_id()
tanID.Name = 'Tangent ID'
tanID.BHdebug =  LeafDebug

tanSolver = tan_solve()
tanSolver.BHdebug = SolverDebug
tanSolver.Name = "Tangent Solver"

tanSol = b3.Sequence([tanID, tanSolver])
tanSol.Name = "TanID+Solv"
tanSol.BHdebug =  LeafDebug


algID = algebra_id()
algID.Name = "Algebra ID"
algID.BHdebug = LeafDebug

algSolver = algebra_solve()
algSolver.Name = "Algebra Solver"
algSolver.BHdebug = LeafDebug

algSol = b3.Sequence([algID, algSolver])
algSol.Name = "Algebra ID and Solve"
algSol.BHdebug = SolverDebug

#  sin(th) OR cos(th)
scID = sincos_id()
scID.Name = "Sin Cos ID"
scID.BHdebug = SolverDebug

scSolver = sincos_solve()
scSolver.Name = "Sine Cosine Solver"
scSolver.BHdebug =  LeafDebug

scSol = b3.Sequence([scID,scSolver])
scSol.Name = "SinCos ID+Solve"
scSol.BHdebug = SolverDebug
 
# sin(th) AND cos(th) in same eqn
sacID = sinandcos_id()
sacID.Name = "Sin Cos ID"
sacID.BHdebug = True

sacSolver = sinandcos_solve()
sacSolver.Name = "Sine Cosine Solver"
sacSolver.BHdebug = True

sacSol = b3.Sequence([sacID,sacSolver])
sacSol.Name = "Sin AND Cos ID+Solve"
sacSol.BHdebug = SolverDebug
 
# x^2 + y^2 trick from Craig (eqn 4.65)
x2z2_Solver = x2z2_id_solve()
x2z2_Solver.Name = 'X2Y2 id and solver'
x2z2_Solver.BHdebug = True



# two equations one unknown,
SimuEqnID = simu_id()
SimuEqnID.Name = 'Simultaneous Eqn ID'
SimuEqnID.BHdebug = False
SimuEqnSolve = simu_solver()
SimuEqnSolve.Name = 'Simultaneous Eqn solver'
Simu_Eqn_Sol = b3.Sequence([SimuEqnID, SimuEqnSolve])
 #
 #  Equation Transforms
 #
 
sub_trans = sub_transform()
sub_trans.Name = "Substitution Transform"
sub_trans.BHdebug = LeafDebug

sumOfAnglesT = sum_id()  # we should change name of this to 'transform'
sumOfAnglesT.BHdebug = False
sumOfAnglesT.Name = "Sum of Angles Transform"

sumOfAnglesSolve = sum_solve()
sumOfAnglesSolve.Name = "Sum of Angles Solve"

updateL = updateL()
updateL.Name = "updateL Transform"
updateL.BHdebug = False
  

compDetect = comp_det()
compDetect.Name = "Completion Detect" 
compDetect.BHdebug = True

#           ONE BT TO RULE THEM ALL!
#   Higher level BT nodes here
#

sc_tan = b3.Sequence([b3.OrNode([tanSol, scSol]), rk])


# this is the current working version
# it's also possible to build customized BT
worktools = b3.Priority([algSol, sc_tan, Simu_Eqn_Sol, sacSol, x2z2_Solver])



subtree = b3.RepeatUntilSuccess(b3.Sequence([asgn, worktools]), 6)
solveRoutine = b3.Sequence([sub_trans, subtree,  updateL, compDetect])

topnode = b3.RepeatUntilSuccess(solveRoutine, 7) #max 10 loops

ikbt.root = topnode 

logdir = 'logs/'
       
if not os.path.isdir(logdir):  # if this doesn't exist, create it.
    os.mkdir(logdir)

#
#     Logging setup
#    
if(robot == 'MiniDD'):

    ikbt.log_flag = 2  # log exits:  1=SUCCESS only, 2=BOTH S,F
    ikbt.log_file = open(logdir + 'BT_MiniDD_node_log.txt', 'w')
    ikbt.log_file.write('MiniDD Solution Node Log\n')
    
    scSol.BHdebug = False
    scID.BHdebug = False
    scSolver.BHdebug = False
    
    tanSol.BHdebug = False
    
if(robot == 'Chair_Helper'):
    
    ikbt.log_flag = 2  # log exits:  1=SUCCESS only, 2=BOTH S,F
    ikbt.log_file = open(logdir + 'BT_ChHelper_node_log.txt', 'w')
    ikbt.log_file.write('Robot Solution Node Log\n')


if(robot == 'Wrist'):    
    ikbt.log_flag = 2  # log exits:  1=SUCCESS only, 2=BOTH S,F
    ikbt.log_file = open(logdir + 'BT_Wrist_node_log.txt', 'w')
    ikbt.log_file.write('Robot Solution Node Log\n')
    #print ' ----------------------------   INITIAL KINEMATIC EQUATION ----------------------'
    #print R.mequation_list[0]   # print the classic matrix equation
    #print ' --------------------------------------------------------------------------------'
    tanSol.BHdebug = False
    tanSolver.BHdebug = False
    #tanID.BHdebug = True
    
    
    
if (robot == 'Puma' ):  # Puma debug setup
    ikbt.log_flag = 2  # log exits:  1=SUCCESS only, 2=BOTH S,F
    ikbt.log_file = open(logdir + 'BT_Puma_node_log.txt', 'w')
    ikbt.log_file.write('Puma Node Log --\n') 
    
    T = True
    F = False 
   
    sumOfAnglesSolve.BHdebug = F
   
    tanSolver.BHdebug = F
    tanID.BHdebug = F
    
    sacSol.BHdebug = F
    sacID.BHdebug = F
    sacSolver.BHdebug = F
    scSol.BHdebug = F
    scID.BHdebug = F
    scSolver.BHdebug = F
    
    x2z2_Solver.BHdebug = T
    sumOfAnglesT.BHdebug = F

    compDetect.BHdebug = F
    compDetect.FailAllDone = F # set it up to SUCCEED when there is more work to do. (not default)
    algID.BHdebug = F
    algSolver.BHdebug = F
    tanSol.BHdebug = F 
#
#    Set up the blackboard for solution
#
bb = b3.Blackboard()


##   Generate the lists of soln candidate equations from the matrix equations
[L1, L2, L3p] = R.scan_for_equations(unknowns)  # lists of 1unk and 2unk equations
bb.set('eqns_1u', L1)   # eqns with one unk
bb.set('eqns_2u', L2)   #           two unks
bb.set('eqns_3pu', L3p)   #        three or more unks

R.sum_of_angles_transform(unknowns) #get the sum of angle

bb.set('Robot', R)
bb.set('unknowns', unknowns)



################################################################################
#
#           Perform the Computation via ticking the BT
#


#  Off we go: tick the BT
print "Ticking IK BT for ", R.name, " -------------------------\n\n"

ikbt.tick("Test a full solver", bb)

unks = bb.get('unknowns')
Tm = bb.get('Tm')
R = bb.get('Robot')


if TEST_DATA_GENERATION:
    # Now we're going to save some results for use in tests.
    print ' Storing results for test use'
    test_pickle_dir = 'Test_pickles/'       
    name = test_pickle_dir + R.name + 'test_pickle.p'
    with open(name,'wb') as pf:
        pickle.dump( [R, unks], pf)
    quit()

    

final_groups = matching.matching_func(R.notation_collections, R.solution_nodes)
# # matching, now integrated into the latex report 
# uncomment for debugging

# print "sorted final notation groups"
# for a_set in final_groups:
#    print a_set
output_solution_graph(R)
output_latex_solution(R,unks, final_groups)
op.output_python_code(R, final_groups)
oc.output_cpp_code(R, final_groups)




#################################################
# print out all eqnuations that used to solve variables
# uncomment for debugging

print "equations evaluated"
for one_unk in unks:
    print one_unk.symbol
    print one_unk.eqntosolve
    print one_unk.secondeqn
    print '\n'


#
#
#
################################################################################

# define symbols that appear in solutions
sp.var('r_11 r_12 r_13 r_21 r_22 r_23 r_31 r_32 r_33 Px Py Pz')

ntests = 0
if(robot == 'Chair_Helper'):
    fs = 'Chair_Helper   FAIL'
    for u in unks:
        print '\n Asserting: ', u.symbol, ' = ',
        if(u.symbol == d_1):
            ntests += 1
            assert(u.nsolutions == 1), fs+' n(d_1)'
            print str(u.solutions[0])
            assert(u.solutions[0] == Pz - l_4*r_33), fs + '  [d_1]'
        if(u.symbol == th_2):
            ntests += 1
            assert(u.nsolutions == 2), fs+' n(th_2)'
            print str(u.solutions[0]) + ', ' + str(u.solutions[1])
            assert(u.solutions[0] ==  sp.asin((Px-l_1-l_4*r_13)/l_2) ), fs + ' [th_2a]'
            assert(u.solutions[1] == -sp.asin((Px-l_1-l_4*r_13)/l_2)+sp.pi ), fs + ' [th_2b]'


ntests = 0
##  Test Assertions
if(robot == 'Wrist'):
    pass

fs = '\n         Warning: \n   No Assertions yet for ' + robot
if (robot == 'Puma'):
    print fs

string = 'test robot '+robot

print '\n\n\n                            ',string,'  PASSES \n\n\n'


