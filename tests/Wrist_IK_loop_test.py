#!/usr/bin/python
#  Python inverse kinematic equations for Wrist

#  This test performs the following:
#
#  1) Generate a reachable 4x4 pose
#  2) run the generated IK Python code for "Wrist" robot
#  3) take resulting list of joint-solutions and put them back into
#          forward kinematics
#  4) compare the generated 4x4 matrices to the original pose
#
#  Running instructions:
#
#   > cd IKBT/
#   > pyton -m tests.Wrist_IK_loop_test
#
#   final results for each solution are error -- should be very small
#
#
import numpy as np
from math import sqrt
from math import atan2
from math import cos
from math import sin

from ikbtbasics.pykinsym import *
from ikbtfunctions.helperfunctions import *
from ikbtbasics.kin_cl import *
from ikbtbasics.ik_classes import *     # special classes for Inverse kinematics in sympy
from ikbtfunctions.ik_robots import *


pi = np.pi



#  Declare the parameters



# Code to solve the unknowns
def ikin_Wrist(T):
    if(T.shape != (4,4)):
        print "bad input to "+funcname
        quit()
#define the input vars
    r_11 = T[0,0]
    r_12 = T[0,1]
    r_13 = T[0,2]
    r_21 = T[1,0]
    r_22 = T[1,1]
    r_23 = T[1,2]
    r_31 = T[2,0]
    r_32 = T[2,1]
    r_33 = T[2,2]
    Px = T[0,3]
    Py = T[1,3]
    Pz = T[2,3]

#
# Caution:    Generated code is not yet validated
#


    solvable_pose = True


    #Variable:  A
    As1 = atan2(r_21, r_11)
    As2 = atan2(-r_21, -r_11)


    #Variable:  B
    Bs2 = atan2(-r_31, r_11/cos(As2))
    Bs1 = atan2(-r_31, r_11/cos(As1))


    #Variable:  C
    Cs2 = atan2(r_32/cos(Bs1), r_33/cos(Bs1))
    Cs1 = atan2(r_32/cos(Bs2), r_33/cos(Bs2))

##################################
#
#package the solutions into a list for each set
#
###################################

    solution_list = []
    #(note trailing commas allowed in python
    solution_list.append( [  As1,  Bs1,  Cs2,  ] )
    #(note trailing commas allowed in python
    solution_list.append( [  As2,  Bs2,  Cs1,  ] )
    if(solvable_pose):
        return(solution_list)
    else:
        return(False)


#
#    TEST CODE
#
if __name__ == "__main__":

#  4x4 transforms which are pure rotations

    def RotX4_N(t):
      return(np.matrix([
        [1,         0,           0,      0],
        [0, np.cos(t),  -np.sin(t),      0],
        [0, np.sin(t),   np.cos(t),      0],
        [0,0,0,1.0]
        ]))

    def RotY4_N(t):
      return(np.matrix([
        [ np.cos(t),   0,      np.sin(t),    0],
        [0,            1,          0    ,    0],
        [-np.sin(t),   0,      np.cos(t),    0],
        [0,0,0,1]
        ]))

    def RotZ4_N(t):
      return(np.matrix([
        [ np.cos(t),  -np.sin(t),       0,    0],
        [ np.sin(t),   np.cos(t),       0,    0],
        [ 0,              0,            1,    0],
        [0,0,0,1]
        ]))

    px = 0.2   # desired EE position
    py = 0.3
    pz = 0.6
    th = np.pi/7  # just a random angle

    # generate a 4x4 pose to test IK

    T1 = RotX4_N(th) * RotY4_N(2*th)  # combine two rotations
    T1[0,3] = px
    T1[1,3] = py
    T1[2,3] = pz

    # try the Puma IK

    sol_list = ikin_Wrist(T1)

    i = 0
    for sol in sol_list:
        print ''
        print 'Solution ', i
        i+=1
        print sol

#########3 try to plug back into FK model

robot = 'Wrist'

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

sp.var('A B C')
i=0
# for each solution, compare FK(sol) with T01
for sol in sol_list:
    pose = {A : sol[0], B : sol[1], C : sol[2]}
    T2 = forward_kinematics_N(M, pose, M.pvals)
    maxe = -9999999.99
    print '- - - - - '
    print T2-T1
    print '- - - - - '
    for k in [0,1,2,3]:
        for j in [0,1,2]:
            e = T1[k,j]-T2[k,j]
            #print '<<',e,'>>'
            if np.abs(e) > maxe:
                maxe = np.abs(e)
    print 'Solution ',i,': ', maxe
    i += 1
