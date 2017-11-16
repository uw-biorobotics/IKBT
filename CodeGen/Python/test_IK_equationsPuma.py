#!/usr/bin/python
#  Python inverse kinematic equations for Puma

import numpy as np
from math import sqrt
from math import atan2
from math import cos
from math import sin

pi = np.pi

##  define some wrappers for tighter notation
#def atan2(y,x):
    #return np.arctan2(y,x)


# Code to solve the unknowns
def ikin_Puma(T):
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

#    Declare

    d_3 = 0.25
    d_4 = 0.20
    a_2 = 0.1
    a_3 = 0.5
#
# Caution:    Generated code is not yet validation

    solvable_pose = True


    #Variable:  th_1
    th_1s1 = atan2(Px, -Py) + atan2(sqrt(Px**2 + Py**2 - d_3**2), -d_3)
    th_1s2 = atan2(Px, -Py) + atan2(-sqrt(Px**2 + Py**2 - d_3**2), -d_3)


    #Variable:  th_3
    th_3s3 = atan2(-2*a_2*d_4, 2*a_2*a_3) + atan2(sqrt(4*a_2**2*a_3**2 + 4*a_2**2*d_4**2 - (Pz**2 - a_2**2 - a_3**2 - d_4**2 + (Px*cos(th_1s2) + Py*sin(th_1s2))**2)**2), Pz**2 - a_2**2 - a_3**2 - d_4**2 + (Px*cos(th_1s2) + Py*sin(th_1s2))**2)
    th_3s2 = atan2(-2*a_2*d_4, 2*a_2*a_3) + atan2(-sqrt(4*a_2**2*a_3**2 + 4*a_2**2*d_4**2 - (Pz**2 - a_2**2 - a_3**2 - d_4**2 + (Px*cos(th_1s1) + Py*sin(th_1s1))**2)**2), Pz**2 - a_2**2 - a_3**2 - d_4**2 + (Px*cos(th_1s1) + Py*sin(th_1s1))**2)
    th_3s1 = atan2(-2*a_2*d_4, 2*a_2*a_3) + atan2(sqrt(4*a_2**2*a_3**2 + 4*a_2**2*d_4**2 - (Pz**2 - a_2**2 - a_3**2 - d_4**2 + (Px*cos(th_1s1) + Py*sin(th_1s1))**2)**2), Pz**2 - a_2**2 - a_3**2 - d_4**2 + (Px*cos(th_1s1) + Py*sin(th_1s1))**2)
    th_3s4 = atan2(-2*a_2*d_4, 2*a_2*a_3) + atan2(-sqrt(4*a_2**2*a_3**2 + 4*a_2**2*d_4**2 - (Pz**2 - a_2**2 - a_3**2 - d_4**2 + (Px*cos(th_1s2) + Py*sin(th_1s2))**2)**2), Pz**2 - a_2**2 - a_3**2 - d_4**2 + (Px*cos(th_1s2) + Py*sin(th_1s2))**2)


    #Variable:  th_23
    th_23s2 = atan2(Pz*(-a_2*cos(th_3s2) - a_3) - (-Px*cos(th_1s1) - Py*sin(th_1s1))*(a_2*sin(th_3s2) - d_4), Pz*(a_2*sin(th_3s2) - d_4) + (-Px*cos(th_1s1) - Py*sin(th_1s1))*(-a_2*cos(th_3s2) - a_3))
    th_23s4 = atan2(Pz*(-a_2*cos(th_3s4) - a_3) - (-Px*cos(th_1s2) - Py*sin(th_1s2))*(a_2*sin(th_3s4) - d_4), Pz*(a_2*sin(th_3s4) - d_4) + (-Px*cos(th_1s2) - Py*sin(th_1s2))*(-a_2*cos(th_3s4) - a_3))
    th_23s1 = atan2(Pz*(-a_2*cos(th_3s3) - a_3) - (-Px*cos(th_1s2) - Py*sin(th_1s2))*(a_2*sin(th_3s3) - d_4), Pz*(a_2*sin(th_3s3) - d_4) + (-Px*cos(th_1s2) - Py*sin(th_1s2))*(-a_2*cos(th_3s3) - a_3))
    th_23s3 = atan2(Pz*(-a_2*cos(th_3s1) - a_3) - (-Px*cos(th_1s1) - Py*sin(th_1s1))*(a_2*sin(th_3s1) - d_4), Pz*(a_2*sin(th_3s1) - d_4) + (-Px*cos(th_1s1) - Py*sin(th_1s1))*(-a_2*cos(th_3s1) - a_3))


    #Variable:  th_2
    th_2s2 = th_23s4 - th_3s4
    th_2s4 = th_23s3 - th_3s1
    th_2s1 = th_23s2 - th_3s2
    th_2s3 = th_23s1 - th_3s3


    #Variable:  th_4
    th_4s3 = atan2(r_13*sin(th_1s2) - r_23*cos(th_1s2), r_13*cos(th_1s2)*cos(th_23s4) + r_23*sin(th_1s2)*cos(th_23s4) - r_33*sin(th_23s4))
    th_4s7 = atan2(r_13*sin(th_1s1) - r_23*cos(th_1s1), r_13*cos(th_1s1)*cos(th_23s3) + r_23*sin(th_1s1)*cos(th_23s3) - r_33*sin(th_23s3))
    th_4s2 = atan2(-r_13*sin(th_1s1) + r_23*cos(th_1s1), -r_13*cos(th_1s1)*cos(th_23s2) - r_23*sin(th_1s1)*cos(th_23s2) + r_33*sin(th_23s2))
    th_4s6 = atan2(-r_13*sin(th_1s2) + r_23*cos(th_1s2), -r_13*cos(th_1s2)*cos(th_23s1) - r_23*sin(th_1s2)*cos(th_23s1) + r_33*sin(th_23s1))
    th_4s4 = atan2(-r_13*sin(th_1s2) + r_23*cos(th_1s2), -r_13*cos(th_1s2)*cos(th_23s4) - r_23*sin(th_1s2)*cos(th_23s4) + r_33*sin(th_23s4))
    th_4s1 = atan2(r_13*sin(th_1s1) - r_23*cos(th_1s1), r_13*cos(th_1s1)*cos(th_23s2) + r_23*sin(th_1s1)*cos(th_23s2) - r_33*sin(th_23s2))
    th_4s8 = atan2(-r_13*sin(th_1s1) + r_23*cos(th_1s1), -r_13*cos(th_1s1)*cos(th_23s3) - r_23*sin(th_1s1)*cos(th_23s3) + r_33*sin(th_23s3))
    th_4s5 = atan2(r_13*sin(th_1s2) - r_23*cos(th_1s2), r_13*cos(th_1s2)*cos(th_23s1) + r_23*sin(th_1s2)*cos(th_23s1) - r_33*sin(th_23s1))


    #Variable:  th_5
    th_5s4 = atan2((-r_13*sin(th_1s2) + r_23*cos(th_1s2))/sin(th_4s6), -r_13*sin(th_23s1)*cos(th_1s2) - r_23*sin(th_1s2)*sin(th_23s1) - r_33*cos(th_23s1))
    th_5s8 = atan2((-r_13*sin(th_1s2) + r_23*cos(th_1s2))/sin(th_4s5), -r_13*sin(th_23s1)*cos(th_1s2) - r_23*sin(th_1s2)*sin(th_23s1) - r_33*cos(th_23s1))
    th_5s1 = atan2((-r_13*sin(th_1s2) + r_23*cos(th_1s2))/sin(th_4s3), -r_13*sin(th_23s4)*cos(th_1s2) - r_23*sin(th_1s2)*sin(th_23s4) - r_33*cos(th_23s4))
    th_5s2 = atan2((-r_13*sin(th_1s1) + r_23*cos(th_1s1))/sin(th_4s7), -r_13*sin(th_23s3)*cos(th_1s1) - r_23*sin(th_1s1)*sin(th_23s3) - r_33*cos(th_23s3))
    th_5s6 = atan2((-r_13*sin(th_1s1) + r_23*cos(th_1s1))/sin(th_4s1), -r_13*sin(th_23s2)*cos(th_1s1) - r_23*sin(th_1s1)*sin(th_23s2) - r_33*cos(th_23s2))
    th_5s3 = atan2((-r_13*sin(th_1s1) + r_23*cos(th_1s1))/sin(th_4s2), -r_13*sin(th_23s2)*cos(th_1s1) - r_23*sin(th_1s1)*sin(th_23s2) - r_33*cos(th_23s2))
    th_5s7 = atan2((-r_13*sin(th_1s1) + r_23*cos(th_1s1))/sin(th_4s8), -r_13*sin(th_23s3)*cos(th_1s1) - r_23*sin(th_1s1)*sin(th_23s3) - r_33*cos(th_23s3))
    th_5s5 = atan2((-r_13*sin(th_1s2) + r_23*cos(th_1s2))/sin(th_4s4), -r_13*sin(th_23s4)*cos(th_1s2) - r_23*sin(th_1s2)*sin(th_23s4) - r_33*cos(th_23s4))


    #Variable:  th_6
    th_6s3 = atan2(r_12*sin(th_23s1)*cos(th_1s2) + r_22*sin(th_1s2)*sin(th_23s1) + r_32*cos(th_23s1), -r_11*sin(th_23s1)*cos(th_1s2) - r_21*sin(th_1s2)*sin(th_23s1) - r_31*cos(th_23s1))
    th_6s2 = atan2(r_12*sin(th_23s4)*cos(th_1s2) + r_22*sin(th_1s2)*sin(th_23s4) + r_32*cos(th_23s4), -r_11*sin(th_23s4)*cos(th_1s2) - r_21*sin(th_1s2)*sin(th_23s4) - r_31*cos(th_23s4))
    th_6s4 = atan2(r_12*sin(th_23s3)*cos(th_1s1) + r_22*sin(th_1s1)*sin(th_23s3) + r_32*cos(th_23s3), -r_11*sin(th_23s3)*cos(th_1s1) - r_21*sin(th_1s1)*sin(th_23s3) - r_31*cos(th_23s3))
    th_6s1 = atan2(r_12*sin(th_23s2)*cos(th_1s1) + r_22*sin(th_1s1)*sin(th_23s2) + r_32*cos(th_23s2), -r_11*sin(th_23s2)*cos(th_1s1) - r_21*sin(th_1s1)*sin(th_23s2) - r_31*cos(th_23s2))

##################################
#
#package the solutions into a list for each set
#
###################################

    solution_list = []
    #(note trailing commas allowed in python
    solution_list.append( [  th_1s2,  th_23s4,  th_2s2,  th_3s4,  th_4s3,  th_5s1,  th_6s2,  ] )
    #(note trailing commas allowed in python
    solution_list.append( [  th_1s1,  th_23s3,  th_2s4,  th_3s1,  th_4s7,  th_5s2,  th_6s4,  ] )
    #(note trailing commas allowed in python
    solution_list.append( [  th_1s1,  th_23s2,  th_2s1,  th_3s2,  th_4s2,  th_5s3,  th_6s1,  ] )
    #(note trailing commas allowed in python
    solution_list.append( [  th_1s2,  th_23s1,  th_2s3,  th_3s3,  th_4s6,  th_5s4,  th_6s3,  ] )
    #(note trailing commas allowed in python
    solution_list.append( [  th_1s2,  th_23s4,  th_2s2,  th_3s4,  th_4s4,  th_5s5,  th_6s2,  ] )
    #(note trailing commas allowed in python
    solution_list.append( [  th_1s1,  th_23s2,  th_2s1,  th_3s2,  th_4s1,  th_5s6,  th_6s1,  ] )
    #(note trailing commas allowed in python
    solution_list.append( [  th_1s1,  th_23s3,  th_2s4,  th_3s1,  th_4s8,  th_5s7,  th_6s4,  ] )
    #(note trailing commas allowed in python
    solution_list.append( [  th_1s2,  th_23s1,  th_2s3,  th_3s3,  th_4s5,  th_5s8,  th_6s3,  ] )
    if(solvable_pose):
        return(solution_list)
    else:
        return(False)



if __name__ == "__main__":
    from ikbtbasics.pykinsym import *
    #from ikbtbasics.ik_classes import *     # special classes for Inverse kinematics in sympy


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

    list = ikin_Puma(T1)

    print list

