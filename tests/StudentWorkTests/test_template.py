import sympy as sp
from kin_cl import *
from sys import exit

# modified by BH, local version in current dir
import b3 as b3          # behavior trees

# local modules
import setup_node
from helperfunctions import *
from ik_classes import *     # special classes for Inverse kinematics in sympy

from updateT import * 
from tan_solver import *
from sincos_solver import *
from sinANDcos_solver import *
from sub_transform import *
from algebra_solver import *

sp.init_printing()

ik_tester = b3.BehaviorTree()

print ""
print "          Testing IK solution "
print ""
print ""


# generic variables for any maniplator
((th_1, th_2, th_3, th_4, th_5, th_6)) = sp.symbols(('th_1', 'th_2', 'th_3', 'th_4', 'th_5', 'th_6'))
((d_1, d_2, d_3, d_4, d_5, d_6)) = sp.symbols(('d_1', 'd_2', 'd_3', 'd_4', 'd_5', 'd_6'))
((h,l_0, l_1, l_2, l_3, l_4)) = sp.symbols(('h','l_0', 'l_1', 'l_2', 'l_3', 'l_4'))
((a_2, a_3)) = sp.symbols(('a_2', 'a_3'))

sp.var('th_12, th_23, th_34, th_45, th_56')
sp.var('c_12 s_12 c_23 s_23 c_34 s_34 c_45 s_45 c_56 s_56 c_13 s_13')

########################################################
#
#     Robot Parameters

# standardize on the order "alpha N-1, a N-1, d N, theta N' for the DH table columns.
params_example = [l_1,l_2,l_4]

# Copy paste paramenters from tastCases file
# build tree around the test methods

# Gary 2011, 
# DOF : 6 
# mentods to test:m1, m2, m3, m6
Gary11_dh = sp.Matrix([
    [0.,        0.,         0.,         th_1],
    [sp.pi/2,   0.,         l_1,        th_2],
    [0.,        l_2,        0.,         th_3],
    [sp.pi,     l_3,        0.,         th_4],
    [sp.pi,     0.,         0.,         th_5],
    [sp.pi,     0.,         d_6,        0.]
    ])
    
Gary11_vv = [1, 1, 1, 1, 1, 0]
Gary11_unk = [th_1, th_2, th_3, th_4, th_5, d_6]

dh = Gary11_dh
vv = Gary11_vv
unknowns = Gary11_unk

M = mechanism(dh, params, vv)
M.forward_kinematics()
Ts = M.T_06

# sp.pprint(Ts)

# for t in Ts:
    # print "term: "
    # sp.pprint(t)
    
ikbt = b3.BehaviorTree()
bb = b3.Blackboard()
bb.set("unknowns", unknowns)
bb.set("vv", vv)
bb.set("dh", dh)

setup = init_setup()

LeafDebug = False
SolverDebug = True

tanID = tan_id()
tanID.Name = 'Tangent ID'
tanID.BHdebug =  LeafDebug

tanSolver = tan_solve()
tanSolver.BHdebug = LeafDebug
tanSolver.Name = "Tangent Solver"

# update 2-equ