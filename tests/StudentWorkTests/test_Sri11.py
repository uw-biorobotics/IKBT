import sympy as sp
from kin_cl import *
from sys import exit

# modified by BH, local version in current dir
import b3 as b3          # behavior trees

# local modules
from setup_node import*
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
((h,l_0, l_1, l_2, l_3, l_4, l_5)) = sp.symbols(('h','l_0', 'l_1', 'l_2', 'l_3', 'l_4', 'l_5'))
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

# Srisuan 2011,
# DOF: 6
# methods to test: m1, m4, m5, m3, m6
Srisuan11_dh = sp.Matrix([
    [0.,        0.,         d_1,        0.],
    [0.,        0.,         0.,         th_2],
    [sp.pi/2,   0.,         l_0,        th_3],
    [sp.pi/2,   0.,         d_4,        sp.pi/2],
    [0.,        0.,         0.,         th_5],
    [sp.pi/2,   0.,         0.,         th_6]
    ])
    
Srisuan11_vv = [0, 1, 1, 0, 1, 1]
Srisuan11_unk = [d_1, th_2, th_3, d_4, th_5, th_6]

dh = Srisuan11_dh
vv = Srisuan11_vv
unknowns = Srisuan11_unk

easySetup = True
treeTest = False

if easySetup:

    M = mechanism(dh, params, vv)
    M.forward_kinematics()
    Ts = M.T_06

    sp.pprint(Ts)

    for t in Ts:
        print "term: "
        sp.pprint(t)
    
if treeTest:

    ikbt = b3.BehaviorTree()
    bb = b3.Blackboard()
    bb.set("unknowns", unknowns)
    bb.set("vv", vv)
    bb.set("dh", dh)

    setup = init_setup()
    setup.Name = "Initial Setup"

    LeafDebug = False
    SolverDebug = True

    # common methods (under selector, repeat till failure)
    tanID = tan_id()
    tanID.Name = 'Tangent ID'
    tanID.BHdebug =  LeafDebug

    tanSolver = tan_solve()
    tanSolver.Name = "Tangent Solver"
    tanSolver.BHdebug = LeafDebug
    
    #build module
    tanMod = b3.Sequence([tanID, tanSolver]) 
    
    subID = sub_transform()
    subID.Name = 'Sub transform'
    subID.BHdebug = False
    
    algID = algebra_id()
    algID.Name = 'algebra ID'
    algID.BHdebug = True
    
    algSolver = algebra_solve()
    algSolver.Name = 'algebra Solver'
    algSolver.BHdebug = True
    
    subMod = b3.Sequence([subID, algID, algSolver])
    
    sincosID = sincos_id()
    sincosID.Name = 'Sin or cos ID'
    
    sincosSolver = sincos_solve()
    sincosSolver.Name = 'sin or cos solver'
    
    sincosMod = b3.Sequence([sincosID, sincosSolver])
    
    commonTricks = b3.Priority([subMod, tanMod, sincosMod])
    repeatCommons = b3.RepeatUntilFailure(commonTricks, 3)
    
    update = updateT()
    update.Name = 'update T'
    
    
    compDet = comp_det()
    oneRound = b3.Sequence([repeatCommons, update, compDet])
    
    repeatRound = b3.RepeatUntilSuccess(oneRound, 3) 
    
    ikbt.root = b3.Sequence([setup, repeatRound])
    ikbt.tick('test for L11', bb)
    
    
    
    
