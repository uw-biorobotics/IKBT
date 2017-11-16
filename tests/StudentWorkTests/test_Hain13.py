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
from sum_transform import *

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

# Haines 2013
# DOF 6

Haines13_dh = sp.Matrix([
    [0.,        0.,         d_1,        0.],
    [sp.pi/2,   0.,         l_2,        th_2],
    [sp.pi/2,   0.,         0.,         th_3],
    [0.,        l_4,        0.,         th_4],
    [0.,        l_5,         0.,        th_5],
    [sp.pi/2,     0.,       l_6,        th_6]
    ])
    
Haines13_vv = [0, 1, 1, 1, 1, 1]
Haines13_unk = [d_1, th_2, th_3, th_4, th_5, th_6]


dh = Haines13_dh
vv = Haines13_vv
unknowns = Haines13_unk

easySetup = False
treeTest = True

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
    tanSolver.BHdebug = False
    
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
    algSolver.BHdebug = False
    
    algMod = b3.Sequence([algID, algSolver])
    
    sincosID = sincos_id()
    sincosID.Name = 'Sin or cos ID'
    sincosID.BHdebug = False
    
    sincosSolver = sincos_solve()
    sincosSolver.Name = 'sin or cos solver'
    
    sincosMod = b3.Sequence([sincosID, sincosSolver])
    
    #sum_solver
    sumID = sum_id()
    sumID.Name = "sum of anlge ID"
    
    sumSol = sum_solve()
    sumSol.Name = "sum of angle solve"
    
    #sin and cos
    sandcID = sinandcos_id()
    sandcID.Name = "sin and cos ID"
    
    sandcSolver = sinandcos_solve()
    sandcSolver.Name = "sin adn cos Solver"
    
    sandcMod = b3.Sequence([sandcID, sandcSolver])
    
    commonTricks = b3.Priority([subID, algMod, tanMod, sincosMod, sandcMod, sumSol])
    
    #commonTricks = b3.Priority([subID, algMod, tanMod, sincosMod, sumSol])
    repeatCommons = b3.RepeatUntilFailure(commonTricks, 3)
    
    #important to invert to Success to continue to T update
    convertToSucc = b3.Inverter(repeatCommons)
    
    update = updateT()
    update.Name = 'update T'
    update.BHdebug = False
    
    compDet = comp_det()
    compDet.Name = 'Completion Detection'
    compDet.BHdebug = True
    
    oneRound = b3.Sequence([sumID, convertToSucc, update, compDet])
    
    repeatRound = b3.RepeatUntilSuccess(oneRound, 4) 
    
    ikbt.root = b3.Sequence([setup, repeatRound])
    ikbt.tick('test for L11', bb)
    
    unkns = bb.get('unknowns')
    for unk in unkns:
        if unk.solved:
            print unk.symbol
            print unk.solutions
            
    prt_tree = False
    
    if prt_tree:
    
        print '\n\n ====================== Results =====================\n'
        #  Print some stuff to show the test worked!!
        print '                        ',R.name,'\n\n'
        print " inverse kinematics solutions found: "
        unks = bb.get('unknowns')
        for u in unks:
            if(u.solved):
                print u.symbol, ' = {'
                for s in u.solutions:
                    sp.pprint(s)
                print '}'
                print u.nsolutions, ' solutions, by the ', u.solvemethod, ' method.'
                print 'Solution Order: ',u.solveorder
                print ' '        
                
        print '===========   Current Ts Matrix (RHS) ================'
        Tm = bb.get('Tm')
        sp.pprint( notation_squeeze(Tm.Ts) )

        print ' === for following Craigs solution: '
        print ' [1,4] '
        sp.pprint ( notation_squeeze(Tm.Td[0,3]))
        print '= '
        sp.pprint( notation_squeeze(Tm.Ts[0,3]) )

        print '========== Solution Tree output ================'

        sequence = []
        nodes = []
        for n in R.solutiontreenodes:
            sequence.append(n.variable.solveorder)
            nodes.append(n)
        nodes.sort(key=dict(zip(nodes,sequence)).get)

        for n in nodes:
            print '-> ',n.variable.solveorder
            n.output()
    
