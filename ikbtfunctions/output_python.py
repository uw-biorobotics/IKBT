#!/usr/bin/python
#
#   Generate python output code of the IK solution
#

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
#import numpy as np
from ikbtbasics.kin_cl import *
from ikbtfunctions.helperfunctions import *
from ikbtbasics.ik_classes import *     # special classes for Inverse kinematics in sympy
#

importString = '''#!/usr/bin/python
#  Python inverse kinematic equations for **Robot**

import numpy as np
from math import sqrt
from math import atan2
from math import cos
from math import sin
from math import acos
from math import asin

pi = np.pi

'''
#
#   Output python code to simplify and numerically evaluate the forward kinematic equations
#

def output_FK_python_code(Robot):

    DirName = 'CodeGen/Python/'
    orig_name  = Robot.name.replace('test: ', '')
    fname = DirName + 'FK_equations'+orig_name+'.py'
    f = open(fname, 'w')

    importString = '''#!/usr/bin/python
#  Python forward kinematic equations for **Robot**

import numpy as np
from math import sqrt
from math import atan2
from math import cos
from math import sin
from math import acos
from math import asin

pi = np.pi

'''
    importString = importString.replace('**Robot**', Robot.name)
    print(importString, file=f)

    matclass = '''class Matrix:
    def __init__(self,A):
        Matrix.A = A
        Matrix.rows = len(A)
        Matrix.cols = len(A[0])

    def __repr__(self):
        res = '\\n'
        for i in range(self.rows):
            for j in range(self.cols):
                res += f'{self.A[i][j]:10.3f} '
            res += '\\n'
        return res
    '''

    print(matclass, file=f)

    indent = '' # 4 spaces

    # parameter Declarations (a_3, d_5, etc).

    print('#\n#      Robot Parameters \n#',file=f)
    tmp = '\n'
    if(Robot.Mech.pvals != {}):  # if we have numerical values stored
        for p in Robot.params:
            val = str(Robot.Mech.pvals[p])
            tmp += str(p) + ' = ' + val + '\n'
    else:                        # no stored numerical values
        for p in Robot.params:
            tmp += str(p) + ' = XXXXX    # deliberate undeclared error!  USER needs to give numerical value \n'
    par_decl_str = tmp

    print(par_decl_str, file=f)


    # joint variables:
    print('Debug: joint variables: ', Robot.variables)

    print('#\n#     Robot Joint Variables \n#',file=f)

    for v in Robot.variables:
        ss = str(v).split('_')   # get joint subscript(s)
        print('Variabl: SofA: ', str(v), ss)
        if len(ss) > 1:
            if len(ss[1]) > 1:  # we have a sum_of_angles
                print('Sum of ang found in variable: ', str(v))
                subs = [*ss[1]] # make list of
                soa = str(v) + ' = ' # e.g. 'th_23 = '
                for s in subs:
                    print('Debug subscript s:',ss, s)
                    # find var with this subscript:
                    for v1 in Robot.variables:
                        if s in str(v1) and len(str(v1).split('_')[1]) == 1: # avoid soa subscripts
                            soa += f' {str(v1)} +'
                print(indent + soa[:-1], file=f)
            else:
                print(indent + f'{str(v)} = 1.0   # 1.0= dummy value',file=f)

    funcname = 'Fkin_'+orig_name
    print('''
# Code to compute Forward Kinematics ''', file=f)
    #print('def', funcname +'():', file=f) # no indent

    print(indent + '''
#############################################################
#
#   Forward Kinematics
#
#############################################################
 ''', file=f)


    Fkeqns = Robot.Mech.forward_kinematics()  # need to redo this????

    Tfk = (Robot.Mech.T_06)

    Tfks = str(Tfk)

    print('T06 = ' + Tfks,file=f)
    print('',file=f)
    print('print(f"T06 has {T06.rows} rows and {T06.cols} cols")',file=f)
    print('',file=f)
    print('print(T06)', file=f)
    print('',file=f)

    f.close()




#
#   Output python code to   evaluate the inverse kinematic equations
#

def output_python_code(Robot, groups):

    importString = '''#!/usr/bin/python
#  Python inverse kinematic equations for **Robot**

import numpy as np
from math import sqrt
from math import atan2
from math import cos
from math import sin
from math import acos
from math import asin

pi = np.pi

'''
    fixed_name = Robot.name.replace(r'_', r'\_')  # this is for LaTex output
    fixed_name = fixed_name.replace('test: ','')
    orig_name  = Robot.name.replace('test: ', '')

    DirName = 'CodeGen/Python/'
    fname = DirName + 'IK_equations'+orig_name+'.py'
    f = open(fname, 'w')

    importString = importString.replace('**Robot**', Robot.name)
    print(importString, file=f)

    # parameter Declarations (a_3, d_5, etc).
    tmp = '\n'
    if(Robot.Mech.pvals != {}):  # if we have numerical values stored
        for p in Robot.params:
            val = str(Robot.Mech.pvals[p])
            tmp += str(p) + ' = ' + val + '\n'
    else:                        # no stored numerical values
        for p in Robot.params:
            tmp += str(p) + ' = XXXXX    # deliberate undeclared error!  USER needs to give numerical value \n'
    par_decl_str = tmp


    nlist = Robot.solution_nodes
    #nlist.sort( ) # sort by solution order


    indent = '    ' # 4 spaces

    funcname = 'ikin_'+fixed_name 
    print('''
# Auto Generated Code to solve the unknowns
#        parameter:  T   4x4 numerical target for T06
#
''', file=f)
    print('def', funcname +'(T):', file=f) # no indent
    print(indent+'if(T.shape != (4,4)):', file=f)
    print(indent*2 + 'print ( "bad input to "+funcname'+')', file=f)
    print(indent*2 + 'quit()', file=f)
    print('''#define the input vars 
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

    ''', file=f)
    
    print('#  Declare the parameters', file=f)
    print(par_decl_str, file=f)


    print(indent + 'print ( " Caution - this code has no solution checking.")', file=f)
    print(indent + 'print ("in case of domain errors, change the test position / orientation ")', file=f)
    print(indent + 'print ( " to a pose reachable by your specific robot")', file=f)
    print(indent + '', file=f)
    print(indent + 'solvable_pose = True', file=f)
    print(indent + '''
 
#############################################################
#
# Future reachable pose checking code (autogenerated) will go here
#
#    This code will check asin / acos args etc to determine
#
#        solvable_pose = True/False
#
#############################################################
 ''', file=f) 
 
    for node in nlist:  # for each solved var
        print('\n', file=f)
        print(indent + '#Variable: ', str(node.symbol), file=f)
        nsolns = node.unknown.nsolutions #len(node.solution_with_notations.values())
        nvers  = Robot.nversions
        colindex = node.unknown.solveorder-1  # select the unknown
        for rowindex in range(nvers): # go through the versions
            # get the solution equation version
            solEqnVer = Robot.FinalEqnMatrix[rowindex][colindex]
            print('Python Output: Solution Equation Version: ', solEqnVer)
            if re.search('asin', str(solEqnVer.RHS)) or re.search('acos', str(solEqnVer.RHS)):
                print ('  Found asin/acos solution ...', solEqnVer.LHS , ' "=" ',solEqnVer.RHS)
                tmp = re.search('\((.*)\)',str(solEqnVer.RHS))
                print(indent + 'if (solvable_pose and abs', tmp.group(0), ' > 1):', file=f)
                print(indent*2 + 'solvable_pose = False', file=f)
                print(indent + 'else:', file=f)
                tmp = str(solEqnVer.LHS) + ' = ' + str(solEqnVer.RHS)
                print(indent*2 + tmp, file=f)
            if re.search('atan', str(solEqnVer.RHS)):
                print('  Found atan2 solution ...', solEqnVer.LHS , ' "=" ',solEqnVer.RHS)
                tmp = re.search('\((.*)\)',str(solEqnVer.RHS))
                tmp = str(solEqnVer.LHS) + ' = ' + str(solEqnVer.RHS)
                print(indent + tmp, file=f)
            if node.solvemethod == 'algebra':
                print('  Found algebra solution ... ' , solEqnVer.LHS , ' = ', solEqnVer.RHS)
                print(indent + str(solEqnVer.LHS) + ' = ' + str(solEqnVer.RHS), file=f)

    print('''
##################################
#
#package the solutions into a list for each set
#
###################################
''', file=f)


    ###########################################################
    #
    #   Output of solution sets
    #
    ###########################################################
    # now group matching is done outside, in ikSolver

    #groups = mtch.matching_func(Robot.notation_collections, Robot.solution_nodes)

    grp_lists = []
    for g in groups:
        gs = []
        for t in g:
            print('g: ', g, 't: ', t)
            gs.append(str(t))

        #print(gs.sort, file=f) # in place
        grp_lists.append(gs)

    print(indent +  'solution_list = []', file=f)
    for g in grp_lists:
        g.sort()
        print(indent + '#(note trailing commas allowed in python', file=f)
        print(indent +  'solution_list.append( [ ', file=f)
        for v in g:
            print( v + ', ', file=f),
        print('] )', file=f)


    # we are done.   Return
    print(indent + 'if(solvable_pose):', file=f)
    print(indent*2 + 'return(solution_list)', file=f)
    print(indent + 'else: ', file=f)
    print(indent*2 + 'return(False)', file=f)

    # __main__()  code for testing:
    print('''

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

    list = ''' + funcname + '''(T1)

    i = 0
    for sol in list:
        print ''
        print 'Solution ', i
        i+=1
        print sol


    ''', file=f)


    f.close()
