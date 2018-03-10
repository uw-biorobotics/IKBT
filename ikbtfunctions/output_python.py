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

def output_python_code(Robot, groups):

    fixed_name = Robot.name.replace(r'_', r'\_')  # this is for LaTex output
    fixed_name = fixed_name.replace('test: ','')
    orig_name  = Robot.name.replace('test: ', '')

    DirName = 'CodeGen/Python/'
    fname = DirName + 'IK_equations'+orig_name+'.py'
    f = open(fname, 'w')
    print >> f, '''#!/usr/bin/python
#  Python inverse kinematic equations for ''' + fixed_name + '''

import numpy as np
from math import sqrt
from math import atan2
from math import cos
from math import sin

pi = np.pi


'''

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


    print >>f, '#  Declare the parameters'
    print >>f, par_decl_str


    nlist = Robot.solution_nodes
    nlist.sort( ) # sort by solution order


    indent = '    ' # 4 spaces

    funcname = 'ikin_'+fixed_name
    print >> f, '''
# Code to solve the unknowns '''
    print >>f, 'def', funcname +'(T):' # no indent
    print >>f, indent+'if(T.shape != (4,4)):'
    print >>f, indent*2 + 'print "bad input to "+funcname'
    print >>f, indent*2 + 'quit()'
    print >>f, '''#define the input vars
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

    '''
    print >>f, indent + 'solvable_pose = True'
    for node in nlist:  # for each solved var
        print >>f, '\n'
        print >>f, indent + '#Variable: ', str(node.symbol)
        for sol in node.solution_with_notations.values():
            if re.search('asin', str(sol.RHS)) or re.search('acos', str(sol.RHS)):
                #print '  Found asin/acos solution ...', sol.LHS , ' "=" ',sol.RHS
                tmp = re.search('\((.*)\)',str(sol.RHS))
                print >>f, indent + 'if (solvable_pose and abs', tmp.group(0), ' > 1):'
                print >>f, indent*2 + 'solvable_pose = False'
                print >>f, indent + 'else:'
                tmp = str(sol.LHS) + ' = ' + str(sol.RHS)
                print >>f, indent*2 + tmp
            if re.search('atan', str(sol.RHS)):
                print '  Found atan2 solution ...', sol.LHS , ' "=" ',sol.RHS
                tmp = re.search('\((.*)\)',str(sol.RHS))
                tmp = str(sol.LHS) + ' = ' + str(sol.RHS)
                print >>f, indent + tmp
            if node.solvemethod == 'algebra':
                print '  Found algebra solution ... ' , sol.LHS , ' = ', sol.RHS
                print >>f, indent + str(sol.LHS) + ' = ' + str(sol.RHS)

    print >> f, '''
##################################
#
#package the solutions into a list for each set
#
###################################
'''


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
            print 'g: ', g, 't: ', t
            gs.append(str(t))

        #print >>f, gs.sort # in place
        grp_lists.append(gs)

    print >>f, indent +  'solution_list = []'
    for g in grp_lists:
        g.sort()
        print >>f, indent + '#(note trailing commas allowed in python'
        print >>f, indent +  'solution_list.append( [ ',
        for v in g:
            print >>f,  v + ', ',
        print >>f, '] )'


    # we are done.   Return
    print >>f, indent + 'if(solvable_pose):'
    print >>f, indent*2 + 'return(solution_list)'
    print >>f, indent + 'else: '
    print >>f, indent*2 + 'return(False)'

    # __main__()  code for testing:
    print >>f, '''

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


    '''


    f.close()
