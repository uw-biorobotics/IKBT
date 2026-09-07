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
import ikbtbasics.numeric_ik as nik   # joint_symbols():  joints in chain order
from ikbtbasics.ik_classes import *     # special classes for Inverse kinematics in sympy
#

import re


#  Console chatter from the generator:  the solution equation it is about to
#  emit, and the joint-variable scan.  Off by default -- it is one line per
#  version per variable, which on a 6-DOF arm buries the solve's own output.
#  Set True (or -v on the self-test) to get it back.
VERBOSE = False


def _say(*args):
    if VERBOSE:
        print(*args)


def py_identifier(name):
    """A robot name turned into a valid Python identifier.

       NOT the same string as the LaTeX label.  output_latex needs '_' escaped
       as '\\_';  Python needs the opposite -- underscores are legal in an
       identifier and backslashes are not.  Reusing the LaTeX-escaped name here
       emitted

           def ikin_Chair\\_Helper(T):
           SyntaxError: unexpected character after line continuation character

       for every robot whose name contains '_' or '-':  Chair_Helper,
       ICP5p5_A21, Arm_3, Raven-II.  It is fatal for the hybrid branch, whose
       derived arms are named by the edit they encode and therefore ALWAYS
       contain underscores (KinovaLite_d_5_0).
    """

    ident = re.sub(r'\W', '_', str(name))
    if ident and ident[0].isdigit():
        ident = '_' + ident          # an identifier may not start with a digit
    return ident


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
    print('\n\n\n                       Starting FK Python Output work \n\n\n')

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
    #  Mech.params, NOT Robot.params.  Robot.params holds what ik_robots.py
    #  DECLARED;  Mech.params also holds what forward_kinematics() had to
    #  INVENT -- the ca_i / sa_i for a twist angle that is not a multiple of 90
    #  degrees.  Those appear in the equations below, so a module that declares
    #  only the first list emits code referencing an undefined name:  measured
    #  on Craig417, `NameError: name 'sa2' is not defined` on the first call to
    #  ikin_Craig417().  (Pre-existing;  found 2026-09-03.)
    decl_params = list(getattr(Robot.Mech, 'params', None) or Robot.params)
    if(Robot.Mech.pvals != {}):  # if we have numerical values stored
        for p in decl_params:
            val = str(Robot.Mech.pvals[p])
            tmp += str(p) + ' = ' + val + '\n'
    else:                        # no stored numerical values
        for p in decl_params:
            tmp += str(p) + ' = XXXXX    # deliberate undeclared error!  USER needs to give numerical value \n'
    par_decl_str = tmp

    print(par_decl_str, file=f)


    # joint variables:
    _say('Debug: joint variables: ', Robot.variables)

    print('#\n#     Robot Joint Variables \n#',file=f)

    for v in Robot.variables:
        ss = str(v).split('_')   # get joint subscript(s)
        _say('Variabl: SofA: ', str(v), ss)
        if len(ss) > 1:
            if len(ss[1]) > 1:  # we have a sum_of_angles
                _say('Sum of ang found in variable: ', str(v))
                subs = [*ss[1]] # make list of
                soa = str(v) + ' = ' # e.g. 'th_23 = '
                for s in subs:
                    _say('Debug subscript s:',ss, s)
                    # find var with this subscript:
                    for v1 in Robot.variables:
                        if s in str(v1) and len(str(v1).split('_')[1]) == 1: # avoid soa subscripts
                            soa += f' {str(v1)} +'
                print(indent + soa[:-1], file=f)
            else:
                print(indent + f'{str(v)} = 1.0   # 1.0= dummy value',file=f)

    funcname = 'Fkin_' + py_identifier(orig_name)
    print('''
# Code to compute Forward Kinematics ''', file=f)

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
    print('\n\n\n                       Starting IK Python Output work \n\n\n')

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
    #  Mech.params, NOT Robot.params.  Robot.params holds what ik_robots.py
    #  DECLARED;  Mech.params also holds what forward_kinematics() had to
    #  INVENT -- the ca_i / sa_i for a twist angle that is not a multiple of 90
    #  degrees.  Those appear in the equations below, so a module that declares
    #  only the first list emits code referencing an undefined name:  measured
    #  on Craig417, `NameError: name 'sa2' is not defined` on the first call to
    #  ikin_Craig417().  (Pre-existing;  found 2026-09-03.)
    decl_params = list(getattr(Robot.Mech, 'params', None) or Robot.params)
    if(Robot.Mech.pvals != {}):  # if we have numerical values stored
        for p in decl_params:
            val = str(Robot.Mech.pvals[p])
            tmp += str(p) + ' = ' + val + '\n'
    else:                        # no stored numerical values
        for p in decl_params:
            tmp += str(p) + ' = XXXXX    # deliberate undeclared error!  USER needs to give numerical value \n'
    par_decl_str = tmp


    nlist = Robot.solution_nodes


    indent = '    ' # 4 spaces

    funcname = 'ikin_' + py_identifier(orig_name)

    #  MODULE LEVEL, and before the def:  printed inside the function body at
    #  column 0 they close the function early and make the next indented line
    #  an IndentationError.  At module level they are also inspectable and
    #  overridable by a caller.
    print('#  Declare the parameters (link lengths etc.)', file=f)
    print(par_decl_str, file=f)

    #####################################################################
    #
    #   THE RETURN CONTRACT.
    #
    #   ikin_*() returns JOINTS ONLY, in CHAIN order, with the names emitted
    #   alongside.  The sum-of-angle variables are still computed -- later
    #   solutions depend on them -- but they are intermediates, not joints, so
    #   they are not returned.
    #
    #   Chain order comes from the DH table via numeric_ik.joint_symbols():
    #   NOT from the unknown list, which is extended with SOA variables, and
    #   NOT from solve order, which is an artifact of how the tree happened to
    #   solve this particular arm.
    #
    #####################################################################
    jnames = [str(s) for s in nik.joint_symbols(Robot.Mech)]
    order  = [nd.unknown.name for nd in Robot.solution_nodes]   # column order
    joint_cols = [j for j in jnames if j in order]
    unsolved   = [j for j in jnames if j not in order]
    aux_cols   = [nm for nm in order if nm not in jnames]

    print('#  Joint values returned by %s(), in this order:' % funcname, file=f)
    print('JOINT_NAMES = %r' % joint_cols, file=f)
    print('#  Sum-of-angle intermediates:  computed, but NOT returned', file=f)
    print('AUX_NAMES   = %r' % aux_cols, file=f)
    if unsolved:
        print('#  WARNING:  these joints were NOT solved, so they are absent',
              file=f)
        print('#            from every returned branch:  %r' % unsolved, file=f)
    print('', file=f)

    print('''
# Auto Generated Code to solve the unknowns
#        parameter:  T   4x4 numerical target for T06
#
''', file=f)
    print('def', funcname +'(T):', file=f) # no indent
    print(indent+'if(T.shape != (4,4)):', file=f)
    #  funcname is a variable HERE, not in the generated module -- emitting
    #  it bare produced `print("bad input to "+funcname)`, a NameError the
    #  moment anyone passed a wrongly-shaped T.  Bake the name in as a literal.
    print(indent*2 + 'print ( "bad input to %s" )' % funcname, file=f)
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
        colindex = node.unknown.solveorder-1  # select the unknown
        #  ONE assignment per DISTINCT version.  A variable solved early shares
        #  its versions between matrix rows (Puma's th_1:  2 versions, 8 rows),
        #  so walking the rows emitted the same line four times.  Dedup on the
        #  LHS in first-seen order -- set() would reorder, and unstably.
        eqnlist = []
        seen = set()
        for rowindex in range(Robot.nversions):
            e = Robot.FinalEqnMatrix[rowindex][colindex]
            if str(e.LHS) in seen:
                continue
            seen.add(str(e.LHS))
            eqnlist.append(e)

        for solEqnVer in eqnlist: # go through the versions
            _say('Python Output: Solution Equation Version: ', solEqnVer)
            if re.search('asin', str(solEqnVer.RHS)) or re.search('acos', str(solEqnVer.RHS)):
                _say('  Found asin/acos solution ...', solEqnVer.LHS , ' "=" ',solEqnVer.RHS)
                tmp = re.search('\((.*)\)',str(solEqnVer.RHS))
                print(indent + 'if (solvable_pose and abs', tmp.group(0), ' > 1):', file=f)
                print(indent*2 + 'solvable_pose = False', file=f)
                print(indent + 'else:', file=f)
                tmp = str(solEqnVer.LHS) + ' = ' + str(solEqnVer.RHS)
                print(indent*2 + tmp, file=f)
            if re.search('atan', str(solEqnVer.RHS)):
                _say('  Found atan2 solution ...', solEqnVer.LHS , ' "=" ',solEqnVer.RHS)
                tmp = re.search('\((.*)\)',str(solEqnVer.RHS))
                tmp = str(solEqnVer.LHS) + ' = ' + str(solEqnVer.RHS)
                print(indent + tmp, file=f)
            if node.solvemethod == 'algebra':
                _say('  Found algebra solution ... ' , solEqnVer.LHS , ' = ', solEqnVer.RHS)
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

    #  Rows come from Robot.solListMatrix:  an ORDERED list of lists whose
    #  columns are in solve order.  `groups` is the same data as a SET of
    #  tuples, so iterating it orders the branches by string hashing -- fine
    #  for a set, wrong for generated source that ought to be diffable.  The
    #  set is kept as a fallback for a caller that has only that.
    rows = getattr(Robot, 'solListMatrix', None)
    if not rows:
        rows = [list(g) for g in sorted(groups)]

    print(indent + 'solution_list = []', file=f)
    print(indent + '#  each row is one solution branch, in JOINT_NAMES order',
          file=f)
    for row in rows:
        vals = [row[order.index(j)] for j in joint_cols]
        print(indent + 'solution_list.append( [ ' + ', '.join(vals) + ' ] )',
              file=f)


    # we are done.   Return
    print(indent + 'if(solvable_pose):', file=f)
    print(indent*2 + 'return(solution_list)', file=f)
    print(indent + 'else: ', file=f)
    print(indent*2 + 'return(False)', file=f)

    #  A labelled view of the same answer, for callers who would rather
    #  not index by position.  The numeric list stays the fast path.
    print('''

#
#   The same solutions, keyed by joint name.
#
def ''' + funcname + '''_labeled(T):
    sols = ''' + funcname + '''(T)
    if sols is False:
        return False
    return [dict(zip(JOINT_NAMES, s)) for s in sols]
''', file=f)

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

    #  NOTE, all three fixed here:  the generated __main__ used PYTHON 2 print
    #  STATEMENTS (`print ''`), which made the module unimportable under
    #  python3 -- "SyntaxError: Missing parentheses in call to 'print'".  It
    #  also bound the result to `list`, shadowing the builtin, and iterated it
    #  without checking for the False that ikin_*() returns on an unreachable
    #  pose, giving "TypeError: bool is not iterable" instead of saying so.
    # try the Puma IK

    sols = ''' + funcname + '''(T1)

    if sols is False:
        print('  no solution:  that pose is not reachable by this arm')
    else:
        print('  joint order: ', JOINT_NAMES)
        for i, sol in enumerate(sols):
            print('')
            print('Solution ', i)
            print(dict(zip(JOINT_NAMES, sol)))


    ''', file=f)


    f.close()

    print('\n\n\n                       End of Python Output work \n\n\n')
