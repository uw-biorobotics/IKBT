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


class acos_dc(sp.Function):
    """Marker for a DOMAIN-CHECKED arccosine in the generated python."""


class asin_dc(sp.Function):
    """Marker for a DOMAIN-CHECKED arcsine in the generated python."""


class sqrt_dc(sp.Function):
    """Marker for a DOMAIN-CHECKED square root in the generated python."""


def dc_rewrite(e):
    r"""One expression with every acos/asin swapped for its checked twin.

       THE GUARD GOES AT THE POINT OF USE, which is the only place it can be
       written down wrong-proof.  What this replaced hoisted the guard out:
       it dug the argument out of the RHS and emitted

           if (solvable_pose and abs(<argument>) > 1):
               solvable_pose = False
           else:
               th_5v1 = acos(<argument>)

       and getting `<argument>` right is then a separate problem that can be
       got wrong -- and was.  It was pulled out of the PRINTED RHS with a
       greedy `re.search(r'\((.*)\)', ...)`, so for `acos(x) + atan2(y, z)`
       the test was on `(x) + atan2(y, z)`, the wrong quantity;  for a sum of
       two arcsines it tested neither.  acos was then called out of range
       anyway and the generated module RAISED.  (BH, 2026-09-29: "why not just
       write a domain-checked version of acos()".)

       There is nothing to extract here, so there is nothing to extract
       wrongly:  every arccosine is checked, however many there are and
       however deeply nested.

       THE CONTRACT IS NaN, NOT AN EXCEPTION.  An out-of-range argument means
       the posture does not exist at this pose -- data, not a fault -- and it
       propagates through the arithmetic that follows to the finiteness test
       at the end.  The C++ needs no equivalent rewrite: std::acos ALREADY
       returns NaN out of domain, where python's math.acos raises.  Two
       spellings, one contract, and scripts/cpp_expr_check asserts it.

       It also retires a whole class of failure that was not about arccosine
       at all:  a tripped guard used to leave the variable UNBOUND, so the
       next line that read it raised UnboundLocalError rather than reporting
       the pose unreachable."""

    e = sp.sympify(e).replace(sp.acos, acos_dc).replace(sp.asin, asin_dc)

    #  SQUARE ROOTS TOO, and they need matching differently:  sympy has no
    #  sqrt Function -- sqrt(x) is Pow(x, 1/2) -- so .replace(sp.sqrt, ...)
    #  matches nothing.  Only the two exponents that PRINT as a square root
    #  are rewritten;  generated code contains no other fractional power
    #  (checked across the robot set, 2026-09-29: only **2).
    half = sp.Rational(1, 2)

    def _is_root(p):
        return isinstance(p, sp.Pow) and p.exp in (half, -half)

    def _root_dc(p):
        return sqrt_dc(p.base) if p.exp == half else 1 / sqrt_dc(p.base)

    return e.replace(_is_root, _root_dc)


def py_identifier(name):
    """A robot name turned into a valid Python identifier.

       NOT the same string as the LaTeX label.  output_latex needs '_' escaped
       as '\\_';  Python needs the opposite -- underscores are legal in an
       identifier and backslashes are not.
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
from math import isfinite

pi = np.pi


#  DOMAIN-CHECKED arccosine and arcsine.
#
#  An argument outside [-1, 1] means the posture this branch describes does
#  not exist at this pose.  That is DATA, not a fault:  the solver enumerates
#  combinations of each unknown's branches without discarding the ones the
#  arm cannot adopt, and on the hybrid path the closed form belongs to a
#  SIMPLIFIED arm that genuinely cannot reach every pose the true one can.
#
#  So they return NaN and let it propagate to the finiteness test at the end,
#  rather than raising.  math.acos raises;  C++'s std::acos already returns
#  NaN, which is why the generated C++ needs no equivalent wrapper.
def acos_dc(x):
    return acos(x) if -1.0 <= x <= 1.0 else float('nan')


def asin_dc(x):
    return asin(x) if -1.0 <= x <= 1.0 else float('nan')


#  Same story:  math.sqrt raises on a negative, C++'s std::sqrt returns NaN.
#  A negative discriminant means this branch has no solution here.
def sqrt_dc(x):
    return sqrt(x) if x >= 0.0 else float('nan')

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
from math import isfinite

pi = np.pi


#  DOMAIN-CHECKED arccosine and arcsine.
#
#  An argument outside [-1, 1] means the posture this branch describes does
#  not exist at this pose.  That is DATA, not a fault:  the solver enumerates
#  combinations of each unknown's branches without discarding the ones the
#  arm cannot adopt, and on the hybrid path the closed form belongs to a
#  SIMPLIFIED arm that genuinely cannot reach every pose the true one can.
#
#  So they return NaN and let it propagate to the finiteness test at the end,
#  rather than raising.  math.acos raises;  C++'s std::acos already returns
#  NaN, which is why the generated C++ needs no equivalent wrapper.
def acos_dc(x):
    return acos(x) if -1.0 <= x <= 1.0 else float('nan')


def asin_dc(x):
    return asin(x) if -1.0 <= x <= 1.0 else float('nan')


#  Same story:  math.sqrt raises on a negative, C++'s std::sqrt returns NaN.
#  A negative discriminant means this branch has no solution here.
def sqrt_dc(x):
    return sqrt(x) if x >= 0.0 else float('nan')

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
    #  degrees.
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


    #  NO RECOMPUTE.  T_06 is already on the mechanism -- kinematics_pickle()
    #  ran forward_kinematics() before this and the pickle carries the result.
    #  This line used to call forward_kinematics() again and drop the return
    #  value on the floor (`Fkeqns` was never read): 2.0 s of symbolic FK per
    #  report on Puma, for nothing.  (Measured 2026-09-27.)
    Tfk = Robot.Mech.T_06

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

def output_python_code(Robot, groups, known=None):
    """Write the generated python IK for `Robot`.

       Output filename convention:

       known=None       CodeGen/Python/IK_equations<Robot>.py, ikin_<Robot>(T)
                        -- an unconditional closed form
       known='th_2'     CodeGen/Python/IK_conditional<Robot>.py,
                        ikin_<Robot>_given(T, th_2) -- the ONE-VARIABLE branch's
                        closed form, valid only where th_2 is right
"""

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
from math import isfinite

pi = np.pi


#  DOMAIN-CHECKED arccosine and arcsine.
#
#  An argument outside [-1, 1] means the posture this branch describes does
#  not exist at this pose.  That is DATA, not a fault:  the solver enumerates
#  combinations of each unknown's branches without discarding the ones the
#  arm cannot adopt, and on the hybrid path the closed form belongs to a
#  SIMPLIFIED arm that genuinely cannot reach every pose the true one can.
#
#  So they return NaN and let it propagate to the finiteness test at the end,
#  rather than raising.  math.acos raises;  C++'s std::acos already returns
#  NaN, which is why the generated C++ needs no equivalent wrapper.
def acos_dc(x):
    return acos(x) if -1.0 <= x <= 1.0 else float('nan')


def asin_dc(x):
    return asin(x) if -1.0 <= x <= 1.0 else float('nan')


#  Same story:  math.sqrt raises on a negative, C++'s std::sqrt returns NaN.
#  A negative discriminant means this branch has no solution here.
def sqrt_dc(x):
    return sqrt(x) if x >= 0.0 else float('nan')

'''
    fixed_name = Robot.name.replace(r'_', r'\_')  # this is for LaTex output
    fixed_name = fixed_name.replace('test: ','')
    orig_name  = Robot.name.replace('test: ', '')

    DirName = 'CodeGen/Python/'
    fname = DirName + ('IK_conditional' if known else 'IK_equations') + orig_name + '.py'
    f = open(fname, 'w')

    importString = importString.replace('**Robot**', Robot.name)
    print(importString, file=f)

    # parameter Declarations (a_3, d_5, etc).
    tmp = '\n'
    #  Mech.params, NOT Robot.params.  Robot.params holds what ik_robots.py
    #  DECLARED;  Mech.params also holds what forward_kinematics() had to
    #  INVENT -- the ca_i / sa_i for a twist angle that is not a multiple of 90
    #  degrees.
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

    funcname = 'ikin_' + py_identifier(orig_name) + ('_given' if known else '')

    #  The argument list, and the one extra name the function body may use.
    arglist = 'T' + (', ' + known if known else '')

    #  MODULE LEVEL, and before the def:  printed inside the function body at
    #  column 0 they close the function early and make the next indented line
    #  an IndentationError.  At module level they are also inspectable and
    #  overridable by a caller.
    print('#  Declare the parameters (link lengths etc.)', file=f)
    print(par_decl_str, file=f)

    #
    #   THE RETURN CONTRACT.
    #
    #   ikin_*() returns JOINTS ONLY, in CHAIN order, with the names emitted
    #   alongside.  The sum-of-angle variables are still computed -- later
    #   solutions depend on them -- but they are intermediates, not joints, so
    #   they are not returned.
    jnames = [str(s) for s in nik.joint_symbols(Robot.Mech)]
    order  = [nd.unknown.name for nd in Robot.solution_nodes]   # column order
    #  The assumed-known joint was never solved, so it is not in `order` -- but
    #  it IS known, by assumption, and leaving it out would return a joint
    #  vector with a hole in it.  It is a column like any other;  its value is
    #  the argument.
    joint_cols = [j for j in jnames if j in order or j == known]
    unsolved   = [j for j in jnames if j not in order and j != known]
    aux_cols   = [nm for nm in order if nm not in jnames]

    if known:
        print('#  EVERY branch is returned, in a FIXED position, whether or', file=f)
        print('#  not it exists at this pose -- a branch that does not comes', file=f)
        print('#  back with NaN in it.  IK_onevar%s.py indexes branches by' % orig_name, file=f)
        print('#  position, so row i must be the same branch at every value.', file=f)
    else:
        print('#  Only the branches that EXIST at the goal pose are returned,', file=f)
        print('#  so the count varies with the pose.  False means none do.', file=f)
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
    if known:
        print('#  CONDITIONAL.  %s is an INPUT, not an output:  these equations'
              % known, file=f)
        print('#  hold only where its value is right.  IK_onevar%s.py searches'
              % orig_name, file=f)
        print('#  for the values that are, and is what you should normally call.',
              file=f)
        print('KNOWN_VARIABLE = %r' % known, file=f)
        print('', file=f)
    print('def', funcname + '(%s):' % arglist, file=f) # no indent
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
    


    #  SILENT ON THE CONDITIONAL PATH.  These three lines are advice for a
    #  human calling ikin once.
    if not known:
        print(indent + 'print ( " Caution - this code has no solution checking.")', file=f)
        print(indent + 'print ("in case of domain errors, change the test position / orientation ")', file=f)
        print(indent + 'print ( " to a pose reachable by your specific robot")', file=f)
    print(indent + '', file=f)
    print(indent + '''
#############################################################
#
#   REACHABILITY is decided at the END, from the answer itself.
#
#   Every arccosine and arcsine below goes through acos_dc / asin_dc, which
#   return NaN rather than raising when their argument leaves [-1, 1].  A NaN
#   propagates through the arithmetic that follows, so a posture this arm
#   cannot adopt at this pose arrives at the bottom of this function as a
#   non-finite number and is recognised there.
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
        #  LHS in first-seen order.
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
            rhs = solEqnVer.RHS

            #  ONE PLAIN ASSIGNMENT PER VERSION.  The arccosine domain test is
            #  inside acos_dc(), at the point of use -- see dc_rewrite().
            #  This used to be three independent `if`s emitting a hoisted
            #  guard, and a RHS holding both an asin and an atan2 got the
            #  guarded assignment AND an unguarded one, the unguarded one
            #  winning.
            has_dc = bool(sp.sympify(rhs).atoms(sp.asin, sp.acos))
            if has_dc or re.search('atan', str(rhs)) \
                    or node.solvemethod == 'algebra':
                #  WHICH versions get an assignment is unchanged.
                _say('  emit ', solEqnVer.LHS, ' = ', rhs)
                print(indent + str(solEqnVer.LHS) + ' = '
                      + str(dc_rewrite(rhs)), file=f)

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
    #  columns are in solve order.
    rows = getattr(Robot, 'solListMatrix', None)
    if not rows:
        rows = [list(g) for g in sorted(groups)]

    print(indent + 'solution_list = []', file=f)
    print(indent + '#  each row is one solution branch, in JOINT_NAMES order',
          file=f)
    for row in rows:
        #  `known` has no column in the solution matrix -- nothing solved it --
        #  so its cell is the argument's own name.
        vals = [known if j == known else row[order.index(j)] for j in joint_cols]
        print(indent + 'solution_list.append( [ ' + ', '.join(vals) + ' ] )',
              file=f)


    # we are done.   Return
    #
    #   THE TWO ENTRY POINTS DIFFER HERE, DELIBERATELY.
    #
    #   A row holding a non-finite value is a posture that does not exist at
    #   this pose -- an arccosine out of range, a negative discriminant.  What
    #   to do with it depends on who is asking.
    #
    if not known:
        #   THE UNCONDITIONAL FORM DROPS THEM.  A caller wants the postures
        #   the arm can actually adopt, and the count legitimately varies with
        #   the pose.  This used to discard ALL branches whenever ANY one of
        #   them was non-finite, which threw away exact answers: KR16 at the
        #   probe pose has four postures that reproduce it to 5.6e-16 and four
        #   that do not exist, and reported "unreachable" for the lot.
        print(indent + '#  Keep the postures that exist.  A row with a', file=f)
        print(indent + '#  non-finite value is one that does not exist at this', file=f)
        print(indent + '#  pose -- an arccosine out of range, or a negative', file=f)
        print(indent + '#  discriminant -- and the count varies with the pose.', file=f)
        print(indent + 'reachable = [r for r in solution_list', file=f)
        print(indent + '             if all(isfinite(v) for v in r)]', file=f)
        print(indent + 'if not reachable:', file=f)
        print(indent*2 + 'return(False)     #  no posture at all reaches T', file=f)
        print(indent + 'return(reachable)', file=f)
    else:
        #   THE CONDITIONAL FORM KEEPS THEM, IN PLACE.  IK_onevar<Robot>.py
        #   calls this at hundreds of values of the assumed variable and
        #   builds ONE ERROR CURVE PER BRANCH, indexing by POSITION -- so
        #   row i must be the same branch at every value.  Dropping a row at
        #   some values and not others would stitch one branch's error onto
        #   another's curve and wreck the per-branch domain edges the search
        #   hunts roots at.  The caller sees the NaN and reads it as "this
        #   branch is undefined here", which is exactly what errors_*() does.
        print(indent + '#  EVERY row, in a FIXED position, NaN and all.  The', file=f)
        print(indent + '#  1-D search indexes branches by position and needs', file=f)
        print(indent + '#  row i to be the same branch at every value of', file=f)
        print(indent + '#  %s;  it reads a non-finite row as "this branch' % known, file=f)
        print(indent + '#  is undefined here".', file=f)
        print(indent + 'return(solution_list)', file=f)

    #  A labelled view of the same answer, for callers who would rather
    #  not index by position.  The numeric list stays the fast path.
    print('''

#
#   The same solutions, keyed by joint name.
#
def ''' + funcname + '''_labeled(''' + arglist + '''):
    sols = ''' + funcname + '''(''' + arglist + ''')
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

    sols = ''' + funcname + '''(''' + ('T1' if not known else 'T1, 0.3') + ''')

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
