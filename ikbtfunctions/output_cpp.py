#!/usr/bin/python
#
#   Generate C++ output code of the IK solution
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
from sympy.printing.tree import  pprint_nodes, print_tree
#import numpy as np
from ikbtbasics.kin_cl import *
from ikbtfunctions.helperfunctions import *
import ikbtbasics.numeric_ik as nik   # joint_symbols(): chain order
from ikbtbasics.ik_classes import *     # special classes for Inverse kinematics in sympy
#
import pickle     # for storing pre-computed FK eqns

class cpp_output:
    def init(self):
        self.f = None     #file ptr
        self.level = 0
        self.indent = '    '

    def line(self,str):
        lines = str.split('\n')
        for l in lines:
            # correct exponential operators for squaring:   x**2 --> x*x
            #     (** is not valid C++)
            if re.search(r'\*\*', l):
                print('fixing exponential ** notation')
                r = re.compile(r'(\w+)\*\*2')
                s2 = r.sub(r'\1*\1',l)
                r = re.compile(r'([a-z]{3}\([^\)]+?\))\*\*2')  #  sin(x)**2 or cos(x)**2
                l = r.sub(r'\1*\1', s2)  # substitute --> x*x
            print(self.indent*self.level + l, file=self.f)

    def push(self):
        self.level += 1
        print(self.indent*self.level + '{', file=self.f)
    def pop(self):
        print(self.indent*self.level + '}', file=self.f)
        self.level -= 1

def output_cpp_code(Robot, solution_groups):

    fixed_name = Robot.name.replace(r'_', r'\_')  # this is for LaTex output
    fixed_name = fixed_name.replace('test: ','')
    orig_name  = Robot.name.replace('test: ', '')

    #  THE RETURN CONTRACT -- see the long note in output_python.py.  Joints
    #  only, in DH chain order;  the sum-of-angle variables are computed but
    #  not returned.  Two C-specific reasons this matters more here than in
    #  Python:  solution_list was declared [64][6] while a 6-DOF arm with one
    #  SOA variable wrote SEVEN columns, which is a buffer overrun, and C has
    #  no way to hand back a name with a value, so the order IS the contract.
    jnames = [str(s) for s in nik.joint_symbols(Robot.Mech)]
    order  = [nd.unknown.name for nd in Robot.solution_nodes]
    joint_cols = [j for j in jnames if j in order]
    unsolved   = [j for j in jnames if j not in order]
    aux_cols   = [nm for nm in order if nm not in jnames]

    rows = getattr(Robot, 'solListMatrix', None)
    if not rows:
        rows = [list(g) for g in sorted(solution_groups)]

    n_joints   = max(1, len(joint_cols))
    n_branches = max(1, len(rows))

    c = cpp_output()

    DirName = 'CodeGen/Cpp/'
    fname = DirName + 'IK_equations'+orig_name+'.cpp'
    f = open(fname, 'w')
    c.f = f
    c.level = 0
    c.indent = '    '

    c.line('''//
//  C++ inverse kinematic equations for ''' + fixed_name + '''


#include <math.h>
#include <stdio.h>
#include <iostream>

double pi = 3.1415926;

//  Sizes of the answer.  These were hardcoded [64][6];  an arm whose solution
//  used a sum-of-angles variable wrote a seventh column into a row of six.
#define IK_NJOINTS   ''' + str(n_joints) + '''
#define IK_NBRANCHES ''' + str(n_branches) + '''

//  Column order of every solution row (joints only, DH chain order):
//      ''' + ', '.join(joint_cols) + '''
//  Computed but NOT returned (sum-of-angle intermediates):
//      ''' + (', '.join(aux_cols) if aux_cols else '(none)') + '''

// ikin() modifies solution_list in-place and
// returns 1 for valid solutions and 0 for no solutions
int ikin(double T[4][4], double solution_list[IK_NBRANCHES][IK_NJOINTS]);

''')
    if unsolved:
        c.line('//  WARNING: these joints were NOT solved and are absent from')
        c.line('//           every row:  ' + ', '.join(unsolved))

    ###################
    #   Variable and parameter declarations

    sol_vars = set() #all variables used in solutions
    # now handled outside, in ikSolver
    #solution_groups = mtch.matching_func(Robot.notation_collections, Robot.solution_nodes)
    for s in solution_groups:
        sol_vars.update(s)

    #  Variable Declarations
    tmp = 'double'
    for v in sol_vars:
        tmp += ' ' + str(v) + ','

    var_decl_str = tmp[:-1] + ';'   # drop trailing comma and end the line

    # parameter Declarations
    tmp = '\n'
    #  Mech.params, not Robot.params -- same reason as output_python.py:  it
    #  also carries the ca_i / sa_i that forward_kinematics() invented for a
    #  twist angle that is not a multiple of 90 degrees, and those appear in the
    #  equations.  Here the omission is a COMPILE error rather than a runtime
    #  one, since C++ needs every name declared.
    for p in (getattr(Robot.Mech, 'params', None) or Robot.params):
        tmp += 'double '+str(p) + ' = XXXXX ; \\ deliberate undeclared error!  USER needs to give numerical value\n'
    par_decl_str = tmp

    nlist = Robot.solution_nodes

    indent = '    ' # 4 spaces

    funcname = 'ikin'
    c.line('int main()')
    c.push()
    c.line('double T[4][4] = { {1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}, };' )
    c.line('double sol_list[IK_NBRANCHES][IK_NJOINTS] = {0};  // list of solutions')
    c.line('if('+funcname+'(T, sol_list))')
    c.push()
    c.line('std::cout << sol_list;')
    c.pop()
    c.line('else')
    c.push()
    c.line('std::cout <<  "No valid solution" ;')
    c.pop()
    c.pop()
    # end of main

    # ik function
    c.line('\n// Code to solve the unknowns ')
    c.line('\n\n// Declarations')
    c.line('int ' + funcname
           + '(double T[4][4], double solution_list[IK_NBRANCHES][IK_NJOINTS] )')  # no indent
    c.push()
    c.line('''\n//define the input vars
double r_11 = T[0][0];
double r_12 = T[0][1];
double r_13 = T[0][2];
double r_21 = T[1][0];
double r_22 = T[1][1];
double r_23 = T[1][2];
double r_31 = T[2][0];
double r_32 = T[2][1];
double r_33 = T[2][2];
double Px = T[0][3];
double Py = T[1][3];
double Pz = T[2][3];

double argument;

//
// Caution:    Generated code is not yet validated
//

int True = 1;
int False = 0;
''')
    c.line('int solvable_pose = True;')
    c.line('// declare variables in solutions')
    c.line(var_decl_str)
    c.line('')
    c.line('// declare constant parameters (note they will need values!)')
    c.line(par_decl_str)
    for node in nlist:  # for each solved var
        c.line('\n\n')

        c.line('//Variable: '+str(node.symbol))
        solno = 0
        nsolns = node.unknown.nsolutions
        nvers  = Robot.nversions
        print('Cpp Output Gen: ', node, ' has ', nsolns, ' solutions and ',nvers, ' versions')
        # go through the final matrix of equation versions
        colindex = node.unknown.solveorder-1  # select the unknown
        #  ONE assignment per DISTINCT version:  a variable solved early shares
        #  its versions between rows, so walking the rows emitted the same
        #  statement several times.  First-seen order, not set() order.
        eqnlist = []
        seen = set()
        for rowindex in range(nvers):
            e = Robot.FinalEqnMatrix[rowindex][colindex]
            if str(e.LHS) in seen:
                continue
            seen.add(str(e.LHS))
            eqnlist.append(e)

        for solEqnVer in eqnlist: # go through the versions
            print('Cpp Output Gen: Solution Equation Version: ', solEqnVer)
            c.line('\n// solution '+str(solno))
            solno += 1
            solrhs = str(solEqnVer.RHS)
            # detect arcsin() or arccos()
            #
            #  BH   21-Sept
            #  This code needs a serious refactor.  When solution is for example:
            #    th5_ijk = arcsin(x) + atan2(y1,x1)
            #       (here 'x' is a complicated expression)
            #  then it is important that the generated code will test  -1 < x < +1
            #  if this test fails the pose is unreachable and arcsin fails.
            #
            #
            trig = False
            print('\n\nCpp Output Gen: Studying node: ', node.symbol, ' solution ', solno)
            print('solvemethod: ', node.solvemethod)
            print(' LHS: ', str(solEqnVer.LHS))
            print(' RHS: ', solrhs)
            print('argument: ', node.argument)
            print('\n\n')

            c.line('// solvemethod: ' + node.solvemethod )
            c.line('//    argument: ' + str(node.argument )  )

            if(node.solvemethod == 'arcsin' or node.solvemethod == 'arccos'):
               trig = True

            if(trig):
               print('  Found asin/acos solution ...', solEqnVer.LHS , ' "=" ',solEqnVer.RHS)
               c.line('// Arcsin() or Arccos() based solution:')
               c.line('argument = ' + str(node.argument))
               c.line('if (solvable_pose && fabs(argument) > 1)')
               c.push()
               c.line('solvable_pose = False; ')
               c.pop()
               c.line('else if (solvable_pose)')
               c.push()        #  bugus    (here)
               c.line(str(solEqnVer.LHS) + ' = '+node.solvemethod+'(argument);' )
               c.pop()

            if ((not trig) and 'atan2(y,x)' in node.solvemethod):
                c.line(str(solEqnVer.LHS) + ' = ' + solrhs + ';')

            if 'algebra' in node.solvemethod:
                c.line(str(solEqnVer.LHS) + ' = ' + solrhs + ';')

            if 'x2z2' in node.solvemethod:
                print('x2z2 output: ', node.argument)
                c.line(str(solEqnVer.LHS) + ' = ' + solrhs + ';')



            if 'simultaneous eqn' in node.solvemethod:
                print('x2z2 output: ', node.argument)
                c.line(str(solEqnVer.LHS) + ' = ' + solrhs + ';')




    c.line('''
//##################################
//#
//#package the solutions into a list for each set
//#
//###################################
''')


    ###########################################################
    #
    #   Output of solution sets
    #
    ###########################################################


    c.line('//  one row per solution branch;  columns in IK_JOINT order:')
    c.line('//      ' + ', '.join(joint_cols))

    for i, row in enumerate(rows):
        for j, jname in enumerate(joint_cols):
            v = row[order.index(jname)]
            c.line('solution_list['+ str(i) + '][' + str(j) + '] = ' + v
                   + ';   // ' + jname)

    # we are done.   Return
    c.line('\n\n')
    c.line('// return 1 for solved, 0 for no solution')
    c.line('return(solvable_pose);')
    c.pop()

    f.close()
    print('\n\n\n                       End of Cpp Output work \n\n\n')

###################################################################
#
#    Test Code
#


#####################################################################################
# Test code below.  See sincos_solver.py for example
#
class TestSolver010(unittest.TestCase):    # change TEMPLATE to unique name (2 places)
    # def setUp(self):
        # self.DB = False  # debug flag
        #print '===============  Test updateL.py  ====================='
        # return

    def runTest(self):
        self.test_output_cpp()

    def test_output_cpp(self):
        #
        #     Set up robot equations for further solution by BT
        #
        #   Check for a pickle file of pre-computed Mech object. If the pickle
        #       file is not there, compute the kinematic equations
        ####  Using PUMA 560 also tests scan_for_equations() and sum_of_angles_transform()  in ik_classes.py
        #
        #   The famous Puma 560  (solved in Craig)
        #

        # 1)   Read the test pickle for PUMA equations
        test_pickle_dir = 'Test_pickles/'
        name = test_pickle_dir + 'Puma' + 'test_pickle.p'
        try:
            with open(name, 'r') as pick:
                print('\nReading pre-computed forward kinematics TEST info\n')
                [R, unks]  = pickle.load(pick)
        except:
            print('\n\n\n        Testing:  Failed to find data pickle file ... quitting()    \n\n\n')
            # if the test pickle is missing: edit ikSolver.py
            #    line 32: TEST_DATA_GENERATION = True
            #    > python ikSolver.py Puma
            #    (change line 32 back to False)

            quit()
        # 2)   call the function output_cpp_code(R)

        #output_latex_solution(R,unks)

        output_cpp_code(R)

        # 3)   assertions

        print('cpp output file completed')

        #    3.1)   Open CPP output file
        #    3.2)   use assertions to check some lines.


#
#    Can run your test from command line by invoking this file
#
#      - or - call your TestSolverTEMPLATE()  from elsewhere
#

#def run_test():
    #print '\n\n===============  Test output_cpp_code() ====================='
    #testsuite = unittest.TestLoader().loadTestsFromTestCase(TestSolver010)  # replace TEMPLATE
    #unittest.TextTestRunner(verbosity=2).run(testsuite)

if __name__ == "__main__":

    print('\n\n===============  Test output_cpp_code() =====================')
    testsuite = unittest.TestLoader().loadTestsFromTestCase(TestSolver010)  # replace TEMPLATE
    unittest.TextTestRunner(verbosity=2).run(testsuite)
    #unittest.main()






