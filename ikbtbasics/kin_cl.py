#!/usr/bin/python
#

# Copyright 2017 University of Washington

# Developed by Dianmu Zhang and Blake Hannaford
# BioRobotics Lab, University of Washington

# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
import unittest
import re   # regular expressions for output conversions
import sympy as sp
import numpy as np

from pykinsym import *

import ikbtfunctions.helperfunctions as hf


#
#  Configure here for speed if jacobian is not needed
JACOBIAN = True       #  False = disable velocity calculations for FK speed

######################################################################
#
#   Python class for symbolic analysis of serial mechanisms
#        Revision 1      4-Feb-2016
#        Revision 2  1-Apr-2016
#        Revision 3  14-Mar-2017  (add kequation and eqn_set)
#        Revision 4  25-May-2017  (update to unittest debugging)
#

sp.init_printing()
sp.var('x')

#  Kinematic Equation class
class kequation:
    def __init__(self,LHS=x,RHS=x):
        self.LHS = LHS
        self.RHS = RHS
        self.string = str(LHS) + ' = '+ str(RHS)

    def prt(self):
        print self.LHS, ' = ', self.RHS
    #string representation of equations (so other module can print out the equ, instead of a pointer)
    def __repr__(self):
        return "%s = %s" % (self.LHS, self.RHS)
    def __eq__(self,other):
        if other is None:
            return False
        if (self.LHS - other.LHS ==0 and self.RHS - other.RHS ==0):
            return True
        else:
            return False
    def __ne__(self,other):
        if not self.__eq__(other):
            return True
        else:
            return False
    def __hash__(self):
        return hash(str(self.LHS) + str(self.RHS))

    def LaTexOutput(self, align=False):
        tmp = sp.latex(self.RHS)
        # TODO: delete these comments or restore functionality
        #tmp = tmp.replace(r'^(.+)/(.+)', '\\frac{\1}{\2}')
        #tmp = tmp.replace('\\frac{(.+)}{(.+)}(.+)',r'\\frac{\1\3}{\2}')
        tab = ' '
        if align:
            tab = ' &'
        self.string = sp.latex(self.LHS) + tab +  '= ' + tmp

        tmp = self.string
        tmp = tmp.replace(r'th_', r'\theta_')     # change to greek theta
        tmp = re.sub(r'_(\d+)',  r'_{\1}', tmp)   # get all digits of subscript into {}
        tmp = re.sub(r'atan_\{2\}','atan2' , tmp)  # correct atan2 formatting

        return tmp

class matrix_equation:
    def __init__(self, Td=sp.zeros(4), Ts=sp.zeros(4)):
        self.Td = sp.zeros(4)  # LHS (T desired)
        self.Ts = sp.zeros(4)  # RHS (T symbolic)
        ## init 5x5 matrix of kequation() objects
        self.auxeqns = []   # aux equations such as th12 = th_1 + th_2 (!)
        for i in range(0,3):  # just first 3 rows
            for j in range(0,4):  # all 4 cols
                self.Td[i,j] = Td[i,j]
                self.Ts[i,j] = Ts[i,j]
        self.Td[3,3] = 1  # handle row 4
        self.Ts[3,3] = 1
    # put the matrix elements (Td,Ts) into a list of equations
    def get_kequation_list(self):
        list = []
        for i in range(0,3):     # only 3 rows are interesting
            for j in range(0,4):  # all 4 cols are interesting
                list.append(kequation(self.Td[i,j], self.Ts[i,j]))
        return list
    def __repr__(self):
        print '\n  - - - \n'
        sp.pprint(notation_squeeze(self.Td))
        print ' = '
        sp.pprint(notation_squeeze(self.Ts))
        return ' '

class mechanism:

    def __init__(self, dh, params, varvect):
        self.DH = dh
        self.vv = varvect
        self.params = params    # constant parameters a_4 etc
        self.pvals = {}         # dict for numerical param values
        self.jlims = np.array([
        [-np.pi, np.pi],
        [-np.pi, np.pi],
        [-np.pi, np.pi],
        [-np.pi, np.pi],
        [-np.pi, np.pi],
        [-np.pi, np.pi]
        ])
        self.jnum = np.matrix(np.zeros(36).reshape(6,6))


    ###############  compute kinematic transforms and equations for the manipulator (including Jacobian)
    def forward_kinematics(self):
        ###   set up symbolic variables
        if(JACOBIAN):
            # angular velocities of each link
            (self.w_00, self.w_11, self.w_22, self.w_33, self.w_44, self.w_55, self.w_66) =           sp.symbols(('self.w_00','self.w_11','self.w_22','self.w_33','self.w_44','self.w_55','self.w_66'))

            # linear velocities of each link
            (self.v_00, self.v_11, self.v_22, self.v_33, self.v_44, self.v_55, self.v_66) =           sp.symbols(('self.v_00','self.v_11','self.v_22','self.v_33','self.v_44','self.v_55','self.v_66'))

            # joint velocities of each link (qd stands for q-dot)
            (qd_0, qd_1, qd_2, qd_3, qd_4, qd_5, qd_6) = sp.symbols(('qd_0','qd_1','qd_2','qd_3','qd_4','qd_5','qd_6'))

        # standardize on the order "alpha N-1, a N-1, d N, theta N' for the DH table columns.
        al = 0   # Alpha_{n-1)
        a = 1    # a_{n-1}
        d = 2    # d_n
        th = 3   # th_n

        #  symbolic 4x4 transforms for each link
        self.T_01 = Link_S(self.DH[0,al], self.DH[0,a], self.DH[0,d], self.DH[0,th])
        self.T_12 = Link_S(self.DH[1,al], self.DH[1,a], self.DH[1,d], self.DH[1,th])
        self.T_23 = Link_S(self.DH[2,al], self.DH[2,a], self.DH[2,d], self.DH[2,th])
        self.T_34 = Link_S(self.DH[3,al], self.DH[3,a], self.DH[3,d], self.DH[3,th])
        self.T_45 = Link_S(self.DH[4,al], self.DH[4,a], self.DH[4,d], self.DH[4,th])
        self.T_56 = Link_S(self.DH[5,al], self.DH[5,a], self.DH[5,d], self.DH[5,th])

        #  here is the full FK derivation:
        self.T_06 = sp.trigsimp(self.T_01 * self.T_12 * self.T_23 * self.T_34 * self.T_45 * self.T_56)

        # list of T_ij matrices (used in inverse kinematics update
        self.Ts = [self.T_01, self.T_12, self.T_23, self.T_34, self.T_45, self.T_56]

        if(JACOBIAN):
            # Rotation sub matrices:
            self.R_01 = self.T_01[0:3, 0:3]
            self.R_12 = self.T_12[0:3, 0:3]
            self.R_23 = self.T_23[0:3, 0:3]
            self.R_34 = self.T_34[0:3, 0:3]
            self.R_45 = self.T_45[0:3, 0:3]
            self.R_56 = self.T_56[0:3, 0:3]

            # position offset vectors
            self.P_01 = self.T_01[0:3, 3]
            self.P_12 = self.T_12[0:3, 3]
            self.P_23 = self.T_23[0:3, 3]
            self.P_34 = self.T_34[0:3, 3]
            self.P_45 = self.T_45[0:3, 3]
            self.P_56 = self.T_56[0:3, 3]


        ###################################################
        #
        #   Select axes for application of sum-of-angles
        #
        # propagation steps below will only be trig simplified if consecutive axes
        # are parallel to save time
        #   if \theta_j =0, then we should look for sin(theta_j-1 + theta_j) etc.
        #

        simp = np.zeros(6)
        for j in range(1,5):  # we will only trigsimp if \alpha_N-1 == {0,pi} signifying
                            # parallel axes
            if(self.DH[j,al] == 0 or self.DH[j,al] ==  sp.pi):
                simp[j] = 1


        if(JACOBIAN):
            # velocity propagation for the Jacobian matrix
            ##########################
            # angular
            self.v_00 = sp.Matrix([0,0,0])
            self.w_00 = sp.Matrix([0,0,0])

            self.w_11 = self.R_01.T * self.w_00
            if(self.vv[0] == 1):
                self.w_11 += sp.Matrix([0,0,qd_1])

            self.w_22 = self.R_12.T * self.w_11
            if(self.vv[1] == 1):
                self.w_22 += sp.Matrix([0,0,qd_2])
            if(simp[1]):
                self.w_22 = sp.trigsimp(self.w_22)

            self.w_33 = self.R_23.T * self.w_22
            if(self.vv[2] == 1):
                self.w_33 += sp.Matrix([0,0,qd_3])
            if(simp[2]):
                self.w_33 = sp.trigsimp(self.w_33)

            self.w_44 = self.R_34.T * self.w_33
            if(self.vv[3] == 1):
                self.w_44 += sp.Matrix([0,0,qd_4])
            if(simp[3]):
                self.w_44 = sp.trigsimp(self.w_44)

            self.w_55 = self.R_45.T * self.w_44
            if(self.vv[4] == 1):
                self.w_55 += sp.Matrix([0,0,qd_5])
            if(simp[4]):
                self.w_55 = sp.trigsimp(self.w_55)

            self.w_66 = self.R_56.T * self.w_55
            if(self.vv[5] == 1):
                self.w_66 += sp.Matrix([0,0,qd_6])
            if(simp[5]):
                self.w_66 = sp.trigsimp(self.w_66)

            #########################
            ###linear
            self.v_00 = sp.Matrix([0,0,0])

            self.v_11 = self.R_01.T*(self.v_00 + self.w_00.cross(self.P_01))
            if(self.vv[0] == 0):
                self.v_11 += sp.Matrix([0,0,qd_1])

            self.v_22 = self.R_12.T*(self.v_11 + self.w_11.cross(self.P_12))
            if(self.vv[1] == 0):
                self.v_22 += sp.Matrix([0,0,qd_2])
            if(simp[1]):
                self.v_22 = sp.trigsimp(self.v_22)

            self.v_33 = self.R_23.T*(self.v_22 + self.w_22.cross(self.P_23))
            if(self.vv[2] == 0):
                self.v_33 += sp.Matrix([0,0,qd_3])
            if(simp[2]):
                self.v_33 = sp.trigsimp(self.v_33)

            self.v_44 = self.R_34.T*(self.v_33 + self.w_33.cross(self.P_34))
            if(self.vv[3] == 0):
                self.v_44 += sp.Matrix([0,0,qd_4])
            if(simp[3]):
                self.v_44 = sp.trigsimp(self.v_44)

            self.v_55 = self.R_45.T*(self.v_44 + self.w_44.cross(self.P_45))
            if(self.vv[4] == 0):
                self.v_55 += sp.Matrix([0,0,qd_5])
            if(simp[4]):
                self.v_55 = sp.trigsimp(self.v_55)

            self.v_66 = self.R_56.T*(self.v_55 + self.w_55.cross(self.P_56))
            if(self.vv[5] == 0):
                self.v_66 += sp.Matrix([0,0,qd_6])
            if(simp[5]):
                self.v_66 = sp.trigsimp(self.v_66)

            self.qdot = sp.Matrix([qd_1, qd_2, qd_3, qd_4, qd_5, qd_6])

            self.J66  = ManipJacobian_S(self.v_66, self.w_66, self.qdot)


    ###################################################
    #
    #   Compute the full set of FK equations for input
    #     to an IK solver:
    #
    #    Td         = T01*T12*....T56
    #    T10*Td         = T12*....T56
    #    T21*T10*Td     = T23*...T_56
    #            . . .
    #    T54*T43*T32*T21*T10*Td = T56

    def get_mequation_set(self):
        self.Td = hf.ik_lhs()
        list = []
        lhs = self.Td
        rhs = self.T_01*self.T_12*self.T_23*self.T_34*self.T_45*self.T_56
        list.append(matrix_equation(lhs,rhs))

        lhs = H_inv_S(self.T_01)*self.Td
        rhs =           self.T_12*self.T_23*self.T_34*self.T_45*self.T_56
        list.append(matrix_equation(lhs,rhs))

        lhs = H_inv_S(self.T_12)*H_inv_S(self.T_01)*self.Td
        rhs =                 self.T_23*self.T_34*self.T_45*self.T_56
        list.append(matrix_equation(lhs,rhs))

        lhs = H_inv_S(self.T_23)*H_inv_S(self.T_12)*H_inv_S(self.T_01)*self.Td
        rhs =                           self.T_34*self.T_45*self.T_56
        list.append(matrix_equation(lhs,rhs))

        lhs = H_inv_S(self.T_34)*H_inv_S(self.T_23)*H_inv_S(self.T_12)*H_inv_S(self.T_01)*self.Td
        rhs =                                     self.T_45*self.T_56
        list.append(matrix_equation(lhs,rhs))

        lhs = H_inv_S(self.T_45)*H_inv_S(self.T_34)*H_inv_S(self.T_23)*H_inv_S(self.T_12)*H_inv_S(self.T_01)*self.Td
        rhs =                                               self.T_56
        list.append(matrix_equation(lhs,rhs))
        return list


#########################################################################################
#
#      Numerical Functions
#

# sample pose: {th_1: 30*deg, th_2: 45*deg}
# sample params: {a_1: 2, l_4: 10}
# M: a machanism

def forward_kinematics_N(M, pose, params):
    pp = pose.copy()
    pp.update(params)    # combine the pose and the params
    T1 = M.T_06.subs(pp)    # substitue for all symbols

    # test to make sure all symbols are substituted with numeric values
    Num_check(T1)     # this quits if fails

    T2 = np.matrix(T1)   # convert from sympy to numpy matrix
    return T2







#####################################################################################
# Test code below.  See sincos_solver.py for example
#




class TestSolver008(unittest.TestCase):    # change TEMPLATE to unique name (2 places)
    def setUp(self):
        ((th_1, th_2, th_3, th_4, th_5, th_6)) = sp.symbols(('th_1', 'th_2', 'th_3', 'th_4', 'th_5', 'th_6'))

        #((h, l_3, l_4)) = sp.symbols(('h', 'l_1','l_3', 'l_4'))

        sp.var('h l_1 l_3 l_4')
        self.DB = False  # debug flag
        print '===============  Test kin_cl library  ====================='
        return

    # def tearDown(self):
        # print "===============  END: Test kin_cl library  ====================="

    def runTest(self):
        self.a_test_kin_cl()
        self.a_test_kequation()

    # test Latex output for kequation
    def a_test_kequation(self):   # another kequation test in ik_classes
        e1 = kequation(th_2, sp.sin(th_1)*l_1 + sp.sqrt(l_4))
        e2 = kequation(th_2, sp.sin(th_1)*l_1 / sp.sqrt(l_4))
        print '>>-----------------------------<<'
        print e1
        print e1.LaTexOutput()
        print e1.LaTexOutput(True)
        print '>>-----------------------------<<'
        fs = 'kequation LaTex output  FAIL'
        self.assertTrue(e1.LaTexOutput() == r'\theta_{2} = l_{1} \sin{\left (\theta_{1} \right )} + \sqrt{l_{4}}', fs)
        self.assertTrue(e1.LaTexOutput(True) == r'\theta_{2} &= l_{1} \sin{\left (\theta_{1} \right )} + \sqrt{l_{4}}', fs + ' (align)')

#\theta_{2} = l_{1} \sin{\left (\theta_{1} \right )} + \sqrt{l_{4}}
#\theta_{2} &= l_{1} \sin{\left (\theta_{1} \right )} + \sqrt{l_{4}}

        print '>>-----------------------------<<'
        print e2
        print e2.LaTexOutput()
        print e2.LaTexOutput(True)
        print '>>-----------------------------<<'

        return

    def a_test_kin_cl(self):
        params = [h, l_3, l_4]
        params = {h:5, l_3: 2, l_4: 6}

        # 1 for rotary, 0 for prismatic
        v = [1,1,1,1,1,1]

        # standardize on the order "alpha N-1, a N-1, d N, theta N' for the DH table columns.
        dh = sp.Matrix([
            [sp.pi/2, 0, h,   th_1],
            [0,       0, 0,   th_2   ],
            [-sp.pi/2, 0, l_3, th_3  ],
            [0,     l_4, 0,   th_4  ],
            [0,       0,  0,   0],
            [0,       0,  0,   0]
            ])

        M = mechanism(dh, params, v)

        M.forward_kinematics()

        print ' - - - '
        m = M.T_01*M.T_12
        sp.pprint(notation_squeeze(m))

        fs = 'kinematics class FAIL'
        self.assertTrue(m[0,0]==  sp.cos(th_1)*sp.cos(th_2)-sp.sin(th_1)*sp.sin(th_2), fs)
        self.assertTrue(m[0,1]== -sp.cos(th_1)*sp.sin(th_2)-sp.cos(th_2)*sp.sin(th_1), fs)
        self.assertTrue(m[1,2]== -1, fs)
        self.assertTrue(m[1,3]== -h, fs)


        #   Test eqn_set()
        L = M.get_mequation_set()

        fs = 'intermediate equations FAIL'

        sp.var('r_21 r_22 r_23 Py')

        self.assertTrue(L[2].Td[2,0] == -r_21, fs)
        self.assertTrue(L[2].Td[2,1] == -r_22, fs)
        self.assertTrue(L[2].Td[2,2] == -r_23, fs)
        self.assertTrue(L[2].Td[2,3] == -Py-h, fs)


        if(JACOBIAN and False):    # reactivate this later
            print ' --- Numerical Jacobian ---'
            pose = {th_1: 20*deg, th_2:45*deg, th_3:15*deg, th_4:-21.7*deg}
            M.Jacobian_N(pose)

        #print '\n\n\n            kin_cl.py PASSES all tests \n\n'

#
#    Can run your test from command line by invoking this file
#
#      - or - call your TestSolverTEMPLATE()  from elsewhere
#

if __name__ == "__main__":

    print '\n\n===============  Test kin_cl nodes====================='
    #testsuite = unittest.TestLoader().loadTestsFromTestCase(TestSolver008)  # replace TEMPLATE
    #unittest.TextTestRunner(verbosity=2).run(testsuite)
    unittest.main()


