# Dianmu Zhang 08/17
# new two equations solver based on Herrera-Bendezu
# modified simultanous equation P72

# Copyright 2017 University of Washington

# Developed by Dianmu Zhang and Blake Hannaford 
# BioRobotics Lab, University of Washington

# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
import sympy as sp

from ikbtfunctions.helperfunctions import *
from ikbtbasics.kin_cl import *
from ikbtbasics.ik_classes import *  
from sys import exit
import b3 as b3          # behavior trees


Aw = sp.Wild('Aw')
Bw = sp.Wild('Bw')
Cw = sp.Wild('Cw')
Dw = sp.Wild('Dw')
Ew = sp.Wild('Ew')
 
thx = sp.Wild('thx')
thy = sp.Wild('thy')
thz = sp.Wild('thz')

class simu_id(b3.Action):
    # finding 
    #    c = Asin(th1) + Bcos(th1)
    #    d = Acos(th1) - Bsin(th2)  (nice arctan solution)
    #
  
    def tick(self, tick):
        curr_unk = tick.blackboard.get('curr_unk')
        unknowns = tick.blackboard.get('unknowns')
        R = tick.blackboard.get('Robot')

        one_unk = tick.blackboard.get('eqns_1u')

        found = False
        #pattern1 = Aw * sp.sin(curr_unk.symbol) + Bw * sp.cos(curr_unk.symbol) - Cw
        #pattern2 = Aw * sp.cos(curr_unk.symbol) - Bw * sp.sin(curr_unk.symbol) - Dw

        eq1 = None
        eq2 = None

        eqn_list= []
        if not curr_unk.solved:
            for e in one_unk:
                e_flat = e.RHS - e.LHS
                e_flat = e_flat.expand()
                if e_flat.has(curr_unk.symbol) and e_flat.has(curr_unk.symbol):    
                    e_flat= e_flat.collect(sp.sin(curr_unk.symbol))
                    e_flat= e_flat.collect(sp.cos(curr_unk.symbol))
                    if e_flat not in eqn_list:
                        eqn_list.append(e_flat)

            if self.BHdebug:
                print("potential list: ")
                print(eqn_list)

            for i in range(len(eqn_list)):
                e_flat = eqn_list[i]
                # previouly used sp.match, which fails when expr too complicated
                # collect coefficients manually
                d1 = {}
                d1[Aw] = e_flat.coeff(sp.sin(curr_unk.symbol))
                d1[Bw] = e_flat.coeff(sp.cos(curr_unk.symbol))

                if self.BHdebug:
                    print("considering eqn: ", e_flat)
                    print("d1 content (a, b)")
                    print(d1[Aw], '\n',d1[Bw])

                eq1 = e_flat
                for j in range(i+1, len(eqn_list)):
                    e_flat = eqn_list[j]
                    if e_flat - eq1 == 0 or e_flat + eq1 == 0:
                        continue

                    d2 = {}
                    d2[Aw] = e_flat.coeff(sp.cos(curr_unk.symbol))
                    d2[Bw] = -e_flat.coeff(sp.sin(curr_unk.symbol))

                    if self.BHdebug:
                        print("considering eqn: ", e_flat)
                        print("d2 content (a, b)")
                        print(d2[Aw],'\n',d2[Bw])

                    eq2 = e_flat
                    if (d1[Aw] == d2[Aw] or d1[Aw] == -d2[Aw]) \
                        and (d1[Bw] == d2[Bw] or d1[Bw] == -d2[Bw]):
                        found = True
                        if self.BHdebug:
                            print("found two equ two unknown")
                            print(eq1)
                            print(eq2)
                    # it's also possible the order is reversed
                    # if that's the case, swap
                    elif (d1[Aw] == d2[Bw] or d1[Aw] == -d2[Bw]) \
                        and (d1[Bw] == d2[Aw] or d1[Bw] == -d2[Aw]):
                        print("reverse order")
                        found = True
                        temp = eq1
                        eq1 = eq2
                        eq2 = temp

                    if found:
                        curr_unk.solvemethod += "simultaneous eqn"
                        curr_unk.eqntosolve = kequation(0, eq1)
                        curr_unk.secondeqn = kequation(0, eq2)
                        tick.blackboard.set('curr_unk', curr_unk)
                        return b3.SUCCESS

        return b3.FAILURE

class simu_solver(b3.Action):
    def tick(self, tick):
        curr_unk = tick.blackboard.get('curr_unk')
        eq1 = curr_unk.eqntosolve.RHS
        eq2 = curr_unk.secondeqn.RHS

        unknowns = tick.blackboard.get('unknowns')
        R = tick.blackboard.get('Robot')


        A = eq1.coeff(sp.sin(curr_unk.symbol))
        B = eq1.coeff(sp.cos(curr_unk.symbol))

        C = A*sp.sin(curr_unk.symbol) + B*sp.cos(curr_unk.symbol) - eq1
        C = C.simplify()

        D = A*sp.cos(curr_unk.symbol) - B*sp.sin(curr_unk.symbol) - eq2
        D = D.simplify()


        if C == 0 and D == 0:
            print("Simultaneous Eqn Unsuccessful: divded by 0")
            return b3.FAILURE


        sol = sp.atan2(A*C - B*D, A*D + B*C)
        
        curr_unk.solutions=[sol]
        # enable test for atan(0,0) case
        curr_unk.argument = sp.Abs(A*C - B*D) + \
                            sp.Abs(A*D + B*C)
        curr_unk.nsolutions = 1

        curr_unk.set_solved(R, unknowns)

        return b3.SUCCESS

class TestSolverm7(unittest.TestCase):
    def setUp(self):
        self.DB = False  # debug flag
        print('\n\n===============  Test sincos Solver  =====================')
        return
    
    def runTest(self):
        self.test_m7()
            
    def test_m7(self):
        sp.var('Px Py Pz th_1 th_23 th_3 a_3 a_2 d_4')
        exp1 = Pz*sp.sin(th_23) + a_2*sp.cos(th_3) + a_3 + (-Px*sp.cos(th_1) - Py*sp.sin(th_1))*sp.cos(th_23)

        exp2 = Pz*sp.cos(th_23) - a_2*sp.sin(th_3) + d_4 + (Px*sp.cos(th_1) + Py*sp.sin(th_1))*sp.sin(th_23)

        keq1 = kequation(0, exp1)
        keq2 = kequation(0, exp2)
        one_ls = [keq1, keq2]

        uth1 = unknown(th_1)
        uth2 = unknown(th_3)
        uth23 = unknown(th_23)

        unknowns = [uth1, uth2, uth23]


        Rob = Robot()
        ik_tester = b3.BehaviorTree()



        # two equations one unknown,
        SimuEqnID = simu_id()
        SimuEqnID.Name = 'Simultaneous Eqn ID'
        SimuEqnSolve = simu_solver()
        SimuEqnSolve.Name = 'Simultaneous Eqn solver'
        Simu_Eqn_Sol = b3.Sequence([SimuEqnID, SimuEqnSolve])

        ik_tester.root = Simu_Eqn_Sol
        bb = b3.Blackboard()
        bb.set('curr_unk', uth23)
        bb.set('eqns_1u', one_ls)
        bb.set('Robot', Rob)
        bb.set('unknowns', unknowns)

        ik_tester.tick("testing two_eqn_m7", bb)

        curr = bb.get('curr_unk')
        print(curr.solutions)
                
def run_test():
    suite2 = unittest.TestLoader().loadTestsFromTestCase(TestSolverm7)
    unittest.TextTestRunner(verbosity=2).run(suite2)

            

