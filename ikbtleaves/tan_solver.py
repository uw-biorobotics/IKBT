#!/usr/bin/python
#
#   TEMPLATE
#     BT Nodes for Testing, ID, Solving
#

#  Identify and solve sets of two equations in which
#    sin(x)is in one and cos(x) is in the other
#
#   x = atan2(sin(x), cos(x))
#
# move loop logic outside of the node 21-Jun-2017 -DZ

# Copyright 2017 University of Washington

# Developed by Dianmu Zhang and Blake Hannaford
# BioRobotics Lab, University of Washington

# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
import sympy as sp  
import numpy as np
from sys import exit
import unittest

from ikbtfunctions.helperfunctions import *
from ikbtbasics.kin_cl import *
from ikbtbasics.ik_classes import *     # special classes for Inverse kinematics in sympy

from ikbtleaves.updateL import *
from ikbtleaves.comp_detect import *
from ikbtleaves.assigner_leaf  import *
from sympy.assumptions.assume import global_assumptions

import b3 as b3          # behavior trees

sp.var('th_23')

Aw = sp.Wild('Aw')
Bw = sp.Wild('Bw')
Cw = sp.Wild('Cw')
Dw = sp.Wild('Dw')
 


class tan_id(b3.Action):    # action leaf for ID eqns solved by atan2()    
    def tick(self, tick):
        unknowns = tick.blackboard.get('unknowns')   # the current list of unknowns
        R = tick.blackboard.get('Robot')
        # get the current assignment
        u = tick.blackboard.get('curr_unk')
        one_unk = tick.blackboard.get('eqns_1u')
        two_unk = tick.blackboard.get('eqns_2u')
            
        if(self.BHdebug):
            print("running: ", self.Name," , input RHS:")
            ulen= len(unknowns)
            print('tan_id working on', ulen ,' unknowns:')
        # identify unknowns in T where one equation can be solved by
        #          arctan(y,x)
        found = False
                
        # only if not identified as solvable by tangent yet
        if (not u.solvable_tan) and (not u.solved):  
            terms = [sp.sin(u.symbol), sp.cos(u.symbol)]


            sin_eqn = []
            cos_eqn = []

            for e in (one_unk + two_unk):   # spot the ones with common factors as well


                # fix the eqn, but not changing the original equation - DZ
                tmp = e.RHS-e.LHS
                sp.var('dummy')
                lhs =  dummy-dummy  # x-x is not same thing as 0 (!)
                if(not (tmp).has(u.symbol)):
                    continue        # only look at equations having the current unknown in them
                if(self.BHdebug):
                    print("\n\n  tan_id:        Looking for unknown: ", u.symbol, " in equation: ", )
                    print(e )
                    print("  which has ", count_unknowns(unknowns, e.RHS), " unknown(s) in RHS")
                    print("     and    ", count_unknowns(unknowns, e.LHS), " unknown(s) in LHS")
            
                if (tmp.has(sp.sin(u.symbol)) and tmp.has(sp.cos(u.symbol))):
                    continue   # this should be caught by sinANDcos solver
                    
                

                if tmp.has(sp.sin(u.symbol)):
                    sin_eqn.append(e)
                if tmp.has(sp.cos(u.symbol)):
                    cos_eqn.append(e)




            # Wilds must exclude sin/cos of THIS unknown, so they are built
            # here rather than reusing the module-level ones.  An unconstrained
            # 'Cw*cos(u) + Dw' is ambiguous -- it matches anything via
            # Cw = (expr - Dw)/cos(u) -- and sympy returns exactly that
            # degenerate reading when the coefficient is negative, e.g.
            #   -3*d_3*cos(th_1) + 4.95  ->  {Cw: 4.95/cos(th_1), Dw: -3*d_3*cos(th_1)}
            # The count_unknowns(d2[Dw]) screen below then rejected a pair the
            # leaf can solve, so a negative coefficient silently defeated it.
            Awx = sp.Wild('Awx', exclude=terms)
            Bwx = sp.Wild('Bwx', exclude=terms)
            Cwx = sp.Wild('Cwx', exclude=terms)
            Dwx = sp.Wild('Dwx', exclude=terms)

            for es in sin_eqn:
                estst = (es.RHS - es.LHS).collect(terms)  # get all the sin(th)s collected
                d1 = estst.match(Awx*sp.sin(u.symbol) + Bwx)
                if self.BHdebug: 
                    print('---')
                    print("\nsin equ: ")
                    print(u.eqntosolve)
                    print("\nsin(): coefficients are : ")
                    print(d1[Awx] )
                    print('---')
                for ec in cos_eqn:  
                    ectst = (ec.RHS - ec.LHS).collect(terms)  # get all the cos(th)s collected
                    d2 = ectst.match(Cwx*sp.cos(u.symbol) + Dwx)
                    if self.BHdebug: 
                        print("\ncos equ: ")
                        print(u.secondeqn)
                        print("\ncos(): coefficients")
                        print(d2[Cwx]          )
                    
                    # check some things about potential solvable equations
                    assert(d1 is not None and d2 is not None), 'somethings wrong!'
                    co = d1[Awx]/d2[Cwx]   # take ratio
                    # it's not solvable if (simplified) coefficient contains unknowns, or other parts have unknowns
                
                    #print 
                    #print 'tan_id: (',u.symbol,')   0 =  Aw*sin(th_XX)+Bw , 0 = Cw*cos(th_XX) + Dw '
                    #print 'Aw: ', d1[Awx], '   Bw: ', d1[Bwx]
                    #print 'Cw: ', d2[Cwx], '   Dw: ', d2[Dwx]
                    
                    
                    too_many_unknowns = False
                    if count_unknowns(unknowns, co) > 0 or count_unknowns(unknowns, d1[Bwx]) >0 or count_unknowns(unknowns, d2[Dwx]) > 0:
                        too_many_unknowns = True


                    # a good match / solution candidate
                    if not too_many_unknowns:  
                        found = True  # found both terms for at least one variable
                        u.eqntosolve = kc.kequation(0, estst)
                        u.secondeqn = kc.kequation(0, ectst)
                        u.readytosolve = True 
                        # u.eqntosolve and secondeqn are already set up above 
                        print('tan_id:  able to solve', u.symbol)
                        if count_unknowns(unknowns, co) > 0: #cancellable unsolved term, add the nonzero assumption
                            global_assumptions.add(sp.Q.nonzero(d2[Cwx]))                            
                        u.solvemethod += "atan2(y,x)"
                        u.solvable_tan = True
                        
                    if(self.BHdebug and u.readytosolve):
                        print('\n              tan_id: Identified Solution: ', u.symbol)
                        e1tmp = u.eqntosolve.RHS + u.eqntosolve.LHS 
                        e2tmp = u.secondeqn.RHS + u.secondeqn.LHS
                        print('                       ', u.eqntosolve, '  (', count_unknowns(unknowns, e1tmp), 'unks)')
                        for u in get_unknowns(unknowns, e1tmp):
                            print(u.symbol, )
                        print('')
                        print('                       ', u.secondeqn, '  (', count_unknowns(unknowns, e2tmp), 'unks)')
                        for u in get_unknowns(unknowns, e1tmp):
                            print(u.symbol, )
                        print('')
                        print('')
                    if found:
                        break
                if found:
                    break

            #continue #stop looking at equations for the same unknown                      
        tick.blackboard.set('curr_unk', u)
        tick.blackboard.set('unknowns',unknowns)   # the current list of unknowns
        if u.solvable_tan:
            return b3.SUCCESS
        else:
            return b3.FAILURE
        
      
# solve eqns in T where two equations can be solved for one variable 
#    using atan2()
class tan_solve(b3.Action):    # Solve sin cos equation pairs
    def tick(self, tick):
        Tm = tick.blackboard.get('Tm')   # the current matrix equation  
        unknowns = tick.blackboard.get('unknowns')
        R = tick.blackboard.get('Robot') 
        u = tick.blackboard.get('curr_unk')
        #create a list of symbols for (unsolved) variables
        # will use it for get_variable

        unk_unsol = []
        for unk in unknowns:
            if not unk.solved:
                unk_unsol.append(unk)
                if(self.BHdebug):
                    print('tan_solve(): Not yet solved: ', unk.symbol)

        fsolved = False
        if u.solvable_tan:
            if(self.BHdebug):
                print("tan_solve: I'm trying to solve: ", u.symbol)
                print("  Using tangent() and 2 eqns:" )
                print(u.eqntosolve)
                print(u.secondeqn)
            
            fs = "tan_solve:  Somethings Wrong!"
            
            try:
                x  = u.eqntosolve.LHS
            except:
                print("problematic step: %s"%u.symbol)
                print(u.eqntosolve)
                
            rhs = u.eqntosolve.RHS
            # exclude sin/cos of this unknown -- an unconstrained Wild makes
            # 'Aw*sin(u) + Bw' ambiguous and sympy returns a degenerate
            # decomposition for negative coefficients.  See tan_id.
            _terms = [sp.sin(u.symbol), sp.cos(u.symbol)]
            Aw = sp.Wild("Aw", exclude=_terms)
            Bw = sp.Wild("Bw", exclude=_terms)
            d  = rhs.match(Aw*sp.sin(u.symbol)+Bw)
            
            assert(d != None), fs
            assert(count_unknowns(unknowns, d[Bw])==0), fs
            
            # now the second equation for this variable
            x2 = u.secondeqn.LHS # it's 0
            rhs2 = u.secondeqn.RHS
            d2 = rhs2.match(Aw*sp.cos(u.symbol)+Bw)
            
            assert(d2 != None), fs
            assert(count_unknowns(unknowns, d2[Bw])==0), fs

            #construct solutions
            print('tan_solver Denominators: ', d[Aw], d2[Aw])

            co = d[Aw]/d2[Aw] #coefficients of Y and X
            Y = x-d[Bw]
            X = x2-d2[Bw]
            
            # the reason it can only test one d[Aw] is that 
            # the two eqn are pre-screened by the ID
            # safer way to do it is to get the unsolved unknown number 
            # from d[Aw] and d2[Aw] and use the max
            co_unk = get_variables(unk_unsol, d[Aw]) #get the cancelled unknown in the coefficients
            fsolved = True
            # if coefficient doesn't have unsolved unknowns
            if len(co_unk) == 0: 
                # this is critical for "hidden dependency"
                # can't use 'co', since it might have cancelled the parent (solved) variable
                sol = sp.atan2(Y/d[Aw],X/d2[Aw]) 
                u.solutions.append(sol)
                u.tan_solutions.append(sol)
                u.tan_eqnlist.append(u.eqntosolve)
                u.tan_eqnlist.append(u.secondeqn)
                u.nsolutions = 1
               
            else: 
                sol1 = sp.atan2(Y/co,X)
                sol2 = sp.atan2(-Y/co, -X)
                u.solutions.append(sol1) #co_unk > 0
                u.solutions.append(sol2) #co_unk < 0
                
                u.tan_solutions.append(sol1)
                u.tan_solutions.append(sol2)
                
                u.tan_eqnlist.append(u.eqntosolve)
                u.tan_eqnlist.append(u.secondeqn)
                # The branch is selected by the sign of the factor used to
                # scale both atan2 arguments, which is A2 (= d2[Aw]) -- NOT A1.
                # Scaling atan2(y,x) preserves the angle only for a positive
                # factor; sol2 covers the negative case.  These previously
                # recorded d[Aw] (A1), which agrees only when A1 and A2 happen
                # to share a sign; for opposite signs the labels were inverted.
                u.assumption.append(sp.Q.positive(d2[Aw]))
                u.assumption.append(sp.Q.negative(d2[Aw]))
                u.nsolutions = 2

                # note that set_solved is doen in ranker (ranking sincos, and tan sols)

        if(fsolved):  
            tick.blackboard.set('curr_unk', u)          
            tick.blackboard.set('unknowns', unknowns)
            tick.blackboard.set('Robot', R)
            return b3.SUCCESS
        else:
            return b3.FAILURE
#############################################
#
#  Unit test code:
#    

class test_tan_id(b3.Action):    # tester for your ID    
   def tick(self, tick):
        test_number = tick.blackboard.get('test_number')
        R = tick.blackboard.get('Robot')
        if R is None:
            R = Robot()   # if not there, make one_unk
        # set up bb data for testing  
        Td = ik_lhs()      # eij elements
        Ts = sp.zeros(5)
        
        if(test_number == 1):
            Ts = sp.zeros(5)
            Ts[0,1] = l_1*sp.sin(th_3) + l_2*sp.cos(th_3)  # should skip this but get rest
            Ts[1,1] = l_1*sp.sin(th_2) + 15      # test equation pairs
            Ts[1,2] = l_3*sp.cos(th_2) + 99
            Ts[1,3] = sp.sin(th_5)*l_3        # not a match
            Ts[2,0] = l_1*sp.sin(th_3) + l_2
            Ts[2,1] = l_3*sp.cos(th_3) + l_4
            Ts[2,2] = l_5*sp.sin(th_23)          # sum-of-angles variables
            Ts[2,3] = l_1*sp.cos(th_23) + 50
            Ts[0,2] = (l_1+l_2)*sp.sin(th_4)
            Ts[0,3] =  l_3*sp.cos(th_4)            
            
        elif(test_number == 2):
            # unsolved sum of angle testing!!
            #  
            Ts = sp.zeros(5)
            Ts[0, 0] = sp.sin(th_2 + th_3) * sp.sin(th_4) #two sols
            Ts[0, 3] = sp.sin(th_2 + th_3) * sp.cos(th_4) 
            
            # should be solvable:
            Ts[1,1] = l_1*sp.sin(th_2) + 15      # test equation pairs
            Ts[1,2] = l_3*sp.cos(th_2) + 99
            
            # test equation that caused bug
            #th_4 = atan2(r_13*cos(th_2)*cos(th_3) - r_23*sin(th_3) - r_33*sin(th_2)*cos(th_3), -r_13*sin(th_2) - r_33*cos(th_2)) 
             
            # Test with an unsolved scalar (i.e. common factor which is unknown)
            Ts[2,2] = sp.sin(th_5)*sp.sin(th_6)
            Ts[2,0] = -sp.sin(th_5)*sp.cos(th_6) # two solns!
            
        elif(test_number == 3):       
            Ts = sp.zeros(5)
            # should be solvable:
            Ts[1,1] = l_1*sp.sin(th_3) + 15      # test equation pairs
            Ts[1,2] = l_3*sp.cos(th_3) + 99
            
            #   th_3 should be solved first 
            Ts[0,1] = r_13*sp.cos(th_2)*sp.cos(th_3) - r_23*sp.sin(th_3) - r_33*sp.sin(th_2)*sp.cos(th_3)
            Ts[1,0] = -r_13*sp.sin(th_2) - r_33*sp.cos(th_2)
            
        else:
            print('tan_solver:   UNKNOWN TEST NUMBER: ', test_number)
            return b3.FAILURE

        testm = matrix_equation(Td,Ts)
        R.mequation_list = [testm]
        
        uth1 = unknown(th_1)  # clear and reset the unknowns
        uth2 = unknown(th_2)
        uth3 = unknown(th_3)
        uth4 = unknown(th_4)
        uth5 = unknown(th_5)
        uth6 = unknown(th_6)
        uth23 = unknown(th_23)

        variables = [uth1, uth2, uth3, uth4, uth5, uth6, uth23]
        
        [L1, L2, L3p] = R.scan_Mequation(testm, variables)  # lists of 1unk, 2unk, and 3+unk equations
        tick.blackboard.set('eqns_1u', L1)
        tick.blackboard.set('eqns_2u', L2)
        tick.blackboard.set('eqns_3pu', L3p)
        tick.blackboard.set('unknowns',variables)
        tick.blackboard.set('Robot',R)    
        return b3.SUCCESS
class TestSolver004(unittest.TestCase):    # change TEMPLATE to unique name (2 places)
    DB = False
    def setUp(self):
        self.DB = False  # debug flag
        print('===============  Test tan solver  =====================')
        return
    
    def runTest(self):
        self.test_tansolver()
        self.test_tanB_roundtrip_single_solution()
        self.test_tanB_roundtrip_two_branches()
        self.test_tanB_negative_cos_coefficient_solves()
        self.test_tanB_assumption_labels_match_valid_branch()

    #####################################################################
    #  Numeric round-trip tests.
    #
    #  test_tansolver() above compares against exact sympy expressions, which
    #  pins the current FORM rather than the correctness of the answer.  These
    #  substitute numbers into whatever the solver returns and check that it
    #  actually satisfies both original equations.
    #
    #  The solver takes two paths:
    #     A1*sin(u) + B1 = 0,  A2*cos(u) + B2 = 0
    #   - if A1 has no unsolved unknowns:  one solution, atan2(Y/A1, X/A2)
    #   - otherwise: the shared unsolved factor is cancelled via co = A1/A2 and
    #     TWO solutions are emitted, because scaling both atan2 arguments is
    #     only angle-preserving when the scale factor A2 is positive.

    def run_tan(self, sin_expr, cos_expr, sym, coeff_unknowns=()):
        '''Drive tan_id + tan_solve on 0 == sin_expr and 0 == cos_expr.
           `coeff_unknowns` are extra UNSOLVED unknowns appearing in the
           coefficients (that is what selects the two-branch path).
           Returns the unknown for `sym`.  Note set_solved() is deliberately
           NOT called by this leaf -- the rank leaf does it -- so u.solved
           stays False here and we check u.solutions directly.'''
        u = unknown(sym)
        unknowns = [u] + [unknown(s) for s in coeff_unknowns]

        t_id = tan_id()
        t_id.BHdebug = self.DB
        t_sl = tan_solve()
        t_sl.BHdebug = self.DB

        ik_tester = b3.BehaviorTree()
        ik_tester.root = b3.Sequence([t_id, t_sl])

        bb = b3.Blackboard()
        bb.set('curr_unk', u)
        bb.set('unknowns', unknowns)
        bb.set('eqns_1u', [kequation(0, sin_expr), kequation(0, cos_expr)])
        bb.set('eqns_2u', [])
        bb.set('Robot', Robot())
        ik_tester.tick("tan numeric roundtrip", bb)
        return bb.get('curr_unk')

    def residuals(self, sol, sin_expr, cos_expr, sym, subs=None):
        '''Return (|sin eqn residual|, |cos eqn residual|) at the solution.'''
        s = sol if subs is None else sol.subs(subs)
        e1 = sin_expr if subs is None else sin_expr.subs(subs)
        e2 = cos_expr if subs is None else cos_expr.subs(subs)
        r1 = complex(sp.N(e1.subs(sym, s)))
        r2 = complex(sp.N(e2.subs(sym, s)))
        return abs(r1), abs(r2)

    def test_tanB_roundtrip_single_solution(self):
        '''Coefficients free of unsolved unknowns -> the single-solution path.'''
        sp.var('th_1')
        fs = ' tan_solver numeric roundtrip FAIL'
        th_true = 0.6
        A1, A2 = 2, 3
        B1 = -A1*float(sp.sin(th_true))
        B2 = -A2*float(sp.cos(th_true))
        sin_expr = A1*sp.sin(th_1) + B1
        cos_expr = A2*sp.cos(th_1) + B2

        u = self.run_tan(sin_expr, cos_expr, th_1)

        self.assertEqual(len(u.solutions), 1, fs + ' (expected the 1-solution path)')
        self.assertAlmostEqual(float(sp.N(u.solutions[0])), th_true, places=9,
                               msg=fs + ' (recovered the wrong angle)')
        r1, r2 = self.residuals(u.solutions[0], sin_expr, cos_expr, th_1)
        self.assertAlmostEqual(r1, 0.0, places=9, msg=fs + ' (sin eqn not satisfied)')
        self.assertAlmostEqual(r2, 0.0, places=9, msg=fs + ' (cos eqn not satisfied)')

    def test_tanB_roundtrip_two_branches(self):
        '''A shared unsolved factor d_3 in both coefficients cancels in
           co = A1/A2, giving the two-branch path.  Exactly one branch is
           correct for a given sign of d_3; the pair must bracket the truth.'''
        sp.var('th_1 d_3')
        fs = ' tan_solver two-branch roundtrip FAIL'
        th_true, d3_val = 0.6, 2.0
        # A1 = 2*d_3, A2 = 3*d_3  -- same sign, so co = 2/3 is unknown-free
        sin_expr = 2*d_3*sp.sin(th_1) - 2*d3_val*float(sp.sin(th_true))
        cos_expr = 3*d_3*sp.cos(th_1) - 3*d3_val*float(sp.cos(th_true))

        u = self.run_tan(sin_expr, cos_expr, th_1, coeff_unknowns=[d_3])

        self.assertEqual(len(u.solutions), 2, fs + ' (expected the 2-branch path)')
        subs = {d_3: d3_val}
        ok = [i for i, s in enumerate(u.solutions)
              if max(self.residuals(s, sin_expr, cos_expr, th_1, subs)) < 1e-9]
        self.assertEqual(len(ok), 1, fs + ' (exactly one branch should be valid)')
        # and the branches must be pi apart
        vals = [float(sp.N(s.subs(subs))) for s in u.solutions]
        self.assertAlmostEqual(abs(abs(vals[0] - vals[1]) - float(sp.pi)), 0.0, places=9,
                               msg=fs + ' (branches are not pi apart)')

    def test_tanB_negative_cos_coefficient_solves(self):
        '''Regression: a NEGATIVE cos coefficient must solve, same as positive.

           tan_id and tan_solve decomposed with an unconstrained Wild, and
           'Cw*cos(u) + Dw' is ambiguous -- it matches anything via
           Cw = (expr - Dw)/cos(u).  For a negative coefficient sympy returned
           exactly that degenerate reading:
               -3*d_3*cos(th_1) + 4.95 -> {Cw: 4.95/cos(th_1), Dw: -3*d_3*cos(th_1)}
           the count_unknowns(d2[Dw]) screen then rejected the pair, and the
           leaf silently declined a problem it can solve -- while the identical
           problem with the sign flipped solved fine.  Fixed by excluding
           sin/cos of the unknown from the Wilds.'''
        sp.var('th_1 d_3')
        fs = ' tan_solver negative-cos-coefficient FAIL'
        th_true, d3_val = 0.6, 2.0
        subs = {d_3: d3_val}
        sin_expr  =  2*d_3*sp.sin(th_1) - 2*d3_val*float(sp.sin(th_true))

        # the degenerate match that used to defeat the leaf
        cos_neg = -3*d_3*sp.cos(th_1) + 3*d3_val*float(sp.cos(th_true))
        Cw, Dw = sp.Wild('Cw'), sp.Wild('Dw')
        self.assertTrue(cos_neg.match(Cw*sp.cos(th_1) + Dw)[Dw].has(sp.cos(th_1)),
                        fs + ' (unconstrained Wild no longer degenerates -'
                             ' the exclude= guards may now be redundant)')
        # ... and the constrained form the leaf now uses, which does not
        Cx = sp.Wild('Cx', exclude=[sp.sin(th_1), sp.cos(th_1)])
        Dx = sp.Wild('Dx', exclude=[sp.sin(th_1), sp.cos(th_1)])
        self.assertFalse(cos_neg.match(Cx*sp.cos(th_1) + Dx)[Dx].has(sp.cos(th_1)), fs)

        # both signs must now solve, and give the same true angle
        for name, sign in (('negative', -3), ('positive', 3)):
            cos_expr = sign*d_3*sp.cos(th_1) - sign*d3_val*float(sp.cos(th_true))
            u = self.run_tan(sin_expr, cos_expr, th_1, coeff_unknowns=[d_3])
            self.assertEqual(len(u.solutions), 2,
                             fs + ' (%s cos coefficient did not solve)' % name)
            ok = [i for i, s in enumerate(u.solutions)
                  if max(self.residuals(s, sin_expr, cos_expr, th_1, subs)) < 1e-9]
            self.assertEqual(len(ok), 1,
                             fs + ' (%s: exactly one branch should be valid)' % name)
            self.assertAlmostEqual(float(sp.N(u.solutions[ok[0]].subs(subs))), th_true,
                                   places=9,
                                   msg=fs + ' (%s: wrong angle recovered)' % name)

    def test_tanB_assumption_labels_match_valid_branch(self):
        '''The recorded branch assumptions must name the coefficient that
           actually selects the branch.

           Scaling both atan2 arguments preserves the angle only for a POSITIVE
           factor, and that factor is A2 (d2[Aw]); solutions[1] covers A2 < 0.
           The assumptions previously recorded A1 (d[Aw]), which agrees only
           when A1 and A2 share a sign.  With opposite signs the labels were
           inverted: assumption[0] said "A1 positive" while the branch valid
           under that condition was solutions[1].

           Harmless in itself -- u.assumption is only appended and printed,
           never consumed -- but wrong, and it would bite the moment anything
           selects a branch from these labels.'''
        sp.var('th_1 d_3')
        fs = ' tan_solver assumption-label FAIL'
        th_true, d3_val = 0.6, 2.0      # d_3 POSITIVE
        subs = {d_3: d3_val}
        sin_expr = 2*d_3*sp.sin(th_1) - 2*d3_val*float(sp.sin(th_true))

        for name, a2 in (('same-sign', 3), ('opposite-sign', -3)):
            cos_expr = a2*d_3*sp.cos(th_1) - a2*d3_val*float(sp.cos(th_true))
            u = self.run_tan(sin_expr, cos_expr, th_1, coeff_unknowns=[d_3])
            self.assertEqual(len(u.assumption), 2, fs)

            valid = [i for i, s in enumerate(u.solutions)
                     if max(self.residuals(s, sin_expr, cos_expr, th_1, subs)) < 1e-9]
            self.assertEqual(len(valid), 1, fs + ' (%s)' % name)

            # assumption[i] pairs with solutions[i]; the one that holds for
            # d_3 = +2 must be the label on the branch that actually works.
            # NB bool() on a sympy AppliedPredicate is always True -- it does
            # not evaluate the predicate -- so evaluate the sign explicitly.
            holds = []
            for i, a in enumerate(u.assumption):
                expr = a.arguments[0] if hasattr(a, 'arguments') else a.args[-1]
                val = float(expr.subs(subs))
                if (val > 0) == (a.function == sp.Q.positive):
                    holds.append(i)
            self.assertEqual(len(holds), 1, fs + ' (%s: labels not exclusive)' % name)
            self.assertEqual(holds[0], valid[0],
                             fs + ' (%s: assumption labels the wrong branch)' % name)


    def test_tansolver(self):
        ik_tester = b3.BehaviorTree() 
        tan_setup = test_tan_id()
        tanID = tan_id()
        tanID.Name = "tan ID"
        tanID.BHdebug = False
        tanSOL = tan_solve()
        tanSOL.BHdebug = False
        tanSOL.Name = 'Tangent Solver'
        asgn = assigner()
        
        subtree = b3.Sequence([asgn, tanID, tanSOL])
        repeats = b3.Repeater(subtree, max_loop = 15)
        
        #updateL01.name = 'update Transform and eqn lists'
        bb = b3.Blackboard()
       
        
        
        ik_tester.root= b3.Sequence([tan_setup, repeats])
        sp.var('r_11 r_12 r_13 r_21 r_22 r_23  r_31 r_32 r_33  Px Py Pz') # needed for test results 
        
        print('\n\n  ----------     tan solver TEST 1 --------------\n\n')
        bb.set('test_number',1) # go to test 1
        ik_tester.tick("tan_id test", bb)    
        
        
        #   Test the results 
        variables = bb.get('unknowns')  
        fs = 'tan_solver test 1 FAIL'
        ntests = 0
        for v in variables:  
            if(v.symbol == th_5):
                self.assertTrue(v.solved == False, fs) 
            if(self.DB):
                print('\n--------------------      ', v.symbol)
            if(self.DB and v.solved):
                sp.pprint (v.solutions[0])
                if(v.nsolutions == 2):
                    sp.pprint(v.solutions[1])
            if(v.symbol == th_1):
                ntests += 1
                self.assertTrue(not v.solved, fs+ ' [th_1]')
            if(v.symbol == th_2):
                ntests += 1
                #self.assertTrue(v.solved, fs+' [th_2]')
                self.assertTrue(v.solutions[0] - sp.atan2((r_22-15)/l_1, (r_23-99)/l_3) == 0,fs+' [th_2]')
            if(v.symbol == th_3):
                ntests += 1
                #self.assertTrue(v.solved, fs + '  [th_3]')
                sp.pprint(v.solutions[0])
                self.assertTrue(v.solutions[0] == sp.atan2((r_31-l_2)/l_1, (r_32-l_4)/l_3),fs + '  [th_3]')
            if(v.symbol == th_4):
                ntests += 1
                #self.assertTrue(v.solved, fs + '  [th_4]')
                self.assertTrue(v.solutions[0] == sp.atan2(r_13/(l_1+l_2), Px/l_3),fs + ' [th_4]')
            if(v.symbol == th_23):
                ntests += 1
                #self.assertTrue(v.solved, fs + '  [th_4]')
                self.assertTrue(v.solutions[0] == sp.atan2(r_33/(l_5), (Pz-50)/l_1), fs + ' [th_23]')
                
        self.assertTrue(ntests == 5, fs + '  assertion count failure ')
        print('Passed: ', ntests, ' asserts')

        
        print('\n\n  ----------     tan solver TEST 2 --------------\n\n')
        
        bb2 = b3.Blackboard()
        bb2.set('test_number',2) # go to test 2

        ik_tester.tick("tan_id test", bb2)    
        
        #   Test the results 
        variables = bb2.get('unknowns')
        print('>> Test 2 Asserts')

        fs = 'tan_solver test 2 FAIL'
        fs2 = 'wrong assumption' 
        ntests = 0
        for v in variables: 
            if v.solved: 
                print('\n--------------------      ', v.symbol)
                sp.pprint (v.solutions[0])
                if(v.nsolutions == 2):
                    sp.pprint(v.solutions[1])            
            if(v.symbol == th_1):
                ntests += 1
                self.assertTrue(not v.solved, fs + ' [th_1]')
            if(v.symbol == th_2):
                ntests += 1
                #self.assertTrue(v.solved, fs + ' [th_2]')
                self.assertTrue(v.nsolutions == 1, fs + ' [th_2]')
                self.assertTrue(v.solutions[0] == sp.atan2((r_22-15)/l_1,(r_23-99)/l_3), fs + ' [th_2]' )
                
            if v.symbol == th_4:
                ntests += 1
                self.assertTrue(not v.solved, fs + ' [th_4]')
                
            if v.symbol == th_5:
                ntests += 1
                self.assertTrue(not v.solved, fs +  ' [th_5]')
                
            if v.symbol == th_6:
                ntests += 1
                #self.assertTrue(v.solved, fs + ' [th_6]')
                self.assertTrue(v.solutions[0] == sp.atan2( -r_33, r_31), fs)
                self.assertTrue(v.solutions[1] == sp.atan2(r_33, -r_31), fs)
                print('Assumptions for ', v.symbol)               # should set assumptions if canceling an unk.
                print('    ', sp.pprint (v.assumption[0]) )
                print('    ', sp.pprint (v.assumption[1]))

        self.assertTrue(ntests == 5, 'tan_solver:   Assert count    FAIL')
        print('Passed: ', ntests, ' asserts')
        print("global assumptions")
        print(global_assumptions)
        


#
#    Can run your test from command line by invoking this file
#
#      - or - call your TestSolverTEMPLATE()  from elsewhere
#


def run_test():
    print('\n\n===============  Test tan_solver.py =====================')
    testsuite = unittest.TestLoader().loadTestsFromTestCase(TestSolver004)  # replace TEMPLATE 
    unittest.TextTestRunner(verbosity=2).run(testsuite)

if __name__ == "__main__":
    print('\n\n===============  Test tan_solver.py =====================')
    testsuite = unittest.TestLoader().loadTestsFromTestCase(TestSolver004)  # replace TEMPLATE 
    unittest.TextTestRunner(verbosity=2).run(testsuite)
    #unittest.main()

