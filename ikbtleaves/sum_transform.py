#!/usr/bin/python
#
#     BT Nodes for specific symbolic steps

# Copyright 2017 University of Washington

# Developed by Dianmu Zhang and Blake Hannaford
# BioRobotics Lab, University of Washington

# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
import sympy as sp
from sys import exit

from ikbtfunctions.helperfunctions import *
from ikbtbasics.kin_cl import *
from ikbtbasics.ik_classes import *     # special classes for Inverse kinematics in sympy

import b3 as b3          # behavior trees

# retrieve thxy from thx, thy
def find_xy(thx, thy):
    # lookup table for thxy (new: 3 parallel axes for axes 2,3,4)
    thxy_lookup = {th_1: [th_12], th_2:[th_12, th_23, th_234], th_3:[th_23, th_34, th_234], \
                    th_4:[th_34, th_45, th_234], th_5:[th_45, th_56], th_6:[th_56], \
                    }
    # one symbol in common is the th_xy we're looking for
    thx_s = set(thxy_lookup[thx])
    thy_s = set(thxy_lookup[thy])
    thxy_s = thx_s.intersection(thy_s)
    thxy = thxy_s.pop()
    return thxy
    

class sum_id(b3.Action):   ##  we should change this name since its a transform
    def tick(self, tick):
        matr_equ = tick.blackboard.get('Tm')                # current matrix equation  

        unknowns = tick.blackboard.get("unknowns")
                
        Tmatrix = matr_equ.Ts
        
        Tmatrix_squeeze = sp.simplify(notation_squeeze(Tmatrix))
        
        if(self.BHdebug):
            #print 'sum of angles transform: input: '
            #sp.pprint(matr_equ.Ts)
            ##print out the squeezed form
            print 'sum of angles transform: squeezed test input:'
            sp.pprint(Tmatrix_squeeze)
            
        unkn_sums_sym = set() #keep track of joint variable symbols
        
        thx = sp.Wild('thx')
        thy = sp.Wild('thy')
        sgn = sp.Wild('sgn')
        
        success_flag = False
        
        for i in range(3):
            for j in range(4):
                # need new ways to identify thx +/- thy
                # notation_squeeze does not pick up - cases
                expr = Tmatrix[i, j]    
                
            
                sub_sin = expr.find(sp.sin(thx + sgn * thy)) #returns a subset of expressions with the query pattern, this finds sin(thx) too
                sub_cos = expr.find(sp.cos(thx + sgn * thy))
                    
                found = False
                while len(sub_sin) > 0 and not found:
                    sin_expr = sub_sin.pop()
                    d = sin_expr.match(sp.sin(thx + sgn * thy))
                    if d[thx] != 0 and d[sgn] != 0 and d[thy] != 0: #has to be joint variable
                        found = True

                    
                while len(sub_cos) > 0 and not found:
                    cos_expr = sub_cos.pop()
                    d = cos_expr.match(sp.cos(thx + sgn * thy))
                    if d[thx] != 0 and d[sgn] != 0 and d[thy] != 0:
                        found = True
                    
                if found:
                    success_flag = True
                    th_xy = find_xy(d[thx], d[thy])
                     #if not exists in the unknown list (this requires proper hashing), create variable
                    if th_xy not in unkn_sums_sym:
                        print "found 'joint' (sumofangle) variable: "
                        print th_xy
                        #  try moving soa equation to Tm.auxeqns
                        unkn_sums_sym.add(th_xy) #add into the joint variable set
                        newjoint = unknown(th_xy)
                        #newjoint.joint_eq = d[thx] + d[sgn] * d[thy]
                        unknowns.append(newjoint) #add it to unknowns list 
                        tmpeqn = kequation(th_xy, d[thx] + d[sgn] * d[thy])
                        print 'sumofanglesT: appending ', tmpeqn
                        matr_equ.auxeqns.append(tmpeqn)
                        print d[thx] + d[sgn]*d[thy]
                        #Tmatrix = Tmatrix.subs(d[thx] + d[sgn] * d[thy], th_xy) #substitute all thx +/- thy expression with th_xy
                        matr_equ.Ts = matr_equ.Ts.subs(d[thx] + d[sgn] * d[thy], th_xy) #substitute all thx +/- thy expression with th_xy
                        matr_equ.Td = matr_equ.Td.subs(d[thx] + d[sgn] * d[thy], th_xy) #substitute all thx +/- thy expression with th_xy
       
        tick.blackboard.set("Tm", matr_equ)# we've got to keep the blackboard tags standard
        tick.blackboard.set("unknowns", unknowns)# we've got to keep the blackboard tags standard
        
        if(success_flag):
            return b3.SUCCESS  # when does this ID return FAILURE????
        else:
            return b3.FAILURE
        
class test_sum_id(b3.Action):
    
    # new test not based on a complete robot 
    #   (simpler to understnand what is correct output)
    #
    def tick(self, tick):
        variables = []
        x = sp.var('th_1 th_2 th_3 th_4 th_5 th_6')
        test_id = tick.blackboard.get('test_id')
        
        i = 1
        for v in x:
            unk = unknown(v)
            variables.append(unk)
            unk.n = i
            i += 1
            
        
        Ts = sp.zeros(4)
        
        if test_id == 1: #thxy = thx + thy
            Ts[1,1] =  sp.sin(th_1)*sp.cos(th_2)+sp.cos(th_1)*sp.sin(th_2)  # sin(th12)
            Ts[1,2] =  sp.cos(th_1)*sp.cos(th_2)-sp.sin(th_1)*sp.sin(th_2)  # cos(th12)
            Ts[2,1] =  l_1*sp.sin(th_1)*sp.cos(th_2)+l_1*sp.cos(th_1)*sp.sin(th_2)+l_2*sp.sin(th_3)  # l_1sin(th12)+l2sin(th3)
            Ts[2,2] = sp.cos(th_6)*(sp.cos(th_4)*sp.cos(th_5)*(sp.cos(th_2)*sp.cos(th_3)-sp.sin(th_2)*sp.sin(th_3)) + \
                sp.sin(th_5)*(-1)*(sp.cos(th_2)*sp.sin(th_3)+sp.sin(th_2)*sp.cos(th_3)) ) - \
                sp.sin(th_4)*sp.sin(th_6)*(sp.cos(th_2)*sp.cos(th_3)-sp.sin(th_2)*sp.sin(th_3))
            Ts[3,1] =  0
            Ts[3,2] =  0
            
        if test_id == 2: #thxy = thx - thy
            Ts[1,1] = sp.sin(th_1)*sp.cos(th_2) - sp.cos(th_1)*sp.sin(th_2) # sin(th1-2)
            Ts[1,2] = sp.cos(th_1)*sp.cos(th_2) + sp.sin(th_1)*sp.sin(th_2) # cos(th1-2)
            Ts[2,1] = l_1*sp.sin(th_1)*sp.cos(th_2) - l_1*sp.cos(th_1)*sp.sin(th_2)+l_2*sp.sin(th_3)
            Ts[2,2] = sp.cos(th_6)*(sp.cos(th_4)*sp.cos(th_5)*(sp.cos(th_2)*sp.cos(th_3)-sp.sin(th_2)*sp.sin(th_3)) + \
                sp.sin(th_5)*(-1)*(sp.cos(th_2)*sp.sin(th_3)+sp.sin(th_2)*sp.cos(th_3)) ) - \
                sp.sin(th_4)*sp.sin(th_6)*(sp.cos(th_2)*sp.cos(th_3)-sp.sin(th_2)*sp.sin(th_3))
                
            Ts[0,1] = sp.sin(th_4)*sp.cos(th_5) - sp.cos(th_4)*sp.sin(th_5) # sin(th4-5)
        
        Ts = sp.simplify(Ts)  #  this actually converts sum of angles but doesn't create new vars.
        
        Td = ik_lhs()
        Tm = matrix_equation(Td,Ts)
            #write on blackboard
        tick.blackboard.set("Robot", Robot())
        tick.blackboard.set("Tm", Tm)
        tick.blackboard.set("unknowns", variables)     
        return b3.SUCCESS

    

# after joint varible th_xy and th_x are solved, solve th_y = th_xy - th_y  (for thxy = thx + thy)
# 
class sum_solve(b3.Action):
    def tick(self, tick):
        unknowns = tick.blackboard.get("unknowns")
        R = tick.blackboard.get('Robot')
        Tm = tick.blackboard.get('Tm')
        solvedtag = False
           
        thx = sp.Wild('thx')
        thy = sp.Wild('thy')
        sgn = sp.Wild('sgn')
        
        #for unk in unknowns:
            #if unk.n == 0 and unk.solved: #joint varible has order of 0

        if len(Tm.auxeqns) > 0:
            for e in Tm.auxeqns:
                #d = unk.joint_eq.match(thx + sgn * thy)
                print e
                d = e.RHS.match(thx + sgn * thy)
                
                unka = find_obj(d[thx], unknowns)
                #print unka
                unkb = find_obj(d[thy], unknowns)
                #print unkb
                if unka == None:
                    print "variable %s doesn't exist" %(d[thx])
                elif unkb == None:
                    print "variable %s doesn't exist" %(d[thy])
                else:

                    if unka.solved and (not unkb.solved):
                        sol = (e.LHS - unka.symbol)/d[sgn]
                        unkb.solutions.append(sol)
                        unkb.nsolutions = 1
                        unkb.set_solved(R, unknowns)
                        solvedtag = True
                        
                        if self.BHdebug:
                            print "I'm solving %s from joint variable %s" % (unkb.symbol, e.LHS)
                            print "solution: %s" % sol
                        
                    elif unkb.solved and (not unka.solved):
                        sol = e.LHS - d[sgn] *unkb.symbol
                        unka.solutions.append(sol)
                        unka.nsolutions = 1
                        unka.set_solved(R, unknowns)
                        solvedtag = True
                        
                        if self.BHdebug:
                            print "I'm solving %s from joint variable %s" % (unka.symbol, e.LHS)
                            print "solution: %s" % sol
        #tick.blackboard.set('test_id', test_id)
        tick.blackboard.set('unknowns', unknowns)
        tick.blackboard.set('Robot', R)
        if solvedtag:
            return b3.SUCCESS
        
        return b3.FAILURE
            
class test_sum_solve(b3.Action):
    def tick(self, tick):
        unknowns = tick.blackboard.get('unknowns')
        R = tick.blackboard.get('Robot')
        #mark joint var as solved
        for unk in unknowns:
            if unk.n == 0:
                unk.solved = True
        
        th_1_obj = find_obj(th_1, unknowns)
        th_1_obj.set_solved(R, unknowns)
        
        th_5_obj = find_obj(th_5, unknowns)
        th_5_obj.set_solved(R, unknowns)
        tick.blackboard.set('Robot', R)
        tick.blackboard.set('unknowns', unknowns)
        return b3.SUCCESS

if __name__ == "__main__":
    ik_tester = b3.BehaviorTree()

    s1 = sum_id()
    st1 = test_sum_id()
    test = b3.Sequence([st1, s1])
    
    testNum = 1
    
    if testNum==1:

        print '\n\n\n         - - -       Sum of Angles Test 1        - - -\n\n'

        ik_tester.root = test

        bb = b3.Blackboard()
        bb.set('test_id', 1)

        ik_tester.tick("Test the sum of angle identifier", bb)

        unknowns = bb.get("unknowns")

        print "----------"
        print "test results"

        print "found sum of angles: "
        for unkn in unknowns:
            if unkn.n == 0:
                print unkn
                
        print "new T matrix"

        m = bb.get("Tm")
        sp.pprint(m.Ts)
         
         
        fs = ' sum of angles transform FAIL'
        assert(m.Ts[1,1] == sp.sin(th_12)), fs
        assert(m.Ts[1,2] == sp.cos(th_12)), fs
        assert(m.Ts[2,1] == l_1*sp.sin(th_12)+l_2*sp.sin(th_3)), fs
        
        assert(m.Ts[2,2].has(sp.sin(th_23)*sp.sin(th_5))), fs

        # the computed answer
        A = m.Ts[2,2]
        # the right answer
        B =  -(sp.sin(th_23)*sp.sin(th_5) - sp.cos(th_23)*sp.cos(th_4)*sp.cos(th_5))*sp.cos(th_6) - sp.sin(th_4)*sp.sin(th_6)*sp.cos(th_23)  
        
        A = sp.expand(A)
        B = sp.expand(B)
        
        print ' - - -LHS: - - '
        print         A
        print ' - - - - - '
        print ' - - -RHS: - - '
        print       B
        print ' - - - - - '
        assert (A==B), ' Special A==B assert'
    
    testNum = 2
    
    if testNum == 2:
        print '\n\n\n         - - -       Sum of Angles Test 2        - - -\n\n'

        s_solve = sum_solve()
        s_solve.BHdebug = True
        s_solve.Name = "sum of angle solve"

        s_solve_t = test_sum_solve()
        
        ik_tester2 = b3.BehaviorTree()
        test2 = b3.Sequence([st1, s1, s_solve_t, s_solve])
        ik_tester2.root = test2
        bb2 = b3.Blackboard()
        
        bb2.set('test_id', 2)
        
        ik_tester2.tick("test sum_solve", bb2)
        unknowns = bb2.get("unknowns")
       
        for unk in unknowns:
            if unk.solved:
                print unk.symbol
                print unk.solutions
                
        test_id = bb2.get('test_id')
        fs = 'Sum-of-Angles solver failed'
        assert (unknowns[0].solved),fs
        assert (unknowns[1].solved),fs #th_2 = th_12 - th_1
        assert (unknowns[2].solved),fs #th_3 = th_23 - th_2
        
        if test_id == 1:
            assert (unknowns[1].solutions[0] == -th_1 + th_12), fs
            assert (unknowns[2].solutions[0] == -th_2 + th_23), fs
            
        if test_id == 2:
            fs2 = 'Sum-of-Angles solver for thxy = thx - thy failed'
            assert (unknowns[1].solutions[0] == th_1 - th_12), fs2 #th_2 = th_1 - th_12
            assert (unknowns[2].solutions[0] == -th_2 + th_23), fs2 #th_3 = th_23 - th_2
            assert (unknowns[3].solutions[0] == th_45 + th_5), fs2 #th_4 = th_45 + th5
            
    print '\n\n\n             Sum Transform + Solver Passes ALL tests \n\n'
    
    
