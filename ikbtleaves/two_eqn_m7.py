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
import b3 as b3
import ikbtbasics.eqn_sanity as _eqsan


Aw = sp.Wild('Aw')
Bw = sp.Wild('Bw')
Cw = sp.Wild('Cw')
Dw = sp.Wild('Dw')
Ew = sp.Wild('Ew')
 
thx = sp.Wild('thx')
thy = sp.Wild('thy')
thz = sp.Wild('thz')


def _partner_of(u, e1, e2):
    """Is e2 the PARTNER of e1 -- and if so, written the way simu_solver reads
       it?  Returns e2, or -e2, or None when e2 is not a partner at all.

       THE PAIR.  Two equations solve together for u when they read

           e1:   0 = A*sin(u) + B*cos(u) - C
           e2:   0 = A*cos(u) - B*sin(u) - D

       with the SAME A and B.  simu_solver reads A and B off e1 alone, so e2
       has to supply exactly those, and the two then collapse to one atan2.

       WHY A SIGN FLIP IS ALLOWED.  Every equation here is stored as
       "expression = 0", and `0 = X` and `0 = -X` state the same thing -- both
       hold exactly when X is zero.  So each equation has two equally true
       spellings, and only one of them is the spelling simu_solver reads
       coefficients from;  handed the other it would extract -A and -B and
       produce nonsense.  Hence: return whichever spelling fits, not merely a
       yes/no.  (That is what "canonical form" meant in this function's former
       name -- the one agreed spelling out of the two.)

       A == B == 0 means u appears in e1 with no sin or cos of it at all;  both
       atan2 arguments would vanish and the "solution" would be nan.  Reject.

       Private: callers want solvable_pair(), which does not have to know
       which equation plays e1."""
    A = e1.coeff(sp.sin(u))
    B = e1.coeff(sp.cos(u))
    if A == 0 and B == 0:
        return None
    a2 = e2.coeff(sp.cos(u))
    b2 = -e2.coeff(sp.sin(u))
    if a2 == A and b2 == B:
        return e2
    if a2 == -A and b2 == -B:
        return -e2
    return None


def solvable_pair(u, ea, eb):
    '''These two equations in solver order -- (e1, e2) -- or None if they do
       not solve together for u.

           e1:   0 = A*sin(u) + B*cos(u) - C
           e2:   0 = A*cos(u) - B*sin(u) - D

       ORDER MATTERS, AND THE CALLER CANNOT KNOW IT.  A and B come from e1
       alone, so the two equations are not interchangeable -- but a caller
       holding an unordered pair out of the equation list has no way to tell
       which is which.  BOTH assignments are tried here, so the question asked
       is the one the caller actually has:  do these two solve together, and
       in what order?

       Rejects, before either assignment is tried, a pair that is one
       statement twice:  e and -e state the same thing, so the "pair" carries
       no second constraint and the solve would be underdetermined.'''

    if ea - eb == 0 or ea + eb == 0:
        return None

    for e1, e2 in ((ea, eb), (eb, ea)):
        second = _partner_of(u, e1, e2)
        if second is not None:
            return e1, second
    return None


class simu_id(b3.Action):
    #  Find a pair of equations in ONE unknown of the form
    #
    #    c = A sin(th1) + B cos(th1)
    #    d = A cos(th1) - B sin(th1)     (nice arctan solution)
    #
    #  Which of the two plays which role is solvable_pair()'s business, not
    #  this loop's.
  
    def tick(self, tick):
        curr_unk = tick.blackboard.get('curr_unk')
        unknowns = tick.blackboard.get('unknowns')
        R = tick.blackboard.get('Robot')

        one_unk = tick.blackboard.get('eqns_1u')

        eqn_list= []
        if not curr_unk.solved:
            for e in one_unk:
                e_flat = e.RHS - e.LHS
                e_flat = e_flat.expand()
                # must carry a sin or cos of the unknown: this solver reads
                # coefficients via .coeff(sin(u)) / .coeff(cos(u)), so a purely
                # algebraic appearance of u (e.g. an SOA definition) yields
                # A == B == 0 and an atan2(0,0) == nan "solution".
                # 'or' not 'and': a valid pair may have B == 0, in which case
                # e1 carries only sin(u) and e2 only cos(u).
                if e_flat.has(sp.sin(curr_unk.symbol)) or e_flat.has(sp.cos(curr_unk.symbol)):    
                    e_flat= e_flat.collect(sp.sin(curr_unk.symbol))
                    e_flat= e_flat.collect(sp.cos(curr_unk.symbol))
                    if e_flat not in eqn_list:
                        eqn_list.append(e_flat)

            if self.BHdebug:
                print("potential list: ")
                print(eqn_list)

            for i in range(len(eqn_list)):
                cand1 = eqn_list[i]

                if self.BHdebug:
                    print("considering eqn: ", cand1)
                    print("  (a, b) = ", cand1.coeff(sp.sin(curr_unk.symbol)),
                          '\n           ', cand1.coeff(sp.cos(curr_unk.symbol)))

                for j in range(i+1, len(eqn_list)):
                    cand2 = eqn_list[j]

                    pair = solvable_pair(curr_unk.symbol, cand1, cand2)
                    if pair is None:
                        continue
                    eq1, eq2 = pair

                    if self.BHdebug:
                        if eq1 is cand2:
                            print("reverse order")
                        print("found two eqns in one unknown")
                        print(eq1)
                        print(eq2)
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
        #  REFUSE AN EQUATION THAT DOES NOT CONSTRAIN THIS VARIABLE.
        #  `0 = 0` and friends are satisfied by every value, so "solving" one
        #  yields an arbitrary answer that looks like a result.  See
        #  ikbtbasics/eqn_sanity.py for what counts as constraining.
        #
        #  Reporting FAILURE rather than a bogus solution is the leaf's whole
        #  contribution here:  a caller can respond to FAILURE, and cannot
        #  detect a confidently wrong answer.
        if not _eqsan.constrains(curr_unk.eqntosolve, curr_unk.name, R):
            _eqsan.reject(curr_unk, 'simultaneous eqn solver')
            return b3.FAILURE



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

#######################################################################
#  Test code:
class TestSolver005(unittest.TestCase):
    '''Simultaneous-equation solver (paper solver #5).

       Solves the pair
             C = A*sin(u) + B*cos(u)
             D = A*cos(u) - B*sin(u)
       via u = atan2(A*C - B*D, A*D + B*C).'''

    def setUp(self):
        self.DB = False  # debug flag
        print('\n\n===============  Test simultaneous eqn Solver  =====================')
        return

    def runTest(self):
        self.test_m7_puma_th23()
        self.test_m7_numeric_roundtrip()
        self.test_m7_accepts_negated_second_eqn()
        self.test_m7_accepts_zero_B()
        self.test_m7_rejects_algebraic_pair()
        self.test_m7_rejects_mixed_sign_pair()
        self.test_m7_solvable_pair_tries_both_role_assignments()

    #####################################################################
    #  helpers

    def run_leaf(self, exprs, target, presolved=()):
        '''Tick simu_id + simu_solver once over the equation list `exprs`
           (each meaning "expr == 0"), trying to solve unknown `target`.
           `presolved` are symbols already solved by earlier leaves -- the
           realistic state, since an equation only reaches eqns_1u once every
           OTHER unknown in it is solved (count_unknowns skips solved ones).
           Returns (status, curr_unk).'''
        u_target = unknown(target)
        unknowns = [u_target]
        for s in presolved:
            up = unknown(s)
            up.solved = True
            up.nsolutions = 1     # nsolutions defaults to 0, which would zero
            up.solutions = [s]    # out nversions in set_solved
            unknowns.append(up)

        SimuEqnID = simu_id()
        SimuEqnID.Name = 'Simultaneous Eqn ID'
        SimuEqnID.BHdebug = self.DB
        SimuEqnSolve = simu_solver()
        SimuEqnSolve.Name = 'Simultaneous Eqn solver'
        SimuEqnSolve.BHdebug = self.DB

        ik_tester = b3.BehaviorTree()
        ik_tester.root = b3.Sequence([SimuEqnID, SimuEqnSolve])

        bb = b3.Blackboard()
        bb.set('curr_unk', u_target)
        bb.set('eqns_1u', [kequation(0, e) for e in exprs])
        bb.set('Robot', Robot())
        bb.set('unknowns', unknowns)

        status = ik_tester.tick("testing two_eqn_m7", bb)
        return status, bb.get('curr_unk')

    def puma_th23_pair(self):
        '''The PUMA th_23 equation pair (Craig) -- the motivating case.'''
        sp.var('Px Py Pz th_1 th_23 th_3 a_3 a_2 d_4')
        exp1 = Pz*sp.sin(th_23) + a_2*sp.cos(th_3) + a_3 \
               + (-Px*sp.cos(th_1) - Py*sp.sin(th_1))*sp.cos(th_23)
        exp2 = Pz*sp.cos(th_23) - a_2*sp.sin(th_3) + d_4 \
               + (Px*sp.cos(th_1) + Py*sp.sin(th_1))*sp.sin(th_23)
        return exp1, exp2

    #####################################################################
    #  positive cases

    def test_m7_puma_th23(self):
        '''The leaf must fire on the PUMA th_23 pair and produce one solution.'''
        sp.var('th_1 th_23 th_3')
        exp1, exp2 = self.puma_th23_pair()
        fs = ' simultaneous eqn solver FAIL'

        status, u = self.run_leaf([exp1, exp2], th_23, presolved=[th_1, th_3])

        self.assertEqual(status, b3.SUCCESS, fs + ' (leaf did not fire)')
        self.assertTrue(u.solved, fs + ' (not marked solved)')
        self.assertEqual(u.nsolutions, 1, fs + ' (wrong solution count)')
        self.assertEqual(len(u.solutions), 1, fs)
        self.assertIn('simultaneous eqn', u.solvemethod, fs)
        # the solution must not still depend on the variable being solved
        self.assertFalse(u.solutions[0].has(th_23), fs + ' (solution contains th_23!)')

    def test_m7_numeric_roundtrip(self):
        '''Substitute numbers into the symbolic solution and check it really
           satisfies BOTH original equations.  Independent of expression form,
           so it catches sign errors that a symbolic comparison would miss.'''
        import math
        sp.var('Px Py Pz th_1 th_23 th_3 a_3 a_2 d_4')
        exp1, exp2 = self.puma_th23_pair()
        fs = ' simultaneous eqn numeric roundtrip FAIL'

        # The pair is only consistent when A^2+B^2 == C^2+D^2, so build the
        # numbers from a chosen true angle rather than picking them freely.
        th23_true = 0.7
        A_v, B_v = 1.1, -0.45
        C_v = A_v*math.sin(th23_true) + B_v*math.cos(th23_true)
        D_v = A_v*math.cos(th23_true) - B_v*math.sin(th23_true)

        th1_v, Py_v = 0.4, 0.0
        Pz_v = A_v                          # A  =  Pz
        Px_v = -B_v/math.cos(th1_v)         # B  = -(Px*cos(th_1) + Py*sin(th_1))
        th3_v, a2_v = -0.9, 0.8
        a3_v = -C_v - a2_v*math.cos(th3_v)  # C  = -(a_2*cos(th_3) + a_3)
        d4_v = a2_v*math.sin(th3_v) - D_v   # D  =   a_2*sin(th_3) - d_4

        vals = {Px: Px_v, Py: Py_v, Pz: Pz_v, th_1: th1_v,
                th_3: th3_v, a_2: a2_v, a_3: a3_v, d_4: d4_v}

        # sanity: the fixture itself is consistent at the true angle
        chk = dict(vals); chk[th_23] = th23_true
        self.assertAlmostEqual(float(exp1.subs(chk)), 0.0, places=9,
                               msg=fs + ' (fixture inconsistent)')
        self.assertAlmostEqual(float(exp2.subs(chk)), 0.0, places=9,
                               msg=fs + ' (fixture inconsistent)')

        status, u = self.run_leaf([exp1, exp2], th_23, presolved=[th_1, th_3])
        self.assertEqual(status, b3.SUCCESS, fs)

        got = float(u.solutions[0].subs(vals))
        self.assertAlmostEqual(got, th23_true, places=9,
                               msg=fs + ' (recovered wrong angle)')

        # and the recovered angle satisfies both original equations
        back = dict(vals); back[th_23] = got
        self.assertAlmostEqual(float(exp1.subs(back)), 0.0, places=9, msg=fs)
        self.assertAlmostEqual(float(exp2.subs(back)), 0.0, places=9, msg=fs)

    def test_m7_solvable_pair_tries_both_role_assignments(self):
        """The leaf must not care which order the two equations arrive in.

           A and B are read from e1 ALONE, so the pair is NOT symmetric -- but
           the caller pulls two equations out of a list and cannot know which
           should play e1.  solvable_pair() therefore tries both.

           NOTE both assignments are legitimate, and they are NOT the same
           assignment.  Handed (second, first) the function keeps `second` as
           e1 and returns -first as e2, which is canonical for A' = -B,
           B' = A -- a pi/2 relabelling of the same pair.  It cancels in the
           atan2, so BOTH orders solve to the same angle.  That equivalence,
           not a particular ordering, is what is asserted here.

           This is the path the old code covered with a second, open-coded
           call to the partner test;  nothing tested it."""
        sp.var('AA BB CC DD th_23')
        fs = ' solvable_pair role-assignment FAIL'
        u = th_23
        first  = AA*sp.sin(u) + BB*sp.cos(u) - CC
        second = AA*sp.cos(u) - BB*sp.sin(u) - DD

        fwd = solvable_pair(u, first, second)
        rev = solvable_pair(u, second, first)
        self.assertIsNotNone(fwd, fs + ' (rejected a valid pair)')
        self.assertIsNotNone(rev, fs + ' (rejected the same pair reversed)')

        #  each returned pair must be internally canonical: e2 as returned is
        #  what _partner_of would produce for that e1
        for label, (e1, e2) in (('forward', fwd), ('reversed', rev)):
            self.assertEqual(sp.simplify(_partner_of(u, e1, e2) - e2), 0,
                             fs + ' (%s pair is not self-consistent)' % label)

        #  AND THE ANSWER IS THE SAME EITHER WAY -- end to end through the leaf
        th = 0.7
        subs = {AA: 2.0, BB: 3.0}
        subs[CC] = float((AA*sp.sin(u) + BB*sp.cos(u)).subs(subs).subs({u: th}))
        subs[DD] = float((AA*sp.cos(u) - BB*sp.sin(u)).subs(subs).subs({u: th}))

        angles = []
        for order in ([first, second], [second, first]):
            status, unk = self.run_leaf(order, u)
            self.assertEqual(status, b3.SUCCESS,
                             fs + ' (leaf failed on one equation order)')
            angles.append(float(unk.solutions[0].subs(subs)))

        self.assertAlmostEqual(angles[0], th, places=9,
                               msg=fs + ' (forward order got %g)' % angles[0])
        self.assertAlmostEqual(angles[1], th, places=9,
                               msg=fs + ' (reversed order got %g)' % angles[1])

        #  one statement twice is NOT a pair -- it adds no second constraint
        self.assertIsNone(solvable_pair(u, first, first), fs + ' (e with e)')
        self.assertIsNone(solvable_pair(u, first, -first), fs + ' (e with -e)')

    def test_m7_accepts_negated_second_eqn(self):
        '''e2 and -e2 both state the same thing, so a uniformly negated second
           equation is still solvable -- and must give the canonical answer.'''
        sp.var('AA BB CC DD th_23')
        fs = ' simultaneous eqn negated-e2 FAIL'
        eq1 = AA*sp.sin(th_23) + BB*sp.cos(th_23) - CC
        eq2 = -(AA*sp.cos(th_23) - BB*sp.sin(th_23) - DD)   # uniformly negated

        status, u = self.run_leaf([eq1, eq2], th_23)

        self.assertEqual(status, b3.SUCCESS, fs)
        expected = sp.atan2(AA*CC - BB*DD, AA*DD + BB*CC)
        self.assertEqual(sp.simplify(u.solutions[0] - expected), 0, fs)

    def test_m7_accepts_zero_B(self):
        '''B == 0 is legal: e1 then carries only sin(u) and e2 only cos(u).
           Guards the sin/cos screen against being tightened to "and".'''
        sp.var('AA CC DD th_23')
        fs = ' simultaneous eqn B==0 FAIL'
        eq1 = AA*sp.sin(th_23) - CC
        eq2 = AA*sp.cos(th_23) - DD

        status, u = self.run_leaf([eq1, eq2], th_23)

        self.assertEqual(status, b3.SUCCESS, fs + ' (rejected a valid pair)')
        expected = sp.atan2(AA*CC, AA*DD)
        self.assertEqual(sp.simplify(u.solutions[0] - expected), 0, fs)

    #####################################################################
    #  negative cases -- these are the two bugs

    def test_m7_rejects_algebraic_pair(self):
        '''Equations mentioning u with no sin/cos of it give A == B == 0, so
           both atan2 arguments vanish and the "solution" is nan.  The screen
           used to be `has(u) and has(u)`, which let these through.'''
        sp.var('th_23 l_1 l_2')
        fs = ' simultaneous eqn algebraic-pair FAIL'
        eq1 = th_23 - l_1
        eq2 = th_23 - l_2

        status, u = self.run_leaf([eq1, eq2], th_23)

        self.assertEqual(status, b3.FAILURE, fs + ' (accepted an algebraic pair)')
        self.assertFalse(u.solved, fs + ' (marked solved with a nan solution)')

    def test_m7_rejects_mixed_sign_pair(self):
        '''A-sign and B-sign were tested independently, admitting pairs the
           solver cannot actually handle.  Here e2 has A negated but B not;
           simu_solver would then compute D = 2*A*cos(u) + D, still a function
           of u, so the resulting atan2 is not a solution at all.'''
        sp.var('AA BB CC DD th_23')
        fs = ' simultaneous eqn mixed-sign FAIL'
        eq1 = AA*sp.sin(th_23) + BB*sp.cos(th_23) - CC
        eq2 = -AA*sp.cos(th_23) - BB*sp.sin(th_23) - DD     # A flipped, B not

        status, u = self.run_leaf([eq1, eq2], th_23)

        self.assertEqual(status, b3.FAILURE, fs + ' (accepted a mixed-sign pair)')
        self.assertFalse(u.solved, fs)
        if u.solved:   # extra diagnosis if it ever regresses
            self.assertFalse(u.solutions[0].has(th_23),
                             fs + ' (solution still contains th_23)')


def run_test():
    suite2 = unittest.TestLoader().loadTestsFromTestCase(TestSolver005)
    unittest.TextTestRunner(verbosity=2).run(suite2)


if __name__ == "__main__":

    print('\n\n===============  Test simultaneous eqn solver =====================')
    run_test()

            

