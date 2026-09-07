#!/usr/bin/python
#
#   invariant_gen.py --  the kinematic invariant generator
#
#   Every other IKBT solver shares one precondition:  the target unknown must
#   ALREADY be isolated in a conveniently-shaped equation.  The system
#   recognizes solvable forms;  it does not manufacture them.  x2y2_transform
#   was the sole exception, and it is a hand-written special case of a much more
#   general move.
#
#   This leaf generalizes it.  Both sides of a matrix equation  Td = Ts  are the
#   same 4x4 transform written two ways, so ANY scalar function f gives a valid
#   new scalar equation  f(Td) = f(Ts).  Choose f invariant under the trailing
#   rotations and the result typically carries FEWER unknowns than any raw
#   element equation.  This is Pieper's classic opening move, and how the
#   position/orientation decoupling is found for arms that admit it.
#
#   The leaf produces no solutions of its own -- it is a pure equation generator
#   feeding the existing solvers -- so it introduces no new solution
#   multiplicities and does not disturb Robot.create_solution_set().
#
#   Yields, on the shipped FK pickles:  Puma's meqn 2 ||P||^2 drops to ONE
#   unknown, Kawasaki's likewise;  KinovaLite gains nothing.  See
#   IKdocs/DEV_NOTES.md.
#
#   Copyright 2026 University of Washington
#
#   Developed by Blake Hannaford
#   BioRobotics Lab, University of Washington

import unittest

import sympy as sp

from ikbtfunctions.helperfunctions import *
from ikbtbasics.kin_cl import *
from ikbtbasics.ik_classes import *     # special classes for Inverse kinematics in sympy

import b3 as b3          # behavior trees


#####################################################################
#
#   Tunables
#

#  How many matrix equations to mine.  None = all of them.  The premultiplied
#  equations (Craig's  T_01^-1 * Td = T_12*...  and friends) are where the good
#  invariants live, so do not lower this without measuring.
N_MEQNS = None

#  Only equations the solvers can actually consume are worth generating.
#  1 or 2 unknowns land in L1/L2; 3+ land in L3p, which no solver reads, so
#  admitting them is pure cost on every later scan.
MAX_UNKNOWNS = 2

#  Reject invariants that simplify to a monster.  A huge equation slows every
#  downstream scan and produces an unreadable report even if it does solve.
MAX_OPS = 80


#####################################################################
#
#   The invariant registry.
#
#   Each entry is (name, function), where function(Td, Ts) returns the pair
#   (lhs, rhs).  To add or remove a family, edit INVARIANTS -- that is the
#   entire extension mechanism.
#

def inv_norm_P(Td, Ts):
    '''||P||^2.  Invariant under any rotation applied after the translation,
       so it annihilates the trailing joints.  This is the workhorse and it is
       the generalization of what x2y2_transform does by pattern match.'''
    return (sum(Td[i, 3]**2 for i in range(3)),
            sum(Ts[i, 3]**2 for i in range(3)))


def inv_trace_R(Td, Ts):
    '''trace(R).  Equals 1 + 2*cos(phi) for rotation angle phi, so it collapses
       a whole rotation sequence into one scalar.  Earns nothing on Puma or
       Kawasaki (4-5 unknowns, gated out) but it is the classic isolator for a
       spherical wrist, and it is cheap.'''
    return (sum(Td[i, i] for i in range(3)),
            sum(Ts[i, i] for i in range(3)))


def _make_P_dot_col(k):
    '''P . (column k of R):  the projection of the position vector on a tool
       axis.  Non-vacuous, unlike column-column dot products -- see the note
       below.  Its payoff is UNPROVEN:  it never got below 3 unknowns on either
       Puma or Kawasaki.  Kept because it is one line to drop.'''
    def f(Td, Ts):
        return (sum(Td[i, 3]*Td[i, k] for i in range(3)),
                sum(Ts[i, 3]*Ts[i, k] for i in range(3)))
    f.__name__ = 'inv_P_dot_col%d' % k
    return f


#  NOTE:  column-COLUMN dot products (col_a . col_b) are deliberately absent.
#  Ts is built from real rotation matrices, so those simplify to a literal 0 or
#  1 and the resulting equation carries ZERO unknowns -- vacuous.  Only products
#  involving the position column say anything.

INVARIANTS = [
    ('|P|^2',    inv_norm_P),
    ('trace(R)', inv_trace_R),
    ('P.col0',   _make_P_dot_col(0)),
    ('P.col1',   _make_P_dot_col(1)),
    ('P.col2',   _make_P_dot_col(2)),
]


#####################################################################
#
#   Sum-of-angles back-substitution
#

def soa_subs_dict(R, unknowns):
    '''Build  {th_23: th_2 + th_3, ...}  from the sum-of-angles definitions in
       R.kequation_aux_list.

       sum_of_angles_sub() files those as kequation(th_new, th_i + th_j), i.e.
       LHS is a bare symbol.  The aux list ALSO holds equations from
       x2y2_transform (and from this leaf), whose LHS is an arbitrary
       expression, so filter on shape:  bare Symbol LHS that names a declared
       unknown, RHS an Add of symbols.'''

    unk_syms = set(u.symbol for u in unknowns)
    d = {}
    for e in R.kequation_aux_list:
        lhs, rhs = e.LHS, e.RHS
        if not isinstance(lhs, sp.Symbol):
            continue
        if lhs not in unk_syms:
            continue
        if not isinstance(rhs, sp.Add):
            continue
        if lhs in rhs.free_symbols:      # would be a circular definition
            continue
        d[lhs] = rhs
    return d


def best_form(expr, unknowns, soa_defs):
    '''Return the more useful of {expr, expr with SOA variables expanded}.

       sympy cannot reduce  cos(th_2 - th_23)  because it does not know that
       th_23 == th_2 + th_3.  Substituting the definition and re-simplifying
       turns it into cos(th_3) -- which is what takes the Puma meqn-1 ||P||^2
       invariant from 2 unknowns down to 1.

       The substitution is all-or-nothing, and going the other way (expanding a
       th_23 that was helping) makes things worse, so both forms are scored and
       the better one wins:  fewer unknowns first, then fewer operations.  That
       ordering also means an expression whose expansion re-exposes a genuine
       sum of angles keeps its collapsed form automatically, with no need to
       call sum_of_angles_sub() -- which mutates R and the unknowns list and has
       no business running inside a candidate evaluation.'''

    def score(e):
        return (count_unknowns(unknowns, e), sp.count_ops(e))

    best = expr
    if soa_defs and any(s in expr.free_symbols for s in soa_defs):
        alt = sp.simplify(expr.subs(soa_defs))
        if score(alt) < score(best):
            best = alt
    return best


#####################################################################
#
#   Redundancy screen
#

def element_forms(meqns):
    '''Flatten every element equation of every mined matrix equation to
       (Td[i,j] - Ts[i,j]), expanded, for comparison against candidates.'''

    out = []
    for Meq in meqns:
        for i in range(3):
            for j in range(4):
                e = sp.expand(Meq.Td[i, j] - Meq.Ts[i, j])
                if e != 0:
                    out.append(e)
    return out


def is_redundant(flat, known):
    '''True if the candidate (already flattened to LHS-RHS, expanded) merely
       restates something already on the books.

       `known` holds the flattened forms of every element equation plus every
       candidate kept so far.  Two reasons both matter:

       - Some invariants DEGENERATE.  P . col_k reduces to the bare element
         equation for P_k whenever the rotation block is (or simplifies to) the
         identity, and scan_for_equations() already put that on the blackboard.
         Re-emitting it in the aux list is worse than useless:  updateL files aux
         equations as kequation(0, LHS-RHS), a different SHAPE from the
         kequation(Td, Ts) the scanner produces, so the existing "if e1 not in
         self.l1" dedup does not catch it and the duplicate survives.

       - Different matrix equations produce the SAME relation, often with every
         sign flipped.  Measured on Puma:  two of nine generated equations were
         an exact negated pair.  kequation equality does not see that.

       Hence both an exact and a negated match count -- e1 and -e1 state the
       same thing.  Compared on expanded forms rather than via simplify():
       element equations are already near-expanded, and this runs for every
       candidate.'''

    if flat == 0:
        return True                     # vacuous: 0 == 0
    for e in known:
        if sp.expand(flat - e) == 0 or sp.expand(flat + e) == 0:
            return True
    return False


#####################################################################
#
#   Generation
#

def generate_invariants(R, unknowns, debug=False):
    '''Mine R.mequation_list for scalar invariants worth solving.

       Returns a list of kequation, NOT yet appended to anything -- keeping the
       computation separate from the side effect is what makes it testable.'''

    meqns = R.mequation_list
    if N_MEQNS is not None:
        meqns = meqns[:N_MEQNS]

    soa_defs = soa_subs_dict(R, unknowns)
    if debug:
        print('invariant_gen: SOA definitions: ', soa_defs)

    #  Everything already stated, flattened for the redundancy screen:  the raw
    #  element equations, plus whatever is already in the aux list (SOA
    #  definitions, x2y2's output, and this leaf's own output from an earlier run).
    known = element_forms(meqns)
    known += [sp.expand(e.LHS - e.RHS) for e in R.kequation_aux_list]

    keepers = []
    for k, Meq in enumerate(meqns):
        for name, f in INVARIANTS:
            lhs_raw, rhs_raw = f(Meq.Td, Meq.Ts)

            rhs = best_form(sp.simplify(rhs_raw), unknowns, soa_defs)
            lhs = best_form(sp.simplify(lhs_raw), unknowns, soa_defs)

            nunk = count_unknowns(unknowns, rhs) + count_unknowns(unknowns, lhs)
            nops = sp.count_ops(rhs) + sp.count_ops(lhs)

            if debug:
                print('invariant_gen: meqn %d  %-9s unks=%d ops=%d' % (k, name, nunk, nops))

            #  nunk == 0 means the invariant is vacuous (no unknown to solve for).
            #  nunk > MAX_UNKNOWNS lands in L3p, which no solver consumes.
            if nunk < 1 or nunk > MAX_UNKNOWNS:
                continue
            if nops > MAX_OPS:
                if debug:
                    print('invariant_gen:    -- too big (%d ops), discarding' % nops)
                continue

            flat = sp.expand(lhs - rhs)
            if is_redundant(flat, known):
                if debug:
                    print('invariant_gen:    -- already stated, discarding')
                continue

            newe = kequation(lhs, rhs)
            known.append(flat)

            if debug:
                print('invariant_gen:    ++ KEEPING meqn %d %s:  %s = %s' % (k, name, lhs, rhs))
            keepers.append(newe)

    return keepers


class invariant_gen(b3.Action):
    '''BT leaf wrapper.  A transform, not a solver:  no ID/solve split, because
       it produces equations rather than solutions.  x2z2_transform is the
       precedent for a transform living in the worktools Priority.

       The class default is OFF;  bt_assembly.make_leaves() switches the
       tree's instance on.  Set nodes['invariantGen'].enabled = False to get
       the inert behaviour back.  Costs and benefits: IKdocs/DEV_NOTES.md.'''

    enabled = False

    def tick(self, tick):
        if not self.enabled:
            return b3.FAILURE

        #  One shot.  The invariants of a matrix equation do not change, so a
        #  second run can only re-emit duplicates -- and a leaf that keeps
        #  returning SUCCESS makes the enclosing Priority succeed forever.
        #
        #  The flag lives on the blackboard, not on R:  R comes out of a pickle
        #  and must not be assumed to carry attributes pickle never stored.
        if tick.blackboard.get('invariants_done'):
            return b3.FAILURE

        R = tick.blackboard.get('Robot')
        unknowns = tick.blackboard.get('unknowns')

        #  Nothing left to generate equations FOR.  This matters because of
        #  where the leaf sits:  last in the worktools Priority, it is reached
        #  only on a tick where every solver failed -- and on a robot that
        #  solves cleanly, the first such tick is the one after the last
        #  variable is solved.  Without the guard, Puma generated 9 large and
        #  useless equations (count_unknowns skips solved variables, so
        #  mostly-solved monsters slip under the <= 2 threshold).
        if all(u.solved for u in unknowns):
            if self.BHdebug:
                print('invariant_gen: every unknown already solved - nothing to do')
            return b3.FAILURE

        tick.blackboard.set('invariants_done', True)

        if self.BHdebug:
            print('\nrunning: ', self.Name)
            print('invariant_gen: mining %d matrix equations' % len(R.mequation_list))

        keepers = generate_invariants(R, unknowns, debug=self.BHdebug)

        if len(keepers) == 0:
            print('invariant_gen: no invariant beat the gate - nothing generated')
            return b3.FAILURE

        #  updateL (ikbtleaves/updateL.py:46) and Robot.scan_for_equations()
        #  both fold kequation_aux_list back into L1/L2/L3p, so appending here
        #  is all that is needed to hand these to the solvers.
        for e in keepers:
            R.kequation_aux_list.append(e)

        print('invariant_gen: generated %d new equation(s):' % len(keepers))
        for e in keepers:
            print('     ', e.LHS, ' = ', e.RHS)

        tick.blackboard.set('Robot', R)
        return b3.SUCCESS


#####################################################################
#####################################################################
#
#   Test code
#

class TestSolver012(unittest.TestCase):
    '''Kinematic invariant generator.'''

    def setUp(self):
        self.DB = False   # debug flag
        print('\n\n===============  Test invariant generator  =====================')
        return

    def runTest(self):
        self.test_invA_fires_on_puma_like_meqn()
        self.test_invB_numeric_roundtrip()
        self.test_invC_soa_backsub_wins()
        self.test_invD_declines_when_nothing_beats_the_gate()
        self.test_invE_is_one_shot()
        self.test_invF_rejects_oversized_invariant()
        self.test_invG_rejects_restatement_of_an_element_eqn()
        self.test_invH_rejects_negated_duplicate()
        self.test_invI_skips_when_everything_is_solved()
        self.test_invJ_dedups_candidates_within_one_run()
        self.test_invK_is_off_by_default()

    #################################################################
    #  helpers

    def puma_like_meqn(self):
        '''A stripped-down stand-in for a premultiplied Puma matrix equation.

           The position column is Craig's planar 2-link arm reaching into the
           th_23 wrist offset.  Every element equation has 2 unknowns, but
           ||P||^2 collapses to one, in th_3 -- exactly the real result measured
           on fk_eqns/Puma_pickle.p, meqn 2.'''
        sp.var('Px Py Pz th_2 th_3 th_23 a_2 a_3 d_3 d_4')

        Td = sp.zeros(4)
        Ts = sp.zeros(4)

        #  rotation block: identity on both sides -> trace is vacuous here, so
        #  ||P||^2 is the only invariant that can fire.  Keeps the test focused.
        for i in range(3):
            Td[i, i] = 1
            Ts[i, i] = 1

        Td[0, 3] = Px
        Td[1, 3] = Py
        Td[2, 3] = Pz
        Ts[0, 3] = a_2 + a_3*sp.cos(th_3) - d_4*sp.sin(th_3)
        Ts[1, 3] = d_3
        Ts[2, 3] = a_3*sp.sin(th_3) + d_4*sp.cos(th_3)

        return matrix_equation(Td, Ts)

    def build_robot(self, meqn, unknown_syms, aux=()):
        R = Robot()
        R.mequation_list = [meqn]
        R.kequation_aux_list = list(aux)
        unknowns = [unknown(s) for s in unknown_syms]
        return R, unknowns

    def run_leaf(self, R, unknowns, node=None, bb=None):
        '''Tick the leaf once.  Returns (status, R, bb) so a caller can tick the
           SAME node/blackboard again to exercise the one-shot guard.'''
        if node is None:
            node = invariant_gen()
            node.Name = 'Invariant Generator'
            node.BHdebug = self.DB
            node.enabled = True     # off by default in the tree; on for these tests
        if bb is None:
            bb = b3.Blackboard()
            bb.set('Robot', R)
            bb.set('unknowns', unknowns)

        tester = b3.BehaviorTree()
        tester.root = node
        status = tester.tick('testing invariant_gen', bb)
        return status, bb.get('Robot'), bb, node

    #################################################################
    #  positive cases

    def test_invA_fires_on_puma_like_meqn(self):
        '''||P||^2 must fire and must hand back a ONE-unknown equation, where
           every source element equation had two.'''
        sp.var('th_2 th_3 th_23')
        fs = ' invariant_gen FAIL'

        R, unknowns = self.build_robot(self.puma_like_meqn(), [th_2, th_3, th_23])
        status, R, bb, node = self.run_leaf(R, unknowns)

        self.assertEqual(status, b3.SUCCESS, fs + ' (leaf did not fire)')
        self.assertGreater(len(R.kequation_aux_list), 0,
                           fs + ' (nothing appended to kequation_aux_list)')

        counts = [count_unknowns(unknowns, e.RHS) + count_unknowns(unknowns, e.LHS)
                  for e in R.kequation_aux_list]
        self.assertIn(1, counts, fs + ' (no 1-unknown equation generated)')

        #  nothing above MAX_UNKNOWNS got through
        self.assertLessEqual(max(counts), MAX_UNKNOWNS,
                             fs + ' (an over-threshold equation slipped past the gate)')

    def test_invB_numeric_roundtrip(self):
        '''The generated equation must be TRUE at an FK-consistent numeric
           assignment.  Independent of expression form, so it catches sign and
           algebra errors that comparing to an expected expression would miss.'''
        import math
        sp.var('Px Py Pz th_2 th_3 th_23 a_2 a_3 d_3 d_4')
        fs = ' invariant_gen numeric roundtrip FAIL'

        meqn = self.puma_like_meqn()
        R, unknowns = self.build_robot(meqn, [th_2, th_3, th_23])
        status, R, bb, node = self.run_leaf(R, unknowns)
        self.assertEqual(status, b3.SUCCESS, fs)

        #  pick joint/link values, then derive Px,Py,Pz from the FK itself so
        #  the two sides of the matrix equation really do describe one pose
        vals = {th_3: 0.6, a_2: 0.43, a_3: 0.02, d_3: 0.15, d_4: 0.43}
        subs_p = {Px: float(meqn.Ts[0, 3].subs(vals)),
                  Py: float(meqn.Ts[1, 3].subs(vals)),
                  Pz: float(meqn.Ts[2, 3].subs(vals))}
        vals.update(subs_p)

        for e in R.kequation_aux_list:
            resid = float((e.LHS - e.RHS).subs(vals))
            self.assertAlmostEqual(resid, 0.0, places=9,
                                   msg=fs + ' (residual %g for %s = %s)'
                                       % (resid, e.LHS, e.RHS))

    def test_invC_soa_backsub_wins(self):
        '''sympy cannot reduce cos(th_2 - th_23) on its own.  Given the SOA
           definition th_23 = th_2 + th_3 in the aux list, back-substitution
           must turn a 2-unknown invariant into a 1-unknown one.'''
        sp.var('th_2 th_3 th_23 a_2 a_3')
        fs = ' invariant_gen SOA back-substitution FAIL'

        raw = a_2**2 + 2*a_2*a_3*sp.cos(th_2 - th_23) + a_3**2
        soa = {th_23: th_2 + th_3}
        unknowns = [unknown(th_2), unknown(th_3), unknown(th_23)]

        self.assertEqual(count_unknowns(unknowns, raw), 2,
                         fs + ' (fixture is not the 2-unknown case)')

        got = best_form(raw, unknowns, soa)
        self.assertEqual(count_unknowns(unknowns, got), 1,
                         fs + ' (did not reduce to one unknown: ' + str(got) + ')')
        self.assertTrue(got.has(th_3), fs + ' (reduced form lost th_3)')

        #  ... and it must NOT expand when expanding makes things worse:
        #  cos(th_23) is 1 unknown, cos(th_2+th_3) is 2.
        good = sp.cos(th_23) + a_2
        self.assertEqual(best_form(good, unknowns, soa), good,
                         fs + ' (expanded a form that was already better)')

    #################################################################
    #  negative cases -- these are the guards

    def test_invD_declines_when_nothing_beats_the_gate(self):
        '''A matrix equation whose invariants all carry too many unknowns must
           produce nothing and return FAILURE, so the enclosing Priority falls
           through to whatever comes after it.'''
        sp.var('Px Py Pz th_1 th_2 th_3 th_4 l_1 l_2 l_3 l_4')
        fs = ' invariant_gen gate FAIL'

        Td = sp.zeros(4)
        Ts = sp.zeros(4)
        for i in range(3):
            Td[i, i] = 1
            Ts[i, i] = 1
        Td[0, 3] = Px
        Td[1, 3] = Py
        Td[2, 3] = Pz
        #  four independent joints in the position column, none of which cancel
        #  under ||P||^2 -- 4 unknowns survive, over the threshold
        Ts[0, 3] = l_1*sp.cos(th_1) + l_2*sp.cos(th_2)
        Ts[1, 3] = l_3*sp.cos(th_3) + l_4*sp.cos(th_4)
        Ts[2, 3] = l_1*sp.sin(th_1) + l_2*sp.sin(th_2) + l_3*sp.sin(th_3)

        R, unknowns = self.build_robot(matrix_equation(Td, Ts),
                                       [th_1, th_2, th_3, th_4])
        status, R, bb, node = self.run_leaf(R, unknowns)

        self.assertEqual(status, b3.FAILURE, fs + ' (fired on an unusable meqn)')
        self.assertEqual(len(R.kequation_aux_list), 0,
                         fs + ' (appended equations anyway)')

    def test_invG_rejects_restatement_of_an_element_eqn(self):
        '''P . col_k degenerates to the bare element equation for P_k when the
           rotation block is the identity.  Those are already on the blackboard
           via scan_for_equations(), and updateL files aux equations in a
           different shape so the existing dedup would not catch them.  The
           generator must not emit them.'''
        sp.var('Px Py Pz th_2 th_3 th_23 a_2 a_3 d_3 d_4')
        fs = ' invariant_gen redundancy screen FAIL'

        meqn = self.puma_like_meqn()      # identity rotation block
        R, unknowns = self.build_robot(meqn, [th_2, th_3, th_23])
        status, R, bb, node = self.run_leaf(R, unknowns)

        self.assertEqual(status, b3.SUCCESS, fs + ' (leaf did not fire at all)')

        elems = element_forms([meqn])
        for e in R.kequation_aux_list:
            flat = sp.expand(e.LHS - e.RHS)
            for el in elems:
                self.assertNotEqual(sp.expand(flat - el), 0,
                                    fs + ' (emitted element eqn: %s = %s)' % (e.LHS, e.RHS))
                self.assertNotEqual(sp.expand(flat + el), 0,
                                    fs + ' (emitted negated element eqn: %s = %s)'
                                        % (e.LHS, e.RHS))

        #  what survives is the genuine invariant, and only that
        self.assertEqual(len(R.kequation_aux_list), 1,
                         fs + ' (expected exactly the ||P||^2 equation, got %d)'
                             % len(R.kequation_aux_list))

    def test_invH_rejects_negated_duplicate(self):
        '''Different matrix equations produce the same relation with every sign
           flipped -- measured on Puma, two of nine outputs were an exact
           negated pair.  kequation equality does not see that, so the screen
           has to compare both signs of the flattened form.'''
        sp.var('a_2 d_4 th_3')
        fs = ' invariant_gen negated-duplicate FAIL'

        flat = sp.expand(a_2*sp.cos(th_3) - d_4*sp.sin(th_3))

        self.assertTrue(is_redundant(flat, [flat]),
                        fs + ' (missed an exact duplicate)')
        self.assertTrue(is_redundant(sp.expand(-flat), [flat]),
                        fs + ' (missed a NEGATED duplicate)')
        self.assertTrue(is_redundant(sp.S.Zero, []),
                        fs + ' (accepted the vacuous 0 == 0)')
        self.assertFalse(is_redundant(sp.expand(flat + a_2), [flat]),
                         fs + ' (rejected a genuinely new equation)')

    def test_invI_skips_when_everything_is_solved(self):
        '''Sitting last in the worktools Priority, the leaf is reached only when
           every solver failed -- which, on a robot that solves cleanly, first
           happens right AFTER the last variable is solved.  Generating there is
           pure waste:  count_unknowns skips solved variables, so mostly-solved
           monsters slip under the <= 2 gate.  Measured on Puma: 9 useless
           equations at ~3x the solve wall clock.'''
        sp.var('th_2 th_3 th_23')
        fs = ' invariant_gen all-solved guard FAIL'

        R, unknowns = self.build_robot(self.puma_like_meqn(), [th_2, th_3, th_23])
        for u in unknowns:
            u.solved = True

        global generate_invariants
        real = generate_invariants
        calls = []

        def counting(*args, **kwargs):
            calls.append(1)
            return real(*args, **kwargs)

        generate_invariants = counting
        try:
            status, R, bb, node = self.run_leaf(R, unknowns)
        finally:
            generate_invariants = real

        self.assertEqual(calls, [], fs + ' (generated anyway)')
        self.assertEqual(status, b3.FAILURE, fs + ' (did not decline)')
        self.assertEqual(len(R.kequation_aux_list), 0, fs + ' (appended equations)')

    def test_invJ_dedups_candidates_within_one_run(self):
        '''Two matrix equations sharing a position column give the SAME ||P||^2,
           and swapping Td/Ts gives its exact negation.  Neither duplicate is an
           element equation, so the only thing that can catch them is feeding
           each keeper back into the screen as it is accepted.  Real robots hit
           this:  Puma emitted an exact negated pair across matrix equations.'''
        sp.var('th_2 th_3 th_23')
        fs = ' invariant_gen within-run dedup FAIL'

        same    = self.puma_like_meqn()            # identical relation
        flipped = self.puma_like_meqn()
        flipped.Td, flipped.Ts = flipped.Ts, flipped.Td   # -> negated relation

        R, unknowns = self.build_robot(self.puma_like_meqn(), [th_2, th_3, th_23])
        R.mequation_list = [R.mequation_list[0], same, flipped]

        status, R, bb, node = self.run_leaf(R, unknowns)
        self.assertEqual(status, b3.SUCCESS, fs + ' (leaf did not fire)')
        self.assertEqual(len(R.kequation_aux_list), 1,
                         fs + ' (expected 1 equation from 3 copies, got %d:\n%s)'
                             % (len(R.kequation_aux_list),
                                '\n'.join('   %s = %s' % (e.LHS, e.RHS)
                                          for e in R.kequation_aux_list)))

    def test_invK_is_off_by_default(self):
        '''A freshly constructed leaf must be inert.

           MEASURED, with it live in the fallback position:  Puma 28s -> 159s and
           Kawasaki 27s -> 126s, both for byte-identical output, and KinovaLite
           43s -> >10min while still solving nothing.  It is wired into the tree
           as a documented extension point, not as a working default, and this
           test is what keeps somebody from flipping that by accident.'''
        sp.var('th_2 th_3 th_23')
        fs = ' invariant_gen default-off FAIL'

        self.assertFalse(invariant_gen.enabled,
                         fs + ' (class default is on!)')

        node = invariant_gen()
        node.Name = 'Invariant Generator'
        node.BHdebug = self.DB
        #  note: deliberately NOT setting node.enabled

        R, unknowns = self.build_robot(self.puma_like_meqn(), [th_2, th_3, th_23])
        status, R, bb, node = self.run_leaf(R, unknowns, node=node)

        self.assertEqual(status, b3.FAILURE, fs + ' (fired while disabled)')
        self.assertEqual(len(R.kequation_aux_list), 0,
                         fs + ' (generated equations while disabled)')

    def test_invE_is_one_shot(self):
        '''The second tick must not RECOMPUTE the invariants.

           Note carefully what this does and does not test.  Returning FAILURE
           on the second tick is NOT evidence the guard works:  generate_invariants
           already drops candidates that are in kequation_aux_list, so with the
           guard deleted the second tick still finds nothing new and still
           returns FAILURE.  (Verified by mutation:  removing the guard leaves a
           FAILURE-only assertion green.)

           What the guard actually buys is cost.  This leaf sits inside
           RepeatUntilSuccess(6) nested in RepeatUntilSuccess(10), so it can be
           ticked ~60 times per solve, and each run is dozens of sp.simplify()
           calls over full FK expressions.  So spy on generate_invariants and
           assert it is not called again.'''
        sp.var('th_2 th_3 th_23')
        fs = ' invariant_gen one-shot FAIL'

        R, unknowns = self.build_robot(self.puma_like_meqn(), [th_2, th_3, th_23])
        status1, R, bb, node = self.run_leaf(R, unknowns)
        self.assertEqual(status1, b3.SUCCESS, fs + ' (first tick did not fire)')
        n_after_first = len(R.kequation_aux_list)

        global generate_invariants
        real = generate_invariants
        calls = []

        def counting(*args, **kwargs):
            calls.append(1)
            return real(*args, **kwargs)

        generate_invariants = counting
        try:
            status2, R, bb, node = self.run_leaf(R, unknowns, node=node, bb=bb)
        finally:
            generate_invariants = real

        self.assertEqual(calls, [], fs + ' (second tick recomputed the invariants)')
        self.assertEqual(status2, b3.FAILURE, fs + ' (second tick fired again)')
        self.assertEqual(len(R.kequation_aux_list), n_after_first,
                         fs + ' (second tick appended duplicates)')

    def test_invF_rejects_oversized_invariant(self):
        '''An invariant with few enough unknowns but an enormous expression is
           still rejected:  it would slow every later scan and produce an
           unreadable report.'''
        sp.var('th_2 th_3 th_23')
        fs = ' invariant_gen size gate FAIL'

        global MAX_OPS
        saved = MAX_OPS
        try:
            MAX_OPS = 1          # nothing real can be this small
            R, unknowns = self.build_robot(self.puma_like_meqn(), [th_2, th_3, th_23])
            status, R, bb, node = self.run_leaf(R, unknowns)
            self.assertEqual(status, b3.FAILURE, fs + ' (size gate did not bite)')
            self.assertEqual(len(R.kequation_aux_list), 0, fs)
        finally:
            MAX_OPS = saved


def run_test():
    suite = unittest.TestLoader().loadTestsFromTestCase(TestSolver012)
    unittest.TextTestRunner(verbosity=2).run(suite)


if __name__ == "__main__":
    print('\n\n===============  Test invariant_gen.py =====================')
    run_test()
