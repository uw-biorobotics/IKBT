#!/usr/bin/python
#
#   parallel_triple.py --  the transform for THREE CONSECUTIVE PARALLEL AXES
#
#   Pieper's condition is satisfied by three consecutive axes that intersect at
#   a point OR that are mutually parallel -- parallel being the limiting case
#   where the intersection point recedes to infinity.  dh_analysis has always
#   reported both kinds;  every solver leaf was written for the intersecting
#   one.  This leaf is the parallel case.
#
#   See IKdocs/parallel_triple_refs.md for the derivation, the published
#   algorithm this implements a step of, and the measurements below.
#
#   WHAT IT DOES, in one sentence:  split each position component into the part
#   that still contains an unsolved unknown and the part that does not, move the
#   known part over to the numeric side, and take |P|^2 of what is left.
#
#   WHY THAT IS THE WHOLE TRICK.  On a parallel triple the two link vectors turn
#   in the same plane, so
#
#       |a_j * u(q) + a_j+1 * u(q + q_mid)|^2
#             =  a_j^2 + a_j+1^2 + 2*a_j*a_j+1*cos(q_mid)
#
#   -- the law of cosines, one unknown, solvable by the existing arccos leaf.
#   The cross term collapses ONLY if the trailing offsets have already been
#   moved to the known side:  leave a d_5*cos(th_234) in the sum and squaring it
#   against a_2*sin(th_2) reintroduces th_2, giving two unknowns instead of one.
#   That is why the two existing transforms fail here -- x2z2_transform squares
#   pairs of RAW position equations, and invariant_gen takes |P|^2 over the
#   WHOLE symbolic side, wrist terms included.
#
#   Otherwise this leaf is invariant_gen's smaller sibling, and reuses its
#   machinery -- soa_subs_dict(), best_form(), element_forms(), is_redundant().
#
#   On UR5 it supplies step (12) of IK-Geo's Section IV-B algorithm:  the
#   solver reaches th_1, th_5, th_6, th_234 on its own -- steps (8)-(11) -- and
#   stops for want of the law-of-cosines equation for the middle joint.
#
#   Copyright 2026 University of Washington
#
#   Developed by Blake Hannaford
#   BioRobotics Lab, University of Washington

import unittest

import sympy as sp

import b3 as b3          # behavior trees

from ikbtfunctions.helperfunctions import count_unknowns
from ikbtbasics.kin_cl import kequation
import ikbtbasics.dh_analysis as da

#  Reused wholesale.  These are invariant_gen's, and there is no second copy.
from ikbtleaves.invariant_gen import (soa_subs_dict, best_form,
                                      element_forms, is_redundant)


#####################################################################
#
#   Tunables
#

#  An equation in ONE unknown is the point of this leaf -- it exists to restock
#  eqns_1u.  Two would land in L2 and need a partner that does not exist here.
MAX_UNKNOWNS = 1

#  Same reasoning as invariant_gen's MAX_OPS:  a huge equation slows every later
#  scan and produces an unreadable report even when it solves.  Higher than
#  invariant_gen's 80 because the law-of-cosines form carries the whole known
#  side with it, and that side is a pose expression, not a monster.
MAX_OPS = 220


def parallel_axis_triples(R, unknowns):
    '''The PARALLEL Pieper triples of this robot, or [] if there are none or the
       DH table cannot be read.

       Cheap:  pure DH-table arithmetic, no FK and no pickles, milliseconds.
       That is what makes this leaf safe to gate on -- an arm with no parallel
       triple pays one table scan and the leaf declines.'''

    try:
        M = R.Mech
        ndof = da.ndof_from_unknowns(unknowns)
        return [t for t in da.pieper_triples(M.DH, M.pvals, ndof)
                if t.get('kind') == 'parallel']
    except Exception as e:
        print('parallel_triple: cannot analyse the DH table -- %s: %s'
              % (type(e).__name__, e))
        return []


def split_known(expr, unknowns):
    '''(known, unknown) -- the additive terms of `expr` that contain no unsolved
       unknown, and those that do.

       Term by term on the EXPANDED form, because "known" is a property of a
       term, not of the expression:  a_2*cos(th_2) + d_5*sin(th_234) is half
       known the moment th_234 is solved, and it is exactly that half we need to
       move across.  count_unknowns() already ignores solved variables, which is
       what makes this track the solve as it progresses.'''

    e = sp.expand(expr)
    terms = e.args if isinstance(e, sp.Add) else (e,)
    known = sp.S.Zero
    unk = sp.S.Zero
    for t in terms:
        if count_unknowns(unknowns, t) == 0:
            known += t
        else:
            unk += t
    return known, unk


def generate_parallel_equations(R, unknowns, debug=False):
    '''Mine R.mequation_list for law-of-cosines equations in one unknown.

       Returns a list of kequation, NOT appended to anything -- keeping the
       computation separate from the side effect is what makes it testable,
       exactly as in invariant_gen.'''

    soa_defs = soa_subs_dict(R, unknowns)
    meqns = R.mequation_list

    known_forms = element_forms(meqns)
    known_forms += [sp.expand(e.LHS - e.RHS) for e in R.kequation_aux_list]

    keepers = []

    def consider(lhs_raw, rhs_raw, what, k):
        '''Score one candidate and keep it if it beats every gate.'''
        try:
            rhs = best_form(sp.simplify(rhs_raw), unknowns, soa_defs)
            lhs = best_form(sp.simplify(lhs_raw), unknowns, soa_defs)
        except Exception as e:
            if debug:
                print('parallel_triple: meqn %d %s did not simplify -- %s'
                      % (k, what, e))
            return

        nunk = count_unknowns(unknowns, rhs) + count_unknowns(unknowns, lhs)
        nops = sp.count_ops(rhs) + sp.count_ops(lhs)
        if debug:
            print('parallel_triple: meqn %d  %-10s unks=%d ops=%d'
                  % (k, what, nunk, nops))
        if nunk < 1 or nunk > MAX_UNKNOWNS:
            return
        if nops > MAX_OPS:
            if debug:
                print('parallel_triple:    -- too big (%d ops), discarding' % nops)
            return
        flat = sp.expand(lhs - rhs)
        if is_redundant(flat, known_forms):
            if debug:
                print('parallel_triple:    -- already stated, discarding')
            return
        known_forms.append(flat)
        keepers.append(kequation(lhs, rhs))
        if debug:
            print('parallel_triple:    ++ KEEPING meqn %d %s:  %s = %s'
                  % (k, what, lhs, rhs))

    for k, Meq in enumerate(meqns):
        #  Partition the three position components, then square.  Td is the
        #  known side already, so the known half of Ts crosses over with a
        #  minus sign and the two sides stay balanced.
        lhs_raw = sp.S.Zero
        rhs_raw = sp.S.Zero
        for i in range(3):
            kn, un = split_known(Meq.Ts[i, 3], unknowns)
            lhs_raw += (Meq.Td[i, 3] - kn)**2
            rhs_raw += un**2

            #  STEP (13) of the published algorithm, as well as (12).  The NORM
            #  above gives the middle joint of the triple;  the individual
            #  COMPONENTS give the first one, once the middle is known -- that
            #  is IK-Geo's `R12(p23 + R23 p34) = R10 p06 - ...` solved by
            #  Subproblem 1 (an atan2 over two components).
            #
            #  These look like the element equations the scanner already has,
            #  and algebraically they are:  (Td_i - kn) - un == Td_i - Ts_i.
            #  What is new is the SOA expansion best_form() applies -- the raw
            #  element equation counts th_2 and th_23 as two unknowns, while
            #  th_23 -> th_2 + th_3 with th_3 already solved leaves exactly one.
            #  The expanded form is a different expression, so it survives
            #  is_redundant() on its own merits rather than by accident.
            consider(Meq.Td[i, 3] - kn, un, 'P[%d]' % i, k)

        consider(lhs_raw, rhs_raw, '|P_res|^2', k)

    return keepers


class parallel_triple_transform(b3.Action):
    '''A transform, not a solver:  it emits equations, it does not set_solved().

       SUCCESS  at least one new one-unknown equation was appended
       FAILURE  no parallel triple, nothing new to say, or already said it

       WHY IT IS NOT ONE-SHOT, unlike invariant_gen.  Its whole value depends on
       WHICH variables are solved -- the wrist offsets only move to the known
       side once the trailing joints are known -- so running once at the top of
       the solve is running too early.  It re-runs whenever the solved set has
       changed since its last attempt, and declines otherwise, which bounds it
       at one run per variable solved rather than one per tick.'''

    def __init__(self):
        super(parallel_triple_transform, self).__init__()
        self.Name = 'Parallel Triple Transform'
        self.BHdebug = False

    def tick(self, tick):
        bb = tick.blackboard
        R = bb.get('Robot')
        unknowns = bb.get('unknowns')

        if R is None or not unknowns:
            return b3.FAILURE

        #  Nothing left to generate equations for.  Same guard, and the same
        #  reason, as invariant_gen:  last-ish in the Priority, this leaf is
        #  reached on the tick after the final variable is solved.
        if all(u.solved for u in unknowns):
            return b3.FAILURE

        #  The gate.  Cheap enough to re-evaluate every tick, and it is the
        #  whole reason this leaf can afford to be slow when it does fire.
        triples = parallel_axis_triples(R, unknowns)
        if not triples:
            return b3.FAILURE

        #  Re-run only when the solve has moved on since the last attempt.
        sig = tuple(sorted(u.name for u in unknowns if u.solved))
        if bb.get('parallel_triple_sig') == sig:
            return b3.FAILURE
        bb.set('parallel_triple_sig', sig)

        if self.BHdebug:
            print('\nrunning: ', self.Name)
            print('parallel_triple: parallel triples at ',
                  [t['axes'] for t in triples], ' solved so far: ', list(sig))

        keepers = generate_parallel_equations(R, unknowns, debug=self.BHdebug)

        if not keepers:
            if self.BHdebug:
                print('parallel_triple: nothing beat the gate this pass')
            return b3.FAILURE

        for e in keepers:
            R.kequation_aux_list.append(e)
        print('parallel_triple: generated %d equation(s) in one unknown'
              % len(keepers))

        bb.set('Robot', R)
        return b3.SUCCESS


#####################################################################
#
#   Test code
#

class TestSolver023(unittest.TestCase):
    '''The parallel-triple transform.  Arithmetic and gating only -- no solve,
       so these run in milliseconds and need no pickle.'''

    def runTest(self):
        self.test_ptA_split_known_partitions_by_solved()
        self.test_ptB_gate_declines_without_a_parallel_triple()
        self.test_ptC_gate_fires_on_UR5()
        self.test_ptD_law_of_cosines_collapses()

    def _unk(self, name, solved):
        from ikbtbasics.kin_cl import unknown
        u = unknown(sp.Symbol(name))
        u.solved = solved
        return u

    def test_ptA_split_known_partitions_by_solved(self):
        '''A term is known iff it holds no UNSOLVED unknown -- which is what
           lets the leaf track the solve instead of running once.'''
        fs = ' parallel_triple FAIL'
        th_2, th_234, a_2, d_5 = sp.symbols('th_2 th_234 a_2 d_5')
        unks = [self._unk('th_2', False), self._unk('th_234', True)]
        kn, un = split_known(a_2*sp.cos(th_2) + d_5*sp.sin(th_234), unks)
        self.assertEqual(sp.simplify(kn - d_5*sp.sin(th_234)), 0, fs)
        self.assertEqual(sp.simplify(un - a_2*sp.cos(th_2)), 0, fs)

        #  Before th_234 is solved, the SAME expression is entirely unknown.
        unks2 = [self._unk('th_2', False), self._unk('th_234', False)]
        kn2, un2 = split_known(a_2*sp.cos(th_2) + d_5*sp.sin(th_234), unks2)
        self.assertEqual(kn2, 0, fs + ' (nothing is known yet)')

    def test_ptB_gate_declines_without_a_parallel_triple(self):
        '''Puma has an INTERSECTING wrist triple and no parallel one.  The leaf
           must decline:  Pieper-satisfied is not the gate, parallel is.'''
        fs = ' parallel_triple FAIL'
        from ikbtfunctions.ik_robots import robot_params
        import contextlib, io
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            dh, vv, params, pvals, unks = robot_params('Puma')

        class M: pass
        class Rb: pass
        M.DH = dh
        M.pvals = pvals
        Rb.Mech = M
        self.assertEqual(parallel_axis_triples(Rb, unks), [],
                         fs + ' (Puma has no parallel triple)')

    def test_ptC_gate_fires_on_UR5(self):
        '''UR5 is the motivating robot: par(2,3,4), the three parallel elbow
           axes.  Cheap DH arithmetic, so this is a real gate test.'''
        fs = ' parallel_triple FAIL'
        from ikbtfunctions.ik_robots import robot_params
        import contextlib, io
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            dh, vv, params, pvals, unks = robot_params('UR5')

        class M: pass
        class Rb: pass
        M.DH = dh
        M.pvals = pvals
        Rb.Mech = M
        tr = parallel_axis_triples(Rb, unks)
        self.assertEqual(len(tr), 1, fs + ' (UR5 has exactly one parallel triple)')
        self.assertEqual(tr[0]['axes'], (2, 3, 4), fs)

    def test_ptD_law_of_cosines_collapses(self):
        '''The property the whole leaf rests on:  two link vectors turning in
           one plane give  a^2 + b^2 + 2ab*cos(mid)  -- one unknown -- and the
           SAME sum with a trailing offset left in does NOT.'''
        fs = ' parallel_triple FAIL'
        th_2, th_23, th_234, a_2, a_3, d_5 = sp.symbols(
            'th_2 th_23 th_234 a_2 a_3 d_5')
        unks = [self._unk('th_2', False), self._unk('th_3', False),
                self._unk('th_23', False), self._unk('th_234', True)]
        soa = {th_23: sp.Symbol('th_2') + sp.Symbol('th_3')}

        x = a_2*sp.cos(th_2) + a_3*sp.cos(th_23)
        y = a_2*sp.sin(th_2) + a_3*sp.sin(th_23)
        collapsed = best_form(sp.simplify(x**2 + y**2), unks, soa)
        self.assertEqual(count_unknowns(unks, collapsed), 1,
                         fs + ' (law of cosines must leave ONE unknown)')

        #  Leave the wrist offset in and it does not collapse -- this is the
        #  x2z2 / invariant_gen failure, asserted rather than described.
        x2 = x + d_5*sp.cos(th_234)
        y2 = y + d_5*sp.sin(th_234)
        notcollapsed = best_form(sp.simplify(x2**2 + y2**2), unks, soa)
        self.assertGreater(count_unknowns(unks, notcollapsed), 1,
                           fs + ' (offset left in MUST leave >1 unknown)')


def run_test():
    testsuite = unittest.TestLoader().loadTestsFromTestCase(TestSolver023)
    unittest.TextTestRunner(verbosity=2).run(testsuite)


if __name__ == '__main__':
    print('\n\n  Running tests on parallel_triple.py\n\n')
    run_test()
