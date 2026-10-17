#!/usr/bin/python
#
#   disambiguate.py --  remove solution branches that an UNUSED equation rules out
#
#   Runs once, after every unknown is solved.  A solver leaf sometimes splits
#   an unknown into two branches because a sign it needed was not yet known,
#   e.g. tan_solver on Palm13:
#
#       r_13 = cos(th_1)*cos(th_3),   r_33 = sin(th_1)*cos(th_3)
#       th_1 = atan2( r_33,  r_13)    if cos(th_3) > 0
#       th_1 = atan2(-r_33, -r_13)    if cos(th_3) < 0
#
#   The equations that were used cannot tell the two apart, but the arm's
#   other equations often can (for Palm13, Py together with r_23 gives
#   cos(th_3), sign included).  This leaf looks for such equations and, when
#   it finds them, replaces the branch pair with ONE solution.
#
#   NOTHING HERE IS SPECIFIC TO ONE ROBOT.  For each unknown u with two or
#   more solutions:
#
#     1.  TARGET.  One expression valid for every branch of u, possibly still
#         holding unknowns S whose values decide the split.  A solver that
#         split on a sign records it in u.unsplit (tan_solver does).  With no
#         record, the target is u itself:  atan2(sin u, cos u), or u for a
#         prismatic joint, so S = {u}.
#     2.  DECIDING EQUATIONS.  The scalar equations of the matrix equations
#         whose unknowns lie in S, u, the unknowns solved BEFORE u, and at
#         most ONE further unknown X, tried in turn.  Unknowns solved before u
#         cannot depend on u, so no circular dependence is created;  X is
#         eliminated, so it never appears in the answer.  Where u itself
#         appears, sin(u) and cos(u) are replaced by the target's atan2
#         arguments.
#     3.  ELIMINATE.  sin(s) and cos(s) of every unknown s in S and X (or s
#         itself, for a prismatic joint) become polynomial variables, with
#         sin^2 + cos^2 = 1 for each.  A lex Groebner basis then holds,
#         wherever the equations pin them down, polynomials LINEAR in one of
#         those variables with the rest already solved -- read off by
#         back-substitution.  The end-effector-pose entries (r_ij, Px, ...)
#         are variables too, not constants:  the equations hold only on the
#         poses the arm can reach, and over independent constants the
#         over-determined system the answer needs would be inconsistent.
#         Measured on Wachtveitl:  0.17 s, and the sign of sin(th_34) comes
#         out of Px, Pz and r_33 with cos(th_3) eliminated on the way.
#     4.  CHECK, NUMERICALLY.  At several random joint-poses q, build the
#         end-effector-pose T = FK(q), evaluate the new expression, and require
#         that it reproduces u's value in q.  A leading coefficient that
#         vanishes on every reachable pose would pass the algebra and fail here.
#     5.  REPLACE u's solutions with the one expression, and rebuild every
#         unknown's dependencies and version names (record_versions()).
#
#   Always returns SUCCESS:  a pair it cannot resolve stays as it was.
#
#   ADDING ANOTHER SPLIT.  A solver leaf that branches on an undetermined sign
#   sets u.unsplit = {'solutions': [...the branches...], 'expr': ...one
#   expression valid for all of them...}.  If 'expr' is atan2(y, x), then y
#   and x must be EXACTLY sin(u) and cos(u), not just proportional to them --
#   step 2 substitutes them for sin(u) and cos(u).  The record is used only
#   while u.solutions still equals those branches, so rank_leaf choosing a
#   different leaf's answer disables it.  Branches with no record are tried
#   with the default target of step 1.
#
#   Copyright 2026 University of Washington
#
#   Developed by Blake Hannaford
#   BioRobotics Lab, University of Washington

import contextlib
import itertools
import signal
import threading
import time
import unittest

import numpy as np
import sympy as sp

import b3 as b3          # behavior trees

#  Random joint-poses for the numerical check, and how close the new
#  expression must come to the true joint value at each of them.
N_CHECK = 6
CHECK_TOL = 1e-6

#  Time limits:  one Groebner basis, and the whole search for one unknown.
#  The search for a GENUINE branch pair (Puma's elbow) always runs to the
#  budget, since nothing will be found, so this is what it costs per pair.
GROEBNER_SECONDS = 2.0
SECONDS_PER_UNKNOWN = 20.0

POSE_SYMBOLS = ([sp.Symbol(f'r_{i + 1}{j + 1}') for i in range(3) for j in range(3)]
                + [sp.Symbol(n) for n in ('Px', 'Py', 'Pz')])


def scalar_equations(R):
    '''Every scalar equation (expr == 0) of every matrix equation, once each.
       A negated copy counts as the same equation.'''
    out, seen = [], set()
    for meq in R.mequation_list:
        for i in range(3):
            for j in range(4):
                e = meq.Td[i, j] - meq.Ts[i, j]
                if e == 0:
                    continue
                key = min(str(e), str(-e))
                if key not in seen:
                    seen.add(key)
                    out.append(e)
    out.sort(key=sp.count_ops)
    return out


def _trig_symbols(syms):
    '''{sin(s): _sin_s, cos(s): _cos_s} for each s.'''
    rep = {}
    for s in syms:
        rep[sp.sin(s)] = sp.Symbol(f'_sin_{s}')
        rep[sp.cos(s)] = sp.Symbol(f'_cos_{s}')
    return rep


def _to_polynomial(expr, elim, rep):
    '''Rewrite expr with sin/cos of the unknowns replaced by symbols (rep).
       Returns (expr, variables) where variables are the replaced symbols of
       the unknowns in `elim` plus any bare (prismatic) unknown of `elim`;  or
       None when an unknown is left inside a function, e.g. sin(th_2 + th_3).'''
    new = expr.xreplace(rep)
    bare = set()
    for s in elim:
        if not new.has(s):
            continue
        if any(f.has(s) for f in new.atoms(sp.Function)):
            return None
        bare.add(s)
    elim_rep = {rep[k] for k in rep if k.args[0] in elim}
    return new, ({v for v in elim_rep if new.has(v)} | bare)


def _is_revolute(u, eqns):
    return any(e.has(sp.sin(u.symbol)) or e.has(sp.cos(u.symbol)) for e in eqns)


def _target(u, eqns):
    '''Step 1:  the expression to make unknown-free.'''
    rec = u.unsplit
    if rec and list(rec['solutions']) == list(u.solutions):
        return rec['expr']
    if _is_revolute(u, eqns):
        return sp.atan2(sp.sin(u.symbol), sp.cos(u.symbol))
    return u.symbol


def _own_trig(u_sym, target):
    '''{sin(u): y, cos(u): x} when the target is atan2(y, x), else {}.'''
    if isinstance(target, sp.atan2):
        y, x = target.args
        return {sp.sin(u_sym): y, sp.cos(u_sym): x}
    return {}


def _back_substitute(basis, order):
    '''Read variable values off a lex Groebner basis, lowest variable first:
       an element linear in v whose other variables are already solved gives
       v.  Returns {variable: expression} for those that could be read.'''
    vals = {}
    for v in reversed(order):
        higher = set(order[:order.index(v)])
        for g in basis:
            if not g.has(v) or g.free_symbols & higher:
                continue
            g = g.xreplace(vals)
            if set(order) & g.free_symbols - {v}:
                continue
            p = sp.Poly(g, v)
            if p.degree() != 1:
                continue
            a, b = p.all_coeffs()
            if a == 0:
                continue
            vals[v] = -b/a
            break
    return vals


class _Timeout(Exception):
    pass


@contextlib.contextmanager
def _time_limit(seconds):
    '''Raise _Timeout inside the block after `seconds`.  A Groebner basis can
       grow explosively, and an improvement must never stall a solve.  Uses
       SIGALRM, so it limits nothing where that is unavailable (Windows, or
       off the main thread);  the per-unknown budget still applies there.'''
    if (not hasattr(signal, 'setitimer')
            or threading.current_thread() is not threading.main_thread()):
        yield
        return

    def expire(signum, frame):
        raise _Timeout()

    old = signal.signal(signal.SIGALRM, expire)
    signal.setitimer(signal.ITIMER_REAL, seconds)
    try:
        yield
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        signal.signal(signal.SIGALRM, old)


#  WHY A GROEBNER BASIS FITS THIS PROBLEM.
#
#  Once sin(th) and cos(th) are renamed s and c, and s^2 + c^2 = 1 is added,
#  every kinematic equation is a polynomial.  Solving several polynomial
#  equations together is the job Gaussian elimination does for LINEAR ones,
#  and a Groebner basis is its generalisation:  in "lex" order it rewrites the
#  set into an equivalent triangular one, where the last polynomials hold only
#  the last variable, the next ones add one more variable, and so on.  So
#  "eliminate th_3, then read off sin(th_34)" is a property of the basis, not
#  a trick written for one robot.  If an element turns out LINEAR in a
#  variable, that variable has ONE value -- sign included -- which is exactly
#  "the equations decide the branch".  If every element holding it is
#  quadratic, the equations TRIED do not decide it and the leaf leaves both
#  branches.  That is proof the branches are genuine only if every equation
#  was tried, and this leaf brings in at most one extra unknown.
#
#  Introductions, gentlest first:
#    https://en.wikipedia.org/wiki/System_of_polynomial_equations
#    https://en.wikipedia.org/wiki/Gr%C3%B6bner_basis      (see "Elimination")
#    https://en.wikipedia.org/wiki/Elimination_theory
#    Cox, Little & O'Shea, "Ideals, Varieties, and Algorithms" (Springer),
#      ch. 1-3 -- written for undergraduates;  ch. 3 is elimination.
#    sympy:  https://docs.sympy.org/latest/modules/polys/basics.html
#            (section "Groebner bases")
#  Classic inverse kinematics of general 6R arms uses the same family of
#  tools, in the form of resultants:  Raghavan & Roth (1993), Manocha & Canny
#  (1994).

def _eliminate(polys, variables, elim, rep, needed, target_p):
    '''Step 3 on ONE small system.  `variables` are the polynomial variables
       of the unknowns being eliminated;  everything else in the polynomials,
       already-solved unknowns included, is a constant.  Returns the target
       with the needed variables read off a lex Groebner basis, or None.'''
    variables = set(variables)
    polys = list(polys)
    for s in elim:
        sv, cv = rep[sp.sin(s)], rep[sp.cos(s)]
        if sv in variables or cv in variables:
            polys.append(sv**2 + cv**2 - 1)
            variables |= {sv, cv}

    order = sorted(needed, key=str) + sorted(variables - needed, key=str)
    free = set().union(*(p.free_symbols for p in polys))
    pose = sorted(free & set(POSE_SYMBOLS), key=str)
    coeffs = sorted(free - set(order) - set(pose), key=str)
    domain = sp.QQ.frac_field(*coeffs) if coeffs else sp.QQ
    try:
        with _time_limit(GROEBNER_SECONDS):
            G = sp.groebner(polys, *order, *pose, order='lex', domain=domain)
    except (_Timeout, sp.PolynomialError, sp.CoercionFailed, sp.DomainError):
        return None
    if list(G.exprs) == [1]:
        return None
    vals = _back_substitute(list(G.exprs), order)
    if not needed <= set(vals):
        return None
    return target_p.xreplace(vals)


def solve_target(target, S, allowed, eqns, unk_syms, u_sym=None, extras=(),
                 deadline=None):
    '''Steps 2 and 3.  Yield (replacement for `target`, equations used), the
       replacement holding only pose symbols, parameters and the `allowed`
       already-solved unknowns.  u_sym is the unknown being disambiguated,
       when it is not itself in S.  `extras` are the unknowns that may be
       eliminated along the way, one at a time.  Stops at time.monotonic()
       == deadline.

       SMALL SYSTEMS, NOT ALL OF THEM AT ONCE.  Handing Groebner every
       equation that qualifies is correct but can be hopelessly slow:  on
       Wachtveitl, with th_3 to eliminate, that is 12 polynomials in 15
       variables, most of them orientation equations that drag in all nine
       r_ij and say nothing about the branch -- it did not finish in minutes.
       The deciding system there is 3 equations (r_33, Pz, Px) plus two
       sin^2 + cos^2 = 1, and on Palm13 and Sims11 it is likewise ONE more
       equation than there are unknowns to eliminate:  the extra one is the
       equation that decides the sign.  So subsets of n to n+2 equations are
       tried, n = unknowns to eliminate, simplest first.'''
    own = _own_trig(u_sym, target) if u_sym is not None and u_sym not in S else {}
    base = set(S) | ({u_sym} if own else set())

    for X in [set()] + [{x} for x in extras if x not in base]:
        elim = set(S) | X
        rep = _trig_symbols(elim | allowed)
        reach = base | X

        lt = _to_polynomial(target, elim, rep)
        if lt is None:
            return
        target_p, needed = lt

        cands, seen = [], set()
        for e in eqns:
            present = e.free_symbols & unk_syms
            if not present & reach or not present <= reach | allowed:
                continue
            e_sub = e
            if own and u_sym in present:
                e_sub = e.xreplace(own)
                if e_sub.has(u_sym):
                    continue
            p = _to_polynomial(sp.numer(sp.together(e_sub)), elim, rep)
            if p is None or not p[1]:
                continue
            poly = sp.expand(p[0])
            key = min(str(poly), str(-poly))
            if key in seen:
                continue
            seen.add(key)
            cands.append((poly, e, p[1]))
        cands.sort(key=lambda c: (len(c[0].free_symbols & set(POSE_SYMBOLS)),
                                  sp.count_ops(c[0])))
        if not needed <= set().union(*(c[2] for c in cands)) if cands else True:
            continue

        for k in range(1, len(elim) + 3):
            for combo in itertools.combinations(cands, k):
                if deadline is not None and time.monotonic() > deadline:
                    return
                variables = set().union(*(c[2] for c in combo))
                if not needed <= variables:
                    continue
                n = len({s for s in elim
                         if variables & {rep[sp.sin(s)], rep[sp.cos(s)], s}})
                if not n <= k <= n + 2:
                    continue
                new = _eliminate([c[0] for c in combo], variables, elim, rep,
                                 needed, target_p)
                if new is None:
                    continue
                new = new.xreplace({v: k_ for k_, v in rep.items()})
                if new.free_symbols & (elim | {u_sym}):
                    continue
                yield new, [c[1] for c in combo]


class _Checker:
    '''Step 4:  values of everything at random joint-poses of the true arm.'''

    def __init__(self, R):
        import ikbtbasics.numeric_ik as nik
        M = R.Mech
        self.jsyms = list(nik.joint_symbols(M))
        try:
            pv = nik.pvals_numeric(M)
        except ValueError:
            pv = {}
        self.T = sp.Matrix(M.T_06)
        self.params = {p: pv.get(p) for p in self.T.free_symbols - set(self.jsyms)}
        self.aux = [(e.LHS, e.RHS) for e in R.kequation_aux_list
                    if isinstance(e.LHS, sp.Symbol)]
        self.rng = np.random.default_rng(12345)
        self.poses = [self._pose() for _ in range(N_CHECK)]

    def _pose(self):
        vals = {}
        for p, v in self.params.items():
            vals[p] = v if v is not None else float(self.rng.uniform(0.5, 1.5))
        for j in self.jsyms:
            vals[j] = float(self.rng.uniform(-np.pi, np.pi))
        for lhs, rhs in self.aux:
            vals[lhs] = float(rhs.subs(vals))
        Tn = self.T.subs(vals)
        cells = [(i, j) for i in range(3) for j in range(3)] + [(0, 3), (1, 3), (2, 3)]
        for s, k in zip(POSE_SYMBOLS, cells):
            vals[s] = float(Tn[k])
        return vals

    def agrees(self, expr, u, revolute):
        ok = 0
        for vals in self.poses:
            if u.symbol not in vals:
                return False
            try:
                v = complex(sp.N(expr.subs(vals)))
            except (TypeError, ZeroDivisionError):
                continue
            if not np.isfinite(v) or abs(v.imag) > 1e-9:
                continue
            d = v.real - vals[u.symbol]
            if revolute:
                d = (d + np.pi) % (2 * np.pi) - np.pi
            if abs(d) > CHECK_TOL:
                return False
            ok += 1
        return ok >= N_CHECK // 2


def disambiguate_robot(R, unknowns, verbose=False):
    '''Run steps 1-5 over every solved unknown.  Returns a list of
       (unknown name, old solution count, equations used) for those changed.'''
    nodes = sorted(R.solution_nodes, key=lambda n: n.unknown.solveorder)
    if not any(n.unknown.nsolutions > 1 for n in nodes):
        return []
    eqns = scalar_equations(R)
    unk_syms = {u.symbol for u in unknowns}
    checker = _Checker(R)
    changed = []

    for node in nodes:
        u = node.unknown
        if u.nsolutions < 2:
            continue
        target = _target(u, eqns)
        revolute = _is_revolute(u, eqns)
        allowed = {n.unknown.symbol for n in nodes
                   if n.unknown.solveorder < u.solveorder}
        S = (target.free_symbols & unk_syms) - allowed
        extras = [n.unknown.symbol for n in nodes
                  if n.unknown.symbol not in allowed | S | {u.symbol}]
        deadline = time.monotonic() + SECONDS_PER_UNKNOWN
        for new, used in solve_target(target, S, allowed, eqns, unk_syms,
                                      u_sym=u.symbol, extras=extras,
                                      deadline=deadline):
            if not checker.agrees(new, u, revolute):
                continue
            changed.append((u.name, u.nsolutions, used))
            if verbose:
                print(f'disambiguate: {u.name} {u.nsolutions} -> 1 solution,'
                      f' from {len(used)} equations')
            u.solutions = [new]
            u.nsolutions = 1
            u.assumption = []
            u.unsplit = None
            u.add_solvemethod('disambiguated')
            node.solvemethod = u.solvemethod
            break

    if changed:
        R.notation_graph_edges = set()
        for node in nodes:
            node.unknown.record_versions(R, unknowns)
    return changed


class disambiguate(b3.Action):
    '''BT leaf:  see the module header.  Always SUCCESS.'''

    def __init__(self):
        super().__init__()
        self.Name = 'Disambiguate'
        self.BHdebug = False

    def tick(self, tick):
        R = tick.blackboard.get('Robot')
        unknowns = tick.blackboard.get('unknowns')
        if R is None or not unknowns or getattr(R, 'Mech', None) is None:
            return b3.SUCCESS
        if not all(u.solved for u in unknowns):
            return b3.SUCCESS
        try:
            changed = disambiguate_robot(R, unknowns, verbose=True)
        except Exception as e:           # an improvement must never break a solve
            print(f'disambiguate: skipped ({type(e).__name__}: {e})')
            return b3.SUCCESS
        tick.blackboard.set('disambiguated', changed)
        return b3.SUCCESS


#######################################################################
#  Test code

class TestSolver032(unittest.TestCase):
    '''The deciding-equation search, on equations written by hand.  Each
       answer is checked at a reachable configuration, not compared with an
       expected formula:  the search may legitimately find a different
       expression that agrees wherever the arm can reach.'''

    def runTest(self):
        self.test_disA_sign_fixed_by_unused_equation()
        self.test_disB_genuine_pair_left_alone()
        self.test_disC_prismatic()
        self.test_disD_extra_unknown_eliminated()

    def setUp(self):
        print('\n\n===============  Test disambiguate  =====================')

    @staticmethod
    def reaches(new, truth, vals):
        v = float(sp.N(new.subs(vals)))
        return abs((v - truth + np.pi) % (2*np.pi) - np.pi) < 1e-9

    def test_disA_sign_fixed_by_unused_equation(self):
        '''Palm13's shape, written out:  th_1 split on the sign of
           cos(th_3);  r_23 and Py decide it.  Nothing in the search names
           those equations -- it has to find them.'''
        th_1, th_3, l_1, l_3, h = sp.symbols('th_1 th_3 l_1 l_3 h')
        r_13, r_23, r_33, Py = sp.symbols('r_13 r_23 r_33 Py')
        F = {r_13: sp.cos(th_1)*sp.cos(th_3), r_33: sp.sin(th_1)*sp.cos(th_3),
             r_23: -sp.sin(th_3), Py: -h*sp.sin(th_3) - l_1 + l_3*sp.cos(th_3)}
        eqns = [k - v for k, v in F.items()]
        target = sp.atan2(r_33/sp.cos(th_3), r_13/sp.cos(th_3))
        got = list(solve_target(target, {th_3}, set(), eqns, {th_1, th_3},
                                u_sym=th_1))
        self.assertTrue(got, 'no deciding equations found')
        new = got[0][0]
        self.assertFalse(new.has(th_3) or new.has(th_1))
        for q1, q3 in ((0.4, 2.5), (-2.0, 0.3), (2.9, -2.2)):   # both signs of cos(th_3)
            vals = {l_1: 0.7, l_3: 1.3, h: 0.4, th_1: q1, th_3: q3}
            vals.update({k: v.subs(vals) for k, v in F.items()})
            self.assertTrue(self.reaches(new, q1, vals), f'wrong at th_3={q3}')

    def test_disB_genuine_pair_left_alone(self):
        '''One equation in sin and cos fixes nothing:  both branches stay.'''
        th_1, Px, Py, d = sp.symbols('th_1 Px Py d')
        eqns = [Px*sp.sin(th_1) - Py*sp.cos(th_1) - d]
        target = sp.atan2(sp.sin(th_1), sp.cos(th_1))
        self.assertEqual(list(solve_target(target, {th_1}, set(), eqns, {th_1})), [])

    def test_disC_prismatic(self):
        '''A bare prismatic unknown is its own polynomial variable.'''
        d_2, th_1, Px, Pz = sp.symbols('d_2 th_1 Px Pz')
        eqns = [Px - d_2*sp.sin(th_1), Pz + d_2*sp.cos(th_1)]
        got = list(solve_target(d_2, {d_2}, {th_1}, eqns, {d_2, th_1}))
        self.assertTrue(got)
        self.assertFalse(got[0][0].has(d_2))

    def test_disD_extra_unknown_eliminated(self):
        '''Wachtveitl's shape:  th_2 split on the sign of sin(th_34), and the
           deciding equation Px also holds th_3, which must be eliminated
           (with Pz, r_33 and sin^2 + cos^2 = 1) rather than appear.'''
        th_2, th_3, th_34 = sp.symbols('th_2 th_3 th_34')
        l_2, l_3, l_4, h = sp.symbols('l_2 l_3 l_4 h')
        r_13, r_23, r_33, Px, Pz = sp.symbols('r_13 r_23 r_33 Px Pz')
        F = {r_13: sp.sin(th_34)*sp.cos(th_2), r_23: sp.sin(th_2)*sp.sin(th_34),
             r_33: -sp.cos(th_34),
             Px: l_2*sp.cos(th_2)*sp.cos(th_3) + l_3*sp.sin(th_2)
                 + l_4*sp.sin(th_34)*sp.cos(th_2),
             Pz: h + l_2*sp.sin(th_3) - l_4*sp.cos(th_34)}
        eqns = [k - v for k, v in F.items()]
        target = sp.atan2(r_23/sp.sin(th_34), r_13/sp.sin(th_34))
        unk = {th_2, th_3, th_34}
        self.assertEqual(list(solve_target(target, {th_34}, set(), eqns, unk,
                                           u_sym=th_2)), [],
                         'found without eliminating th_3 -- test is not testing it')
        got = list(solve_target(target, {th_34}, set(), eqns, unk,
                                u_sym=th_2, extras=[th_3]))
        self.assertTrue(got, 'th_3 was not eliminated')
        new = got[0][0]
        self.assertFalse(new.free_symbols & unk)
        for q2, q3, q4 in ((0.4, 0.9, 2.0), (-2.0, 2.4, -1.1), (2.9, -0.7, -2.6)):
            vals = {l_2: 0.9, l_3: 0.6, l_4: 0.5, h: 0.2,
                    th_2: q2, th_3: q3, th_34: q3 + q4}
            vals.update({k: v.subs(vals) for k, v in F.items()})
            self.assertTrue(self.reaches(new, q2, vals), f'wrong at {q2, q3, q4}')


if __name__ == '__main__':
    unittest.TextTestRunner().run(TestSolver032())
