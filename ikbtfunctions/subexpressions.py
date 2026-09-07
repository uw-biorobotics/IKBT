#!/usr/bin/python
#
#   subexpressions.py --  name the big pieces, so the equations fit on the page
#
#   THE PROBLEM.  A solved joint variable's closed form is routinely one
#   enormous expression -- Craig417's th_4 is a single atan2 of 87 operations.
#   Printed as one equation it runs off the right margin, and LaTeX cannot fix
#   it:  breqn breaks an equation at its operators, but not inside a \frac, and
#   a fraction of two forty-term sums is one unbreakable box.
#
#   THE FIX (BH).  Shorten the equation at the SOURCE.  Pull the big pieces out,
#   name each one, define them just above, and write the equation compactly in
#   terms of the names -- what a person does by hand, and it helps every reader
#   of the report rather than only the typesetter.
#
#   THE RULE.  A subexpression is worth naming when it involves more than two
#   PREVIOUSLY SOLVED variables.  Dependency count, not size:  that is what
#   makes an expression hard to follow.
#
#   WHAT COUNTS AS "A PIECE".  Splitting the top-level TERMS of a sum does
#   nothing, because every long solution is a SINGLE term -- a term-level rule
#   turns `th_4 = atan2(BIG, BIG)` into `th_4 = K_1` with
#   `K_1 = atan2(BIG, BIG)`, a rename that shortens nothing.  So the split
#   descends into FUNCTION ARGUMENTS (atan2's two arguments are exactly the
#   pieces a person would name) and into the factors of a product.
#
#   ONE NAME PER DISTINCT SUBEXPRESSION.  The pool remembers what it has named,
#   so a piece appearing in eight versions of a variable is defined once and
#   referenced eight times.  That is most of the saving in the "all versions"
#   section.  It is also why the numbering runs across a whole section rather
#   than restarting at each variable:  K_7 means one thing in the document.
#
#   THE PREFIX IS CHECKED, NOT ASSUMED.  `a_1, a_2, ...` is the natural choice
#   and is not available -- a_2 and a_3 are DH link lengths on several robots,
#   so a report would define `a_2 = <expression>` for an arm where a_2 is also
#   0.432 m.  K is free everywhere today, but Wrist's joints are literally named
#   A, B and C, so the prefix is chosen against the robot's own parameter and
#   variable names at run time and falls back when taken.
#
#   Copyright 2026 University of Washington
#
#   Developed by Blake Hannaford
#   BioRobotics Lab, University of Washington

import sympy as sp


#  More than two previously-solved variables:  BH's rule, verbatim.
MIN_DEPS = 3

#  ...and big enough to be worth a name.  A sum-of-angles definition like
#  th_2 + th_3 + th_4 has three dependencies and two operations;  naming it
#  would replace something already short and perfectly readable with an
#  indirection.  This guard is the only thing added to BH's rule, and it only
#  ever suppresses a naming, never causes one.
MIN_OPS = 6

#  Tried in order.  The first whose names are all unused by the robot wins.
PREFIXES = ['K', 'C', 'G', 'W', 'Z']


def dependency_names(Robot):
    """Every symbol name that refers to a previously-solved variable.

       Three spellings of the same thing appear in the report and all of them
       count:  the bare unknown (th_1) in the generic solutions, its per-
       solution name (th_1s2), and its per-version name (th_1v5).  Counting
       only the bare form would score every equation in the "all versions"
       section as having zero dependencies -- which is exactly the section with
       the longest equations in it."""

    names = set()
    for nd in getattr(Robot, 'solution_nodes', []) or []:
        u = getattr(nd, 'unknown', None)
        if u is None:
            continue
        names.add(str(u.name))
        for attr in ('solutionNames', 'LHSversionNames', 'versionNames'):
            names.update(str(n) for n in (getattr(u, attr, None) or []))
    return names


def choose_prefix(Robot, deps=None):
    """A symbol prefix this robot is not already using.

       Checked against the DH parameters AND the joint variables:  a collision
       with either would put two different meanings on one symbol inside a
       single report, which is worse than an ugly name."""

    used = set(deps or dependency_names(Robot))
    used |= {str(p) for p in (getattr(Robot, 'params', None) or [])}
    mech = getattr(Robot, 'Mech', None)
    if mech is not None:
        used |= {str(p) for p in (getattr(mech, 'pvals', None) or {})}
    for nd in getattr(Robot, 'solution_nodes', []) or []:
        u = getattr(nd, 'unknown', None)
        if u is not None:
            used.add(str(getattr(u, 'symbol', '')))

    for pref in PREFIXES:
        if not any(n == pref or n.startswith(pref + '_') for n in used):
            return pref
    return 'KK'                      # everything taken: still unambiguous


class SubexprPool(object):
    """Names the big pieces of a robot's solutions, once each.

       pool = SubexprPool(Robot)
       new_expr, defs = pool.split(expr)

       `defs` is the list of (symbol, subexpression) pairs THIS call created,
       in definition order, ready to print immediately above the equation.
       Pieces already named by an earlier call do not come back -- they are
       already in the document -- so printing every `defs` in order yields each
       definition exactly once, before its first use."""

    def __init__(self, Robot, min_deps=MIN_DEPS, min_ops=MIN_OPS,
                 max_defs=400, max_depth=8):
        self.deps = dependency_names(Robot)
        self.prefix = choose_prefix(Robot, self.deps)
        self.min_deps = min_deps
        self.min_ops = min_ops
        self.max_defs = max_defs

        #  DEEP ENOUGH TO REACH THE RADICAND.  This was 3, and measured on
        #  KinovaLite that stopped the walk before it reached the sqrt inside
        #  th_5/th_6:  the sqrt itself got named, but its argument never did, so
        #  eight definitions came out `K_n = sqrt(<forty terms>)` at 253pt over.
        #  The recursion is bounded by big_enough() rather than by this number
        #  -- it stops as soon as a piece is under min_ops -- so the cap only
        #  needs to be past the deepest thing worth naming.
        self.max_depth = max_depth
        self.named = {}              # subexpression -> symbol
        self.order = []              # (symbol, subexpression), definition order

    #  ------------------------------------------------------------------
    def n_deps(self, e):
        """How many distinct previously-solved variables this piece involves."""
        try:
            return len({s for s in e.free_symbols if str(s) in self.deps})
        except AttributeError:
            return 0

    def all_named(self, e):
        """Is `e` nothing but a combination of names we already created?

           K_59 = K_55 + K_56 + K_57 + K_58 is a definition that defines
           nothing:  every symbol in it is already a name, so the reader must
           visit four more definitions to reconstruct a sum that was perfectly
           readable written out.  Measured on Craig417, where forcing produced
           exactly that chain (BH, 2026-09-04).

           Suppressing the OUTER name -- not the inner ones -- is what keeps the
           width fix.  The terms stay named, because their width is real and a
           name is the only thing that shortens them;  what disappears is the
           extra indirection that merely adds them up."""

        syms = getattr(e, 'free_symbols', None)
        if not syms:
            return False
        known = set(self.named.values())
        return bool(known) and syms.issubset(known)

    def big_enough(self, e):
        """Big enough that a name is an improvement, ignoring dependencies."""
        return (e is not None and not e.is_Atom
                and sp.count_ops(e) >= self.min_ops)

    def worth_naming(self, e):
        """BH's rule, plus the triviality guard."""
        if e is None or e.is_Atom:
            return False
        if self.n_deps(e) < self.min_deps:
            return False
        return sp.count_ops(e) >= self.min_ops

    def _new_symbol(self):
        return sp.Symbol('%s_%d' % (self.prefix, len(self.order) + 1))

    def _pieces(self, e):
        """The parts of `e` a person would consider naming separately.

           Function arguments FIRST, because that is the case that matters:
           the long solutions are single atan2 calls, so their arguments are
           the only decomposition available.  Sums and products contribute
           their terms and factors."""

        if e.is_Atom:
            return []
        if isinstance(e, sp.Function):
            return list(e.args)
        if e.is_Add or e.is_Mul:
            return list(e.args)
        if e.is_Pow:
            #  sqrt(x) IS A Pow, not a Function -- sympy spells it x**(1/2) --
            #  so without this the pool walks straight past it and a
            #  `K_5 = sqrt(<forty terms>)` can never be shortened by anything.
            #  Measured on KinovaLite:  eight definitions 253pt too wide, all of
            #  them a sqrt of an expression nothing had looked inside
            #  (BH, 2026-09-04).  Nothing can break inside a radical either, so
            #  naming the radicand is the only lever there is.
            #
            #  BOTH ARGS, not just the base.  The rebuild below is
            #  e.func(*new_pieces), and Pow needs a base AND an exponent:
            #  handing it one argument raises, the except swallows it, and the
            #  ORIGINAL expression comes back -- so the definitions were emitted
            #  and then not used, which is exactly the shape the bug took
            #  (a `K_n = sqrt(...)` sitting beside the unshortened equation).
            #  The exponent is safe to include:  it is an atom (2, 1/2), and
            #  neither worth_naming() nor big_enough() will name an atom.
            return list(e.args)
        return []

    #  ------------------------------------------------------------------
    def split(self, expr, force=False):
        """(rewritten expression, new definitions created by this call).

           force=True names this expression's pieces WHATEVER their dependency
           count, subject only to the triviality guard.  It is for the caller
           who has MEASURED the equation as too wide for the page:  at that
           point the dependency rule has already had its say and been wrong,
           and the question is no longer "is this worth naming" but "this must
           be made narrower, what is there to name".

           Brad's th_3 is the case it exists for -- a single atan2 whose two
           arguments are 12 operations each and involve two solved variables,
           so every threshold declines it, while the typeset line is 144pt too
           wide."""

        before = len(self.order)
        try:
            out = self._walk(sp.sympify(expr), self.max_depth, force=force)
        except Exception:
            #  A rewrite that cannot be done must never cost us the equation.
            #  The report is still correct with the long form in it;  it is
            #  only harder to read.
            return expr, []
        return out, list(self.order[before:])

    def _walk(self, e, depth, force=False):
        if depth <= 0 or len(self.order) >= self.max_defs:
            return e

        pieces = self._pieces(e)
        if not pieces:
            return e

        new_pieces = []
        changed = False
        for p in pieces:
            #  FORCE NAMES WHAT CANNOT BE BROKEN, and nothing else.  A sum is
            #  breakable -- dmath puts its terms on separate lines by itself --
            #  so naming the terms of one buys no width and costs a definition
            #  apiece, plus an aggregator to add them back up.  Measured on
            #  Craig417:  K_54 and K_59 were each `K_a + K_b + K_c + K_d + ...`,
            #  four names to reconstruct a sum that reads perfectly well written
            #  out (BH, 2026-09-04).
            #
            #  The RECURSION STILL CARRIES force.  Not naming a term is not the
            #  same as leaving it alone:  inside that term there may be an
            #  atan2 whose arguments nothing can break, and those are exactly
            #  what force exists to name.  An earlier cut dropped force here and
            #  the equations stayed too wide.
            #  ...and the same applies to the dependency rule, which is what
            #  actually produced the aggregators:  K_7 = K_3+K_4+K_5+K_6+...
            #  came from naming four terms of one sum because each had three
            #  dependencies.  Sums are breakable either way, so NO rule names
            #  their terms;  both still recurse inside them.
            nameable_here = not e.is_Add
            if nameable_here and (self.worth_naming(p)
                                  or (force and self.big_enough(p))):
                #  Split INSIDE the piece before naming it, so a definition is
                #  itself compact:  K_2 = atan2(K_1, ...) rather than K_2
                #  swallowing everything below it.
                #  force PROPAGATES.  Naming only the top pieces is not enough:
                #  measured on Craig417 th_3, naming the two atan2 terms of the
                #  sum just moved the overflow into the definitions
                #  (K_1 = atan2(-sqrt(...), ...) was itself too wide for the
                #  page).  The recursion is bounded by big_enough() -- it stops
                #  as soon as a piece is under min_ops -- so this does not
                #  shatter the equation, it just keeps going while there is
                #  still something too big to print.
                inner = self._walk(p, depth - 1, force=force)

                #  RE-JUDGE AFTER THE INNER SPLIT.  worth_naming() saw the
                #  piece whole;  what would actually be stored is what is left
                #  once its interior has been named, and that can be trivial.
                #  Measured on Craig417 th_4, naming on the original size gave
                #  `K_2 = K_1*r_21` -- a name for one multiplication, four
                #  times over.  A definition has to earn its indirection at the
                #  size it will be PRINTED, not the size it started.
                if sp.count_ops(inner) < self.min_ops or self.all_named(inner):
                    new_pieces.append(inner)
                    changed = True
                    continue

                sym = self.named.get(inner)
                if sym is None:
                    sym = self._new_symbol()
                    self.named[inner] = sym
                    self.order.append((sym, inner))
                new_pieces.append(sym)
                changed = True
            else:
                sub = self._walk(p, depth - 1, force=force)
                new_pieces.append(sub)
                changed = changed or (sub is not p)

        if not changed:
            return e
        try:
            return e.func(*new_pieces)
        except Exception:
            #  Rebuilding failed (an unusual head).  Keep the original --
            #  a correct long equation beats a short wrong one.
            return e


#####################################################################
#
#   Test code
#

import unittest


class TestSolver025(unittest.TestCase):
    '''Naming the big pieces.

       The property that matters is that the rewrite is FAITHFUL:  a report is
       worse than useless if it is readable and wrong.  Everything else here is
       about the rule firing where it should and staying quiet where it should
       not.'''

    def setUp(self):
        print('\n\n===============  Test subexpression naming  =====================')
        return

    def runTest(self):
        self.test_seA_rewrite_is_faithful()
        self.test_seB_splits_inside_function_arguments()
        self.test_seC_leaves_few_dependency_terms_alone()
        self.test_seD_no_trivial_definitions()
        self.test_seE_shared_pieces_are_named_once()
        self.test_seF_prefix_avoids_a_collision()

    #  A stand-in Robot:  the pool only reads solution_nodes and params.
    class unk(object):
        def __init__(self, name):
            self.name = name
            self.symbol = sp.Symbol(name)
            self.solutionNames = [name + 's1', name + 's2']
            self.LHSversionNames = [name + 'v1', name + 'v2']

    class node(object):
        def __init__(self, u):
            self.unknown = u

    def robot(self, names=('th_1', 'th_2', 'th_3', 'th_4'), params=()):
        R = type('R', (object,), {})()
        R.solution_nodes = [TestSolver025.node(TestSolver025.unk(n))
                            for n in names]
        R.params = [sp.Symbol(p) for p in params]
        R.Mech = None
        return R

    def big(self):
        '''An atan2 whose two arguments each involve three solved variables --
           the shape every long IKBT solution actually has.'''
        t1, t2, t3, a, b = sp.symbols('th_1 th_2 th_3 a_2 d_4')
        y = (a*sp.sin(t1)*sp.cos(t2) + b*sp.cos(t1)*sp.sin(t3)
             - a*sp.sin(t2)*sp.cos(t3) + b*sp.sin(t1)*sp.sin(t2)*sp.cos(t3))
        x = (a*sp.cos(t1)*sp.cos(t2) - b*sp.sin(t1)*sp.sin(t3)
             + a*sp.cos(t2)*sp.cos(t3) - b*sp.cos(t1)*sp.sin(t2)*sp.sin(t3))
        return sp.atan2(y, x)

    def test_seA_rewrite_is_faithful(self):
        '''Substituting the definitions back must recover the original, exactly.

           This is the whole safety property.  The rewrite is presented to the
           reader as the same equation, so it has to BE the same equation --
           and nothing else in the report would catch it if it were not.'''
        fs = ' subexpression faithfulness FAIL'
        pool = SubexprPool(self.robot())
        e = self.big()
        new, defs = pool.split(e)
        self.assertTrue(defs, fs + ' (nothing was named, so nothing is tested)')

        #  Back-substitute in REVERSE definition order:  a later definition may
        #  refer to an earlier one.
        restored = new
        for sym, sub in reversed(defs):
            restored = restored.subs(sym, sub)
        self.assertEqual(sp.simplify(restored - e), 0,
                         fs + ' (the rewritten equation is not the original)')

    def test_seB_splits_inside_function_arguments(self):
        '''The long solutions are a single atan2, so a term-level rule would do
           nothing.  The arguments must be what gets named.'''
        fs = ' subexpression function-argument FAIL'
        pool = SubexprPool(self.robot())
        new, defs = pool.split(self.big())

        self.assertTrue(defs, fs + ' (a 3-dependency atan2 was not split)')
        self.assertLess(sp.count_ops(new), 6,
                        fs + ' (the equation did not actually get shorter)')
        self.assertEqual(type(new).__name__, 'atan2',
                         fs + ' (the atan2 itself was named -- that is a '
                         'rename, not a simplification)')

    def test_seC_leaves_few_dependency_terms_alone(self):
        '''Two dependencies is not more than two.  A rule that fires here would
           bury short equations under names for no benefit.'''
        fs = ' subexpression threshold FAIL'
        pool = SubexprPool(self.robot())
        t1, t2, a = sp.symbols('th_1 th_2 a_2')
        e = sp.atan2(a*sp.sin(t1)*sp.cos(t2) + a*sp.cos(t1)*sp.sin(t2),
                     a*sp.cos(t1)*sp.cos(t2) - a*sp.sin(t1)*sp.sin(t2))
        new, defs = pool.split(e)
        self.assertEqual(defs, [], fs + ' (named a 2-dependency piece)')
        self.assertEqual(new, e, fs + ' (rewrote an equation it should not have)')

    def test_seD_no_trivial_definitions(self):
        '''Every definition must be worth its indirection AT PRINTED SIZE.

           A piece is judged before its interior is split, so what actually
           gets stored can have collapsed to almost nothing:  measured on
           Craig417 th_4, the first cut produced `K_2 = K_1*r_21` four times
           over -- a name for one multiplication.'''
        fs = ' subexpression triviality FAIL'
        pool = SubexprPool(self.robot())
        pool.split(self.big())
        for sym, sub in pool.order:
            self.assertGreaterEqual(
                sp.count_ops(sub), pool.min_ops,
                fs + ' (%s = %s is too small to deserve a name)'
                % (sym, sub))

    def test_seE_shared_pieces_are_named_once(self):
        '''The same subexpression in two equations gets ONE name and ONE
           definition -- which is most of the saving in the versions section,
           where the same inner expressions recur in every row.'''
        fs = ' subexpression sharing FAIL'
        pool = SubexprPool(self.robot())
        e = self.big()
        n1, d1 = pool.split(e)
        n2, d2 = pool.split(e)
        self.assertTrue(d1, fs + ' (first split named nothing)')
        self.assertEqual(d2, [], fs + ' (re-defined pieces it had already named)')
        self.assertEqual(n1, n2, fs + ' (same input, different rewrite)')

    def test_seF_prefix_avoids_a_collision(self):
        '''K is only the DEFAULT.  a_i was the natural choice and is a DH link
           length in 7 of the 32 robots;  a prefix that clashes puts two
           meanings on one symbol in one document.'''
        fs = ' subexpression prefix FAIL'
        self.assertEqual(choose_prefix(self.robot()), 'K',
                         fs + ' (unexpected default prefix)')

        clash = self.robot(params=('K_1', 'a_2'))
        self.assertNotEqual(choose_prefix(clash), 'K',
                            fs + ' (chose a prefix the robot already uses)')

        joints = self.robot(names=('K_1', 'th_2', 'th_3'))
        self.assertNotEqual(choose_prefix(joints), 'K',
                            fs + ' (chose a prefix that is a joint name)')


def run_test():
    t = TestSolver025()
    t.setUp()
    t.runTest()
    print('\n\n            subexpression naming PASSES\n\n')


if __name__ == '__main__':
    unittest.main()
