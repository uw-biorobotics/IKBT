#!/usr/bin/python
#
#   output_onevar_python.py --  generated python for the ONE-VARIABLE method
#
#   The one-variable method answers a robot IKBT cannot solve outright by
#   assuming ONE of its unknowns is known and solving the rest in closed form.
#   That closed form is exact for the TRUE arm -- no DH parameter was touched --
#   but only where the assumed value is right, which is a 1-D set.  Finding it
#   is a 1-D search, and this module writes the code that does it.
#
#   WHAT THE USER GETS, for a one-variable robot called C-Arm:
#
#       IK_onevarC-Arm.py                <- the top level, the search
#         loads IK_conditionalC-Arm.py     closed form, th_2 as an argument
#         loads FK_numericC-Arm.py         the TRUE arm's FK
#
#   THREE FILES, NOT ONE, for the reason the hybrid method has four:  the thing
#   that can go wrong here is using the conditional closed form as though it
#   were an answer, and a file named for what it is makes that visible.
#
#   THE ENTRY POINT IS solve_<Robot>(T).  Unlike the hybrid method there is no
#   posture to choose between phases -- the search finds every branch that
#   reaches T and returns them all, so there is nothing for a caller to commit
#   to in the middle.  The pieces are exposed anyway (branches_at, errors,
#   sweep), because a 1-D error curve is worth plotting when a pose comes back
#   unreachable.
#
#   HOW THE SEARCH WORKS.  See SEARCH_CORE below:  a van der Corput sweep of
#   the assumed variable, local minima of the pose error bracketed per branch,
#   each bracket refined by golden section, and a minimum accepted only if it
#   reaches ZERO.  A reachable pose drives a true root to zero;  a dip that
#   stops short is an artifact of that branch, not a solution.
#
#   Friedman, D.C.W., Kowalewski, T., Jovanovic, R., Rosen, J. and Hannaford, B.
#   "Freeing the serial mechanism designer from inverse kinematic solvability
#   constraints", Applied Bionics and Biomechanics 7(3), 2010, 209-216.
#
#   Copyright 2026 University of Washington
#
#   Developed by Blake Hannaford
#   BioRobotics Lab, University of Washington

import os

import numpy as np
import sympy as sp

import ikbtbasics.numeric_ik as nik
from ikbtfunctions.output_python import py_identifier
from ikbtfunctions.output_numeric_common import (DIR_NAME, MODULE_HEADER,
                                                 POSE_ERROR_CORE,
                                                 write_fk_module)


def search_domain(M, known, ndof=None):
    """(lo, hi, periodic) -- the interval the assumed variable is searched over.

       A revolute joint, and every sum-of-angles variable, is periodic on
       [-pi, pi):  that is the whole domain and the search is complete over it.

       A PRISMATIC joint has no such domain.  Nothing in a DH table records how
       far a slide travels, so the interval is a guess and is documented as one
       in the generated file:  +/- the sum of the arm's own lengths, which is
       past anything the mechanism can reach, at the cost of sampling a range
       that is mostly out of reach.  A user who knows the travel should narrow
       it -- that is why the bounds are module constants and an argument, not
       baked into the loop."""

    ndof = ndof or nik.dof_of(M)
    syms = [str(s) for s in nik.joint_symbols(M, ndof)]
    vv = list(getattr(M, 'vv', None) or [])

    if known in syms and vv:
        i = syms.index(known)
        if i < len(vv) and not vv[i]:          # vv[i] == 0 -> prismatic
            pv = nik.pvals_numeric(M)
            reach = 0.0
            for r in range(ndof):
                for c in (1, 2):               # the a and d columns
                    try:
                        reach += abs(float(sp.Matrix(M.DH)[r, c].subs(pv)))
                    except (TypeError, ValueError):
                        pass
            reach = reach or 1.0
            return -reach, reach, False

    import math
    return -math.pi, math.pi, True


#  The search, emitted verbatim.  A generated module stands on numpy alone and
#  cannot import IKBT, so this is the only copy -- unlike the hybrid method's
#  damped least squares, which is also a library (ikbtbasics/numeric_ik.py)
#  because the hybrid branch is not its only caller.  Nothing but generated
#  code runs this, so a second copy would be one to keep in step for nobody.
#  TestSolver029 generates a module and exercises it instead.
SEARCH_CORE = '''

#############################################################
#
#   The 1-D search over the assumed variable
#
#   The closed form above solves every OTHER joint given a value for
#   KNOWN_VARIABLE.  Feed it the wrong value and it still returns joint
#   vectors -- they just do not reach the goal pose.  So the error between
#   the goal and where those joints actually put the arm is a function of
#   one variable, and the solutions of the true arm are its ZEROS.
#
#   SAMPLING IS A VAN DER CORPUT SEQUENCE.  Each new point falls in the
#   middle of the largest untested gap, with no bias toward either end, and
#   the sequence is a prefix of itself:  raising n_samples re-tests nothing
#   and refines everywhere.  A uniform grid has neither property.
#
#   ONE ERROR CURVE PER BRANCH.  The closed form returns several branches
#   (elbow up/down, wrist flipped), and they are separate functions of the
#   assumed variable with separate zeros -- and separate domains, since a
#   branch vanishes where its own arcsin leaves [-1, 1].  Mixing them into
#   one curve would bracket minima that no single branch has.
#
#   ACCEPTED ONLY IF IT REACHES ZERO.  This is root finding wearing
#   minimisation's clothes:  a reachable pose drives a true solution's error
#   to zero, so a local minimum that stops above ACCEPT_TOL is a feature of
#   that branch's curve and not a solution.  Returning it would be worse
#   than returning nothing.
#
#############################################################


def _vdc(n, base=2):
    """The van der Corput radical inverse of n, in [0, 1)."""
    v, denom = 0.0, 1.0
    while n:
        denom *= base
        n, rem = divmod(n, base)
        v += rem / denom
    return v


def sample_values(n_samples=N_SAMPLES, lo=None, hi=None):
    """`n_samples` values of the assumed variable, ASCENDING.

       Drawn as a van der Corput sequence and then sorted:  which order they
       are drawn in matters if you stop early, but the sweep evaluates all of
       them and the bracketing needs neighbours."""
    lo = SEARCH_LO if lo is None else lo
    hi = SEARCH_HI if hi is None else hi
    span = hi - lo
    return sorted(lo + span * _vdc(i) for i in range(int(n_samples)))


def branches_at_**IDENT**(T, value):
    """Every closed-form branch at one assumed value.

       -> list of joint vectors, NDOF floats each, in JOINT_NAMES order and
       INCLUDING the assumed variable in its own chain position.  [] when the
       closed form reports the pose unreachable for that value, which is
       ordinary: a branch is defined on part of the range, not all of it."""

    T = np.matrix(np.asarray(T, dtype=float))
    try:
        #  errstate:  at some values of the assumed variable the closed form
        #  divides by zero or takes a root of a negative -- ArmRobo does both.
        #  That is the branch being undefined there, which is the same answer
        #  as an arcsine out of range and is DATA, not a fault;  numpy would
        #  otherwise print a RuntimeWarning for each of a thousand samples.
        #  The non-finite result it produces is caught by the isfinite test in
        #  errors_**IDENT**().
        with np.errstate(all='ignore'):
            sols = cond_ik.ikin_**IDENT**_given(T, float(value))
    except Exception:
        #  A domain error inside the closed form means "not here", not "stop".
        return []
    if sols is False or sols is None:
        return []
    return [[float(v) for v in row] for row in sols]


def errors_**IDENT**(T, value):
    """Pose error of each branch at one assumed value;  np.inf where undefined.

       This is the function the search minimises, one entry per branch."""

    out = []
    for q in branches_at_**IDENT**(T, value):
        try:
            with np.errstate(all='ignore'):
                Tq = np.asarray(true_fk.fk_**IDENT**(q), dtype=float)
        except Exception:
            out.append(np.inf)
            continue
        if not np.all(np.isfinite(Tq)):
            out.append(np.inf)
            continue
        out.append(pose_error(Tq, T)[1])
    return out


def sweep_**IDENT**(T, n_samples=N_SAMPLES):
    """The raw scan:  (values, curves) with curves[branch][i] the error.

       For plotting, and for understanding a pose that comes back unreachable.

       ON A PERIODIC DOMAIN THE SCAN IS WRAPPED AT BOTH ENDS:  the last sample
       is repeated below SEARCH_LO and the first above SEARCH_HI.  Without that
       halo a root sitting at either end has no bracket -- it is a minimum with
       only one neighbour -- and the search would miss every solution near
       +/- pi.  The halo values lie outside the range on purpose;  _wrap() puts
       anything found there back inside.

       On a non-periodic (prismatic) range there is nothing to wrap to, so a
       root exactly at an end point is not bracketed.  Widen the range if that
       is a worry -- for a guessed range it is what the guess is for."""

    values = sample_values(n_samples)
    cols = [errors_**IDENT**(T, t) for t in values]
    if PERIODIC and values:
        span = SEARCH_HI - SEARCH_LO
        values = [values[-1] - span] + values + [values[0] + span]
        cols = [cols[-1]] + cols + [cols[0]]
    nb = max([len(c) for c in cols] or [0])
    curves = [[(c[b] if b < len(c) else np.inf) for c in cols]
              for b in range(nb)]
    return values, curves


def _golden(f, a, b, iters=80):
    """Minimise f on [a, b] by golden section.  -> (x, f(x)).

       Golden section and not a derivative method:  near a solution the error
       is a V, not a parabola -- it is a norm going to zero -- so its slope
       jumps sign and nothing based on curvature behaves.  Unimodality on the
       bracket is all this needs, and 80 iterations shrink the bracket by
       0.618**80, far past float resolution."""

    invphi = (5.0 ** 0.5 - 1.0) / 2.0
    invphi2 = (3.0 - 5.0 ** 0.5) / 2.0
    h = b - a
    c, d = a + invphi2 * h, a + invphi * h
    fc, fd = f(c), f(d)
    for _ in range(int(iters)):
        if fc < fd:
            b, d, fd = d, c, fc
            h *= invphi
            c = a + invphi2 * h
            fc = f(c)
        else:
            a, c, fc = c, d, fd
            h *= invphi
            d = a + invphi * h
            fd = f(d)
    return (c, fc) if fc < fd else (d, fd)


def _local_minima(curve):
    """Indices i where curve dips:  curve[i] < curve[i-1] and <= curve[i+1].

       A neighbour of np.inf is allowed and is often where a real solution is:
       inf means the branch is undefined there, so the curve falls off a cliff
       at the edge of that branch's domain and the minimum sits against it."""

    out = []
    for i in range(1, len(curve) - 1):
        if not np.isfinite(curve[i]):
            continue
        if curve[i] < curve[i - 1] and curve[i] <= curve[i + 1]:
            out.append(i)
    return out


def _wrap(t):
    """A value back into [SEARCH_LO, SEARCH_HI) on a periodic domain."""
    if not PERIODIC:
        return t
    span = SEARCH_HI - SEARCH_LO
    return SEARCH_LO + (t - SEARCH_LO) % span


def solve_**IDENT**(T, n_samples=N_SAMPLES, tol=None, refine_iters=80):
    """Goal pose T (4x4) -> every joint vector that reaches it.

       Each entry is a dict:

           q             NDOF floats, JOINT_NAMES order, ready for FK
           known_value   the value of KNOWN_VARIABLE it was found at
           branch        which branch of the closed form it came from
           error         ||dp|| + W_ROT*theta, what ACCEPT_TOL is measured in

       [] means no solution was FOUND, which is not quite "unreachable":  a
       basin narrower than the gap between samples can be stepped over.  Raise
       n_samples if a pose you believe in comes back empty -- the sweep is a
       van der Corput sequence, so the extra points land in the gaps and none
       of the old work is repeated."""

    tol = ACCEPT_TOL if tol is None else tol
    values, curves = sweep_**IDENT**(T, n_samples)

    found = []
    for b, curve in enumerate(curves):

        def err_at(t, b=b):
            e = errors_**IDENT**(T, _wrap(t))
            return e[b] if b < len(e) else np.inf

        for i in _local_minima(curve):
            t_star, e_star = _golden(err_at, values[i - 1], values[i + 1],
                                     refine_iters)
            if e_star > tol:
                continue                 # a dip, not a root -- see the header
            t_star = _wrap(t_star)
            qs = branches_at_**IDENT**(T, t_star)
            if b >= len(qs):
                continue
            found.append({'q': qs[b], 'known_value': float(t_star),
                          'branch': int(b), 'error': float(e_star)})

    #  Two branches can converge on the same posture -- IKBT enumerates version
    #  combinations without discarding duplicates, and a bracket found from
    #  either side lands in the same place.  Same joints, same solution.
    unique = []
    for s in sorted(found, key=lambda s: s['error']):
        if not any(np.max(np.abs(np.array(s['q']) - np.array(u['q']))) < DEDUP_TOL
                   for u in unique):
            unique.append(s)
    return unique


def solve_**IDENT**_labeled(T, n_samples=N_SAMPLES, tol=None):
    """solve_**IDENT**(), with each q keyed by joint name."""
    out = []
    for s in solve_**IDENT**(T, n_samples=n_samples, tol=tol):
        s = dict(s)
        s['q'] = dict(zip(JOINT_NAMES, s['q']))
        out.append(s)
    return out
'''


MAIN_BLOCK = '''

#
#    TEST CODE:  pick a pose the arm can reach, and go back to it
#
if __name__ == "__main__":

    import random

    random.seed(0)
    q_true = [random.uniform(-1.0, 1.0) for _ in range(NDOF)]
    T = true_fk.fk_**IDENT**(q_true)

    print('%s:  searching over %s in [%.3f, %.3f]'
          % (ROBOT, KNOWN_VARIABLE, SEARCH_LO, SEARCH_HI))
    print('  a reachable pose, from joints:')
    print('   ', dict(zip(JOINT_NAMES, [round(v, 4) for v in q_true])))

    sols = solve_**IDENT**(T)
    print('  %d solution(s) found:' % len(sols))
    for s in sols:
        print('    %s = %8.4f   error %.3e' % (KNOWN_VARIABLE,
                                               s['known_value'], s['error']))
        print('      ', dict(zip(JOINT_NAMES, [round(v, 4) for v in s['q']])))
    if not sols:
        print('    none -- try a larger n_samples')
'''


def write_onevar_top(M, name, known, dirname=DIR_NAME, n_samples=128):
    '''Write IK_onevar<name>.py -- the 1-D search over the assumed variable.

       M       the TRUE arm's mechanism (nothing here is approximated)
       name    the robot the user asked about
       known   the variable the closed form assumes is known, e.g. 'th_2'

       Returns the path written.'''

    ident = py_identifier(name)
    ndof = nik.dof_of(M)
    jnames = [str(s) for s in nik.joint_symbols(M, ndof)]
    #  `or 1.0`:  w_rot scales the acceptance tolerance, and a degenerate DH
    #  table (every length zero) would otherwise make it 0 -- a test no root
    #  can pass, so a solve that worked would report nothing found.
    w_rot = float(nik.w_rot_for(M, ndof)) or 1.0
    lo, hi, periodic = search_domain(M, known, ndof)

    path = os.path.join(dirname, 'IK_onevar%s.py' % name)
    with open(path, 'w') as f:
        hdr = MODULE_HEADER.replace('**ROBOT**', name)
        hdr = hdr.replace('**WHAT**',
                          'ONE-VARIABLE inverse kinematics for %s' % name)
        print(hdr, file=f)

        #  SIBLINGS ARE LOADED BY PATH, not imported by name.  A robot name is
        #  not always a python identifier -- 'C-Arm' and 'Raven-II' are not --
        #  and `import IK_conditionalC-Arm` is a syntax error.  The file names
        #  keep the robot's real name, which is what makes a directory listing
        #  readable, so the loader takes the strain.
        print('import os', file=f)
        print('import importlib.util', file=f)
        print('', file=f)
        print('_HERE = os.path.dirname(os.path.abspath(__file__))', file=f)
        print('''

def _sibling(filename, modname):
    """Load a generated module sitting next to this one, by PATH.

       Robot names are not always python identifiers ('C-Arm'), so the file
       name cannot be an import statement."""
    spec = importlib.util.spec_from_file_location(
        modname, os.path.join(_HERE, filename))
    if spec is None:
        raise ImportError('cannot find %s next to %s' % (filename, __file__))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod

''', file=f)
        print("cond_ik = _sibling('IK_conditional%s.py', 'onevar_cond_%s')"
              % (name, ident), file=f)
        print("true_fk = _sibling('FK_numeric%s.py', 'onevar_fk_%s')"
              % (name, ident), file=f)
        print('', file=f)

        print('''
#############################################################
#
#   READ THIS FIRST.
#
#   %s could not be solved in closed form outright.  What
#   IK_conditional%s.py holds is a closed form for every OTHER
#   joint, given a value of %s -- exact for THIS arm, no DH parameter
#   changed, but only where that value is right.
#
#   solve_%s(T) is the entry point:  it searches %s for the
#   values that are right, and returns complete joint vectors that reach T.
#   A joint vector taken straight from the conditional closed form, at a
#   value you picked yourself, does NOT put the robot at the pose you asked
#   for.
#
#############################################################
''' % (name, name, known, ident, known), file=f)

        print('ROBOT          = %r' % name, file=f)
        print('KNOWN_VARIABLE = %r' % known, file=f)
        print('#  Chain order -- what a q vector is, and what comes back.', file=f)
        print('JOINT_NAMES    = %r' % jnames, file=f)
        print('NDOF           = %d' % ndof, file=f)
        print('', file=f)
        print('#  One characteristic arm length per radian:  the weight that puts', file=f)
        print('#  position error and orientation error in comparable units.  Baked', file=f)
        print("#  in because nothing records a model's units and they are not", file=f)
        print('#  consistent across the robot set.  In the DH table\'s units.', file=f)
        print('W_ROT          = %r' % float(w_rot), file=f)
        print('', file=f)
        if periodic:
            print('#  %s is periodic, so this is the WHOLE domain and the'
                  % known, file=f)
            print('#  search over it is complete.', file=f)
        else:
            print('#  %s is PRISMATIC and its travel is not recorded anywhere in'
                  % known, file=f)
            print('#  a DH table, so these bounds are a guess:  the sum of the', file=f)
            print("#  arm's own lengths, which is past anything it can reach.", file=f)
            print('#  Narrow them to the real travel if you know it -- the sweep', file=f)
            print('#  spends its samples evenly over whatever range it is given.', file=f)
        print('SEARCH_LO      = %r' % float(lo), file=f)
        print('SEARCH_HI      = %r' % float(hi), file=f)
        print('PERIODIC       = %r' % bool(periodic), file=f)
        print('', file=f)
        print('#  Samples of the first sweep.  More of them find narrower basins', file=f)
        print('#  and cost one closed-form evaluation each.', file=f)
        print('N_SAMPLES      = %d' % int(n_samples), file=f)
        print('', file=f)
        print('#  A refined minimum is a SOLUTION only if it gets this close --', file=f)
        print('#  a millionth of an arm length.  Loose enough to survive float', file=f)
        print('#  noise in the closed form, tight enough that a local dip which', file=f)
        print('#  is not a root cannot pass.', file=f)
        print('ACCEPT_TOL     = %r' % (1e-6 * float(w_rot)), file=f)
        print('', file=f)
        print('#  Two solutions closer than this in every joint are one solution.',
              file=f)
        print('DEDUP_TOL      = 1e-6', file=f)

        print(POSE_ERROR_CORE, file=f)
        print(SEARCH_CORE.replace('**IDENT**', ident), file=f)
        print(MAIN_BLOCK.replace('**IDENT**', ident), file=f)

    return path


#####################################################################
#
#   Test code
#

import math
import shutil
import tempfile
import importlib.util
import unittest


class TestSolver029(unittest.TestCase):
    '''The one-variable code generator:  the search range, and the search.

       THE SEARCH IS TESTED ON A TOY, not on a robot.  Every piece of it --
       sampling, the per-branch curves, bracketing, the wrap, the refinement,
       what is accepted -- is generic, and a toy whose roots are known by
       arithmetic pins all of it in a second.  Generating a real robot's search
       means solving a real robot first, which is minutes and belongs where the
       rest of the end-to-end checking is:  scripts/numerical_closed_loop_sol_check.py,
       which round-trips C-Arm through exactly these generated files.'''

    def setUp(self):
        print('\n\n==========  Test one-variable code generator  ============')
        return

    def runTest(self):
        self.test_ovgA_revolute_range_is_the_whole_circle()
        self.test_ovgB_prismatic_range_is_a_documented_guess()
        self.test_ovgC_search_finds_every_root()
        self.test_ovgD_a_dip_that_is_not_a_root_is_rejected()
        self.test_ovgE_roots_at_the_wrap_are_found()

    #  ----------------------------------------------------------  fixtures

    class mech(object):
        '''Enough of a mechanism for nik.dof_of / joint_symbols / w_rot_for.'''
        def __init__(self, dh, vv, pvals=None):
            self.DH, self.vv, self.pvals = dh, vv, (pvals or {})

    def two_link(self, prismatic=False):
        """A 2-joint planar arm, padded to 6 DH rows like every real table."""
        th_1, th_2 = sp.symbols('th_1 th_2')
        d_1 = sp.Symbol('d_1')
        first = d_1 if prismatic else th_1
        rows = [[0, sp.Integer(0), first if prismatic else sp.Integer(0),
                 sp.Integer(0) if prismatic else th_1],
                [0, sp.Integer(2), sp.Integer(0), th_2]]
        rows += [[0, sp.Integer(0), sp.Integer(0), sp.Integer(0)]
                 for _ in range(4)]
        vv = [0 if prismatic else 1, 1, 1, 1, 1, 1]
        return TestSolver029.mech(sp.Matrix(rows), vv, {d_1: 1})

    #  A toy pair of sibling modules.  The FK reports the SECOND joint as an x
    #  displacement, so the pose error of a branch is |q[1]|, and the branches
    #  below make that a function of the assumed variable with roots we know.
    FK_TOY = '''
import numpy as np


JOINT_NAMES = ['th_1', 'th_2']
NDOF = 2


def fk_**IDENT**(q):
    th_1, th_2 = [float(v) for v in q]
    return np.array([[1.0, 0.0, 0.0, th_2],
                     [0.0, 1.0, 0.0, 0.0],
                     [0.0, 0.0, 1.0, 0.0],
                     [0.0, 0.0, 0.0, 1.0]], dtype=float)
'''

    #  branch 0:  sin(3t), zero at every multiple of pi/3  -> 6 roots in [-pi, pi)
    #  branch 1:  t/2,     zero at 0                       -> the same posture,
    #                                                          so it dedups away
    #  branch 2:  1 + cos(t)/2, never below 0.5             -> dips, never a root
    IK_TOY = '''
import numpy as np


KNOWN_VARIABLE = 'th_1'


def ikin_**IDENT**_given(T, th_1):
    t = float(th_1)
    return [[t, np.sin(3.0 * t)],
            [t, 0.5 * t],
            [t, 1.0 + 0.5 * np.cos(t)]]
'''

    def build_toy(self, name='ToyOneVar'):
        """Generate a search module over the toy siblings.  -> (dir, module)."""
        d = tempfile.mkdtemp(prefix='ikbt_onevar_')
        ident = py_identifier(name)
        for tmpl, fn in ((self.FK_TOY, 'FK_numeric%s.py' % name),
                         (self.IK_TOY, 'IK_conditional%s.py' % name)):
            with open(os.path.join(d, fn), 'w') as f:
                f.write(tmpl.replace('**IDENT**', ident))

        path = write_onevar_top(self.two_link(), name, 'th_1', dirname=d)
        spec = importlib.util.spec_from_file_location('toy_' + ident, path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return d, mod

    def toy_solutions(self, mod, **kw):
        """solve_*() on the target x = 0, as (known_value, error) pairs."""
        T = np.eye(4)
        solve = getattr(mod, 'solve_' + py_identifier('ToyOneVar'))
        return [(s['known_value'], s['error']) for s in solve(T, **kw)]

    #  ----------------------------------------------------------  the range

    def test_ovgA_revolute_range_is_the_whole_circle(self):
        '''A revolute assumed variable is searched over [-pi, pi), and that is
           the complete domain -- so a search over it is a complete search.'''
        fs = ' onevar search range FAIL'

        lo, hi, periodic = search_domain(self.two_link(), 'th_1')
        self.assertAlmostEqual(lo, -math.pi, msg=fs)
        self.assertAlmostEqual(hi, math.pi, msg=fs)
        self.assertTrue(periodic, fs + ' (a revolute joint is periodic)')

        #  and so is a sum-of-angles variable, which is not in the DH table
        lo, hi, periodic = search_domain(self.two_link(), 'th_12')
        self.assertTrue(periodic, fs + ' (a sum of angles is periodic too)')

    def test_ovgB_prismatic_range_is_a_documented_guess(self):
        '''A prismatic assumed variable has no domain -- nothing records how far
           a slide travels -- so the range is a guess, and must be finite, not
           periodic, and big enough to contain the arm.'''
        fs = ' onevar prismatic range FAIL'

        lo, hi, periodic = search_domain(self.two_link(prismatic=True), 'd_1')
        self.assertFalse(periodic, fs + ' (a slide does not wrap)')
        self.assertLess(lo, 0.0, fs)
        self.assertGreaterEqual(hi, 2.0,
                                fs + " (narrower than the arm's own length)")
        self.assertAlmostEqual(lo, -hi, msg=fs + ' (not symmetric)')

    #  ----------------------------------------------------------  the search

    def test_ovgC_search_finds_every_root(self):
        '''sin(3t) is zero at six values of t in [-pi, pi).  All six, and
           nothing else, and each one to machine precision.'''
        fs = ' onevar search FAIL'

        d, mod = self.build_toy()
        try:
            found = self.toy_solutions(mod)
        finally:
            shutil.rmtree(d, ignore_errors=True)

        want = sorted(k * math.pi / 3.0 for k in range(-3, 3))
        got = sorted(t for t, e in found)
        self.assertEqual(len(got), len(want),
                         fs + ' (found %s, expected %s)'
                         % ([round(t, 4) for t in got],
                            [round(t, 4) for t in want]))
        for g, w in zip(got, want):
            self.assertAlmostEqual(g, w, places=7,
                                   msg=fs + ' (root off by %.2e)' % abs(g - w))
        for t, e in found:
            self.assertLess(e, 1e-9, fs + ' (accepted an error of %.2e)' % e)

        #  branch 1 has its own root at t = 0, which is the same posture as
        #  branch 0's root there -- one solution, not two
        self.assertEqual(len([t for t in got if abs(t) < 1e-9]), 1,
                         fs + ' (the duplicate posture was not deduplicated)')

    def test_ovgD_a_dip_that_is_not_a_root_is_rejected(self):
        '''Branch 2 is 1 + cos(t)/2:  a clean local minimum of 0.5 at t = +/-pi,
           and not a solution.  Accepting it would be worse than returning
           nothing -- the caller would drive the arm somewhere else entirely.'''
        fs = ' onevar acceptance FAIL'

        d, mod = self.build_toy()
        try:
            found = self.toy_solutions(mod)
            #  it IS a local minimum:  the curve is there to be found
            values, curves = getattr(mod, 'sweep_' + py_identifier('ToyOneVar'))(
                np.eye(4))
            self.assertEqual(len(curves), 3, fs + ' (expected three branches)')
            self.assertGreater(min(curves[2]), 0.49,
                               fs + ' (fixture: branch 2 should never reach 0)')
            self.assertLess(min(curves[2]), 0.51, fs + ' (fixture)')
        finally:
            shutil.rmtree(d, ignore_errors=True)

        for t, e in found:
            self.assertLess(e, 1e-9,
                            fs + ' (a minimum of %.3f was returned as a '
                            'solution)' % e)

    def test_ovgE_roots_at_the_wrap_are_found(self):
        '''t = -pi is a root of sin(3t) and sits at the very edge of the range.

           A sweep that does not wrap leaves it with one neighbour, so it is
           never bracketed and never found.  That is a whole posture missing
           from the answer, silently, so it gets its own test.'''
        fs = ' onevar wrap FAIL'

        d, mod = self.build_toy()
        try:
            found = self.toy_solutions(mod)
        finally:
            shutil.rmtree(d, ignore_errors=True)

        edge = [t for t, e in found if abs(abs(t) - math.pi) < 1e-6]
        self.assertEqual(len(edge), 1,
                         fs + ' (the root at the wrap was %s)'
                         % ('found twice, once at each end' if edge
                            else 'not found'))


def run_test():
    suite = unittest.TestLoader().loadTestsFromTestCase(TestSolver029)
    unittest.TextTestRunner(verbosity=2).run(suite)


##   Self Tests
if __name__ == "__main__":
    run_test()
