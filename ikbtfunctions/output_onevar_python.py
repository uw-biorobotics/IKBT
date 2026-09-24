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
#   HOW THE SEARCH WORKS.  See SEARCH_CORE below:  a uniform sweep of the
#   assumed variable, local minima of the pose error bracketed per branch,
#   each bracket refined by golden section, and a minimum accepted only if it
#   reaches ZERO.  A reachable pose drives a true root to zero;  a dip that
#   stops short is an artifact of that branch, not a solution.
#
#   AND THEN THE WHOLE THING AGAIN AT TWICE THE RESOLUTION, until two
#   successive grids agree about the answer.  Two roots can sit closer
#   together than one sample spacing -- C-Arm has poses where they are 0.045
#   rad apart -- and a grid that steps over the pair returns one posture where
#   there are two, with nothing in the output to say so.  A fixed resolution
#   cannot rule that out;  a grid that is refined until the answer repeats
#   can, and costs little more, because the doubling re-uses every sample.
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
#   SAMPLING IS A UNIFORM GRID, AND THE GRID IS HALVED UNTIL THE ANSWER
#   STOPS CHANGING.  Two solutions of a real arm can sit closer together in
#   the assumed variable than one sample spacing -- C-Arm has reachable poses
#   whose roots are 0.045 rad apart, against a spacing of 0.049 at 128
#   samples -- and a scan that steps over such a pair sees one dip where
#   there are two and returns one posture where there are two.  Silently.
#   No fixed resolution can be ARGUED to be enough, so this does not argue
#   one:  it solves at n samples, again at 2n, and again, until two
#   successive resolutions return the SAME set of postures, or MAX_SAMPLES
#   is reached.  Doubling a uniform grid re-uses every point of the coarser
#   one -- n is a power of two, so the values are bit-identical -- and every
#   sample's error is cached, so the whole ladder costs little more than its
#   finest rung.  N_SAMPLES is where the ladder STARTS, not where it stops.
#
#   (This was a van der Corput sequence, whose selling point is that it is a
#   prefix of itself.  That property was never used:  the sweep evaluates
#   every point it draws and then SORTS them, and a sorted van der Corput
#   sequence of 2^k points IS the uniform grid of 2^k points.  At any other
#   count it is an UNEVEN grid, which is strictly worse for this job -- a
#   bracketing scan is only as good as its widest gap.)
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


def sample_values(n_samples=N_SAMPLES, lo=None, hi=None):
    """`n_samples` evenly spaced values of the assumed variable, ASCENDING.

       A UNIFORM GRID.  On a periodic domain the two ends are the same point,
       so the top one is left off and there are exactly n_samples of them;  on
       a finite range both ends are wanted, so there are n_samples + 1.

       Written as lo + span * (k / n) rather than by accumulating a step:  for
       n a power of two, k / n is exact in binary and (2k) / (2n) is the SAME
       float, which is what lets the doubling ladder re-use the coarse grid's
       cached errors instead of recomputing them a bit away."""
    lo = SEARCH_LO if lo is None else lo
    hi = SEARCH_HI if hi is None else hi
    n = int(n_samples)
    span = hi - lo
    top = n if PERIODIC else n + 1
    return [lo + span * (k / float(n)) for k in range(top)]


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


def _cached_errors_**IDENT**(T, value, cache):
    """errors_**IDENT**() memoised on the sample value.

       The doubling ladder revisits every coarse sample at every finer
       resolution, and a sample costs one closed-form solve plus one FK per
       branch.  Keyed on the float itself, which is exact across a doubling
       for a power-of-two n -- see sample_values()."""
    key = float(value)
    if key not in cache:
        cache[key] = errors_**IDENT**(T, key)
    return cache[key]


def _scan_**IDENT**(T, n_samples, cache):
    """(values, curves) at one resolution;  curves[branch][i] is the error.

       EVERY REAL SAMPLE GETS TWO NEIGHBOURS, so every one of them can be the
       middle of a bracket.  On a periodic domain that is a wrap:  the last
       sample is repeated below SEARCH_LO and the first above SEARCH_HI,
       without which a root sitting at either end is a minimum with only one
       neighbour and is never found -- a whole posture missing, silently.
       _wrap() puts anything found out there back inside.

       On a finite (prismatic) range there is nothing to wrap to, so the grid
       is extended by one step past each end instead:  the closed form is as
       evaluable there as anywhere, and a root found just outside a GUESSED
       range is worth more than a root not found."""

    values = sample_values(n_samples)
    if not values:
        return values, []
    cols = [_cached_errors_**IDENT**(T, t, cache) for t in values]
    if PERIODIC:
        span = SEARCH_HI - SEARCH_LO
        values = [values[-1] - span] + values + [values[0] + span]
        cols = [cols[-1]] + cols + [cols[0]]
    else:
        step = (SEARCH_HI - SEARCH_LO) / float(int(n_samples))
        lo_x, hi_x = values[0] - step, values[-1] + step
        values = [lo_x] + values + [hi_x]
        cols = ([_cached_errors_**IDENT**(T, lo_x, cache)] + cols
                + [_cached_errors_**IDENT**(T, hi_x, cache)])
    nb = max([len(c) for c in cols] or [0])
    curves = [[(c[b] if b < len(c) else np.inf) for c in cols]
              for b in range(nb)]
    return values, curves


def sweep_**IDENT**(T, n_samples=N_SAMPLES):
    """The raw scan at ONE resolution:  (values, curves).

       For plotting, and for understanding a pose that comes back unreachable.
       solve_**IDENT**() does not stop at one resolution -- it climbs a ladder
       of them until the answer repeats."""
    return _scan_**IDENT**(T, n_samples, {})


def _golden(f, a, b, iters=80, xtol=1e-15):
    """Minimise f on [a, b] by golden section.  -> (x, f(x)).

       Golden section and not a derivative method:  near a solution the error
       is a V, not a parabola -- it is a norm going to zero -- so its slope
       jumps sign and nothing based on curvature behaves.  Unimodality on the
       bracket is all this needs.

       It stops when the bracket reaches float resolution, which is what the
       tail of those 80 iterations used to be spent on:  0.618**80 is far
       below it, so the last dozen-odd passes were deciding between two floats
       that are equal.  The ladder in solve_**IDENT**() calls this many times
       per resolution, so that tail is worth not walking."""

    invphi = (5.0 ** 0.5 - 1.0) / 2.0
    invphi2 = (3.0 - 5.0 ** 0.5) / 2.0
    h = b - a
    c, d = a + invphi2 * h, a + invphi * h
    fc, fd = f(c), f(d)
    for _ in range(int(iters)):
        if h <= xtol * (1.0 + abs(a) + abs(b)):
            break
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
       at the edge of that branch's domain and the minimum sits against it.

       Index 0 and the last index are never candidates and do not need to be:
       _scan_**IDENT**() has already given every real sample two neighbours."""

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


def _dedup(found):
    """One entry per posture, keeping the most accurate of each.

       Two branches can converge on the same posture -- IKBT enumerates
       version combinations without discarding duplicates, and a bracket found
       from either side lands in the same place.  Same joints, same solution."""
    unique = []
    for s in sorted(found, key=lambda s: s['error']):
        if not any(np.max(np.abs(np.array(s['q']) - np.array(u['q']))) < DEDUP_TOL
                   for u in unique):
            unique.append(s)
    return unique


def _roots_in_**IDENT**(T, values, curve, b, cache, tol, refine_iters):
    """Bracket every dip in one branch's sampled curve and refine it.

       -> the accepted ones, as solve()'s dicts.  This is the whole
       scan-then-optimise step, and it is used both on the global grid and on
       the narrow windows _hunt_near_**IDENT**() opens around what it finds."""

    def err_at(t):
        e = _cached_errors_**IDENT**(T, _wrap(t), cache)
        return e[b] if b < len(e) else np.inf

    out = []
    for i in _local_minima(curve):
        t_star, e_star = _golden(err_at, values[i - 1], values[i + 1],
                                 refine_iters)
        if e_star > tol:
            continue                     # a dip, not a root -- see the header
        t_star = _wrap(t_star)
        qs = branches_at_**IDENT**(T, t_star)
        if b >= len(qs):
            continue
        out.append({'q': qs[b], 'known_value': float(t_star),
                    'branch': int(b), 'error': float(e_star)})
    return out


def _hunt_near_**IDENT**(T, found, h, cache, tol, refine_iters,
                    fanout=16, rounds=3):
    """Look again, finely, in a window one grid step wide around each root.

       ROOTS COME IN CLOSE PAIRS, and a pair closer together than the sample
       spacing is one dip on the grid:  the search finds one of the two and
       nothing says the other is missing.  Refining the WHOLE grid until they
       separate is the general answer and solve() does that too, but it is
       luck -- two successive resolutions can both step over the same pair.

       This is the targeted answer, and it is not luck:  a root that a grid
       hides is by definition within one grid step of a root the grid found,
       so every root that IS found gets its neighbourhood re-scanned at
       `fanout` times the resolution.  Anything new found there is itself
       re-scanned, `fanout` times finer again, for `rounds` rounds.  The cost
       goes with the number of roots, not with the size of the domain."""

    known = list(found)
    frontier = list(found)
    for _ in range(int(rounds)):
        fresh = []
        for s in frontier:
            b, t0 = int(s['branch']), float(s['known_value'])
            vals = [t0 - h + 2.0 * h * (k / float(fanout))
                    for k in range(int(fanout) + 1)]
            curve = []
            for t in vals:
                e = _cached_errors_**IDENT**(T, _wrap(t), cache)
                curve.append(e[b] if b < len(e) else np.inf)
            for r in _roots_in_**IDENT**(T, vals, curve, b, cache, tol,
                                         refine_iters):
                if not any(np.max(np.abs(np.array(r['q']) - np.array(u['q'])))
                           < DEDUP_TOL for u in known + fresh):
                    fresh.append(r)
        if not fresh:
            break
        known += fresh
        frontier = fresh
        h = h / float(fanout)
    return _dedup(known)


def _domain_edges(curve):
    """Adjacent index pairs where one sample is defined and the next is not.

       np.inf means the closed form had no answer there -- an arcsine out of
       range, a division by zero -- so a finite/infinite pair straddles the
       edge of this branch's domain."""
    out = []
    for i in range(len(curve) - 1):
        if np.isfinite(curve[i]) != np.isfinite(curve[i + 1]):
            out.append((i, i + 1))
    return out


def _hunt_edges_**IDENT**(T, values, curve, b, cache, tol, refine_iters,
                     bisect=40, depth=40):
    """Probe inward from every edge of this branch's domain, in halving steps.

       A BASIN CAN BE NARROWER THAN ANY AFFORDABLE GRID, and when it is, it is
       pressed against the edge of the branch's domain -- because that edge is
       where the closed form's denominators vanish and its arcsines leave
       range, so it is where the joint values, and the pose error built from
       them, move fastest.  C-Arm's d_1 goes as 1 / cos(th_2), and it has poses
       whose third and fourth solutions sit within 1e-3 of th_2 = +/- pi/2, in
       a spike whose sides rise at a slope of 1000.  A uniform grid would need
       about six thousand points to see one, and approaching it from inside the
       curve RISES first, so the nearest sample is a local maximum and the
       ordinary scan never even brackets it.

       So: bisect each finite/infinite transition down to the edge, then walk
       back inward in halving steps, one grid step down to nothing.  A
       geometric ladder resolves a feature at any scale for the price of its
       logarithm."""

    def err_at(t):
        e = _cached_errors_**IDENT**(T, _wrap(t), cache)
        return e[b] if b < len(e) else np.inf

    out = []
    for i, j in _domain_edges(curve):
        if np.isfinite(curve[i]):
            inside, outside = values[i], values[j]
        else:
            inside, outside = values[j], values[i]
        step = abs(outside - inside)
        if not step:
            continue

        #  the edge itself, to a part in 2**bisect of one grid step
        near, far = inside, outside
        for _ in range(int(bisect)):
            mid = 0.5 * (near + far)
            if np.isfinite(err_at(mid)):
                near = mid
            else:
                far = mid

        inward = 1.0 if inside > outside else -1.0
        probes = sorted(near + inward * step * (0.5 ** k)
                        for k in range(int(depth)))
        out += _roots_in_**IDENT**(T, probes, [err_at(t) for t in probes], b,
                              cache, tol, refine_iters)
    return out


def _solve_at_**IDENT**(T, n_samples, cache, tol, refine_iters):
    """Every root found at ONE global resolution.  -> what solve() returns.

       Three passes, because roots hide in three ways.  The GRID finds the
       ordinary ones.  The DOMAIN EDGES are probed geometrically, because a
       spike too narrow for the grid is pressed against one of them.  And then
       the neighbourhood of everything found so far is re-scanned, because a
       root the grid stepped over is within one step of a root it did not."""

    values, curves = _scan_**IDENT**(T, n_samples, cache)

    found = []
    for b, curve in enumerate(curves):
        found += _roots_in_**IDENT**(T, values, curve, b, cache, tol,
                                refine_iters)
        found += _hunt_edges_**IDENT**(T, values, curve, b, cache, tol,
                                  refine_iters)

    step = (SEARCH_HI - SEARCH_LO) / float(int(n_samples))
    return _hunt_near_**IDENT**(T, _dedup(found), step, cache, tol, refine_iters)


def _same_postures(a, b):
    """Do two solution lists describe the same set of postures?"""
    if len(a) != len(b):
        return False
    for s in a:
        if not any(np.max(np.abs(np.array(s['q']) - np.array(u['q'])))
                   < DEDUP_TOL for u in b):
            return False
    return True


def solve_**IDENT**(T, n_samples=N_SAMPLES, tol=None, refine_iters=80,
               max_samples=MAX_SAMPLES):
    """Goal pose T (4x4) -> every joint vector that reaches it.

       Each entry is a dict:

           q             NDOF floats, JOINT_NAMES order, ready for FK
           known_value   the value of KNOWN_VARIABLE it was found at
           branch        which branch of the closed form it came from
           error         ||dp|| + W_ROT*theta, what ACCEPT_TOL is measured in
           n_samples     the resolution the answer settled at

       THE RESOLUTION IS NOT ASSUMED, IT IS REACHED.  A pair of roots closer
       together than one sample spacing looks exactly like a single root until
       the grid is fine enough to separate them, and the arms this method
       exists for do have such poses -- so a single-resolution answer is a
       guess.  Two things check it.  Globally, the scan runs at n_samples,
       then at 2*n_samples, and on up, and returns as soon as two successive
       resolutions agree about the whole set of postures.  Locally, and this
       is the one that is not luck, _hunt_near_**IDENT**() re-scans the
       neighbourhood of every root that IS found at far higher resolution,
       because a root the grid hides is within one grid step of one it did
       not.

       max_samples is where the checking gives up.  The solutions returned
       there are still SOUND -- every one of them reaches T to within tol --
       but their completeness is once again unproven, and the 'n_samples'
       field reads max_samples to say so.  Raise it for a pose whose count
       looks wrong.

       [] means no solution was FOUND, which is not quite "unreachable"."""

    tol = ACCEPT_TOL if tol is None else tol
    cache = {}
    n = max(2, int(n_samples))
    prev = None
    while True:
        sols = _solve_at_**IDENT**(T, n, cache, tol, refine_iters)
        for s in sols:
            s['n_samples'] = n
        if prev is not None and _same_postures(prev, sols):
            return sols
        if 2 * n > int(max_samples):
            return sols
        prev = sols
        n *= 2


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
#    THE POSE IS RANDOM AND THE SEED IS PRINTED.  A fixed seed exercises one
#    pose forever, and what goes wrong here is pose-dependent -- a pair of
#    roots too close for the starting grid, a branch undefined over most of
#    the range.  Pass the printed seed back to get the same pose again:
#
#        python3 IK_onevar**ROBOT**.py <seed>
#
if __name__ == "__main__":

    import random
    import sys

    seed = int(sys.argv[1]) if len(sys.argv) > 1 else random.randrange(10 ** 9)
    random.seed(seed)
    q_true = [random.uniform(-1.0, 1.0) for _ in range(NDOF)]
    T = true_fk.fk_**IDENT**(q_true)

    print('%s:  searching over %s in [%.3f, %.3f]   (seed %d)'
          % (ROBOT, KNOWN_VARIABLE, SEARCH_LO, SEARCH_HI, seed))
    print('  a reachable pose, from joints:')
    print('   ', dict(zip(JOINT_NAMES, [round(v, 4) for v in q_true])))

    sols = solve_**IDENT**(T)
    print('  %d solution(s) found, at %d samples:'
          % (len(sols), sols[0]['n_samples'] if sols else N_SAMPLES))

    #  THE ERROR IS RE-MEASURED HERE, from the joint vector actually returned.
    #  The search's own number is what it accepted;  a self-test that prints
    #  that number is testing nothing.
    worst = 0.0
    for s in sols:
        e = pose_error(np.asarray(true_fk.fk_**IDENT**(s['q']), dtype=float),
                       np.asarray(T, dtype=float))[1]
        worst = max(worst, e)
        print('    %s = %8.4f   error %.3e' % (KNOWN_VARIABLE,
                                               s['known_value'], e))
        print('      ', dict(zip(JOINT_NAMES, [round(v, 4) for v in s['q']])))
    if sols:
        print('  worst round-trip error %.3e   (accept tolerance %.1e)'
              % (worst, ACCEPT_TOL))
        #  the pose was built from q_true, so q_true had better be among them
        near = min(max(abs(a - b) for a, b in zip(s['q'], q_true))
                   for s in sols)
        print('  the joints the pose was built from are %s'
              % ('among them (%.1e)' % near if near < 1e-6
                 else 'MISSING -- the closest is %.1e away' % near))
    else:
        print('    none -- try a larger max_samples')
'''


def write_onevar_top(M, name, known, dirname=DIR_NAME, n_samples=128,
                     max_samples=4096):
    '''Write IK_onevar<name>.py -- the 1-D search over the assumed variable.

       M            the TRUE arm's mechanism (nothing here is approximated)
       name         the robot the user asked about
       known        the variable the closed form assumes is known, e.g. 'th_2'
       n_samples    where the resolution ladder starts
       max_samples  where it gives up looking for more

       Returns the path written.'''

    ident = py_identifier(name)
    #  BOTH ROUNDED UP TO A POWER OF TWO.  The ladder doubles, and it only
    #  re-uses the coarse grid's cached errors when lo + span*(2k)/(2n) is the
    #  same float as lo + span*k/n -- which it is exactly when n is a power of
    #  two.  Off a power of two the answers are the same and the work is not.
    n_samples = 1 << max(1, int(n_samples) - 1).bit_length()
    max_samples = max(n_samples, 1 << max(1, int(max_samples) - 1).bit_length())
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
        print('#  Where the resolution ladder STARTS.  solve() scans at this', file=f)
        print('#  many samples, then twice as many, until two grids running agree', file=f)
        print('#  about the answer -- so this is a starting cost, not a limit on', file=f)
        print('#  what can be found.  A sample costs one closed-form evaluation.', file=f)
        print('N_SAMPLES      = %d' % int(n_samples), file=f)
        print('', file=f)
        print('#  ... and where it gives up.  Reaching this means the answer was', file=f)
        print('#  still changing:  the solutions returned are sound, but there', file=f)
        print('#  may be more of them.  Both are powers of two, which is what', file=f)
        print('#  makes a doubling re-use the coarser grid exactly.', file=f)
        print('MAX_SAMPLES    = %d' % int(max_samples), file=f)
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
        print(MAIN_BLOCK.replace('**IDENT**', ident)
                        .replace('**ROBOT**', name), file=f)

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
        self.test_ovgF_two_roots_inside_one_sample_gap()
        self.test_ovgG_a_root_at_the_end_of_a_finite_range()
        self.test_ovgH_a_spike_against_the_edge_of_a_domain()

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

    #  Two roots 0.02 apart.  The error is |(t - 0.40)(t - 0.42)|, whose hump
    #  between them tops out at 1e-4 -- far above ACCEPT_TOL, so this really is
    #  two roots and not one -- but 0.02 is under the sample spacing of any
    #  grid coarser than 512 points, so a single-resolution scan sees one dip.
    #  This is C-Arm's failure in miniature:  it has poses whose roots are 0.045
    #  apart against a spacing of 0.049.
    IK_TOY_PAIR = '''
import numpy as np


KNOWN_VARIABLE = 'th_1'


def ikin_**IDENT**_given(T, th_1):
    t = float(th_1)
    return [[t, (t - 0.40) * (t - 0.42)]]
'''

    #  A root 5e-4 inside the edge of the branch's domain, in a spike of slope
    #  1000, with a HILL between it and the rest of the domain -- so approaching
    #  from inside, the curve RISES, and the grid sample nearest the root is a
    #  local maximum that no scan will ever bracket.  This is C-Arm's third and
    #  fourth solutions at a pose where they sit against th_2 = pi/2, where
    #  d_1 = (...)/cos(th_2) blows up.  A uniform grid needs ~6000 points.
    IK_TOY_EDGE = '''
import numpy as np


KNOWN_VARIABLE = 'th_1'


def ikin_**IDENT**_given(T, th_1):
    t = float(th_1)
    if t > 0.5:
        return False                  # the branch's domain ends here
    d = 0.4995 - t                    # how far below the root we are
    if abs(d) <= 0.004:
        return [[t, 1000.0 * d]]      # the spike:  slope 1000, zero at the root
    return [[t, 4.0 * np.exp(-(abs(d) - 0.004))]]   # the hill that hides it
'''

    #  A root at t = 3.0, which on the prismatic toy's guessed range [-3, 3] is
    #  the very last grid point.  Nothing wraps on a finite range, so without
    #  the grid being extended past each end this sample has one neighbour and
    #  is never bracketed.
    IK_TOY_END = '''
import numpy as np


KNOWN_VARIABLE = 'd_1'


def ikin_**IDENT**_given(T, d_1):
    t = float(d_1)
    return [[t, t - 3.0]]
'''

    def build_toy(self, name='ToyOneVar', ik=None, mech=None, known='th_1',
                  **kw):
        """Generate a search module over the toy siblings.  -> (dir, module)."""
        d = tempfile.mkdtemp(prefix='ikbt_onevar_')
        ident = py_identifier(name)
        for tmpl, fn in (((self.FK_TOY, 'FK_numeric%s.py' % name),
                          (ik or self.IK_TOY, 'IK_conditional%s.py' % name))):
            with open(os.path.join(d, fn), 'w') as f:
                f.write(tmpl.replace('**IDENT**', ident))

        path = write_onevar_top(mech if mech is not None else self.two_link(),
                                name, known, dirname=d, **kw)
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


    def test_ovgF_two_roots_inside_one_sample_gap(self):
        '''Two roots 0.02 apart, on a grid whose spacing is 0.049.

           THE GRID SEES ONE DIP WHERE THERE ARE TWO ROOTS, and that is what
           C-Arm was doing: returning one posture of a pair and saying nothing
           about the other.  Refining the whole grid would eventually separate
           them, but two successive resolutions can both step over the same
           pair, so agreement is not proof.  What makes this sound is the
           window opened around every root that IS found -- a hidden twin is
           within one grid step of its sibling by definition.'''
        fs = ' onevar resolution FAIL'

        d, mod = self.build_toy(ik=self.IK_TOY_PAIR)
        try:
            #  the fixture really is one dip on the starting grid
            values, curves = mod.sweep_ToyOneVar(np.eye(4))
            dips = [i for i in mod._local_minima(curves[0])
                    if curves[0][i] < 1e-3]
            self.assertEqual(len(dips), 1,
                             fs + ' (fixture: 128 samples should show one '
                             'dip, showed %d)' % len(dips))
            found = mod.solve_ToyOneVar(np.eye(4))
        finally:
            shutil.rmtree(d, ignore_errors=True)

        got = sorted(s['known_value'] for s in found)
        self.assertEqual(len(got), 2,
                         fs + ' (found %s, expected both roots)'
                         % [round(t, 5) for t in got])
        for g, w in zip(got, (0.40, 0.42)):
            self.assertAlmostEqual(g, w, places=7,
                                   msg=fs + ' (root off by %.2e)' % abs(g - w))
        for s in found:
            self.assertLess(s['error'], 1e-9,
                            fs + ' (accepted an error of %.2e)' % s['error'])
        self.assertTrue(all('n_samples' in s for s in found),
                        fs + ' (the answer must say what resolution it '
                        'settled at)')

    def test_ovgG_a_root_at_the_end_of_a_finite_range(self):
        '''A root at t = 3.0, the last sample of the prismatic range [-3, 3].

           A periodic domain wraps and a root at either end still has two
           neighbours.  A finite one has nothing to wrap to, so the grid is
           extended one step past each end instead;  without that the last
           sample can never be the middle of a bracket and the root is gone.'''
        fs = ' onevar finite-range end FAIL'

        d, mod = self.build_toy(ik=self.IK_TOY_END,
                                mech=self.two_link(prismatic=True),
                                known='d_1')
        try:
            self.assertFalse(mod.PERIODIC, fs + ' (fixture: should be finite)')
            self.assertAlmostEqual(mod.SEARCH_HI, 3.0, msg=fs + ' (fixture)')
            found = self.toy_solutions(mod)
        finally:
            shutil.rmtree(d, ignore_errors=True)

        self.assertEqual(len(found), 1,
                         fs + ' (found %s, expected the root at 3.0)'
                         % [round(t, 5) for t, e in found])
        self.assertAlmostEqual(found[0][0], 3.0, places=7, msg=fs)


    def test_ovgH_a_spike_against_the_edge_of_a_domain(self):
        '''A root in a spike 5e-4 wide, pressed against the domain edge.

           NOT FINDABLE BY REFINING THE GRID at any affordable resolution, and
           not findable by looking near the roots that were found, because this
           one is nowhere near them.  It is findable because of WHERE it is:
           against the edge of the branch's domain, which is where the closed
           form's denominators vanish and everything moves fastest.  Probing
           geometrically inward from that edge costs 40 samples and finds a
           feature of any width.'''
        fs = ' onevar domain-edge FAIL'

        d, mod = self.build_toy(ik=self.IK_TOY_EDGE)
        try:
            #  the fixture really is invisible to the grid:  the sample nearest
            #  the root is a local MAXIMUM, so the scan brackets nothing there
            values, curves = mod.sweep_ToyOneVar(np.eye(4))
            near = [i for i in mod._local_minima(curves[0])
                    if abs(values[i] - 0.4995) < 0.05]
            self.assertEqual(near, [],
                             fs + ' (fixture: the grid should see no dip near '
                             'the root, saw %s)'
                             % [round(values[i], 4) for i in near])
            self.assertTrue(mod._domain_edges(curves[0]),
                            fs + ' (fixture: the branch should have an edge)')
            found = self.toy_solutions(mod)
        finally:
            shutil.rmtree(d, ignore_errors=True)

        self.assertEqual(len(found), 1,
                         fs + ' (found %s, expected the root at 0.4995)'
                         % [round(t, 6) for t, e in found])
        self.assertAlmostEqual(found[0][0], 0.4995, places=9, msg=fs)
        self.assertLess(found[0][1], 1e-9,
                        fs + ' (accepted an error of %.2e)' % found[0][1])


def run_test():
    suite = unittest.TestLoader().loadTestsFromTestCase(TestSolver029)
    unittest.TextTestRunner(verbosity=2).run(suite)


##   Self Tests
if __name__ == "__main__":
    run_test()
