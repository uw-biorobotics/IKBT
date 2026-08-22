#!/usr/bin/python
#
#   dh_analysis.py --  pure DH-table geometry:  which joint-axis triples satisfy
#                      Pieper's condition, and what it would cost to make one
#
#   No forward kinematics, no symbolic solving, no pickles -- so this runs in
#   milliseconds and unit-tests standing on its own.  It supports
#   futurework.md item 1 (hybrid_impl_plan.md phases C and D):  when IKBT cannot
#   solve a robot in closed form, find the single DH parameter whose value is
#   costing us the closed form, and measure what changing it would cost.
#
#       pieper_triples(dh, pvals, ndof)              which triples are satisfied
#       candidate_simplifications(dh, pvals, vv, ndof)  what would satisfy one
#       rank_candidates(...)                         ordered by task-space cost
#
#   ---------------------------------------------------------------------------
#   THE CONVENTION, and three ways to get it wrong
#
#   kin_cl.py:280 fixes the column order:  "alpha N-1, a N-1, d N, theta N", and
#   Link_S/Link_N are Craig's  Rot_x(al_{n-1}) Trans_x(a_{n-1}) Rot_z(th_n)
#   Trans_z(d_n).  So 0-indexed row r holds [al_r, a_r, d_{r+1}, th_{r+1}].
#
#   Joint n's axis is the Z line of frame {n}:  th_n enters as Rot_z(th_n) and
#   Trans_z(d_n) only slides along that same line.  In frame {n},
#
#       axis n    passes through the origin along z
#       axis n+1  passes through [a_n, -sa_n*d_n1, ca_n*d_n1]
#                 along          [0,   -sa_n,      ca_n     ]
#
#   so axes n,n+1 meet iff a_n == 0 (at the origin of {n}), axes n+1,n+2 meet
#   iff a_n1 == 0 (at the origin of {n+1}), and all three share a point iff
#   those two meeting points coincide -- d_n1 == 0.
#
#   The three traps, all of which fail SILENTLY (see
#   scripts/axis_triple_check.py, which checks every rule here against numeric
#   FK geometry over all 32 robots):
#
#     1. `== 0` IS WRONG.  sp.Float(0.0) == 0 is False in sympy, and the DH
#        tables mix Integer 0 with Float 0.0.  Use .is_zero -- which also
#        answers None for an undecidable symbol, the third value we want.
#     2. THE COLLINEAR CASE.  If two of the three axes are the SAME line -- which
#        happens iff a == 0 and sin(alpha) == 0 between them -- there are only two
#        distinct lines, and they are concurrent for ANY d_n1, including when
#        d_n1 is a prismatic joint VARIABLE.  That is the Stanford arm (prismatic
#        sliding along the axis the next joint rotates about), and also Bartell,
#        Palm13, Srisuan11 and Raven-II.
#     3. j IS A 1-BASED JOINT NUMBER used as a 0-based ROW index, running
#        1 .. ndof-2.  Looping from j = 0 names a nonexistent joint 0 and reads
#        a_0 / d_1, which are zero on most arms -- manufacturing a spurious hit.
#   ---------------------------------------------------------------------------
#
#   Copyright 2026 University of Washington
#
#   Developed by Blake Hannaford
#   BioRobotics Lab, University of Washington

import numpy as np
import sympy as sp

from ikbtbasics.pykinsym import Link_N


#  Craig column indices (kin_cl.py:280 uses the same four)
AL, A, D, TH = 0, 1, 2, 3

#  Stand-ins for a symbol that has no pvals entry, per BH:  "Let's set missing
#  pvals to a sensible value for now.  d_x = 1.0, angles = pi/2".  Measured over
#  all 32 robots NO triple-relevant cell actually needs these -- every
#  undecidable cell is a prismatic joint variable, not a missing value -- so
#  this is a safety net, and use of it is reported rather than hidden.
DEFAULT_LENGTH = 1.0
DEFAULT_ANGLE = np.pi/2


###############################################################################
#
#    Zero tests
#

def numeric_pvals(pvals):
    '''Only the numeric entries of pvals.

       forward_kinematics() writes the STRINGS 'np.cos(...)'/'np.sin(...)' into
       pvals for robots whose alpha is not a multiple of pi/2 (kin_cl.py:296;
       Raven-II, ICP5p5_A21).  Substituting those injects strings into sympy.'''

    return {k: v for k, v in (pvals or {}).items() if not isinstance(v, str)}


def cell(dh, pvals, r, c):
    '''One DH cell with pvals resolved, simplified.'''
    return sp.simplify(sp.sympify(dh[r, c]).subs(numeric_pvals(pvals)))


def is_zero(e):
    '''True / False / None(undecidable).

       NEVER use `e == 0` for this:  sp.Float(0.0) == 0 is False.'''
    return sp.simplify(sp.sympify(e)).is_zero


def _z(e):
    '''is_zero as a hard boolean -- undecidable counts as "not zero", which is
       the safe reading for a joint variable (it is not identically zero).'''
    return bool(is_zero(e))


###############################################################################
#
#    Which triples satisfy Pieper's condition
#

def joint_triples(ndof):
    '''Valid j for the triple of axes (j, j+1, j+2).

       j is a 1-based JOINT number used as a 0-based ROW index -- see trap 3 in
       the header.  Restricting the range is also what keeps the zero-padded
       rows that sub-6-DOF robots must carry from manufacturing triples.'''

    return list(range(1, max(1, ndof - 1)))


def triple_report(dh, pvals, j):
    '''Every condition value for the triple (j, j+1, j+2).  Pure data, so a
       caller (or a test) can see WHY a triple did or did not qualify.'''

    a_j, a_j1 = cell(dh, pvals, j, A),  cell(dh, pvals, j+1, A)
    d_j1      = cell(dh, pvals, j, D)
    al_j, al_j1 = cell(dh, pvals, j, AL), cell(dh, pvals, j+1, AL)
    sa_j  = sp.simplify(sp.sin(al_j))
    sa_j1 = sp.simplify(sp.sin(al_j1))

    #  two axes are collinear iff a == 0 and sin(alpha) == 0.  alpha = pi still
    #  gives the same LINE -- only the direction flips -- so sin, not alpha.
    collin_j_j1  = _z(a_j)  and _z(sa_j)
    collin_j1_j2 = _z(a_j1) and _z(sa_j1)

    intersect = (_z(a_j) and _z(a_j1)
                 and (_z(d_j1) or collin_j1_j2 or collin_j_j1))
    parallel = _z(sa_j) and _z(sa_j1)

    terms = {'a_%d' % j: a_j, 'a_%d' % (j+1): a_j1, 'd_%d' % (j+1): d_j1,
             'al_%d' % j: al_j, 'al_%d' % (j+1): al_j1}
    undecidable = sorted(k for k, v in terms.items() if is_zero(v) is None)

    return {'axes': (j, j+1, j+2),
            'intersect': bool(intersect),
            'parallel': bool(parallel),
            'collinear_j_j1': bool(collin_j_j1),
            'collinear_j1_j2': bool(collin_j1_j2),
            'terms': terms,
            'undecidable': undecidable}


def pieper_triples(dh, pvals, ndof):
    '''Satisfied triples, by kind:  [{'axes': (j,j+1,j+2), 'kind': ...}, ...].

       'intersect' and 'parallel' are reported separately, and one triple can
       be both.  An empty list means Pieper's SUFFICIENT condition is not met
       anywhere -- which does NOT mean the arm has no closed form.  Measured:
       9 of the 32 robots have no triple and still solve completely.'''

    out = []
    for j in joint_triples(ndof):
        r = triple_report(dh, pvals, j)
        if r['intersect']:
            out.append({'axes': r['axes'], 'kind': 'intersect', 'report': r})
        if r['parallel']:
            out.append({'axes': r['axes'], 'kind': 'parallel', 'report': r})
    return out


def has_pieper_triple(dh, pvals, ndof):
    '''True iff at least one triple qualifies.  This is the gate the hybrid
       branch is the INVERSE of:  an arm with a triple has a closed form, so if
       IKBT failed on it that is a solver defect, not a geometry problem.'''

    return len(pieper_triples(dh, pvals, ndof)) > 0


###############################################################################
#
#    Fast numeric FK  (for the displacement metric)
#

def joint_cell(dh, vv, r):
    '''(row, column) holding row r's joint variable, and its kind.

       vv[r] == 1 -> rotary, the variable is th_{r+1} in column TH
       vv[r] == 0 -> prismatic, the variable is d_{r+1} in column D'''

    rotary = (vv[r] == 1)
    return (r, TH if rotary else D), ('rot' if rotary else 'pris')


def length_scale(dh, pvals, vv, ndof):
    '''A characteristic length for the arm:  the median magnitude of its
       non-zero constant a and d cells (1.0 if it has none).

       Used for two things that need a length and have no other source, both
       documented where they are used:  the prismatic sampling range, and the
       default weight that converts a rotation error into a length.  M.jlims
       exists but is a hardcoded +-pi placeholder that nothing sets per robot,
       so it is no help.'''

    mags = []
    for r in range(ndof):
        (jr, jc), _ = joint_cell(dh, vv, r)
        for c in (A, D):
            if (r, c) == (jr, jc):
                continue                      # a joint variable, not a length
            e = cell(dh, pvals, r, c)
            if e.free_symbols:
                continue                      # undecidable
            v = abs(float(e))
            if v > 0:
                mags.append(v)
    return float(np.median(mags)) if mags else 1.0


def compile_rows(dh, pvals, vv, ndof):
    '''Pre-resolve every constant cell to a float, ONCE.

       Returns (rows, slots, kinds, defaulted):  `rows` is an ndof x 4 float
       array with 0.0 parked in each joint-variable slot, `slots` gives the
       (r,c) of each row's joint variable, `kinds` its type, and `defaulted`
       names any cell that had to fall back on DEFAULT_LENGTH/DEFAULT_ANGLE.

       Doing this once is what makes the metric affordable:  the sampling loop
       is then pure numpy, with no sympy subs per sample.'''

    num = numeric_pvals(pvals)
    rows = np.zeros((ndof, 4))
    slots, kinds, defaulted = [], [], []

    for r in range(ndof):
        (jr, jc), kind = joint_cell(dh, vv, r)
        slots.append((jr, jc))
        kinds.append(kind)
        for c in range(4):
            if (r, c) == (jr, jc):
                continue                      # filled in per sample
            e = sp.sympify(dh[r, c]).subs(num)
            for s in sorted(e.free_symbols, key=str):
                defaulted.append('%s (row %d col %d)' % (s, r, c))
                e = e.subs(s, DEFAULT_ANGLE if c == AL else DEFAULT_LENGTH)
            rows[r, c] = float(e)
    return rows, slots, kinds, defaulted


def fk_numeric(rows, ndof):
    '''T_0N from pre-resolved numeric rows.  Link_N is pure numpy.'''
    T = np.matrix(np.eye(4))
    for r in range(ndof):
        T = T * Link_N(rows[r, AL], rows[r, A], rows[r, D], rows[r, TH])
    return T


def sample_joints(kinds, rng, L):
    '''One random joint vector.

       Sampling ranges are stated here rather than read from the robot, because
       there is nowhere to read them from:  +-pi for a revolute joint, and
       +-length_scale for a prismatic one.  Both are declared, not discovered.'''

    return [rng.uniform(-np.pi, np.pi) if k == 'rot' else rng.uniform(-L, L)
            for k in kinds]


###############################################################################
#
#    What a simplification would cost, in task space
#

def displacement_metric(dh, dh_simp, pvals, vv, ndof, n=200, seed=0,
                        w_rot=None):
    '''How far the simplified arm's end effector is from the true one.

       Sampled over joint space with the SAME joint values fed to both arms.
       Returns mean and max of

           dp        ||p_true - p_simp||                  (length units)
           dth       angle of R_true^T R_simp              (radians)
           combined  dp + w_rot*dth                        (length units)

       `combined` is the ranking key, and it exists because the whole point of
       ranking in task space is to put zeroing a LENGTH and snapping an ANGLE in
       comparable units.  It is BH's angle/axis scalar from hybrid_impl_plan.md
       question 3 -- position error plus the rotation angle scaled by a length.

       w_rot defaults to length_scale(), i.e. one radian of orientation error
       counts as much as one characteristic link length.  BH's stated convention
       is 1 metre per radian;  the DH tables carry no units (KinovaLite is in
       mm), so pass w_rot=1000.0 explicitly for mm to get exactly that.'''

    L = length_scale(dh, pvals, vv, ndof)
    if w_rot is None:
        w_rot = L

    rows_a, slots_a, kinds, defaulted_a = compile_rows(dh, pvals, vv, ndof)
    rows_b, slots_b, _, defaulted_b = compile_rows(dh_simp, pvals, vv, ndof)

    rng = np.random.default_rng(seed)
    dps, dths = [], []
    for _ in range(n):
        q = sample_joints(kinds, rng, L)
        for i, (r, c) in enumerate(slots_a):
            rows_a[r, c] = q[i]
        for i, (r, c) in enumerate(slots_b):
            rows_b[r, c] = q[i]

        Ta = fk_numeric(rows_a, ndof)
        Tb = fk_numeric(rows_b, ndof)

        dps.append(float(np.linalg.norm(np.array(Ta[:3, 3] - Tb[:3, 3]))))
        Rrel = np.array(Ta[:3, :3]).T @ np.array(Tb[:3, :3])
        #  Rotation angle via atan2, NOT arccos((tr-1)/2).  arccos has infinite
        #  derivative at R = I, so an O(1e-16) rounding error in the trace comes
        #  out as O(1e-8) radians -- which is exactly the regime that matters
        #  here, because a candidate that changes only a LENGTH must measure a
        #  clean zero orientation error.  atan2 of the skew norm against the
        #  trace is linear in the perturbation, and gives exactly 0.0 for R = I.
        skew = np.array([Rrel[2, 1] - Rrel[1, 2],
                         Rrel[0, 2] - Rrel[2, 0],
                         Rrel[1, 0] - Rrel[0, 1]])
        dths.append(float(np.arctan2(0.5*np.linalg.norm(skew),
                                     (np.trace(Rrel) - 1.0)/2.0)))

    dps, dths = np.array(dps), np.array(dths)
    comb = dps + w_rot*dths
    return {'mean_dp': float(dps.mean()),   'max_dp': float(dps.max()),
            'mean_dth': float(dths.mean()), 'max_dth': float(dths.max()),
            'mean_combined': float(comb.mean()), 'max_combined': float(comb.max()),
            'w_rot': float(w_rot), 'n': n, 'seed': seed,
            'defaulted': sorted(set(defaulted_a) | set(defaulted_b))}


###############################################################################
#
#    Candidate simplifications
#

def nearest_pi_multiples(x):
    '''The multiples of pi worth snapping x to, nearest first.

       BOTH neighbours are returned, not just the nearest, because sin(alpha)==0
       is satisfied by 0 and by pi alike -- but alpha = pi FLIPS the axis, so the
       two choices give very different arms.  Which one is cheaper in task space
       is not decidable from the angle alone, so both become candidates and the
       ranking settles it.  (alpha = pi/2 is exactly equidistant, and that is the
       single most common value in these tables.)'''

    k = x/np.pi
    lo, hi = int(np.floor(k)), int(np.ceil(k))
    opts = sorted({lo*np.pi, hi*np.pi}, key=lambda t: (abs(t - x), t))
    return opts


def apply_edits(dh, edits):
    '''A copy of dh with each edit's cell replaced.'''
    out = sp.Matrix(dh)
    for e in edits:
        r, c = e['cell']
        out[r, c] = sp.sympify(e['to'])
    return out


def _edit(dh, pvals, r, c, to, kind, name):
    frm = cell(dh, pvals, r, c)
    try:
        delta = abs(float(frm) - float(sp.sympify(to)))
    except TypeError:
        delta = None                      # undecidable -- reported, not guessed
    return {'cell': (r, c), 'symbol': name, 'from': frm, 'to': sp.sympify(to),
            'kind': kind, 'delta': delta}


def candidate_simplifications(dh, pvals, vv, ndof):
    '''Every single-triple change that would satisfy Pieper's condition.

       One candidate per (triple, route).  Enumeration is exhaustive and tiny --
       at most 4 triples x (3 intersect routes + 1 parallel route) x 2 choices
       per snapped angle -- so this is arithmetic, not an optimisation problem.

       Routes to make axes (j, j+1, j+2) CONCURRENT:

         'zero_offsets'  zero whichever of a_j, a_j1, d_j1 are non-zero.
                         UNAVAILABLE when d_j1 is a prismatic joint variable --
                         you cannot zero a joint variable, and pretending
                         otherwise is how the collinear case gets missed.
         'collinear_hi'  zero a_j, a_j1 and snap al_j1 to a multiple of pi, so
                         axes j+1 and j+2 become one line.  Needs no d_j1 = 0,
                         which is what makes it the route for a prismatic arm.
         'collinear_lo'  same, snapping al_j so axes j and j+1 become one line.

       Route to make them PARALLEL:

         'parallel'      snap al_j and al_j1 each to a multiple of pi.

       Already-satisfied triples are skipped -- there is nothing to simplify.'''

    cands = []
    for j in joint_triples(ndof):
        rep = triple_report(dh, pvals, j)
        if rep['intersect'] or rep['parallel']:
            continue                          # nothing to buy here

        a_j, a_j1 = rep['terms']['a_%d' % j], rep['terms']['a_%d' % (j+1)]
        d_j1 = rep['terms']['d_%d' % (j+1)]
        al_j, al_j1 = rep['terms']['al_%d' % j], rep['terms']['al_%d' % (j+1)]

        #  is d_j1 a joint variable?  (row j's variable is in column D exactly
        #  when joint j+1 is prismatic)
        (jr, jc), _ = joint_cell(dh, vv, j)
        d_is_joint = ((jr, jc) == (j, D))

        zeros = []
        if not _z(a_j):
            zeros.append(_edit(dh, pvals, j,   A, 0, 'zero', 'a_%d' % j))
        if not _z(a_j1):
            zeros.append(_edit(dh, pvals, j+1, A, 0, 'zero', 'a_%d' % (j+1)))

        def add(kind, route, edits, blocked=None):
            cands.append({'axes': (j, j+1, j+2), 'kind': kind, 'route': route,
                          'edits': edits, 'n_edits': len(edits),
                          'magnitude': None if any(e['delta'] is None for e in edits)
                                       else float(sum(e['delta'] for e in edits)),
                          'undecidable': rep['undecidable'],
                          'blocked': blocked,
                          'dh_simp': apply_edits(dh, edits)})

        #  --- intersect, by zeroing the offsets
        if d_is_joint:
            #  recorded rather than dropped:  "this arm cannot be fixed this way"
            #  is a finding, and silently omitting the route looks like a bug.
            cands.append({'axes': (j, j+1, j+2), 'kind': 'intersect',
                          'route': 'zero_offsets', 'edits': [], 'n_edits': 0,
                          'magnitude': None, 'undecidable': rep['undecidable'],
                          'blocked': 'd_%d is a prismatic joint variable' % (j+1),
                          'dh_simp': None})
        else:
            e = list(zeros)
            if not _z(d_j1):
                e.append(_edit(dh, pvals, j, D, 0, 'zero', 'd_%d' % (j+1)))
            if e:
                add('intersect', 'zero_offsets', e)

        #  --- intersect, by making one pair collinear
        for route, (r_al, al, nm) in (('collinear_hi', (j+1, al_j1, 'al_%d' % (j+1))),
                                      ('collinear_lo', (j,   al_j,  'al_%d' % j))):
            if _z(sp.sin(al)):
                continue                      # already collinear on that side
            try:
                x = float(al)
            except TypeError:
                continue                      # undecidable angle: no snap target
            for target in nearest_pi_multiples(x):
                add('intersect', route,
                    list(zeros) + [_edit(dh, pvals, r_al, AL, target, 'snap', nm)])

        #  --- parallel, by snapping both alphas
        snaps = []
        ok = True
        for r_al, al, nm in ((j, al_j, 'al_%d' % j), (j+1, al_j1, 'al_%d' % (j+1))):
            if _z(sp.sin(al)):
                continue
            try:
                x = float(al)
            except TypeError:
                ok = False
                break
            snaps.append((r_al, nm, nearest_pi_multiples(x)))
        if ok and snaps:
            #  cross product of the per-angle choices (at most 2x2)
            combos = [[]]
            for r_al, nm, opts in snaps:
                combos = [c + [(r_al, nm, t)] for c in combos for t in opts]
            for combo in combos:
                add('parallel', 'parallel',
                    [_edit(dh, pvals, r_al, AL, t, 'snap', nm)
                     for r_al, nm, t in combo])

    return cands


def rank_candidates(dh, pvals, vv, ndof, n=200, seed=0, w_rot=None):
    '''candidate_simplifications(), each scored by task-space displacement and
       sorted cheapest first.

       Task-space displacement is the ranking key for the reason
       hybrid_plan.md establishes:  it is the only thing that puts zeroing a
       length and snapping an angle in the same units, and it is cheap enough to
       run inside a BT leaf.  Blocked candidates (nothing to change, or a
       prismatic joint variable in the way) are kept but sort last.'''

    scored = []
    for c in candidate_simplifications(dh, pvals, vv, ndof):
        if c['dh_simp'] is None or not c['edits']:
            c['metric'] = None
            c['cost'] = float('inf')
        else:
            m = displacement_metric(dh, c['dh_simp'], pvals, vv, ndof,
                                    n=n, seed=seed, w_rot=w_rot)
            c['metric'] = m
            c['cost'] = m['mean_combined']
        scored.append(c)

    scored.sort(key=lambda c: (c['cost'], c['n_edits'], str(c['axes']), c['route']))
    return scored


def describe_edits(cand):
    '''"d_5: 57 -> 0" -- one short human-readable line for a candidate.'''
    if cand.get('blocked'):
        return '(blocked: %s)' % cand['blocked']
    return ', '.join('%s: %s -> %s' % (e['symbol'], e['from'], e['to'])
                     for e in cand['edits']) or '(no change needed)'


#####################################################################
#
#   Test code
#

import unittest


def _robot(name):
    '''robot_params() without its console noise.

       Imported lazily and only for the tests:  a BT leaf will import this
       module, and production code here must not depend on ikbtfunctions.'''

    import contextlib
    import io
    from ikbtfunctions.ik_robots import robot_params
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        dh, vv, params, pvals, unks = robot_params(name)
    return dh, vv, pvals, len(unks)


def _kinds(dh, pvals, ndof):
    '''{'int(4,5,6)', 'par(2,3,4)', ...} for one robot.'''
    return set('%s(%d,%d,%d)' % (('int' if t['kind'] == 'intersect' else 'par'),
                                 t['axes'][0], t['axes'][1], t['axes'][2])
               for t in pieper_triples(dh, pvals, ndof))


def _table(al, a, d, th=None):
    '''A 6-row DH table from four column values, all rotary.'''
    th = th or ['th_%d' % (i+1) for i in range(6)]
    return sp.Matrix([[sp.sympify(al), sp.sympify(a), sp.sympify(d),
                       sp.sympify(th[r])] for r in range(6)])


class TestSolver019(unittest.TestCase):
    '''dh_analysis: joint-axis triples, and what simplifying one would cost.

       Every expected value here was cross-checked against an independent
       geometric oracle -- axes built from the numeric FK, concurrency tested
       numerically -- over all 32 robots.  See scripts/axis_triple_check.py.'''

    def setUp(self):
        print('\n\n===============  Test dh_analysis  =====================')
        return

    def runTest(self):
        self.test_dhA_zero_test_handles_float_zero()
        self.test_dhB_triple_range_excludes_joint_zero()
        self.test_dhC_spherical_wrists()
        self.test_dhD_parallel_triples()
        self.test_dhE_collinear_axes()
        self.test_dhF_no_triple_robots()
        self.test_dhG_synthetic_tables()
        self.test_dhH_metric_is_zero_for_no_change()
        self.test_dhI_kinovalite_top_candidate()
        self.test_dhJ_compensation_adds_in_quadrature()
        self.test_dhK_prismatic_route_is_blocked_not_dropped()

    #  ----------------------------------------------------  the zero test

    def test_dhA_zero_test_handles_float_zero(self):
        '''is_zero must see Float 0.0 as zero, and an unknown symbol as
           undecidable.

           This is the highest-severity trap in the module:  sp.Float(0.0) == 0
           is FALSE in sympy, and the DH tables mix Integer 0 with Float 0.0
           (Stanford, Bartell, Palm13 ...).  With `== 0` the detector reports NO
           triples for those robots -- silently, looking like bad geometry
           rather than a broken test.'''
        fs = ' dh_analysis is_zero FAIL'
        self.assertTrue(is_zero(sp.Float(0.0)),  fs + ' (Float 0.0)')
        self.assertTrue(is_zero(sp.Integer(0)),  fs + ' (Integer 0)')
        self.assertTrue(is_zero(sp.sympify('0.0')), fs + " ('0.0')")
        self.assertFalse(is_zero(sp.Float(57.0)), fs + ' (57.0 is not zero)')
        self.assertIsNone(is_zero(sp.Symbol('d_2')),
                          fs + ' (a bare symbol must be UNDECIDABLE, not False)')
        #  and the trap itself, so nobody "simplifies" is_zero back to == 0
        self.assertFalse(sp.Float(0.0) == 0,
                         fs + ' (sympy changed: Float(0.0) == 0 is now True, '
                         'the comment in this module needs updating)')

    #  ----------------------------------------------------  the j range

    def test_dhB_triple_range_excludes_joint_zero(self):
        '''j is a 1-based JOINT number used as a 0-based ROW index.

           j = 0 would name a nonexistent joint 0 and read a_0 / d_1, which are
           zero on most arms -- manufacturing a spurious triple almost
           everywhere.  Brad (3 DOF) is the regression case: it reported five
           triples before the range was restricted.'''
        fs = ' dh_analysis joint_triples FAIL'
        self.assertEqual(joint_triples(6), [1, 2, 3, 4], fs + ' (6 dof)')
        self.assertEqual(joint_triples(5), [1, 2, 3], fs + ' (5 dof)')
        self.assertEqual(joint_triples(3), [1], fs + ' (3 dof)')
        for n in range(1, 10):
            self.assertNotIn(0, joint_triples(n), fs + ' (joint 0 does not exist)')

        dh, vv, pvals, ndof = _robot('Brad')
        self.assertEqual(_kinds(dh, pvals, ndof), set(),
                         fs + ' (Brad, 3 DOF, must report NO triples)')

    #  ----------------------------------------------------  known geometry

    def test_dhC_spherical_wrists(self):
        '''The classic spherical wrist: axes 4,5,6 intersecting.'''
        fs = ' dh_analysis spherical wrist FAIL'
        for name in ('Puma', 'Pumaoffset', 'KawasakiRS007L', 'KR16', 'Khat6DOF'):
            dh, vv, pvals, ndof = _robot(name)
            self.assertIn('int(4,5,6)', _kinds(dh, pvals, ndof),
                          fs + ' (%s has a spherical wrist)' % name)

    def test_dhD_parallel_triples(self):
        '''The parallel branch of the rule -- otherwise untested by the
           intersecting arms above.'''
        fs = ' dh_analysis parallel FAIL'
        for name, want in (('UR5', 'par(2,3,4)'), ('Arm_3', 'par(1,2,3)'),
                           ('Parkman13', 'par(2,3,4)'),
                           ('JennyGuoSp24', 'par(3,4,5)')):
            dh, vv, pvals, ndof = _robot(name)
            self.assertIn(want, _kinds(dh, pvals, ndof),
                          fs + ' (%s should report %s)' % (name, want))

    def test_dhE_collinear_axes(self):
        '''Stanford: the collinear-axes regression test.

           Its prismatic joint 3 slides along the very axis joint 4 rotates
           about, so axes 3 and 4 are ONE line and the triples are concurrent
           for any value of the prismatic variable.  The original rule reported
           none of these three, failing on two counts at once -- float zeros and
           the missing collinear clause.'''
        fs = ' dh_analysis collinear FAIL'
        dh, vv, pvals, ndof = _robot('Stanford')
        got = _kinds(dh, pvals, ndof)
        for want in ('int(2,3,4)', 'int(3,4,5)', 'int(4,5,6)'):
            self.assertIn(want, got, fs + ' (Stanford %s)' % want)

    def test_dhF_no_triple_robots(self):
        '''Arms with no triple of any kind.

           Note three of these SOLVE completely (Sims11, Olson13, Wachtveitl):
           Pieper's condition is sufficient, not necessary, so "no triple" must
           never be asserted to mean "unsolvable".'''
        fs = ' dh_analysis no-triple FAIL'
        for name in ('KinovaLite', 'Issue4', 'KawasakiRS05L', 'Sims11',
                     'Olson13', 'Wachtveitl', 'DZhang', 'Mackler13',
                     'Axtman13', 'ICP5p5_A21'):
            dh, vv, pvals, ndof = _robot(name)
            self.assertEqual(_kinds(dh, pvals, ndof), set(),
                             fs + ' (%s should report no triples)' % name)

    def test_dhG_synthetic_tables(self):
        '''Hand-built extremes, independent of any robot definition.'''
        fs = ' dh_analysis synthetic FAIL'

        #  every alpha zero -> parallel at every legal j;  a != 0 -> no intersect
        par = _table(0, 3, 5)
        k = _kinds(par, {}, 6)
        self.assertEqual(k, {'par(%d,%d,%d)' % (j, j+1, j+2)
                             for j in joint_triples(6)},
                         fs + ' (all-parallel table)')

        #  alpha = pi/2 everywhere and a != 0 -> nothing qualifies
        none = _table(sp.pi/2, 3, 5)
        self.assertEqual(_kinds(none, {}, 6), set(), fs + ' (no-triple table)')

        #  alpha = pi is still PARALLEL (sin(pi) == 0): the axis flips, the line
        #  direction is what matters, not the sign
        flip = _table(sp.pi, 3, 5)
        self.assertTrue(all(t['kind'] == 'parallel'
                            for t in pieper_triples(flip, {}, 6)),
                        fs + ' (alpha = pi must count as parallel)')

    #  ----------------------------------------------------  the metric

    def test_dhH_metric_is_zero_for_no_change(self):
        '''An unmodified table must measure exactly zero displacement.'''
        fs = ' dh_analysis metric FAIL'
        dh, vv, pvals, ndof = _robot('Puma')
        m = displacement_metric(dh, sp.Matrix(dh), pvals, vv, ndof, n=25, seed=3)
        self.assertAlmostEqual(m['max_dp'], 0.0, places=9, msg=fs + ' (dp)')
        self.assertAlmostEqual(m['max_dth'], 0.0, places=9, msg=fs + ' (dth)')

    def test_dhI_kinovalite_top_candidate(self):
        '''KinovaLite: the motivating robot.

           Zeroing d_5 = 57 removes a pure translation in a rotated frame, so
           the position error is EXACTLY 57 at every configuration and the
           orientation is untouched.  hybrid_plan.md measured the same thing,
           and it must rank first -- the next candidate costs 245.'''
        fs = ' dh_analysis KinovaLite FAIL'
        dh, vv, pvals, ndof = _robot('KinovaLite')
        ranked = rank_candidates(dh, pvals, vv, ndof, n=40, seed=1)
        self.assertTrue(ranked, fs + ' (no candidates at all)')

        top = ranked[0]
        self.assertEqual(top['axes'], (4, 5, 6), fs + ' (wrong triple)')
        self.assertEqual(top['kind'], 'intersect', fs + ' (wrong kind)')
        self.assertEqual(len(top['edits']), 1, fs + ' (should be a single edit)')
        self.assertEqual(top['edits'][0]['symbol'], 'd_5', fs + ' (wrong cell)')
        self.assertTrue(is_zero(top['edits'][0]['to']), fs + ' (should zero it)')

        m = top['metric']
        self.assertAlmostEqual(m['mean_dp'], 57.0, places=6,
                               msg=fs + ' (mean ||dp|| must be exactly 57)')
        self.assertAlmostEqual(m['max_dp'], 57.0, places=6, msg=fs + ' (max)')
        self.assertAlmostEqual(m['mean_dth'], 0.0, places=9,
                               msg=fs + ' (orientation must be untouched)')
        self.assertLess(top['cost'], ranked[1]['cost'],
                        fs + ' (must rank strictly first)')

    def test_dhJ_compensation_adds_in_quadrature(self):
        '''Trying to COMPENSATE for a zeroed offset makes things worse.

           Rolling KinovaLite's 57 into d_6 gives 57*sqrt(2) = 80.61, not 0,
           because the two offsets are orthogonal and add in quadrature.
           hybrid_plan.md reports 80.6 mm;  reproducing it is an independent
           check on the metric, since nothing in the code knows that number.'''
        fs = ' dh_analysis compensation FAIL'
        dh, vv, pvals, ndof = _robot('KinovaLite')

        plain = sp.Matrix(dh); plain[4, D] = 0
        comp = sp.Matrix(dh);  comp[4, D] = 0
        comp[5, D] = sp.sympify(dh[5, D]) + 57

        mp = displacement_metric(dh, plain, pvals, vv, ndof, n=40, seed=7)
        mc = displacement_metric(dh, comp, pvals, vv, ndof, n=40, seed=7)
        self.assertAlmostEqual(mp['mean_dp'], 57.0, places=6, msg=fs + ' (plain)')
        self.assertAlmostEqual(mc['mean_dp'], 57.0*np.sqrt(2.0), places=4,
                               msg=fs + ' (compensated should be 57*sqrt(2))')
        self.assertGreater(mc['mean_dp'], mp['mean_dp'],
                           fs + ' (compensating must be WORSE, not better)')

    def test_dhK_prismatic_route_is_blocked_not_dropped(self):
        '''A route that needs zeroing a prismatic joint variable is reported
           BLOCKED, never silently omitted.

           Bartell's d_2 and d_4 are joint variables.  Dropping the route
           quietly would look identical to "this triple needs no work".'''
        fs = ' dh_analysis blocked-route FAIL'
        dh, vv, pvals, ndof = _robot('Bartell')
        cands = candidate_simplifications(dh, pvals, vv, ndof)
        #  Bartell satisfies every triple already, so there is nothing to buy
        self.assertEqual(cands, [], fs + ' (Bartell already has triples)')

        #  a synthetic arm whose middle d IS the joint variable and whose
        #  geometry does not otherwise qualify
        dh2 = _table(sp.pi/2, 0, 5)
        dh2[2, D] = sp.Symbol('d_3')          # joint 3 prismatic
        vv2 = [1, 1, 0, 1, 1, 1]
        blocked = [c for c in candidate_simplifications(dh2, {}, vv2, 6)
                   if c['route'] == 'zero_offsets' and c['blocked']]
        self.assertTrue(blocked,
                        fs + ' (expected a blocked zero_offsets route for the '
                        'prismatic joint variable)')
        self.assertIn('prismatic', blocked[0]['blocked'], fs + ' (say why)')


def run_test():
    suite = unittest.TestLoader().loadTestsFromTestCase(TestSolver019)
    unittest.TextTestRunner(verbosity=2).run(suite)


##   Self Tests
if __name__ == "__main__":
    run_test()
