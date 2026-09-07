#!/usr/bin/python
#
#   axis_triple_check.py --  verify the Pieper-triple rule against real geometry
#
#   The intersecting/parallel-axis test in ikbtbasics/dh_analysis.py is pure
#   DH-cell arithmetic, and a wrong row or column index there gives a detector
#   that is confidently wrong on every robot and looks perfectly plausible.  So
#   the rule is checked against an INDEPENDENT oracle:  build the numeric FK
#   with Link_N, extract each joint axis as a line in the base frame, and test
#   concurrency and parallelism geometrically.  Two paths to one answer.
#
#       python3 -m scripts.axis_triple_check              # all robots
#       python3 -m scripts.axis_triple_check Puma Stanford
#
#   Exit status is 0 when the rule and the geometry agree everywhere.
#
#   It caught all three of the silent failure modes listed in dh_analysis.py's
#   own header, which is why it is worth keeping.
#
#   Copyright 2026 University of Washington
#
#   Developed by Blake Hannaford
#   BioRobotics Lab, University of Washington

import contextlib
import io
import sys
import numpy as np, sympy as sp
from ikbtfunctions.ik_robots import robot_params
from ikbtbasics.pykinsym import Link_N
from ikbtbasics.dh_analysis import triple_report, joint_triples

def numeric_rows(dh, vv, pvals, q, ndof):
    '''Substitute pvals + a joint configuration; return 6 rows of floats.'''
    rows = []
    for r in range(6):
        vals = []
        for c in range(4):
            e = sp.sympify(dh[r, c])
            e = e.subs({k: v for k, v in pvals.items()
                        if not isinstance(v, str)})
            # any symbol left is either a joint variable or an unvalued param
            for sym in sorted(e.free_symbols, key=str):
                nm = str(sym)
                if nm in q:  vals_sub = q[nm]
                else:        vals_sub = 1.0 if nm.startswith(('d','a','l','h')) else np.pi/2
                e = e.subs(sym, vals_sub)
            vals.append(float(e))
        rows.append(vals)
    return rows

def axes_in_base(rows, ndof):
    '''Joint n's axis is the Z axis of frame {n}:  point = origin of {n},
       direction = 3rd column of R_0n.  Returns dict n -> (point, dir).'''
    T = np.matrix(np.eye(4))
    out = {}
    for r in range(ndof):                 # row r carries th_{r+1} -> frame r+1
        al, a, d, th = rows[r]
        T = T * Link_N(al, a, d, th)
        n = r + 1
        out[n] = (np.array(T[:3, 3]).flatten(), np.array(T[:3, 2]).flatten())
    return out

def lines_meet(p1, u1, p2, u2, tol=1e-7):
    '''(intersect?, parallel?) for two lines.'''
    u1 = u1/np.linalg.norm(u1); u2 = u2/np.linalg.norm(u2)
    cr = np.cross(u1, u2)
    par = np.linalg.norm(cr) < 1e-9
    if par:
        # distance between parallel lines
        dv = p2 - p1
        dist = np.linalg.norm(dv - np.dot(dv, u1)*u1)
        return dist < tol, True
    dist = abs(np.dot(p2 - p1, cr/np.linalg.norm(cr)))
    return dist < tol, False

def triple_kind(ax, n, tol=1e-6):
    """Concurrent (all three lines through one point) / parallel / neither.

       The common point must be solved from a NON-PARALLEL pair.  Picking
       blindly is wrong: when two of the three axes are collinear (which happens
       whenever a==0 and sin(alpha)==0 between them) the 3x2 system is
       rank-deficient and lstsq returns a meaningless 'intersection'."""
    if not all(k in ax for k in (n, n+1, n+2)):
        return None
    P = [ax[n], ax[n+1], ax[n+2]]
    U = [p[1]/np.linalg.norm(p[1]) for p in P]

    par = all(np.linalg.norm(np.cross(U[0], u)) < 1e-9 for u in U)

    #  every pair must at least meet (or be collinear)
    for (i, j) in ((0,1), (1,2), (0,2)):
        met, _ = lines_meet(P[i][0], U[i], P[j][0], U[j], tol)
        if not met:
            return {'concurrent': False, 'parallel': bool(par)}
    if par:
        #  three mutually parallel lines that "meet" are one single line
        return {'concurrent': True, 'parallel': True}

    #  find a pair that is NOT parallel and intersect it
    pair = None
    for (i, j) in ((0,1), (1,2), (0,2)):
        if np.linalg.norm(np.cross(U[i], U[j])) > 1e-9:
            pair = (i, j); break
    i, j = pair
    A = np.column_stack([U[i], -U[j]]); b = P[j][0] - P[i][0]
    t = np.linalg.lstsq(A, b, rcond=None)[0]
    X = P[i][0] + t[0]*U[i]
    #  X must lie on ALL three lines
    scale = max(1.0, max(np.linalg.norm(p[0]) for p in P))
    for k in range(3):
        w = X - P[k][0]
        off = np.linalg.norm(w - np.dot(w, U[k])*U[k])
        if off > tol*scale:
            return {'concurrent': False, 'parallel': False}
    return {'concurrent': True, 'parallel': False}

def formula(dh, vv, pvals, n):
    """The rule AS SHIPPED -- delegates to ikbtbasics.dh_analysis.

       This is the point of the script:  it validates the module the solver
       actually uses, not a copy of it that can drift.  If dh_analysis changes
       and this stops agreeing with the geometry, that is the bug report."""
    r = triple_report(dh, pvals, n)
    return {'concurrent': bool(r['intersect']), 'parallel': bool(r['parallel'])}


from ikbtfunctions.ik_robots import ROBOT_LIST
ROBOTS = sys.argv[1:] or [r for r in ROBOT_LIST]
print('%-16s %-6s  %-28s %-28s %s' % ('robot','triple','geometric (from FK)',
                                      'formula (from DH cells)','match'))
print('-'*108)
allok = True
for name in ROBOTS:
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        dh, vv, params, pvals, unks = robot_params(name)
    ndof = len([u for u in unks])
    # joint variable names
    for trial, seed in enumerate([1,2,3]):
        rng = np.random.default_rng(seed)
        q = {}
        for r in range(ndof):
            for c in (2,3):
                e = sp.sympify(dh[r,c])
                for sym in e.free_symbols:
                    q.setdefault(str(sym), float(rng.uniform(-1.7,1.7)))
        rows = numeric_rows(dh, vv, pvals, q, ndof)
        ax = axes_in_base(rows, ndof)
        res = {}
        for n in joint_triples(ndof):
            g = triple_kind(ax, n)
            if g is None: continue
            res[n] = g
        if trial == 0:
            ref = res
        else:
            for n in res:
                if res[n] != ref[n]:
                    print('  !! %s triple %d not configuration-invariant: %s vs %s'
                          % (name, n, ref[n], res[n]))
                    allok = False
    for n in sorted(ref):
        f = formula(dh, vv, pvals, n)
        ok = (f == ref[n])
        if not ok: allok = False
        print('%-16s %-6s  %-28s %-28s %s'
              % (name, '(%d,%d,%d)'%(n,n+1,n+2),
                 'conc=%-5s par=%-5s'%(ref[n]['concurrent'],ref[n]['parallel']),
                 'conc=%-5s par=%-5s'%(f['concurrent'],f['parallel']),
                 'OK' if ok else '<<< MISMATCH'))
    print('-'*108)
print('ALL AGREE' if allok else 'THERE ARE MISMATCHES')
sys.exit(0 if allok else 1)
