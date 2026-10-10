#!/usr/bin/python
#
#   hybrid_test.py --  run the HYBRID branch on robots whose exact answer is
#                      known, and check it against that answer.
#
#       python3 -m scripts.hybrid_test                       # Puma, KawasakiRS007L
#       python3 -m scripts.hybrid_test Puma --poses 50 --delta 0.10
#       python3 -m scripts.hybrid_test --no-cpp
#
#   WHY.  Raven-II is the only robot in ROBOT_LIST that needs the hybrid method,
#   so its code generators had one real customer, and that customer's answer
#   is not known independently.  This sends robots that SOLVE SYMBOLICALLY down
#   the hybrid branch on purpose (bt_assembly.build_hybrid_test_bt):  the
#   derived arm is the real one with every nonzero link length scaled by
#   (1 + delta), which keeps every Pieper triple, so it still solves.  Phase I
#   seeds come from that arm;  Phase II refines them against the TRUE arm.
#
#   THE ORACLE IS THE ROBOT'S OWN CLOSED FORM.  A refined posture that reaches
#   the pose must be one of the exact symbolic solutions, so this asks more than
#   numerical_closed_loop_sol_check can for Raven-II -- not only "does it reach
#   the pose" but "is it one of the right answers".
#
#   The run is named '<robot>_HT' (ik_robots.HYBRID_TEST_SUFFIX), so its report,
#   code and FK pickle never overwrite the real robot's.  It needs the real
#   robot's symbolic code, CodeGen/Python/IK_equations<robot>.py, and makes it
#   (python3 ikSolver.py <robot>) when it is missing.
#
#   FAILS (exit 1) when the hybrid solve does not finish, when a converged
#   refined posture is not an exact solution, or when Phase I gave a pose seeds
#   and none of them converged.  REPORTED, not asserted:  how many of the exact
#   postures the seeds find (damped least squares stays in its seed's basin,
#   and two seeds may share one), and poses where Phase I gives NO seeds -- the
#   scaled arm cannot reach every pose the true one can, which is the hybrid
#   method's known limit, not a defect.
#
#   Copyright 2026 University of Washington
#   Developed by Blake Hannaford
#   BioRobotics Lab, University of Washington

import argparse
import importlib.util
import os
import shutil
import subprocess
import sys

import numpy as np

DEFAULT_ROBOTS = ['Puma', 'KawasakiRS007L']
PYDIR = os.path.join('CodeGen', 'Python')


def load(path, modname):
    spec = importlib.util.spec_from_file_location(modname, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def solve_hybrid(base, delta):
    '''Solve <base>_HT down the test tree.  Returns (alias, error text or None).'''
    from ikbtfunctions.ik_robots import HYBRID_TEST_SUFFIX
    from ikbtfunctions.ik_driver import (load_robot, run_solver,
                                         solved_anything, ensure_logdir)
    from ikbtfunctions.bt_assembly import build_hybrid_test_bt
    from ikbtfunctions.output_python import py_identifier

    alias = base + HYBRID_TEST_SUFFIX
    top = os.path.join(PYDIR, f'IK_hybrid_{alias}.py')
    if os.path.exists(top):
        os.remove(top)                  # so a stale one cannot pass for this run

    #  The alias has the real robot's FK exactly, so start from its pickle
    #  rather than recompute it (minutes, for a 6-DOF arm).
    pk = os.path.join('fk_eqns', f'{alias}_pickle.p')
    real_pk = os.path.join('fk_eqns', f'{base}_pickle.p')
    if not os.path.exists(pk) and os.path.exists(real_pk):
        shutil.copyfile(real_pk, pk)

    ensure_logdir()
    M, R, unks = load_robot(alias)
    bt = build_hybrid_test_bt(delta=delta, codegen=True, quiet=True)
    R, unks, bb = run_solver(R, unks, bt, create_solutions=False)
    if not solved_anything(bb):
        return alias, 'the derived arm solved nothing'
    if not os.path.exists(top):
        return alias, f'no {top} -- the derived arm did not solve completely'
    return alias, None


def compare(base, alias, n_poses, seed, tol=1e-8):
    '''Refined hybrid postures vs. the exact symbolic ones, over random poses.'''
    from ikbtfunctions.ik_robots import robot_params
    from ikbtfunctions.output_python import py_identifier

    sym_path = os.path.join(PYDIR, f'IK_equations{base}.py')
    if not os.path.exists(sym_path):
        print(f'  {sym_path} missing -- solving {base} symbolically first')
        subprocess.run([sys.executable, 'ikSolver.py', base], check=True,
                       stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    sym = load(sym_path, f'hybtest_sym_{py_identifier(base)}')
    hyb = load(os.path.join(PYDIR, f'IK_hybrid_{alias}.py'),
               f'hybtest_hyb_{py_identifier(alias)}')
    ikin = getattr(sym, f'ikin_{py_identifier(base)}')
    refine_all = getattr(hyb, f'refine_all_{py_identifier(alias)}')
    fk = getattr(hyb.true_fk, f'fk_{py_identifier(alias)}')

    _, vv, _, pvals, _ = robot_params(base)
    revolute = [v == 1 for v in vv][:hyb.NDOF]
    lmax = max([abs(float(v)) for v in pvals.values()
                if isinstance(v, (int, float))] or [1.0])

    def same(p, q):
        d = np.asarray(p, float) - np.asarray(q, float)
        d = np.where(revolute, (d + np.pi) % (2*np.pi) - np.pi, d)
        return np.max(np.abs(d)) < 1e-6

    def distinct(qs):
        out = []
        for q in qs:
            if not any(same(q, p) for p in out):
                out.append(q)
        return out

    def reaches(q, T):
        return np.max(np.abs(np.asarray(fk(q), float) - T)) < tol

    rng = np.random.default_rng(seed)
    tally = {'poses': n_poses, 'exact': 0, 'found': 0, 'unsound': 0,
             'no_seeds': 0, 'stuck': 0, 'probe': 0, 'seeds': 0, 'converged': 0}
    for _ in range(n_poses):
        q0 = np.array([rng.uniform(-np.pi, np.pi) if r else rng.uniform(-lmax, lmax)
                       for r in revolute])
        T = np.asarray(fk(q0), float)

        rows = ikin(np.matrix(T))
        rows = [] if rows is False else [np.asarray(r, float) for r in rows]
        S = distinct([q for q in rows if np.all(np.isfinite(q)) and reaches(q, T)])

        res = refine_all(T)
        tally['seeds'] += len(res)
        conv = [np.asarray(r['q'], float) for r in res if r['converged']]
        tally['converged'] += len(conv)
        H = distinct(conv)

        tally['exact'] += len(S)
        tally['found'] += sum(any(same(h, s) for h in H) for s in S)
        tally['unsound'] += sum(not any(same(h, s) for s in S) for h in H)
        tally['no_seeds'] += (len(res) == 0)
        tally['stuck'] += (len(res) > 0 and len(H) == 0)
        tally['probe'] += any(same(h, q0) for h in H)
    return tally


def main():
    ap = argparse.ArgumentParser(description='Run the hybrid branch on robots'
                                 ' whose exact answer is known, and check it.')
    ap.add_argument('robots', nargs='*', default=DEFAULT_ROBOTS)
    ap.add_argument('--delta', type=float, default=0.05,
                    help='link-length scale of the derived arm is 1 + DELTA')
    ap.add_argument('--poses', type=int, default=20)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--no-cpp', action='store_true',
                    help='skip cpp_closed_loop_check --hybrid')
    a = ap.parse_args()

    failures = []
    rows = []
    for base in a.robots:
        alias, err = solve_hybrid(base, a.delta)
        if err:
            failures.append(f'{alias}: {err}')
            rows.append(f'  {alias:22s} SOLVE FAILED -- {err}')
            continue
        t = compare(base, alias, a.poses, a.seed)
        rows.append(f"  {alias:22s} {t['poses']:5d} poses   refined postures "
                    f"exact: {t['found']}/{t['exact']}   unsound {t['unsound']}   "
                    f"no seeds {t['no_seeds']}   stuck {t['stuck']}   probe {t['probe']}/{t['poses']}   "
                    f"seeds converged {t['converged']}/{t['seeds']}")
        if t['unsound']:
            failures.append(f"{alias}: {t['unsound']} refined posture(s) are not "
                            'exact solutions')
        if t['stuck']:
            failures.append(f"{alias}: {t['stuck']} pose(s) had seeds and none "
                            'converged')
        if not a.no_cpp:
            p = subprocess.run([sys.executable, '-m', 'scripts.cpp_closed_loop_check',
                                '--hybrid', alias], capture_output=True, text=True)
            ok = p.returncode == 0
            rows.append(f"  {'':22s} C++ --hybrid: {'ok' if ok else 'FAILED'}")
            if not ok:
                failures.append(f'{alias}: cpp_closed_loop_check --hybrid failed\n'
                                + p.stdout[-1500:])

    print(f'\n  hybrid test, derived arm = every link length x {1 + a.delta:g}')
    print('\n'.join(rows))
    if failures:
        print('\n  FAILED:\n    ' + '\n    '.join(failures))
        sys.exit(1)
    print('\n  OK')


if __name__ == '__main__':
    main()
