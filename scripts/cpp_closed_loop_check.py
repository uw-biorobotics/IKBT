#!/usr/bin/python
#
#   cpp_closed_loop_check.py --  is the GENERATED C++ actually correct?
#
#       python3 -m scripts.cpp_closed_loop_check                # the default robots
#       python3 -m scripts.cpp_closed_loop_check Puma Stanford  # just these
#       python3 -m scripts.cpp_closed_loop_check --keep         # don't re-solve
#       python3 -m scripts.cpp_closed_loop_check --all --keep   # every robot
#       python3 -m scripts.cpp_closed_loop_check --compile-only --all
#       python3 -m scripts.cpp_closed_loop_check --gate         # exit 1 on a regression
#
#   The C++ twin of numerical_closed_loop_sol_check.py, and it asks TWO
#   questions where that one asks one:
#
#     SOUNDNESS   pick q -> T = FK(q) -> compiled ikin_*(T) -> every returned
#                 branch must satisfy FK(branch) == T.  T comes from the
#                 robot's own forward kinematics, so it is reachable by
#                 construction and there is nothing to look up or trust.
#
#     FIDELITY    the same robot and the same pose through the generated
#                 PYTHON must give the same rows, in the same order, to 1e-12.
#
#   Fidelity is the one that earns its keep.  The C++ generator is derived from
#   the python generator, and this is what turns "derived from" into an
#   assertion: a round trip through FK cannot see a wrong column order, a
#   dropped branch or two versions swapped, because every permutation of a
#   correct answer is still a correct answer.  Agreement can see all three.
#
#   THREE KINDS OF COMPILE FAILURE, and only one of them is a regression:
#
#     XXXXX        the deliberate compile stop for a parameter the robot has
#                  no pvals entry for.  Expected;  see CodeGen/HOWTO.txt.
#     undeclared   a solution that references the variable it solves for.  An
#                  UPSTREAM solver defect -- the same one that shows up in
#                  python as UnboundLocalError, which is why these robots are
#                  in expected.UNCHECKABLE.  C++ catches it at compile time
#                  instead of at run time, which is the better of the two.
#     other        a real defect in this generator.
#
#   Skips cleanly with no g++, the way write_latex_fitted() degrades with no
#   pdflatex.
#
#   EVERYTHING FLUSHES.  A sweep over 32 robots is minutes of solving, and
#   python buffers stdout when it is not a tty -- under `> log` the first run
#   of this script wrote a 0-byte file for its whole life and then everything
#   at once.  Same lesson, same fix, as ikbtbasics/numeric_ik.py's _say().
#
#   Copyright 2026 University of Washington
#
#   Developed by Blake Hannaford
#   BioRobotics Lab, University of Washington

import argparse
import io
import contextlib
import os
import shutil
import subprocess
import sys
import tempfile

import numpy as np

from ikbtfunctions.output_cpp_common import cpp_identifier, inline_src
from scripts.expected import EXPECT, UNCHECKABLE, judge_counts
from scripts.numerical_closed_loop_sol_check import (Q_PROBE, TOL, robot_fk,
                                                     import_generated_ik,
                                                     solve_robot)


CPP_DIR = os.path.join('CodeGen', 'Cpp')

#  Python and C++ do the same IEEE operations in the same order, so they agree
#  far better than this;  the slack is for the two printers rounding a literal
#  differently in the last place and that difference being amplified through a
#  long expression.  Measured worst case across the robot set is reported at
#  the end of every run -- if it creeps up, something has diverged.
AGREE_TOL = 1e-9

DEFAULT_ROBOTS = sorted(EXPECT)


#  The harness appended to a copy of the generated file.  Reads 16 doubles for
#  T on stdin, prints the branch count and then one line per branch.
#
#  APPENDED, not #included:  the generated file is a complete translation unit
#  with no header beside it (that is the point of inlining Cpp_src/), and
#  appending needs no include path and cannot pick up a stale copy.
DRIVER = '''

#ifdef IKBT_DRIVER
#include <cstdio>
int main(void)
{
    Mat4 T;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            if (std::scanf("%lf", &T[i][j]) != 1)
                return 2;
    SolutionList sols = **CALL**;
    std::printf("%d\\n", (int) sols.size());
    for (size_t i = 0; i < sols.size(); ++i) {
        for (size_t j = 0; j < sols[i].size(); ++j)
            std::printf("%s%.17g", j ? " " : "", sols[i][j]);
        std::printf("\\n");
    }
    return 0;
}
#endif
'''


def classify(stderr):
    '''Which of the three kinds of compile failure this is.'''

    if 'XXXXX' in stderr:
        return 'XXXXX'
    if 'was not declared in this scope' in stderr:
        return 'undeclared'
    return 'other'


def build(name, extra_arg=None, keep_dir=None):
    '''Compile IK_equations<name>.cpp plus the driver.

       Returns (binary path or None, kind, message).  kind is 'ok', 'missing',
       'XXXXX', 'undeclared' or 'other'.'''

    src = os.path.join(CPP_DIR, 'IK_equations%s.cpp' % name)
    if not os.path.exists(src):
        return None, 'missing', 'no generated C++ at %s' % src

    ident = cpp_identifier(name)
    call = 'ikin_%s(T%s)' % (ident, ', %s' % extra_arg if extra_arg else '')

    tmp = keep_dir or tempfile.mkdtemp(prefix='ikbt_cpp_')
    cpath = os.path.join(tmp, 'check_%s.cpp' % ident)
    bpath = os.path.join(tmp, 'check_%s' % ident)
    with open(cpath, 'w') as f:
        f.write(open(src).read())
        f.write(DRIVER.replace('**CALL**', call))

    cc = subprocess.run(['g++', '-std=c++11', '-O2', '-Wall', '-Wextra',
                         '-DIKBT_DRIVER', cpath, '-o', bpath, '-lm'],
                        capture_output=True, text=True)
    if cc.returncode != 0:
        kind = classify(cc.stderr)
        first = next((l for l in cc.stderr.split('\n') if 'error:' in l), '')
        return None, kind, first.split('error: ')[-1][:80]
    warn = [l for l in cc.stderr.split('\n') if 'warning:' in l]
    return bpath, 'ok', ('%d warning(s)' % len(warn)) if warn else ''


def run_binary(bpath, T):
    '''Feed one 4x4 to the compiled IK;  returns a list of joint vectors.'''

    stdin = '\n'.join(' '.join('%.17g' % T[i][j] for j in range(4))
                      for i in range(4))
    r = subprocess.run([bpath], input=stdin, capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError('the compiled IK exited %d' % r.returncode)
    lines = [l for l in r.stdout.split('\n') if l.strip()]
    if not lines:
        raise RuntimeError('the compiled IK printed nothing')
    n = int(lines[0])
    return [[float(x) for x in l.split()] for l in lines[1:1 + n]]


def check_one(name, verbose=False):
    '''Compile, round-trip and compare one robot.

       Returns a dict.  Never raises:  a robot that could not be checked comes
       back with its reason in 'note'.'''

    res = {'name': name, 'kind': None, 'note': '', 'good': 0, 'total': 0,
           'agree': None, 'worst_fk': None}

    bpath, kind, msg = build(name)
    res['kind'] = kind
    res['note'] = msg
    if bpath is None:
        return res

    try:
        fk, jnames = robot_fk(name)
    except Exception as e:
        res['note'] = 'FK unavailable -- %s' % str(e)[:60]
        return res

    q_true = [float(x) for x in Q_PROBE[:len(jnames)]]
    T = np.asarray(fk(q_true), dtype=float)

    try:
        rows = run_binary(bpath, T)
    except Exception as e:
        res['note'] = '%s: %s' % (type(e).__name__, str(e)[:60])
        return res

    res['total'] = len(rows)
    if not rows:
        res['note'] = 'ikin_*() reports the pose unreachable'
        return res

    #  SOUNDNESS.  The C++ columns are in JOINT_NAMES order, which IS the DH
    #  chain order (the generator builds it by filtering joint_symbols), so a
    #  returned row is already a q vector.
    worst = 0.0
    for i, row in enumerate(rows):
        try:
            err = float(np.max(np.abs(np.asarray(fk(row), dtype=float) - T)))
        except Exception:
            continue
        worst = max(worst, err) if err < 1e3 else worst
        if err < TOL:
            res['good'] += 1
        elif verbose:
            print('      branch %-3d FK error %.2e' % (i, err))
    res['worst_fk'] = worst

    #  FIDELITY.  The same pose through the generated python.
    try:
        fn, cols = import_generated_ik(name)
        jcol = [cols.index(j) for j in jnames]
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            psols = fn(np.matrix(T))
        if psols is False:
            res['agree'] = 'python reports the pose unreachable'
        elif len(psols) != len(rows):
            res['agree'] = ('python returned %d branches, C++ %d'
                            % (len(psols), len(rows)))
        else:
            worst_a = 0.0
            for prow, crow in zip(psols, rows):
                pq = [float(prow[k]) for k in jcol]
                worst_a = max(worst_a,
                              float(np.max(np.abs(np.array(pq)
                                                  - np.array(crow)))))
            res['agree'] = worst_a
    except Exception as e:
        res['agree'] = '%s: %s' % (type(e).__name__, str(e)[:50])

    return res


###############################################################################
#
#    The FK / Jacobian emitter, elementwise
#
#  A UNIT CHECK, not a path check.  fk_<R>() and jacobian_<R>() are what the
#  hybrid refinement and the one-variable search are measured against, so an
#  error in either would be reported by those paths as "did not converge" --
#  a symptom a long way from its cause.  This compares them against their
#  python twins directly, element by element, over random q.
#

FK_DRIVER = """
#include <cstdio>
#include "**HEADER**"
using namespace ikbt;
int main(void)
{
    JointVec q(**NDOF**, 0.0);
    for (int i = 0; i < **NDOF**; ++i)
        if (std::scanf("%lf", &q[i]) != 1)
            return 2;
    Mat4 T = fk_**IDENT**(q);
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            std::printf("%.17g\\n", T[i][j]);
**JAC**
    return 0;
}
"""

FK_DRIVER_JAC = """    Matrix J = jacobian_**IDENT**(q);
    for (size_t i = 0; i < J.size(); ++i)
        for (size_t j = 0; j < J[i].size(); ++j)
            std::printf("%.17g\\n", J[i][j]);
"""


def check_fk(name, n_trials=8, seed=11, verbose=False):
    '''Elementwise agreement of the C++ and python FK (and Jacobian).

       Returns (worst_T, worst_J, note).  worst_* is None when that piece was
       not generated;  a note means the check could not run.'''

    import importlib.util
    import random

    from ikbtfunctions.ik_driver import load_robot
    from ikbtfunctions.output_cpp_common import write_fk_module_cpp
    from ikbtfunctions.output_numeric_common import write_fk_module

    tmp = tempfile.mkdtemp(prefix='ikbt_fk_')
    try:
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            M, R, unks = load_robot(name)
            hpath = write_fk_module_cpp(M, name, jacobian=True, dirname=tmp)
            ppath = write_fk_module(M, name, jacobian=True, dirname=tmp)
    except Exception as e:
        return None, None, '%s: %s' % (type(e).__name__, str(e)[:70])

    ident = cpp_identifier(name)
    ndof = M.ndof

    src = (FK_DRIVER.replace('**HEADER**', os.path.basename(hpath))
                    .replace('**IDENT**', ident)
                    .replace('**NDOF**', str(ndof))
                    .replace('**JAC**',
                             FK_DRIVER_JAC.replace('**IDENT**', ident)))
    cpath = os.path.join(tmp, 'fk_check.cpp')
    bpath = os.path.join(tmp, 'fk_check')
    with open(cpath, 'w') as f:
        f.write(src)

    cc = subprocess.run(['g++', '-std=c++11', '-O2', '-Wall', '-Wextra',
                         '-I', tmp, cpath, '-o', bpath, '-lm'],
                        capture_output=True, text=True)
    if cc.returncode != 0:
        first = next((l for l in cc.stderr.split('\n') if 'error:' in l), '')
        return None, None, 'compile: %s' % first.split('error: ')[-1][:70]

    spec = importlib.util.spec_from_file_location('genfk_' + ident, ppath)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    py_fk = getattr(mod, 'fk_' + ident)
    py_jac = getattr(mod, 'jacobian_' + ident, None)

    rng = random.Random(seed)
    worst_T = 0.0
    worst_J = 0.0 if py_jac else None
    for _ in range(n_trials):
        q = [rng.uniform(-2.5, 2.5) for _ in range(ndof)]
        r = subprocess.run([bpath], input='\n'.join('%.17g' % v for v in q),
                           capture_output=True, text=True)
        if r.returncode != 0:
            return None, None, 'the compiled FK exited %d' % r.returncode
        vals = [float(x) for x in r.stdout.split()]
        cT = np.array(vals[:16]).reshape(4, 4)
        worst_T = max(worst_T, float(np.max(np.abs(cT - np.asarray(py_fk(q),
                                                                  dtype=float)))))
        if py_jac:
            cJ = np.array(vals[16:16 + 6 * ndof]).reshape(6, ndof)
            worst_J = max(worst_J,
                          float(np.max(np.abs(cJ - np.asarray(py_jac(q),
                                                              dtype=float)))))
        if verbose:
            print('      q = %s  dT %.1e' % (['%.2f' % v for v in q], worst_T))

    shutil.rmtree(tmp, ignore_errors=True)
    return worst_T, worst_J, ''


###############################################################################
#
#    The damped-least-squares core, pinned against python
#
#  Cpp_src/ikbt_dls.h is a COPY of ikbtbasics/numeric_ik.solve_numeric(), two
#  languages removed from it (python source -> DLS_CORE string -> C++ header).
#  A copy is a liability, so it is pinned: the same robot, the same seed, the
#  same target, and the refined joint vector, the metric and the ITERATION
#  COUNT all have to match.  The iteration count is the sharp one -- it catches
#  a damping schedule that drifted, which a converged answer would hide.
#

DLS_DRIVER = """
#include <cstdio>
#include "**HEADER**"
using namespace ikbt;
int main(void)
{
    const int n = NDOF_**IDENT**;
    JointVec q0(n, 0.0);
    for (int i = 0; i < n; ++i)
        if (std::scanf("%lf", &q0[i]) != 1) return 2;
    Mat4 Td;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            if (std::scanf("%lf", &Td[i][j]) != 1) return 2;
    double w_rot;
    if (std::scanf("%lf", &w_rot) != 1) return 2;

    SolveResult r = solve_numeric(fk_**IDENT**, jacobian_**IDENT**,
                                  q0, Td, w_rot);
    for (int i = 0; i < n; ++i)
        std::printf("%.17g\\n", r.q[i]);
    std::printf("%.17g\\n%d\\n%d\\n", r.metric, r.iterations,
                r.converged ? 1 : 0);
    std::printf("%s\\n", r.reason);
    return 0;
}
"""


def check_dls(name, n_trials=6, seed=13, verbose=False):
    '''Does Cpp_src/ikbt_dls.h agree with numeric_ik.solve_numeric()?

       Returns (worst_dq, worst_dmetric, iteration mismatches, note).

       VALIDATED ON ROBOTS THAT ALREADY SOLVE EXACTLY, with no dependence on
       the hybrid branch: perturb a known-good pose and confirm it comes back.
       That is how numeric_ik itself is tested, and it is why this check runs
       for any robot with a usable FK rather than only the hybrid ones.'''

    import random

    import ikbtbasics.numeric_ik as nik
    from ikbtfunctions.ik_driver import load_robot
    from ikbtfunctions.output_cpp_common import write_fk_module_cpp

    tmp = tempfile.mkdtemp(prefix='ikbt_dls_')
    try:
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            M, R, unks = load_robot(name)
            hpath = write_fk_module_cpp(M, name, jacobian=True, dirname=tmp)
    except Exception as e:
        return None, None, None, '%s: %s' % (type(e).__name__, str(e)[:70])

    ident = cpp_identifier(name)
    ndof = M.ndof
    w_rot = float(nik.w_rot_for(M, ndof))
    py_fk = nik.fk_callable(M, ndof)
    py_jac = nik.jacobian_callable(M, ndof)

    src = (DLS_DRIVER.replace('**HEADER**', os.path.basename(hpath))
                     .replace('**IDENT**', ident))
    cpath = os.path.join(tmp, 'dls_check.cpp')
    bpath = os.path.join(tmp, 'dls_check')
    with open(cpath, 'w') as f:
        f.write(inline_src(['ikbt_types.h', 'ikbt_linalg.h',
                            'ikbt_pose_error.h', 'ikbt_dls.h']))
        f.write(src)

    cc = subprocess.run(['g++', '-std=c++11', '-O2', '-Wall', '-Wextra',
                         '-I', tmp, cpath, '-o', bpath, '-lm'],
                        capture_output=True, text=True)
    if cc.returncode != 0:
        first = next((l for l in cc.stderr.split('\n') if 'error:' in l), '')
        return None, None, None, 'compile: %s' % first.split('error: ')[-1][:70]

    rng = random.Random(seed)
    worst_q = 0.0
    worst_m = 0.0
    iter_bad = 0
    for _ in range(n_trials):
        q_true = [rng.uniform(-2.0, 2.0) for _ in range(ndof)]
        Td = np.asarray(py_fk(q_true), dtype=float)
        #  Perturb: a seed near, but not at, the answer -- the situation
        #  Phase II is actually in.
        q0 = [v + rng.uniform(-0.25, 0.25) for v in q_true]

        stdin = ('\n'.join('%.17g' % v for v in q0) + '\n'
                 + '\n'.join(' '.join('%.17g' % Td[i][j] for j in range(4))
                              for i in range(4)) + '\n'
                 + '%.17g\n' % w_rot)
        r = subprocess.run([bpath], input=stdin, capture_output=True, text=True)
        if r.returncode != 0:
            return None, None, None, 'the compiled DLS exited %d' % r.returncode
        out = r.stdout.split()
        cq = [float(x) for x in out[:ndof]]
        cm = float(out[ndof])
        cit = int(out[ndof + 1])

        pr = nik.solve_numeric(py_fk, py_jac, q0, Td, w_rot=w_rot)
        worst_q = max(worst_q, float(np.max(np.abs(np.array(cq)
                                                   - np.array(pr['q'])))))
        worst_m = max(worst_m, abs(cm - float(pr['metric'])))
        if cit != int(pr['iterations']):
            iter_bad += 1
            if verbose:
                print('      iterations differ: C++ %d, python %d'
                      % (cit, pr['iterations']))

    shutil.rmtree(tmp, ignore_errors=True)
    return worst_q, worst_m, iter_bad, ''


###############################################################################
#
#    The two fallback paths:  does the C++ answer what the python answers?
#
#  Both are checked the same way and for the same reason as the symbolic
#  path's fidelity test:  a round trip through FK proves an answer is SOUND,
#  and only agreement with the python proves the SET is the same one.  On the
#  one-variable path that distinction is the whole point -- completeness is
#  the question that path exists to answer, and a search that quietly finds
#  six of eight roots round-trips perfectly.
#

ONEVAR_DRIVER = """
#ifdef IKBT_CHECK_DRIVER
#include <cstdio>
int main(void)
{
    Mat4 T;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            if (std::scanf("%lf", &T[i][j]) != 1) return 2;
    std::vector<OneVarSolution> sols = solve_**IDENT**(T);
    std::printf("%d %d\\n", (int) sols.size(), ONEVAR_NDOF);
    for (size_t i = 0; i < sols.size(); ++i) {
        std::printf("%.17g %.17g %d", sols[i].known_value, sols[i].error,
                    sols[i].n_samples);
        for (int j = 0; j < ONEVAR_NDOF; ++j)
            std::printf(" %.17g", sols[i].q[j]);
        std::printf("\\n");
    }
    return 0;
}
#endif
"""

HYBRID_DRIVER = """
#ifdef IKBT_CHECK_DRIVER
#include <cstdio>
int main(void)
{
    Mat4 T;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            if (std::scanf("%lf", &T[i][j]) != 1) return 2;
    std::vector<RefineRecord> all = refine_all_**IDENT**(T);
    std::printf("%d %d\\n", (int) all.size(), NDOF);
    for (size_t i = 0; i < all.size(); ++i) {
        std::printf("%.17g %d %d", all[i].result.metric,
                    all[i].result.iterations, all[i].result.converged ? 1 : 0);
        for (int j = 0; j < NDOF; ++j)
            std::printf(" %.17g", all[i].result.q[j]);
        std::printf("\\n");
    }
    return 0;
}
#endif
"""


def _build_appended(src_path, driver, tmp, tag):
    """Copy a generated .cpp, append a driver, compile it.

       APPENDED rather than a separate file, for the same reason as build():
       the generated file's siblings are found relative to ITSELF, so the copy
       has to live beside them -- which it does, because the copy goes into
       CodeGen/Cpp/ under a temporary name and only the binary goes to tmp."""

    if not os.path.exists(src_path):
        return None, 'not generated: %s' % src_path

    cpath = os.path.join(os.path.dirname(src_path), '_check_%s.cpp' % tag)
    bpath = os.path.join(tmp, 'check_%s' % tag)
    try:
        with open(cpath, 'w') as f:
            f.write(open(src_path).read())
            f.write(driver)
        cc = subprocess.run(['g++', '-std=c++11', '-O2', '-Wall', '-Wextra',
                             '-DIKBT_CHECK_DRIVER', cpath, '-o', bpath, '-lm'],
                            capture_output=True, text=True)
    finally:
        if os.path.exists(cpath):
            os.remove(cpath)

    if cc.returncode != 0:
        first = next((l for l in cc.stderr.split('\n') if 'error:' in l), '')
        return None, 'compile: %s' % first.split('error: ')[-1][:80]
    return bpath, ''


def _feed(bpath, T):
    stdin = '\n'.join(' '.join('%.17g' % T[i][j] for j in range(4))
                       for i in range(4))
    r = subprocess.run([bpath], input=stdin, capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError('the compiled module exited %d' % r.returncode)
    lines = [l for l in r.stdout.split('\n') if l.strip()]
    n, ndof = (int(x) for x in lines[0].split())
    return [[float(x) for x in l.split()] for l in lines[1:1 + n]], ndof


def _match_sets(a, b, tol=1e-6):
    """(matched, only_in_a, only_in_b) between two lists of joint vectors."""
    used = set()
    matched = 0
    only_a = []
    for qa in a:
        hit = None
        for k, qb in enumerate(b):
            if k in used:
                continue
            if len(qa) == len(qb) and \
                    max(abs(x - y) for x, y in zip(qa, qb)) < tol:
                hit = k
                break
        if hit is None:
            only_a.append(qa)
        else:
            used.add(hit)
            matched += 1
    only_b = [q for k, q in enumerate(b) if k not in used]
    return matched, only_a, only_b


def check_onevar(name, n_poses=4, seed=17, verbose=False):
    """Same poses, same root set?  Returns (matched, missing, extra, worst, note)."""

    import importlib.util
    import random

    tmp = tempfile.mkdtemp(prefix='ikbt_ov_')
    ident = cpp_identifier(name)
    try:
        bpath, note = _build_appended(
            os.path.join(CPP_DIR, 'IK_onevar%s.cpp' % name),
            ONEVAR_DRIVER.replace('**IDENT**', ident), tmp, ident)
        if bpath is None:
            return 0, 0, 0, None, note

        ppath = os.path.join('CodeGen', 'Python', 'IK_onevar%s.py' % name)
        if not os.path.exists(ppath):
            return 0, 0, 0, None, 'no python twin at %s' % ppath
        spec = importlib.util.spec_from_file_location('ov_' + ident, ppath)
        mod = importlib.util.module_from_spec(spec)
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            spec.loader.exec_module(mod)
        py_solve = getattr(mod, 'solve_' + ident)
        py_fk = mod.true_fk.__dict__['fk_' + ident]
        ndof = mod.NDOF

        rng = random.Random(seed)
        matched = missing = extra = 0
        worst = 0.0
        for _ in range(n_poses):
            q = [rng.uniform(-1.0, 1.0) for _ in range(ndof)]
            T = np.asarray(py_fk(q), dtype=float)
            crows, _ = _feed(bpath, T)
            cqs = [r[3:] for r in crows]
            with contextlib.redirect_stdout(io.StringIO()):
                pqs = [list(s['q']) for s in py_solve(T)]
            m, only_c, only_p = _match_sets(cqs, pqs)
            matched += m
            missing += len(only_p)          # python found it, C++ did not
            extra += len(only_c)
            #  soundness of what C++ returned, independent of python
            for qq in cqs:
                worst = max(worst, float(np.max(np.abs(
                    np.asarray(py_fk(qq), dtype=float) - T))))
            if verbose:
                print('      pose: C++ %d, python %d, matched %d'
                      % (len(cqs), len(pqs), m))
        return matched, missing, extra, worst, ''
    except Exception as e:
        return 0, 0, 0, None, '%s: %s' % (type(e).__name__, str(e)[:70])
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def check_hybrid(name, n_poses=3, seed=19, verbose=False):
    """Same poses, same refined postures?  Returns (matched, missing, extra,
       worst, note) -- worst is the TRUE arm's pose error at the C++ answers."""

    import importlib.util
    import random

    tmp = tempfile.mkdtemp(prefix='ikbt_hy_')
    ident = cpp_identifier(name)
    try:
        bpath, note = _build_appended(
            os.path.join(CPP_DIR, 'IK_hybrid_%s.cpp' % name),
            HYBRID_DRIVER.replace('**IDENT**', ident), tmp, ident)
        if bpath is None:
            return 0, 0, 0, None, note

        ppath = os.path.join('CodeGen', 'Python', 'IK_hybrid_%s.py' % name)
        if not os.path.exists(ppath):
            return 0, 0, 0, None, 'no python twin at %s' % ppath
        spec = importlib.util.spec_from_file_location('hy_' + ident, ppath)
        mod = importlib.util.module_from_spec(spec)
        with contextlib.redirect_stdout(io.StringIO()):
            spec.loader.exec_module(mod)
        py_refine_all = getattr(mod, 'refine_all_' + ident)
        py_fk = mod.true_fk.__dict__['fk_' + ident]
        ndof = mod.NDOF

        rng = random.Random(seed)
        matched = missing = extra = 0
        worst = 0.0
        for _ in range(n_poses):
            q = [rng.uniform(-1.0, 1.0) for _ in range(ndof)]
            T = np.asarray(py_fk(q), dtype=float)
            crows, _ = _feed(bpath, T)
            #  CONVERGED BRANCHES ONLY, both sides.  A branch that did not
            #  converge is not an answer, and the true arm genuinely fails to
            #  reach some poses in some postures -- comparing those would be
            #  comparing two records of the same non-answer.
            cqs = [r[3:] for r in crows if r[2] > 0.5]
            with contextlib.redirect_stdout(io.StringIO()):
                pres = py_refine_all(T)
            pqs = [list(r['q']) for r in pres if r['converged']]
            m, only_c, only_p = _match_sets(cqs, pqs)
            matched += m
            missing += len(only_p)
            extra += len(only_c)
            for qq in cqs:
                worst = max(worst, float(np.max(np.abs(
                    np.asarray(py_fk(qq), dtype=float) - T))))
            if verbose:
                print('      pose: C++ %d converged, python %d, matched %d'
                      % (len(cqs), len(pqs), m))
        return matched, missing, extra, worst, ''
    except Exception as e:
        return 0, 0, 0, None, '%s: %s' % (type(e).__name__, str(e)[:70])
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('robots', nargs='*', help='robot names (default: the '
                                              'robots scripts/expected.py judges)')
    ap.add_argument('--all', action='store_true', help='every robot in ROBOT_LIST')
    ap.add_argument('--keep', action='store_true', help='do not re-solve')
    ap.add_argument('--compile-only', action='store_true',
                    help='the compile gate:  build, do not run')
    ap.add_argument('--fk', action='store_true',
                    help='check the FK/Jacobian emitter elementwise instead')
    ap.add_argument('--dls', action='store_true',
                    help='pin Cpp_src/ikbt_dls.h against numeric_ik instead')
    ap.add_argument('--onevar', action='store_true',
                    help='one-variable path:  same root set as the python?')
    ap.add_argument('--hybrid', action='store_true',
                    help='hybrid path:  same refined postures as the python?')
    ap.add_argument('--gate', action='store_true',
                    help='exit 1 on any unexpected result')
    ap.add_argument('-v', '--verbose', action='store_true')
    args = ap.parse_args()

    if shutil.which('g++') is None:
        print('no g++ on PATH -- skipping the generated-C++ check')
        return 0

    if args.robots:
        names = args.robots
    elif args.all:
        from ikbtfunctions.ik_robots import ROBOT_LIST
        names = list(ROBOT_LIST)
    else:
        names = DEFAULT_ROBOTS

    if args.onevar or args.hybrid:
        which = 'one-variable' if args.onevar else 'hybrid'
        fn = check_onevar if args.onevar else check_hybrid
        print('%-18s %-9s %-9s %-9s %-12s %s'
              % ('robot', 'matched', 'missing', 'extra', 'worst FK', 'note'),
              flush=True)
        print('-' * 78, flush=True)
        bad = []
        for n in names:
            m, miss, ex, worst, note = fn(n, verbose=args.verbose)
            print('%-18s %-9d %-9d %-9d %-12s %s'
                  % (n, m, miss, ex,
                     'n/a' if worst is None else '%.2e' % worst, note),
                  flush=True)
            if note:
                bad.append('%s: %s' % (n, note))
                continue
            if miss:
                bad.append('%s: the C++ %s search missed %d solution(s) the '
                           'python found' % (n, which, miss))
            if ex:
                bad.append('%s: the C++ %s search returned %d solution(s) the '
                           'python did not' % (n, which, ex))
            if worst is not None and worst > TOL:
                bad.append('%s: a returned branch is %.2e from the goal pose'
                           % (n, worst))
        print('-' * 78, flush=True)
        if bad:
            print('\n%d problem(s):' % len(bad))
            for x in bad:
                print('   ', x)
        else:
            print('\nno problems')
        return 1 if (bad and args.gate) else 0

    if args.dls:
        print('%-18s %-12s %-12s %-10s %s'
              % ('robot', 'max |dq|', 'max |dmetric|', 'iter diff', 'note'),
              flush=True)
        print('-' * 74, flush=True)
        bad = []
        for n in names:
            wq, wm, ib, note = check_dls(n, verbose=args.verbose)
            print('%-18s %-12s %-12s %-10s %s'
                  % (n,
                     'n/a' if wq is None else '%.2e' % wq,
                     'n/a' if wm is None else '%.2e' % wm,
                     'n/a' if ib is None else str(ib),
                     note), flush=True)
            if note:
                bad.append('%s: %s' % (n, note))
            else:
                if wq > 1e-9:
                    bad.append('%s: refined q differs by %.2e' % (n, wq))
                if ib:
                    bad.append('%s: %d of the trials took a different number '
                               'of iterations' % (n, ib))
        print('-' * 74, flush=True)
        if bad:
            print('\n%d problem(s):' % len(bad))
            for x in bad:
                print('   ', x)
        else:
            print('\nno problems')
        return 1 if (bad and args.gate) else 0

    if args.fk:
        #  Generates its own artifacts into a temp dir, so it never needs a
        #  solve and never touches CodeGen/.
        print('%-18s %-14s %-14s %s'
              % ('robot', 'max |dT|', 'max |dJ|', 'note'), flush=True)
        print('-' * 66, flush=True)
        bad = []
        for n in names:
            wT, wJ, note = check_fk(n, verbose=args.verbose)
            print('%-18s %-14s %-14s %s'
                  % (n,
                     'n/a' if wT is None else '%.2e' % wT,
                     'n/a' if wJ is None else '%.2e' % wJ,
                     note), flush=True)
            if note:
                bad.append('%s: %s' % (n, note))
            else:
                if wT > AGREE_TOL:
                    bad.append('%s: FK differs by %.2e' % (n, wT))
                if wJ is not None and wJ > AGREE_TOL:
                    bad.append('%s: Jacobian differs by %.2e' % (n, wJ))
        print('-' * 66, flush=True)
        if bad:
            print('\n%d problem(s):' % len(bad))
            for x in bad:
                print('   ', x)
        else:
            print('\nno problems')
        return 1 if (bad and args.gate) else 0

    if not args.keep:
        for n in names:
            print('solving %s ...' % n, flush=True)
            ok, msg = solve_robot(n)
            if not ok:
                print('   %s' % msg, flush=True)

    print('')
    print('%-18s %-11s %-22s %s'
          % ('robot', 'compile', 'branches reproducing T', 'python agreement'))
    print('-' * 86)

    failures = []
    worst_agree = 0.0
    for n in names:
        if args.compile_only:
            _, kind, msg = build(n)
            print('%-18s %-11s %s' % (n, kind, msg), flush=True)
            if kind == 'other':
                failures.append('%s: compile failed -- %s' % (n, msg))
            elif kind == 'undeclared' and n not in UNCHECKABLE:
                failures.append('%s: undeclared name, and it is not a known '
                                'UNCHECKABLE robot -- %s' % (n, msg))
            continue

        r = check_one(n, verbose=args.verbose)
        if r['kind'] != 'ok':
            print('%-18s %-11s %s' % (n, r['kind'], r['note']), flush=True)
            if r['kind'] == 'other':
                failures.append('%s: compile failed -- %s' % (n, r['note']))
            elif r['kind'] == 'undeclared' and n not in UNCHECKABLE:
                failures.append('%s: undeclared name, not a known defect -- %s'
                                % (n, r['note']))
            continue

        agree = r['agree']
        if isinstance(agree, float):
            worst_agree = max(worst_agree, agree)
            atxt = '%.2e %s' % (agree, 'ok' if agree <= AGREE_TOL else 'DISAGREE')
            if agree > AGREE_TOL:
                failures.append('%s: C++ and python differ by %.2e' % (n, agree))
        else:
            atxt = str(agree)
            if n not in UNCHECKABLE:
                failures.append('%s: could not compare with python -- %s'
                                % (n, agree))

        cnt = '%d of %d' % (r['good'], r['total'])
        if r['worst_fk'] is not None:
            cnt += '  (worst %.1e)' % r['worst_fk']
        print('%-18s %-11s %-22s %s' % (n, 'ok', cnt, atxt), flush=True)

        for complaint in judge_counts(n, r['good'], r['total']):
            failures.append('%s: %s' % (n, complaint))
            print('    ! %s' % complaint)

    print('-' * 86)
    if not args.compile_only:
        print('worst python/C++ disagreement over the run: %.2e' % worst_agree)
    if failures:
        print('\n%d problem(s):' % len(failures))
        for x in failures:
            print('   ', x)
    else:
        print('\nno problems')

    return 1 if (failures and args.gate) else 0


if __name__ == '__main__':
    sys.exit(main())
