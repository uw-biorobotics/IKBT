#!/usr/bin/python
#
#   cpp_expr_check.py --  does the C++ printer agree with the python printer?
#
#       python3 -m scripts.cpp_expr_check           # the standing set
#       python3 -m scripts.cpp_expr_check --robot Puma   # ... plus Puma's own
#                                                        #     solution equations
#       python3 -m scripts.cpp_expr_check -v        # print every comparison
#
#   THE BOTTOM OF THE C++ GENERATOR.  Everything above it -- the IK equations,
#   the FK, the Jacobian, the search -- is sympy expressions printed into a
#   file, so if expr_cpp() and expr_py() disagree about one expression then
#   every artifact built on them is wrong in a way no higher-level check can
#   attribute.  This compares them directly: the same expression, the same
#   random substitutions, evaluated by python and by a compiled C++ program,
#   required to agree to 1e-13 relative.
#
#   SEEDED WITH THE EXPRESSIONS THAT BROKE THE OLD GENERATOR.  `(Px - a_1)**2`
#   is the one that shipped as invalid C++ in Arm_3 and UR5; `pi` is the one
#   that shipped as 3.1415926, eight digits, injecting 3.6e-8 into every
#   solution that contained it.
#
#   Skips cleanly with no g++, the way write_latex_fitted() degrades with no
#   pdflatex:  a missing compiler must not fail a test run.
#
#   Copyright 2026 University of Washington
#
#   Developed by Blake Hannaford
#   BioRobotics Lab, University of Washington

import argparse
import math
import os
import random
import shutil
import subprocess
import sys
import tempfile

import sympy as sp

from ikbtfunctions.output_cpp_common import expr_cpp
from ikbtfunctions.output_numeric_common import expr_py


TOL = 1e-13          # relative;  both sides are IEEE double doing the same ops
N_TRIALS = 12        # random substitutions per expression


def standing_set():
    '''Expressions the generator has to get right, and why each one is here.'''

    Px, Py, Pz, a_1, a_2, d_4, x, y = sp.symbols(
        'Px Py Pz a_1 a_2 d_4 x y')
    th_1, th_2 = sp.symbols('th_1 th_2')

    return [
        #  CASES WHERE PYTHON'S SPELLING IS NOT VALID C++, which is where
        #  an expression printer earns its keep.
        ('compound base squared',      (Px - a_1) ** 2),
        ('trig squared',               sp.sin(th_1) ** 2),
        ('nested compound',            (Px - a_1 * sp.cos(th_1)) ** 2),
        ('cube',                       (Px + a_1) ** 3),
        ('sum of squares under sqrt',  sp.sqrt((Px - a_1) ** 2 + Py ** 2)),

        #  pi at full precision, not 3.1415926.
        ('bare pi',                    sp.pi + x),
        ('pi over two',                sp.atan2(y, x) + sp.pi / 2),
        ('pi over four',               sp.pi / 4 * x),

        #  Rationals:  integer division would make these zero.
        ('rational coefficient',       x / 3),
        ('rational literal',           sp.Rational(1, 2) * x + sp.Rational(2, 7)),

        #  The shapes the solver actually produces.
        ('atan2 pair',                 sp.atan2(Px - a_1, Py + a_2)),
        ('acos of a ratio',            sp.acos((Px ** 2 + Py ** 2 - a_1 ** 2
                                                - a_2 ** 2) / (2 * a_1 * a_2))),
        ('negated acos',               -sp.acos(x / 2)),
        ('asin',                       sp.asin(Pz / d_4)),
        ('abs',                        sp.Abs(Px - a_1)),
        ('reciprocal',                 Px / sp.cos(th_2)),
        ('negative power',             (Px + 2) ** -2),
        ('long sum',                   Px * sp.sin(th_1) * sp.cos(th_2)
                                       - Py * sp.cos(th_1) + Pz * a_2
                                       - d_4 * sp.sin(th_2) + a_1),
        ('deep nest',                  sp.atan2(sp.sqrt(sp.Abs(1 - (Px / d_4) ** 2)),
                                                Px / d_4) + sp.pi / 2),
    ]


def robot_equations(name, limit=40):
    '''Solution equations from a real solve, if one has been run.

       The standing set is hand-picked and therefore biased towards what was
       already known to break.  A robot's own FinalEqnMatrix is not.'''

    import io
    import contextlib

    from ikbtfunctions.ik_driver import load_robot, run_solver
    from ikbtfunctions.bt_assembly import build_default_bt

    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        M, R, unks = load_robot(name)
        R, unks, bb = run_solver(R, unks, build_default_bt())

    out = []
    for row in getattr(R, 'FinalEqnMatrix', []):
        for e in row:
            out.append(('%s: %s' % (name, e.LHS), e.RHS))
            if len(out) >= limit:
                return out
    return out


#  Everything expr_py() may emit, in a namespace eval() can use.  Mirrors the
#  imports at the top of every generated python module.
PY_ENV = {'sin': math.sin, 'cos': math.cos, 'tan': math.tan,
          'asin': math.asin, 'acos': math.acos, 'atan': math.atan,
          'atan2': math.atan2, 'sqrt': math.sqrt, 'fabs': math.fabs,
          'abs': abs, 'exp': math.exp, 'log': math.log, 'pi': math.pi,
          'math': math}


def draw(sym, rng):
    '''A value for one symbol that keeps the standing set in domain.

       Small and away from zero:  an acos argument has to land in [-1, 1] and
       a denominator must not vanish, and a NaN on one side proves nothing
       about the other.'''

    v = rng.uniform(0.3, 1.2)
    return v if rng.random() < 0.5 else -v


def build_cases(exprs, rng):
    '''[(label, expr, subs, python_value)] for every case that python can do.

       A case python cannot evaluate -- out of domain for the draw, or an
       expression expr_py() refuses -- is DROPPED, not failed.  The question
       here is whether the two printers agree, not whether a random draw
       happened to be reachable.'''

    cases = []
    for label, e in exprs:
        syms = sorted(e.free_symbols, key=str)
        try:
            psrc = expr_py(e)
            csrc = expr_cpp(e)
        except ValueError as err:
            print('  skip  %-24s  %s' % (label, err))
            continue
        for _ in range(N_TRIALS):
            subs = {str(s): draw(s, rng) for s in syms}
            env = dict(PY_ENV)
            env.update(subs)
            try:
                val = float(eval(psrc, {'__builtins__': {}}, env))
            except Exception:
                continue             # out of domain for this draw
            if not math.isfinite(val):
                continue
            cases.append((label, csrc, subs, val))
    return cases


def cpp_program(cases):
    '''One translation unit that prints every case's value at 17 digits.'''

    from ikbtfunctions.output_cpp_common import read_src

    out = [read_src('ikbt_types.h'),
           '',
           '#include <cstdio>',
           '',
           'using namespace ikbt;',
           '',
           'int main()',
           '{']
    for i, (label, csrc, subs, _) in enumerate(cases):
        out.append('    {')
        out.append('        //  %s' % label.replace('\n', ' ')[:70])
        for nm, v in sorted(subs.items()):
            out.append('        const double %s = %.17g;' % (nm, v))
        out.append('        const double value = %s;' % csrc)
        out.append('        std::printf("%d %%.17g\\n", value);' % i)
        out.append('    }')
    out.append('    return 0;')
    out.append('}')
    return '\n'.join(out)


def run(exprs, verbose=False, seed=7):
    rng = random.Random(seed)
    cases = build_cases(exprs, rng)
    if not cases:
        print('no cases to check')
        return 1

    src = cpp_program(cases)
    tmp = tempfile.mkdtemp(prefix='ikbt_expr_')
    cpath = os.path.join(tmp, 'expr_check.cpp')
    bpath = os.path.join(tmp, 'expr_check')
    with open(cpath, 'w') as f:
        f.write(src)

    cc = subprocess.run(['g++', '-std=c++11', '-O2', '-Wall', '-Wextra',
                         cpath, '-o', bpath, '-lm'],
                        capture_output=True, text=True)
    if cc.returncode != 0:
        print('COMPILE FAILED -- the generated C++ is not valid C++')
        print(cc.stderr[:3000])
        print('\nsource kept at', cpath)
        return 1
    if cc.stderr.strip():
        print('compiler warnings:\n' + cc.stderr[:2000])

    r = subprocess.run([bpath], capture_output=True, text=True)
    if r.returncode != 0:
        print('the compiled check crashed (exit %d)' % r.returncode)
        return 1

    got = {}
    for line in r.stdout.split('\n'):
        if not line.strip():
            continue
        i, v = line.split()
        got[int(i)] = float(v)

    bad = 0
    worst = 0.0
    worst_label = ''
    for i, (label, csrc, subs, pyval) in enumerate(cases):
        if i not in got:
            print('FAIL  %-24s  no value came back' % label)
            bad += 1
            continue
        cppval = got[i]
        scale = max(1.0, abs(pyval))
        rel = abs(cppval - pyval) / scale
        if rel > worst:
            worst, worst_label = rel, label
        ok = rel <= TOL
        if not ok:
            bad += 1
        if verbose or not ok:
            print('%-5s %-24s  py %+.17g   cpp %+.17g   rel %.2g'
                  % ('ok' if ok else 'FAIL', label, pyval, cppval, rel))

    n_expr = len({c[0] for c in cases})
    print('\n%d expressions, %d evaluations, %d disagreements  '
          '(worst relative %.2g, on %s)'
          % (n_expr, len(cases), bad, worst, worst_label))
    if bad == 0:
        shutil.rmtree(tmp, ignore_errors=True)
    else:
        print('source kept at', cpath)
    return 1 if bad else 0


###############################################################################
#
#    Out-of-domain behaviour
#

#  BOTH LANGUAGES MUST GIVE NaN.  An arccosine whose argument leaves [-1, 1]
#  must yield NaN and propagate it, in BOTH languages -- that is what lets the
#  generated code decide reachability at the end instead of guarding at every
#  arcsine.  C++ gets it for free (std::acos already returns NaN); python does
#  not (math.acos raises), so output_python emits acos_dc / asin_dc.  Two
#  different mechanisms reaching the same behaviour is exactly the kind of
#  thing that drifts, so it is asserted here rather than assumed.
#
#  The PYTHON side is exercised through the helper source output_python
#  actually emits, lifted out of its importString -- not a copy of it.

DOMAIN_CASES = [
    ('acos just outside',      'acos_dc(1.0000001)',        'std::acos(1.0000001)'),
    ('acos far outside',       'acos_dc(1.1131148539915245)',
                               'std::acos(1.1131148539915245)'),
    ('asin far outside',       'asin_dc(-2.0)',             'std::asin(-2.0)'),
    ('NaN propagates through', 'pi - asin_dc(3.0)',         'M_PI - std::asin(3.0)'),
    ('NaN through a product',  'cos(acos_dc(2.0)) * 5.0',
                               'std::cos(std::acos(2.0)) * 5.0'),
    ('sqrt of a negative',     'sqrt_dc(-202.2)',           'std::sqrt(-202.2)'),
    ('sqrt NaN propagates',    '1.0 + sqrt_dc(-1.0)',       '1.0 + std::sqrt(-1.0)'),
    ('sqrt of zero',           'sqrt_dc(0.0)',              'std::sqrt(0.0)'),
    ('inside the domain',      'acos_dc(0.5)',              'std::acos(0.5)'),
    ('sqrt inside the domain', 'sqrt_dc(2.0)',              'std::sqrt(2.0)'),
    ('exactly at the edge',    'acos_dc(1.0)',              'std::acos(1.0)'),
    ('exactly at -1',          'asin_dc(-1.0)',             'std::asin(-1.0)'),
]


def _emitted_helpers():
    """acos_dc / asin_dc exactly as output_python writes them into a module."""

    import ikbtfunctions.output_python as op

    src = op.importString
    start = src.index('def acos_dc')
    ns = {'acos': math.acos, 'asin': math.asin, 'cos': math.cos,
          'sqrt': math.sqrt, 'pi': math.pi, 'float': float}
    exec(compile(src[start:], '<emitted>', 'exec'), ns)
    return ns


def check_domain_behaviour(verbose=False):
    """Do the two languages agree about an out-of-domain arccosine?"""

    from ikbtfunctions.output_cpp_common import read_src

    ns = _emitted_helpers()

    lines = [read_src('ikbt_types.h'), '', '#include <cstdio>', '',
             'using namespace ikbt;', '', 'int main()', '{']
    for i, (_, _, csrc) in enumerate(DOMAIN_CASES):
        lines.append('    std::printf("%d %%.17g\\n", (double)(%s));' % (i, csrc))
    lines.append('    return 0;')
    lines.append('}')

    tmp = tempfile.mkdtemp(prefix='ikbt_dom_')
    cpath = os.path.join(tmp, 'domain_check.cpp')
    bpath = os.path.join(tmp, 'domain_check')
    with open(cpath, 'w') as f:
        f.write('\n'.join(lines))

    cc = subprocess.run(['g++', '-std=c++11', '-O2', '-Wall', '-Wextra',
                         cpath, '-o', bpath, '-lm'],
                        capture_output=True, text=True)
    if cc.returncode != 0:
        print('COMPILE FAILED in the out-of-domain check')
        print(cc.stderr[:1500])
        return 1

    r = subprocess.run([bpath], capture_output=True, text=True)
    got = {}
    for line in r.stdout.split('\n'):
        if line.strip():
            k, v = line.split()
            got[int(k)] = float(v)

    bad = 0
    print('\n  out-of-domain behaviour  (NaN, not an exception, in both)')
    for i, (label, psrc, csrc) in enumerate(DOMAIN_CASES):
        try:
            pv = float(eval(psrc, {'__builtins__': {}}, ns))
            praised = None
        except Exception as e:
            pv, praised = None, '%s: %s' % (type(e).__name__, str(e)[:40])
        cv = got.get(i)

        if praised:
            ok = False
            note = 'python RAISED (%s)' % praised
        elif cv is None:
            ok = False
            note = 'no C++ value'
        elif math.isnan(pv) and math.isnan(cv):
            ok = True
            note = 'both NaN'
        elif math.isnan(pv) != math.isnan(cv):
            ok = False
            note = 'python %s, C++ %s' % (pv, cv)
        else:
            ok = abs(pv - cv) <= 1e-15 * max(1.0, abs(pv))
            note = 'both %.17g' % pv if ok else 'py %.17g vs cpp %.17g' % (pv, cv)
        if not ok:
            bad += 1
        if verbose or not ok:
            print('  %-5s %-24s %s' % ('ok' if ok else 'FAIL', label, note))

    print('  %d case(s), %d disagreement(s)' % (len(DOMAIN_CASES), bad))
    shutil.rmtree(tmp, ignore_errors=True)
    return 1 if bad else 0


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--robot', help='also check this robot\'s own solution '
                                    'equations (runs a solve)')
    ap.add_argument('-v', '--verbose', action='store_true')
    args = ap.parse_args()

    if shutil.which('g++') is None:
        print('no g++ on PATH -- skipping the C++ expression check')
        return 0

    exprs = standing_set()
    if args.robot:
        print('solving %s to harvest its equations ...' % args.robot)
        exprs = exprs + robot_equations(args.robot)

    rc = run(exprs, verbose=args.verbose)
    rc |= check_domain_behaviour(verbose=args.verbose)
    return rc


if __name__ == '__main__':
    sys.exit(main())
