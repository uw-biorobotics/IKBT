#!/usr/bin/python
#
#   eqn_sanity.py --  does this equation actually CONSTRAIN the variable?
#
#   An ID node matches on syntax.  It sees sin(th_2) and cos(th_2) in an
#   equation and offers it as an equation in th_2.  Nothing checks whether the
#   equation says anything about th_2 at all -- and sometimes it does not.
#
#   MEASURED ON UR5 (2026-08-28).  The equation IKBT used to solve th_2 is
#
#       0 = A*sin(th_2) + B*cos(th_2)
#
#       A = -r_31*s5*c6 + r_32*s5*s6 - r_33*c5
#       B = -(r_11*c1 + r_21*s1)*s5*c6 + (r_12*c1 + r_22*s1)*s5*s6
#           - (r_13*c1 + r_23*s1)*c5
#
#   and substituting UR5's own symbolic T_06 gives A == 0 and B == 0
#   IDENTICALLY -- proved with a_2, a_3, d_1, d_4, d_5, d_6 left as free
#   symbols, so it holds for every parameter value and every pose.  th_2 does
#   not appear in the coefficients at all;  the equation is
#
#       0*sin(th_2) + 0*cos(th_2) = 0
#
#   which is satisfied by every value of th_2.  sinANDcos then evaluated
#   atan2(-B, A) = atan2(0, 0) and reported an answer.  Over the 8 solution
#   versions th_2 came back as 0.000, 3.142, 2.614, 5.773 ... against a true
#   -0.600, and NOT ONE of the 8 versions reproduced the pose.  The robot was
#   recorded as "solved 9/9" throughout, because nothing checked.
#
#   WHY A SYMBOLIC TEST CANNOT CATCH IT.  A and B are not zero as expressions
#   -- they are zero only on the manifold where the r_ij come from this robot's
#   forward kinematics.  No amount of simplify() on the equation alone will
#   see it.  sinANDcos_solver's existing guard, assert(A*A + B*B != 0), is a
#   sympy expression compared to 0 and can never fire for this reason.
#
#   SO THE TEST IS NUMERIC, and self-contained:  sample a joint vector q,
#   build T = FK(q) from the robot's own kinematics, substitute that pose and
#   the true values of every OTHER variable, then sweep the target.  A real
#   equation is near zero only at the true value;  a tautology is near zero
#   everywhere.  Generating the probe pose ourselves is what makes the "true
#   values of the other variables" available -- this is not knowledge of the
#   caller's target pose, and nothing here reads one.
#
#   FAIL-SAFE, ALWAYS.  Every failure path returns True ("it constrains"), so
#   a robot whose pvals will not resolve, or an expression lambdify cannot
#   build, behaves exactly as it did before this module existed.  A sanity
#   check must never be the reason a solve is lost.
#
#   Copyright 2026 University of Washington
#
#   Developed by Blake Hannaford
#   BioRobotics Lab, University of Washington

import numpy as np
import sympy as sp

import ikbtbasics.numeric_ik as nik

#  Sweep points for the target variable.  Deliberately not symmetric about 0
#  and not a multiple of pi/2:  a real equation that happens to vanish at one
#  probe point will not vanish at all five, while a tautology vanishes at every
#  point whatever they are.
SAMPLES = (-2.3, -0.7, 0.0, 1.1, 2.9)

TOL = 1e-9

#  Probe poses per mechanism, and verdicts per (equation, variable).  Both are
#  pure functions of the robot, so caching is safe;  the verdict cache is what
#  keeps the check off the hot path -- the same equation is re-offered on every
#  tick until something solves it.
_probe_cache = {}
_verdict_cache = {}


def _probes(R, n=2, seed=17):
    '''n (pose, true-values) pairs generated from the robot's own FK.'''

    key = id(R.Mech)
    if key in _probe_cache:
        return _probe_cache[key]

    M = R.Mech
    pv = nik.pvals_numeric(M)
    jsyms = [str(s) for s in nik.joint_symbols(M)]
    fk = nik.fk_callable(M)

    rng = np.random.default_rng(seed)
    out = []
    for _ in range(n):
        q = rng.uniform(-np.pi, np.pi, len(jsyms))
        T = np.asarray(fk(q), dtype=float)

        sub = dict(pv)
        for i in range(3):
            for j in range(3):
                sub[sp.Symbol('r_%d%d' % (i + 1, j + 1))] = T[i, j]
        for k, nm in enumerate(('Px', 'Py', 'Pz')):
            sub[sp.Symbol(nm)] = T[k, 3]

        truth = dict(zip(jsyms, [float(x) for x in q]))
        #  Sum-of-angle variables are defined in terms of the joints, and the
        #  definitions can nest (th_234 over th_23), so resolve to a fixpoint.
        for _pass in range(4):
            for aux in getattr(R, 'kequation_aux_list', []):
                try:
                    truth[str(aux.LHS)] = float(sp.N(aux.RHS.subs(
                        {sp.Symbol(a): b for a, b in truth.items()})))
                except (TypeError, AttributeError):
                    pass
        out.append((sub, truth))

    _probe_cache[key] = out
    return out


def constrains(eq, varname, R, tol=TOL):
    '''True if `eq` says anything at all about `varname`.

       False ONLY when the residual is flat across the whole sweep on every
       probe pose -- i.e. the equation is satisfied for every value of the
       variable and determines nothing.  Every error path returns True.'''

    if eq is None or not varname:
        return True

    try:
        resid = sp.sympify(eq.RHS) - sp.sympify(eq.LHS)
    except Exception:
        return True

    if not resid.has(sp.Symbol(varname)):
        #  The variable is not even in the expression.  Not this module's
        #  business -- the ID node should not have offered it, but saying
        #  "constrains nothing" here would change behaviour beyond the defect
        #  this module exists for.
        return True

    key = (sp.srepr(resid), varname)
    if key in _verdict_cache:
        return _verdict_cache[key]

    verdict = True
    try:
        flat_everywhere = True
        for pose, truth in _probes(R):
            sub = dict(pose)
            for nm, val in truth.items():
                if nm != varname:
                    sub[sp.Symbol(nm)] = val
            f = sp.lambdify(sp.Symbol(varname), resid.subs(sub), 'numpy')
            if max(abs(complex(f(x))) for x in SAMPLES) > tol:
                flat_everywhere = False
                break
        verdict = not flat_everywhere
    except Exception:
        verdict = True

    _verdict_cache[key] = verdict
    return verdict


def reject(u, reason_leaf=''):
    '''Undo the ID node's offer so the solve node can FAIL cleanly.

       The ID node set readytosolve / eqntosolve / solvemethod before this leaf
       ran.  Leaving them set would have the next leaf in the Priority pick up
       the same dead equation, and would leave a solvemethod string on a
       variable that was never solved.'''

    print('  %s: the equation for %s constrains nothing -- refusing it'
          % (reason_leaf or 'eqn_sanity', u.name))
    u.readytosolve = False
    u.eqntosolve = None
    u.secondeqn = None
    u.solvemethod = ''
    u.solvable_tan = False
