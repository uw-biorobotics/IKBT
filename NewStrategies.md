# Candidate New Solution Strategies for IKBT

Notes on solution techniques *not* currently exploited by the IKBT behavior tree, with an
assessment of what each would cost to add.

Written August 2026, after review of the JAIR 2019 paper and `IKdocs/solExamp.pdf`.

## The structural gap in the current toolbox

The six solvers described in JAIR §3.2 (algebraic, sin-or-cos, tangent, sinANDcos,
simultaneous-equation, x²y²) all share a precondition: **the target unknown must already be
isolated in a conveniently-shaped equation.** The system recognizes solvable forms; it does not
manufacture them.

Evidence in the code:

- `algebra_solver` operates purely on `eqns_1u`. It fetches `eqns_2u` only to print a debug
  count (`ikbtleaves/algebra_solver.py:106`).
- The two "clever" leaves are *cancellation* tricks, not elimination. `tan_solver` divides a
  sin/cos pair so that a shared unsolved factor drops out. `x2y2_transform` squares and adds so
  that unsolved terms cancel through `s² + c² = 1`. Both require the nuisance variable to
  disappear by luck of form.
- There is no polynomial machinery anywhere in the repository — no `Poly`, `resultant`, `roots`,
  `groebner`, or `sp.solve`. All identification is `sp.Wild` template matching against sympy
  expression trees.

So IKBT has no mechanism to **eliminate** a variable between two genuinely coupled equations.
That is the hole all three candidates below aim at, from different directions.

### Incidental finding

`simu_id.tick()` begins with an unconditional `return b3.FAILURE`
(`ikbtleaves/two_eqn_m7.py:46`), with the comment *"temporarily block this and see what
breaks!!!!"*. Paper-solver #5 is therefore dead in the tree at present, although it is still
wired into the `worktools` Priority list in `ikSolver.py`. Worth resolving before measuring the
effect of anything new.

---

## Candidate 1 — Weierstrass substitution + resultant elimination

Substitute `t = tan(θ/2)`, giving

```
sin θ = 2t / (1 + t²)        cos θ = (1 - t²) / (1 + t²)
```

Any equation polynomial in `{sin θ, cos θ}` becomes polynomial in `t` once denominators are
cleared. Two modes of use:

**One unknown, one equation.** Forms such as

```
a·sin θ·cos θ + b·cos²θ + c·sin θ + d = 0
```

arise routinely after sum-of-angle substitution or after squaring, and match *none* of the six
existing templates. Under Weierstrass this is a quartic in `t`; `sp.roots` returns radicals and
`θ = 2·arctan(t)`.

**Two unknowns, two equations.** Substitute both, clear denominators to get bivariate
polynomials `p1(t_i, t_j)`, `p2(t_i, t_j)`, then `sp.resultant(p1, p2, t_j)` eliminates `t_j`
and leaves a univariate polynomial in `t_i`. This is dialytic elimination performed
symbolically. It would be the first leaf to consume `eqns_2u` for genuine elimination.

### Why this does not contradict JAIR §1.1

The published objection to Manocha & Canny was that their method *goes numerical* — eigenvalue
computation in the critical step — so the result is not closed form. A resultant of degree ≤ 4
solved by radicals never leaves the symbolic domain. The technique is standard expert knowledge
rather than a configuration-specific trick, so it remains consistent with the stated design
goals.

### Issues to handle

- **Degree > 4 ⇒ return FAILURE.** By Abel–Ruffini there is no general radical solution. This is
  useful output the system cannot currently produce: a positive statement that no closed form is
  reachable from this equation pair, as distinct from today's silent failure to pattern-match.
- **The θ = π pole.** `t → ∞` at θ = π. When the leading coefficient of the polynomial vanishes,
  that root exists and must be re-added explicitly, or a pose is silently lost.
- **This breaks the `n_s ∈ {1,2}` assumption.** A quartic yields `n_s = 4`. That is exactly the
  open issue flagged in the footnote of `IKdocs/solExamp.pdf` — the row-flipping rule in
  `Robot.create_solution_set()` is only worked out for one and two solutions. Generalizing it to
  a proper Cartesian product with a canonical ordering is a prerequisite. It is self-contained
  and worth doing on its own merits.

### Practical gating

Quadratics are clean and cover a large fraction of real cases. Cubic and quartic radicals are
enormous. Suggested approach: gate on `sp.count_ops` of the resultant, treat degree ≤ 2 as the
primary target, and put degree 3–4 behind a flag. `sp.factor` and `cse` help, as does promoting
recurring subexpressions to named constants (the LaTeX and codegen paths already support this
through `params`).

### Placement in the tree

As a fallback after the cheap pattern matchers have all failed:

```python
worktools = b3.Priority([algSol, sc_tan, Simu_Eqn_Sol, sacSol, x2z2_Solver, weierstrassSol])
```

---

## Candidate 2 — Paden–Kahan geometric subproblems

A different *kind* of knowledge rather than more algebra. Reformulate with product-of-
exponentials,

```
T = exp(ξ̂₁θ₁) · exp(ξ̂₂θ₂) · ... · exp(ξ̂₆θ₆) · g(0)
```

with twist coordinates derived from the DH table already parsed by the front end. The solvers
then become geometric:

- **Subproblem 1** — rotation about one axis carrying point `p` to point `q`. One solution.
- **Subproblem 2** — rotation about two consecutive intersecting axes. Zero, one, or two.
- **Subproblem 3** — rotation about an axis achieving a specified distance. Zero, one, or two.

The real power is the accompanying *cancellation rule*: applying both sides of the kinematic
equation to a point lying on a joint axis annihilates every joint whose axis passes through that
point. This is the geometric analog of the existing substitution transform, but considerably
stronger, and it is how human experts actually crack arms by hand.

**Payoff.** The failure mode is the opposite of Candidate 1: solutions emerge compact and
geometrically interpretable, with no expression swell. This directly serves the human-readability
goal of the paper. Compare the θ₅ and θ₆ PUMA solutions on pp. 476–477 — those pages are what
algebraic accumulation costs.

**Cost.** A second kinematic representation must be maintained, and the leaves would speak in
points, axes, and distances rather than the scalar `kequation` blackboard that every current leaf
shares. This is the architecturally expensive option of the three.

---

## Candidate 3 — Kinematic invariant generator (cheapest)

Generalize `x2y2_transform` from a pattern-matched trick into a systematic generator. From each
matrix equation, emit as new scalar equations:

- `‖P‖²`
- `trace(R)`
- pairwise column dot products of the rotation block

Each of these is invariant under the trailing rotations, so each typically arrives carrying fewer
unknowns than the raw element equations. This is Pieper's classic opening move, and it is how the
position/orientation decoupling gets found for arms that admit it.

**Why it is low risk.** It is a pure equation *generator* feeding the existing solvers. It
requires no new solution algebra, produces no new solution multiplicities, and therefore does not
disturb the `n_s ≤ 2` machinery. Good stepping stone if Candidate 1's solution-set generalization
needs to land first.

---

## Suggested ordering

1. Fix `create_solution_set()` for `n_s > 2` (prerequisite for Candidate 1, independently
   valuable, and already a known open issue).
2. Candidate 3 — cheap, self-contained, immediately testable against currently-failing robots.
3. Candidate 1 — the largest capability gain; start with the one-unknown mode before attempting
   two-unknown resultants.
4. Candidate 2 — a research project in its own right, not an incremental leaf.
