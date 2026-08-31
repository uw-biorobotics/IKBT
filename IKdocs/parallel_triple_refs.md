# Three Parallel Consecutive Axes — method, references, and what IKBT was missing

Background reading for `ikbtleaves/parallel_triple.py`, the transform leaf gated on the
**parallel** branch of the Pieper detector (`ikbtbasics/dh_analysis.py`).

## Why this case is its own thing

Pieper's condition is satisfied by three consecutive axes that **intersect at a point** *or* that are
**mutually parallel** — parallel axes being the limiting case where the intersection point recedes to
infinity. IKBT's detector has always reported both kinds (`pieper_triples()` returns
`kind='intersecting'` or `kind='parallel'`), but every solver leaf was written for the intersecting
case. `UR5`, `Arm_3`, `Parkman13` and `JennyGuoSp24` have parallel triples.

## The measurement that motivated the leaf (2026-08-28)

`UR5` — `par(2,3,4)`, the three parallel elbow axes — was reported `solved 9/9` while **0 of its 8
solution versions reproduced the target pose**. Two independent defects:

1. `th_2` was "solved" from `0*sin(th_2) + 0*cos(th_2) = 0`. Both coefficients vanish identically once
   the robot's own FK is substituted (proved symbolically with all DH parameters left free), so
   `atan2(-B, A)` was `atan2` of two rounding errors. Fixed by `ikbtbasics/eqn_sanity.py`, which every
   solve node now consults.
2. With that refused, `UR5` becomes an honest **4 of 9**: `th_1`, `th_5`, `th_6`, `th_234` solve and
   `th_2`, `th_3`, `th_4` do not. Neither `x2z2_transform` nor `invariant_gen` can restock a
   one-unknown equation for `th_2` — measured, `x2z2` accepted **none of 91** candidate pairs.

## The published method — and how far IKBT already got

**IK-Geo** (Elias & Wen) classifies 6R kinematics into families and gives a closed-form algorithm per
family. Its Table I lists *"three parallel axes and two intersecting axes"* with **UR5** as the
example, and Table II marks that family **closed-form** — no search — using Subproblems 1, 3, 4, 6.

Section IV-B, *Three Parallel Joint Axes*, with `h_2 = h_3 = h_4` the parallel triple:

| step | equation | solves | subproblem |
|---|---|---|---|
| (8)  | `h2' R10 p06 - h2' R45 p56 = h2'(p12 + p23 + p34 + p45)` | `q_1` | SP4 (SP6 with (9) if `p56 != 0`) |
| (9)  | `h2' R10 R06 h6 - h2' R45 h6 = 0`                        | `q_5` | SP4 / SP6 |
| (10) | `R14 R45 h6 = R10 R06 h6`                                | `q_2+q_3+q_4` | SP1 |
| (11) | `R65 R54 h2 = R06' R01 h2`                               | `q_6` | SP1 |
| (12) | `norm(p23 + R23 p34) = norm(R10 p06 - p12 - R14 p45 - R15 p56)` | `q_3` | **SP3** |
| (13) | `R12(p23 + R23 p34) = R10 p06 - p12 - R14 p45 - R15 p56` | `q_2` | SP1 |
| —    | `q_4 = (q_2+q_3+q_4) - q_2 - q_3`                        | `q_4` | subtraction |

Eight solutions. **IKBT independently reproduced steps (8)-(11) and stopped at (12).** The variables it
solved — `th_1`, `th_5`, `th_6`, `th_234` — are exactly those four steps, `th_234` being the
sum-of-angles variable of the parallel triple. That correspondence is the evidence that the gap is one
subproblem wide, not a missing solver family.

## Why the existing transforms cannot produce step (12)

Step (12) is Subproblem 3, *Circle and Sphere*: `min | norm(R(k,th) p1 - p2) - d |`. Concretely the law
of cosines, `norm(...)^2 = a_2^2 + a_3^2 + 2 a_2 a_3 cos(q_3)`, giving `q_3 = +/- acos(.)`.

The load-bearing detail is the **right-hand side**: `R10 p06 - p12 - R14 p45 - R15 p56` subtracts the
wrist offsets, which are *known* by then because `q_1`, `q_5` and `q_2+q_3+q_4` are already solved.

- `x2z2_transform` squares pairs of raw position equations. The `d_5`/`d_6` wrist terms are still
  symbolic, so `th_2` survives in the cross terms and the result carries two unknowns where its
  acceptance test demands one. (Removed from the tree 2026-08-28; `invariant_gen` subsumes it.)
- `invariant_gen` computes `|P|^2` over `R.mequation_list` — the *whole* symbolic side, wrist terms
  included — so it hits the same wall.

**The fix is one step, not one formula:** split each position component into the part that contains an
unsolved unknown and the part that does not, move the known part across to the numeric side, and take
`|P|^2` of what remains. On a parallel triple the cross term collapses to `2 a_j a_(j+1) cos(q_mid)`
and the equation has a single unknown. Everything after that is existing IKBT machinery: `arccos` via
`sincos_solve` (two branches), `q_2` by `atan2`, and `q_4` by the algebra leaf from the SOA definition.

## References

- Elias & Wen, **IK-Geo: Unified Robot Inverse Kinematics Using Subproblem Decomposition**,
  arXiv:2211.05737. <https://arxiv.org/pdf/2211.05737> — Table I (families), Table II (which are
  closed-form), Table III (the six subproblems), **Section IV-B** (this case). Code:
  <https://github.com/rpiRobotics/ik-geo>
- **Automatic Geometric Decomposition for Analytical Inverse Kinematics**, arXiv:2409.14815.
  <https://arxiv.org/html/2409.14815v1> — automatic identification of the parallel case, then
  subproblem decomposition. Architecturally close to IKBT's ID-node + solve-node split.
- **EAIK: A Toolbox for Efficient Analytical Inverse Kinematics by Subproblem Decomposition**.
  <https://ostermd.github.io/EAIK/>
- Hawkins, **Analytic Inverse Kinematics for the Universal Robots UR-5/UR-10 Arms**, Georgia Tech.
  <https://repository.gatech.edu/server/api/core/bitstreams/e56759bc-92c8-43df-aa62-0dc47581459d/content>
  — a hand derivation for this exact arm, useful to check generated equations against.
- Williams, **Universal Robot URe-Series Cobot Kinematics & Dynamics**.
  <https://people.ohio.edu/williams/html/PDF/UniversalRobotKinematics.pdf>
- **321 kinematic structure**, Wikipedia. <https://en.wikipedia.org/wiki/321_kinematic_structure> —
  short history of Pieper's condition and the "meet at infinity" reading.
- Pieper, D. L., *The Kinematics of Manipulators Under Computer Control*, PhD thesis, Stanford, 1968.
- Paden-Kahan subproblems 1-3: the classical ancestors of IK-Geo's Subproblems 1-4.

## Note on the UR5 DH table

`ikbtfunctions/ik_robots.py` gives `a_2 = +0.425`, `a_3 = +0.392`, where Universal Robots publishes
both as **negative**. This is not an error: `a` is the length of a common normal and cannot be
negative, and with three parallel axes the direction of `x_i` is a free choice. Verified numerically —
the repo's table reproduces UR's published FK exactly (`3.3e-16`) under `th_2 += pi, th_4 += pi`, with
no sign changes. Joint **zeros** differ from the UR controller's by `pi` on joints 2 and 4; anyone
comparing generated IK against a real UR5 must account for that. The `d` values are rounded to three
decimals (`d_1 .089` vs `.089159`, `d_4 .109` vs `.10915`, `d_5 .095` vs `.09465`, `d_6 .082` vs
`.0823`).
