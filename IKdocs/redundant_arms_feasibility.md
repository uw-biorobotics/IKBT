# Redundant arms (n > m): symbolic partial IK + null-space motion — feasibility notes

*Started 2026-10-30. Generalized 2026-10-31 from n > 6 to n > m (§0). Planning only, no code
yet. The size claims in §7 are untested guesses; §10 lists the experiments that would settle them.*

## 0. Redundancy is n > m, not n > 6

m is the dimension of the arm's manipulator subspace: the **generic rank** of its 6×n Jacobian
(the maximum over random joint-poses, not the rank at one joint-pose, which drops at
singularities). The arm is redundant when **n > m**, with degree of redundancy **r = n − m**.
"n > 6" is only the special case m = 6. A planar 4R arm (n = 4, m = 3) is redundant by one,
exactly like a 7-joint spatial arm; a planar arm with 6 parallel axes (n = 6, m = 3) is redundant
by three. Measured 2026-10-31: m = n for all 32 robots in `ROBOT_LIST`.

**One rule makes every formula below hold for any n > m.** Replace J by **J_m**, J restricted to
its m-dimensional row space: m independent rows of J, or `U_m^T J` from the SVD `J = U S V^T`.
Then read every "6" that counts task dimensions as m. When m = 6, J_m = J and each formula is
the one written for the 7-joint case, so nothing in that plan changes. Without this rule the
formulas fail on m < 6: J_r is not square, the six-row cross product does not exist, cond(J) is
infinite everywhere, and det(JJ^T), the manipulability, and every 6×6 minor are identically zero.

**When m < 6, the closest-solution step comes first.** A target T off the subspace has no exact
solution, so `q(ψ)` below is exact only after T has been projected onto the subspace (nearest
end-effector-pose under `||dp|| + r*theta`, see `IKdocs/manip_subspace.md`). The routing table
there combines the two questions.

**Summary.** This is feasible for 7-joint arms, and most of the machinery exists. The one-variable
branch already produces the symbolic partial IK. Once that exists, an exact null-space basis comes
from the Jacobian with no symbolic pseudoinverse. Planned deliverables: ∂q/∂ψ, with condition-number
guards, plus two standard cost functions for choosing ψ.

## 1. The one-variable branch is the partial solution

With n joints there are r = n − m redundant ones (§0). Every reachable end-effector-pose T is
reached by an r-dimensional family of joint-poses, the **self-motion manifold**. For r = 1 it is a curve in joint
space: the elbow swinging while the hand stays still. `onevar_ik` declares one unknown known and
solves the rest symbolically, which on a 7-joint arm gives

    q(ψ) = (ψ, f_b(T, ψ)),   b = branch index,  ψ = the joint(s) declared known

This is a closed-form description of the self-motion manifold. It is the joint-angle version of the
SRS "arm angle" (Kreutz-Delgado; Shimizu) and of the analytical Panda IK that uses q7.

| | C-Arm (6 joints, not symbolically solvable) | redundant (n > m) |
|---|---|---|
| ψ | an unknown with a true value | a free choice |
| valid ψ | only the zeros of the error curve | every ψ in the branch's domain |
| 1-D search | find the roots | find the feasible intervals, then optimize a cost (§6) |

Most of `SEARCH_CORE` carries over. The domain-edge probe becomes "where does this branch stop
existing". The root-acceptance step goes away.

## 2. Null-space basis: ∂q/∂ψ

Split J_m (m×n, §0) into J_p (the ψ column(s)) and J_r (the m×m block of solved joints). By the
implicit function theorem,

    ∂q/∂ψ = [ I ; −J_r⁻¹ J_p ]     (n × r)

This is an **exact null-space basis**: moving along it changes the joint-pose without moving the
hand. It needs only q and the generated `jacobian(q)`. It does not need T or a branch index, and it
equals the derivative of f_b on whichever branch passes through q.

**Position level vs. rate level:**

- *Position level* works with q itself. Choose ψ and evaluate the closed form. Each answer is exact,
  with no time steps and no drift. Choosing ψ to minimize a cost H(q(ψ)) (by dH/dψ = ∇H·∂q/∂ψ, or
  just a 1-D search) is the recommended first target.
- *Rate level* is the resolved-rate controller q̇ = J⁺ẋ + (I − J⁺J)z, integrated step by step.
  Integration drifts, so practical controllers add feedback on the end-effector-pose error (CLIK).
  This fits when the input really is a velocity (teleoperation, trajectory streaming).

**Division-free form for r = 1.** The generalized cross product of J_m's m rows (in ℝ^(m+1)),
n_i = (−1)^i·det(J_m with column i removed), is a null vector with no division. Its ψ entry is
±det(J_r), so ∂q/∂ψ = n / n_ψ. The projector is I − J⁺J = n nᵀ/(nᵀn), so the rate-level null term
is q̇_null = n·(nᵀz)/(nᵀn): one dot product per cycle instead of an SVD.

## 3. Charts: why ∂q/∂ψ can blow up while the arm is fine

A **chart** is a coordinate system on part of a curved set, like one page of an atlas. Labeling the
self-motion curve by the value of joint p (ψ = q_p) is a chart.

The unit circle shows how a chart fails. With x as the coordinate, y = ±√(1 − x²). The two signs are
two branches, like the b in f_b. At x = ±1 the circle turns back in x, so dy/dx → ∞ and the
branches merge. **The circle is smooth there; only the coordinate x has failed.** Near x = ±1 you
switch to y, a second chart. A set of charts covering everything is an atlas.

On the arm, where joint p reaches an extreme along the self-motion curve, the tangent has no q_p
component. At that joint-pose ∂q/∂ψ → ∞, two branches merge, and det(J_r) = 0, while J still has
full rank. These are the 1/cos(th_2) spikes the edge probe hunts on C-Arm.

| | what goes singular | meaning | seen by cond(J)? |
|---|---|---|---|
| arm singularity | J_m (m×n) loses rank | the hand cannot move in some direction | yes |
| chart singularity | J_r (m×m) loses rank, J_m does not | ψ is a bad coordinate here | **no** |

J_r is a subset of J's columns, so an arm singularity is always a chart singularity too. The reverse
is not true. A different ψ joint gives a chart with its failures in other places. The arm angle
exists because it fails less often than any single joint angle.

## 4. Planned deliverable: ∂q/∂ψ with condition-number guards

Python and C++, following the existing one-function-per-language pairing:

| python | C++ (in `ikbt::<R>`) | returns |
|---|---|---|
| `dq_dpsi_<R>(q)` | `Eigen::MatrixXd dq_dpsi(const Eigen::VectorXd& q)` | n×r null-space basis |
| `cond_<R>(q)` | `cond(q)` | (cond(J), cond(J_r)) |

- `cond(J)` answers "is the arm near a singularity?" and `cond(J_r)` answers "can ∂q/∂ψ be trusted
  here?" Both come from one `jacobian()` call. The guard is advisory: IKBT reports the numbers and
  the caller picks a threshold.
- cond(J) = σ_max/σ_min over the **m** generically nonzero singular values (numpy `svd`, Eigen
  `JacobiSVD`); taking all six would give ∞ at every joint-pose when m < 6.
- Compute J_r⁻¹J_p by linear solve (`np.linalg.solve` / `partialPivLu`), never by an explicit
  inverse.
- Both condition numbers use the **scaled** Jacobian (§6.3).
- Which joints ψ are is fixed when the code is generated. The module names them, as
  `IK_conditional` names `th_2`.
- Possible later addition for r = 1: return n and det(J_r) instead, which are always finite, so the
  caller sees the blow-up coming instead of receiving an inf.

## 5. Joint limits: none yet

IKBT has never considered joint limits. The only trace is the unused `mechanism.jlims` in
`ikbtbasics/kin_cl.py:346`, hardcoded to ±π and noted as a placeholder in `dh_analysis.py:213`.
Limit vectors per robot belong on the Robot or mechanism class. When they are added, that slot should
be either filled or removed, so limits live in one place. Implementation is deferred.

## 6. Cost functions for choosing ψ

There are two standard ones: joint-limit avoidance and manipulability. Both are written as
**merits** (higher is better).

### 6.1 Joint-limit merit: flat in the middle, smooth falloff at the ends

Normalize each joint, u_i = (q_i − q_min,i)/(q_max,i − q_min,i) ∈ [0, 1]. Normalizing makes a narrow
joint and a wide one comparable, and puts revolute and prismatic joints in the same units. With band
width β (0.1 to start):

    m_i = 1                   β ≤ u_i ≤ 1−β    (flat middle 80%)
    m_i = s(u_i / β)          u_i < β
    m_i = s((1 − u_i) / β)    u_i > 1−β

    s(t) = 6t⁵ − 15t⁴ + 10t³      ("smootherstep")

- **s, not a literal logistic.** A logistic never reaches exactly 0 or 1, so clipping it leaves a
  jump or kink at the band edges. The smootherstep has the same S-shape, reaches 0 at the limit and 1
  at the band edge, and has zero first and second derivatives at both ends.
- **Combine by product, M = Π m_i.** M is 0 if any joint is at a limit, 1 if all joints are safe, and
  smooth everywhere. A `min` would put kinks in the gradient wherever the nearest joint changes and
  can chatter a gradient-driven step. As a cost, −log M = Σ −log m_i is a barrier that is infinite at
  any limit.
- **Flatness is intended.** Over most of the self-motion M = 1 and has no preference, so limits are
  ignored until one gets close. On its own it leaves ψ undetermined, which is why it is paired with
  manipulability.

This replaces an earlier idea, H = 1/min_i d_i, which had the kink problem and no flat zone.

### 6.2 Manipulability (Yoshikawa)

    w = √det(J_m J_mᵀ) = product of the m nonzero singular values

(√det(JJᵀ) itself is identically zero when m < 6.) By Cauchy–Binet, for r = 1, w = ‖n‖, the norm
of the §2 minors vector. The guard and the cost can
share one computation. Typical combinations are to maximize w·M, or to maximize w subject to
M > threshold.

### 6.3 Scale the Jacobian rows first

J's top rows are in length units and its bottom rows are dimensionless. So w and cond(J) change with
the choice of meters or millimeters, and their scale depends on how big the arm is. The DLS code
already deals with this through `w_rot` and `dh_analysis.length_scale()`. Manipulability, cond(J) and
cond(J_r) should all use J scaled the same way, so a threshold means the same thing on every arm.

## 7. A symbolic pseudoinverse only in structured form

Expanded symbolically, J⁺ = J_mᵀ(J_m J_mᵀ)⁻¹ will swell badly (an m×m inverse with trig entries).
Two identities keep it structured:

- **Cauchy–Binet:** det(J_m J_mᵀ) = Σ(m×m minors)² = ‖n‖². Its zero set is exactly the arm singularities,
  and the individual minors show which locked-joint sub-chain went singular.
- **Projection of any right inverse:** J⁺ = (I − n nᵀ/‖n‖²)·G, for example G = J_r⁻¹ padded with a
  zero row, where J_r⁻¹ = adj(J_r)/det(J_r).

To keep the minors small, compute J in a middle frame (3 or 4), since the determinant does not depend
on the frame but entry size does. With a spherical wrist J is block-triangular and the determinant
factors (`pieper_geom_report` finds these triples). Guess: the minors stay manageable on DH-nice arms
and expanded J⁺ does not. **Unmeasured** (§10).

## 8. Algorithms

| # | Method | r | Symbolic part | Notes |
|---|---|---|---|---|
| 1 | Joint-parameterized closed form + optimize ψ (§6) | 1 | all but ψ | Reuses onevar. Exact, no drift. **First target.** |
| 2 | Arm-angle parameterization | 1 | all | SRS arms only, detectable from Pieper triples. A better chart. |
| 3 | Symbolic null vector n(q) at rate level | 1 | n(q) | No division. Useful even with numeric J⁺. |
| 4 | Structured symbolic J⁺ (projector × G) | 1 | factored J⁺ | Only if rate-level control is the goal. |
| 5 | Extended Jacobian (Baillieul) | ≥1 | square inverse | Equivalent to #1 at rate level; has algorithmic singularities. |
| 6 | Numeric SVD J⁺ + projection via `jacobian()` | any | J only | Baseline. Small addition to `ikbt_dls.h`. Test oracle for #3 and #4. Truncate the SVD at rank m. |
| 7 | Declare r joints known | ≥2 | as #1 | C(n,r) candidates and an r-D search. The grid ladder does not scale, but no root-finding is needed. |

## 9. Risks and blockers

- **Chart singularities (§3).** A single joint can be a poor chart globally, so several charts with
  hand-offs may be needed.
- **Candidate ranking.** `onevar_rank` ranks by how easy the remaining solve is (L1 count). For
  redundancy, chart quality (how rarely det(J_r) vanishes) matters too, and the minors could rank it.
- **The 6-row assumption is hard-coded.** `kin_cl.py` has `range(6)`, explicit `T_01..T_56`,
  `w_00..w_66` and a 6×6 `jnum`. `ik_classes.py` builds symbols only up to `th_6`, and sum-of-angles
  stops at `th_456`. `Panda` in `ik_robots.py` is the real Panda with joint 7 removed. Supporting 7+
  rows is mechanical, but it blocks only n > 6: redundant arms with n ≤ 6 (§0) fit the existing
  machinery.
- **Testing carries over.** Soundness: FK(q(ψ)) = T for every sampled ψ. Completeness: ψ = q*_p
  reproduces a known q* on some branch. ∂q/∂ψ: J·(∂q/∂ψ) = 0, and it agrees with a finite difference
  of q(ψ) and with the numeric SVD null space (compared by angle). All fit
  `numerical_closed_loop_sol_check` and the C++ pinning pattern.

## 10. First steps

0. Add a planar 4R test robot (n = 4, m = 3) to `ik_robots.py`. It needs no infrastructure work,
   so `dq_dpsi`, `cond` and the one-variable sweep can be built and tested on it first. It also
   exercises the m < 6 path (§0): closest solution, then self-motion.
1. Extend DH/FK/Jacobian to n rows, and add an unmodified 7-joint Panda.
2. Run onevar on it unchanged. Record which joint it picks and how large the closed forms are.
3. Compute the seven 6×6 minors of the symbolic Jacobian in frame 3 vs. frame 6 and record
   `count_ops`. This decides whether #3 and #4 are practical.
4. Add `dq_dpsi` and `cond` (§4). Both need only `jacobian()`, not a symbolic solve, so they can be
   built and tested on the step-0 planar arm, then on the 7-joint Jacobian from step 1. (A
   full-rank 6-joint arm has no null space to test them on; a rank-deficient one does.)
