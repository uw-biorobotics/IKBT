# Arms with fewer than 6 DOF: the manipulator subspace and "closest solution" IK

*Concept evaluation and design, 2026-10-31. Not implemented.*

## The question

An arm with n < 6 DOF moves on an n-dimensional manifold of SE(3), its **manipulator subspace**:
there are end-effector-poses inside its workspace that it cannot reach. A planar 3-joint arm has
three DH rows but moves only in one plane. Can IKBT derive the subspace symbolically, and what
should IK mean for a target T that is not on it?

**Decision (summary):** for **every** arm whose subspace has dimension m < 6 (section 1.1: m is
the generic Jacobian rank, which can be less than n even when n = 6), IK returns the **closest
reachable end-effector-pose** to the target T under one metric, `||dp|| + r*theta`. There is no
special case for 5-DOF arms. A user who wants a tool frame adds their own 6th DH row.

## 1. The subspace in symbolic form

### Parametric form: this is the FK

`T(q_1..q_n) = FK(q)` *is* a symbolic T-matrix whose elements are functions of n parameters, and
the manipulator subspace is exactly its image. IKBT already computes it for every robot (cached
in `fk_eqns/`). What it does not give is a way to answer "is this T on the subspace?"; that needs
the implicit form.

### 1.1 The dimension is the Jacobian rank, not n

The subspace's dimension m is the **generic rank** of the Jacobian (its maximum over random
joint-poses, not its rank at any one joint-pose, which drops at singularities). Usually m = n, but
not always: a planar arm with **6 parallel revolute axes** has n = 6 and is still confined to a
plane, m = 3. So the trigger for everything in this document is **m < 6**, not n < 6, and the
rank check is the gate rather than an optional first step.

When n > m the arm is also **redundant within its subspace**: an (n − m)-parameter self-motion
reaches every end-effector-pose it can reach at all, so there is no finite solution set to
enumerate. The symbolic path cannot finish (n unknowns, m independent equations), the
one-variable path assumes only one unknown, and Pieper's condition, which presumes a genuine
6-DOF arm, would see parallel triples and could wrongly report a 6-parallel-axis arm as solvable.
Such arms need the redundant-arm treatment (`IKdocs/redundant_arms_feasibility.md`): n − m
parameters declared known, or DLS from a seed, which picks a nearby member of the self-motion.

Measured 2026-10-31: generic rank = n for all 32 robots in `ROBOT_LIST`, so no current robot is
rank-deficient.

#### Computing m exactly: random rational joint-poses

Three ways to get the generic rank:

1. **Symbolic `Matrix.rank()` on J: avoid.** It must decide when a trig expression is
   identically zero, which simplification cannot reliably do. A missed zero inflates the rank,
   which is exactly the error that matters here. It is also slow on a 6×n trig matrix.
2. **Floating-point rank at a few random joint-poses.** The rank drops only on a measure-zero set
   (the singularities), so the maximum over random joint-poses is the generic rank with
   probability 1. Milliseconds via `numeric_ik.jacobian_callable`. The one judgement is the SVD
   tolerance, though at random joint-poses the gap is wide (~1e-15 vs. O(1)). This is how the
   2026-10-31 measurements above were made.
3. **Exact rank at random rational joint-poses: the gate.** No floating point, no tolerance.

**The tangent half-angle substitution.** For a revolute joint, let `t = tan(θ/2)`. For every θ
except θ = π these identities are exact (the double-angle formulas, rewritten):

    cos θ = (1 − t²) / (1 + t²)
    sin θ = 2t / (1 + t²)

A revolute joint enters the DH matrices only through `cos θ` and `sin θ`, so J's entries are
polynomials in those and the DH constants. Choosing a rational t *is* choosing the real joint
angle θ = 2·atan(t); θ itself is never computed, only its cosine and sine, which come out exact.
Example: t = 1/2 gives θ ≈ 53.13°, cos θ = 3/5, sin θ = 4/5 (the 3-4-5 triangle). Rational angles
in radians would not work: sin(1) is transcendental, with no exact representation. And the
special angles 0, π/2, π, which do have exact values, are where arms tend to be singular, the
wrong place to measure a *generic* rank.

Procedure:

- Expand sums of angles first (`sp.expand_trig`), so `sin(th_2 + th_3)` becomes products of the
  individual sines and cosines, which are still rational. `T_06` and `J66` stay in raw joint
  variables (`numeric_ik.py` header), so this is all that is needed.
- Each revolute joint: substitute the two identities with a random rational t. Each prismatic
  joint: a random rational length. DH constants converted to rationals (`sp.Rational(str(x))`).
- The usual α values (0, ±π/2, π) have exact cosines and sines. An α that is not a multiple of π/2
  makes cos α irrational; for such robots use high-precision floats, or treat cos α and sin α by
  the same half-angle substitution.
- Compute the rank of the resulting exact rational matrix over ℚ. Repeat at 3–4 random points and
  take the maximum. Well under a second per robot.

What it guarantees:

- **Lower bound, rigorous.** Rank m at any one point proves the generic rank is at least m.
- **Upper bound, with probability 1.** A random point understates the rank only if it lands on
  the zero set of every nonvanishing m×m minor, a nonzero polynomial in the t's. By the
  Schwartz–Zippel lemma [1, 2], a polynomial of degree d is zero at a point drawn uniformly from
  a set of size S with probability at most d/S, so drawing t from a large range makes a miss
  negligible, and repeated draws make it more so.
- A **proof** of the upper bound needs either every (m+1)×(m+1) minor to be identically zero
  (expensive, and the zero-testing problem of option 1 again), or a geometric cause, since a rank
  drop always has one (e.g. all axes parallel). `pieper_geom_report` and `axis_triple_check`
  already read axis geometry from the DH table and can serve as a cross-check.

θ = π (t → ∞) is the one angle the substitution cannot reach; a single point does not affect a
generic rank. The same substitution is standard in kinematics for turning trig equations into
polynomial ones, e.g. Raghavan and Roth's solution of the general 6R arm [3] and the textbook
treatment of transcendental IK equations [4].

References:

1. J. T. Schwartz, "Fast probabilistic algorithms for verification of polynomial identities,"
   *Journal of the ACM*, 27(4):701–717, 1980.
2. R. Zippel, "Probabilistic algorithms for sparse polynomials," *Proc. EUROSAM '79*, Lecture
   Notes in Computer Science 72, pp. 216–226, Springer, 1979.
3. M. Raghavan and B. Roth, "Inverse kinematics of the general 6R manipulator and related
   linkages," *ASME Journal of Mechanical Design*, 115(3):502–508, 1993.
4. J. J. Craig, *Introduction to Robotics: Mechanics and Control*, 3rd ed., Pearson, 2005,
   Ch. 4 (solution of transcendental equations by the tangent half-angle substitution).

**Routing.** A rank gate before the BT computes m and routes on two independent questions:

| | m = 6 | m < 6 |
|---|---|---|
| **n = m** | today's three paths | closest solution (this document) |
| **n > m** | redundant: self-motion | closest solution, **then** self-motion (e.g. planar 4R) |

"Redundant" means n > m throughout, and one strategy covers every n > m case
(`IKdocs/redundant_arms_feasibility.md` §0).

### 1.2 The subspace is placed by the DH table, not by frame 0

A planar arm's plane is perpendicular to the common direction u of its joint axes. That direction
is z_1 expressed in frame 0, set mainly by the **first DH row** (alpha_0 tilts z_1 away from z_0,
and a_0, d_1 offset it). So the plane is generally **tilted** and offset relative to frame 0, not
parallel to any of its coordinate planes. The constraints are then coordinate-free:
`u . P = c` (the hand stays in the plane) and `R^T u = const` (the hand's frame keeps a fixed
attitude to the plane's normal). The same holds for every subspace: the description must be in
terms of directions the DH table fixes, never in terms of frame 0's axes.

### Implicit form: 6−m constraint equations on T

For a planar 3R arm **whose joint axes happen to be parallel to z_0**: `r13 = r23 = 0`,
`Pz = const` (orthonormality then gives `r31 = r32 = 0`, `r33 = 1`); in general, the
coordinate-free form of 1.2. Three routes, cheapest first:

- **(a) Substitute the IK into the equations no leaf used.** IKBT solves an n-DOF arm from a
  subset of the 12 matrix equations. The leftover equations, with the solved joint expressions
  substituted in, become functions of T alone, and they are the subspace constraints. This needs
  no new algebra, and `disambiguate` already searches the unused equations. `atan2` solutions are
  no obstacle (`cos(atan2(y,x)) = x/sqrt(x^2+y^2)` reduces to algebraic form). The real cost is
  that `asin`/`acos` solutions bring a `±sqrt`, so off the subspace the residual's *value* depends
  on the branch substituted. It is a sound zero test, but a weak measure of distance.
- **(b) Implicitization by elimination.** `c_i, s_i` as variables with `c_i^2 + s_i^2 = 1`, then a
  Groebner basis eliminating the joint variables from `T − FK(q)`. Clean and branch-free, but the
  cost explodes on a general 5-DOF arm (`disambiguate` already has to time-limit its Groebner
  calls).
- **(c) Recognise a displacement subgroup** (Hervé): planar RRR → SE(2), SCARA → Schoenflies,
  spherical wrist → SO(3), PPP → T(3). Textbook closed forms for the subspace and the projection
  onto it. A general 5-DOF arm is not a subgroup.

Caveats: these give equalities only; the reach and joint limits inside the subspace are
inequalities. Route (b) yields the Zariski closure, which can contain points the arm cannot
reach. And the effective DOF can be less than the DH row count, so a numeric Jacobian rank check
at random joint-poses should come first: rank = true dimension, 6 − rank = number of constraints.

The implicit form is **not needed** for the closest-solution design below, which reports the
distance to the subspace directly. It remains useful later as a cheap `in_subspace(T)` test and
as a report section.

## 2. What IK means today for n < 6

18 of the 32 robots in `ROBOT_LIST` have fewer than 6 unknowns:

- **3:** Wrist, Arm_3, Brad
- **4:** Craig417, Palm13, Minder13, Axtman13, ICP5p5_A21
- **5:** Chair_Helper, Bartell, Sims11, MiniDD, Wachtveitl, Frei13, Parkman13, Mackler13, DZhang,
  JennyGuoSp24

The leaves solve n equations out of the 12: whichever ones the BT happened to find. On the
subspace that is exact. Off it:

- **`FK(ikin(T))` is a projection onto the subspace, but an arbitrary one.** FK of any real
  joint-pose is always a valid rigid transform (orthonormal rotation), and always reachable. It
  agrees with T on whatever the solve's equations constrained (combinations of elements, given
  the sum-of-angles and x²+y² transforms) and nothing else. Which components are honoured is an
  accident of the search order, not a choice anyone made.
- **Different branches project to different end-effector-poses**, at different distances from T.
- **A branch can vanish.** An `asin`/`acos` argument or a `sqrt` discriminant can leave its
  domain off the subspace, so rows are dropped and the set can be empty.
- **Nothing checks the input.** A T with a non-orthonormal rotation block is read only through
  the elements the equations use.

So the current contract is "valid only if you somehow generate T on the subspace."

## 3. Design: closest solution for every arm with m < 6

`q* = argmin_q  d(FK(q), T)`, with `d = ||dp|| + r*theta`.

### Rotation representation: angle-axis (already standard)

Every orientation error in the package is already the rotation vector `theta*axis` of
`R_err = R_target * R^T`: `rotation_angle_axis()` / `pose_error()` in `ikbtbasics/numeric_ik.py`,
and the hand-written `Cpp_src/ikbt_pose_error.h`, pinned to the python by `--dls`. `theta` comes
from `atan2`, not `acos`. Not RPY or Euler angles, because:

- `theta` is the true angular distance between the two orientations (the smallest rotation taking
  one to the other); differences of Euler angles are not a distance and blow up near gimbal lock;
- `theta` is in [0, π], so there is no wraparound;
- the Jacobian's ω rows predict exactly how `theta*axis` changes, so the DLS step and the error it
  reduces describe the same quantity.

For small errors, the components of `theta*axis` are approximately the roll/pitch/yaw differences.

### The length r: one scalar, one function

**One magnitude weight on `theta`, no per-axis weights.** `theta` is the same in every frame, so
a scalar r needs no frame and the residual can stay in the base frame, as `pose_error` builds it.

    r = 0.5 * sum_i ( |a_i| + |d_i| )

summed over the **constant** a and d cells only (a prismatic joint's d is its variable and is
excluded). Absolute values, because DH a and d can be negative. Roughly the mean reach: the lever
arm that converts a rotation error into a length, so `||dp|| + r*theta` is correct in any model
unit (ICP5p5_A21 is in millimetres). **Fallback when the sum is 0:** 1 model unit, and the
choice is harmless in both pure cases. A pure wrist (`Wrist`) cannot displace the end-effector,
so `dp` is the same at every joint-pose and only orientation is compared; an all-prismatic arm
cannot rotate it, so `theta` is the same everywhere. In both, r changes only the reported distance,
never which joint-pose is closest. Only a mixed arm with zero constant lengths (e.g. an R-P polar
arm with no offsets) makes the fallback a real choice; no current robot has a zero sum (the
smallest is Brad, 0.18). Exposed as a method the user can override with their own r.

**Prismatic stroke is left out, and that under-weights orientation on prismatic-heavy arms.**
About 15 of the 32 robots have a prismatic joint. On an arm like Stanford, the stroke of `d_3` is
much of the reach, and it is what turns a rotation at the shoulder into displacement at the hand,
so r underestimates the real lever arm. The right fix is to count half the stroke, which needs
joint limits, and IKBT has none yet (`M.jlims` is a ±π placeholder nothing sets per robot). Plan:

1. **Now:** accept and document the underestimate; prismatic-heavy arms can override r.
2. **When joint limits are added** to `ik_robots.py` (range-checking solutions needs them too):
   include half each prismatic joint's stroke in r.

This **replaces both existing defaults**, which are both live today:

- `W_ROT_DEFAULT = 1.0` (one model unit per radian, `ikbtbasics/numeric_ik.py`);
- `w_rot_for()` → `dh_analysis.length_scale()` (median of non-zero constant |a|, |d|), used by the
  generated hybrid and onevar code in both languages, two of whose callers patch a zero result
  with `or 1.0`.

### Work list

1. **The r function** above, and every caller moved onto it.
2. **Step and stopping test must minimise the same thing.** The DLS step minimises the squared
   residual `||e||^2`; acceptance (`m_try < m`) and the stopping test use `||dp|| + r*theta`.
   With an exact target they agree; at an unreachable target their minima are at different
   joint-poses. Simplest fix: make the scalar metric the weighted 2-norm of the residual vector.
   This touches `Cpp_src/ikbt_pose_error.h` and the `--dls` pin.
3. **A `stationary` stop reason** (`||J^T e|| ≈ 0`). Today, when no step improves the metric,
   lambda climbs to its cap and the loop exits as `lam_max`, which reads like a failure when it
   is the answer. The result must report the remaining distance, as data.
4. **Seeds from the symbolic IK branches**: one DLS refinement per branch, then de-duplicate.
   The same structure as the hybrid path's `refine_all`; local minima are per branch.
5. **Fallback seeds.** Off the subspace a branch can vanish, so a T far from the subspace can
   leave no seeds at all. Fallback: the caller's current joint-pose, or a multistart.
6. **On-subspace T is unchanged:** distance 0, the same answers as today.
7. **Closed-loop checks** (`numerical_closed_loop_sol_check`, `cpp_closed_loop_check`) need an
   off-subspace mode that checks the reported distance is a local minimum, not that it is zero.

`solve_numeric` already accepts n < 6 (`test_nikI_sub_six_dof_arm`), and a comment there
anticipates `converged` staying False.

## 4. Considered and dropped

### Ignore tool roll on 5-DOF arms

The idea: a 5-DOF arm gives up roll about z_6, so solve exactly for P plus the direction of z_6.
Measured: rank of the frame-n Jacobian with one angular row removed, 20 random joint-poses each.

| robots | without roll about z | without x | without y |
|---|---|---|---|
| Chair_Helper, Sims11, MiniDD, Wachtveitl, Frei13, Parkman13, Mackler13, DZhang, JennyGuoSp24 | **4** | 5 | 5 |
| Bartell | **5** | 4 | 5 |

The guard fails on 9 of the 10 five-DOF arms, and this follows from the convention. In IKBT's
modified DH, z_n is the last joint's axis and the frame origin lies on it. So a revolute last
joint is exactly a roll about z_n, and it moves neither P nor z_n's direction; joints 1–4 are left
to set 5 numbers. These are roll-capable arms (base yaw, shoulder, elbow, wrist pitch, wrist roll):
what they lack is a **pointing** direction, not roll. Only an arm whose 6th DH row is a real tool
frame, as Bartell's is (`alpha = π/2`), gives up roll. Dropped in favour of one rule for all n < 6.

### Per-axis rotation weights

Per-axis weights need a frame: the same tool roll is a rotation about base x in one joint-pose and
base z in another, so weights in the base frame penalise different physical motions as the arm
moves. They would have to be applied in the tool frame. Dropped: a single magnitude weight on
`theta` is frame-free and enough.

### Drill direction: column 3 only, or a virtual 6th joint

For a drilling task (P and drill direction known, spin phase ωt irrelevant), R's third column
plus P are exactly the parts of T free of ωt. But restricting IKBT to column 3 is not enough:
`get_mequation_set()` (`ikbtbasics/kin_cl.py`) builds all 21 splits `T_0i^-1 * Td * T_j6^-1`, and
the right-peeled ones mix Td's columns. Dropping them loses arms that need them (Raven-II).

The alternative was to make the spin a virtual joint by giving the tool row a `th_6`: the arm
becomes an ordinary 6-DOF problem, any completion of the drill direction is absorbed by `th_6`,
and `th_6` is discarded. This needs a user-supplied tool row, its own derived-robot name and
pickle, a `(P, z)` entry point, and a P-and-z-only closed-loop check. Cancelled: closest solution
covers it, and a user who has a tool adds their own 6th row.
