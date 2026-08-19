# Hybrid Symbolic–Numeric IK: Implementation Plan

A plan for approach 1 of [futurework.md](futurework.md) — when IKBT cannot solve a robot in closed
form, simplify its DH parameters until it *can*, solve the simplified robot symbolically, and correct
the result numerically using the symbolic Jacobian.

Written August 2026. Companion to [NewStrategies.md](NewStrategies.md) and
[ImplementationThoughts.md](ImplementationThoughts.md).

---

## Measurements taken before planning

Approach 1 rests on three assumptions that are cheap to test, so they were tested first. All three
hold, one with a useful surprise.

### 1. The Pieper test is ~25 lines of DH arithmetic, and it separates the robot set cleanly

In this repo's Craig-convention table, row `r` holds `[α_r, a_r, d_{r+1}, θ_{r+1}]`
(`kin_cl.py:280`, "alpha N-1, a N-1, d N, theta N"). Therefore, with 0-indexed rows and 1-indexed
joint axes:

- axes `j` and `j+1` **intersect** iff `a_j = 0`, i.e. `DH[j,1] == 0`
- axes `j` and `j+1` are **parallel** iff `sin(α_j) = 0`, i.e. `sin(DH[j,0]) == 0`
- axes `j, j+1, j+2` **intersect at a common point** iff
  `DH[j,1] == 0 ∧ DH[j+1,1] == 0 ∧ DH[j,2] == 0`  (that is, `a_j = a_{j+1} = d_{j+1} = 0`)
- axes `j, j+1, j+2` are **mutually parallel** iff `sin(DH[j,0]) == 0 ∧ sin(DH[j+1,0]) == 0`

Sanity check on a known-good robot: `KawasakiRS007L` has `a_4 = a_5 = d_5 = 0`, giving the classic
spherical wrist at axes (4,5,6). It solves.

Run over all 33 entries in `robot_params()`'s `List`, exactly **six** robots have no triple of any
kind — and in every one of the six, a *single* DH parameter is responsible:

| robot | cheapest single-parameter fix | magnitude vs. scale |
|---|---|---|
| KinovaLite | `d_5 = 0` → wrist (4,5,6) | 57 of ~690 mm |
| Issue4 | `d_5 = 0` → wrist (4,5,6) | 0.029 of ~0.84 |
| KawasakiRS05L | `a_3 = 0` → triple (3,4,5) | 80 of ~330 mm |
| Wachtveitl | `d_2 = 0` → triple (1,2,3) | 1 of ~3 |
| Olson13 | `d_3 = 0` → triple (2,3,4) | 1 of ~4 |
| Sims11 | `a_2 = 0` → triple (2,3,4) | 5 of ~5 (poor — not a small change) |

So the search in step 1.1 is **not an optimization problem**. It is at most 8 candidate triples ×
2 condition types, enumerable exhaustively in milliseconds.

Two things this also settles:

- `ArmRobo` and `Raven-II` *do* have intersecting triples and still fail. Their empty-`eqns_1u`
  problem (`comp_detect.py:101`) is a defect in the solver, not a property of the geometry. The
  hybrid method is not the fix for those two, and the detector is what tells us so.
- `Chair6DOF` appears in `List` with no definition block, so `robot_params('Chair6DOF')` raises
  `UnboundLocalError` — same family as the missing-comma bug fixed in `b013d9c`.

### 2. The premise of step 2.3 holds on the motivating robot

`KinovaLite` was rebuilt with `d_5 → 0` and run through the **standard, unmodified** tree from
`build_default_bt()`:

| configuration | result |
|---|---|
| `KinovaLite` as committed | 0 of 6 variables solved, ~43 s (recorded in `ImplementationThoughts.md`) |
| `KinovaLite` with `d_5 = 0` | **all 6 solved, 27.7 s** |

Methods chosen: `th_1` sinANDcos, `th_2` simultaneous eqn, `th_3` x2z2 + arcsin, `th_4` atan2,
`th_5` atan2 (best ranked), `th_6` atan2 (best ranked), `th_23` algebra.

No new solver leaf was required. The existing toolbox solves the simplified robot the moment the
geometry admits a closed form. The FK pickle for this run is left in place as
`fk_eqns/KinovaLite_d5zero_pickle.p`.

### 3. The simplification is a good Newton seed — and the obvious "compensation" heuristic is wrong

Over 2000 uniformly sampled poses, comparing true FK against simplified FK:

| variant | mean ‖Δp‖ | max ‖Δp‖ | max Δθ |
|---|---|---|---|
| `d_5 → 0` (naive) | 57.0 mm | 57.0 mm | 0.00° |
| `d_5 → 0`, `d_6 += 57` (length-compensated) | 80.6 mm | 80.6 mm | 0.00° |

8.3% of reach, with **zero** orientation error — a good starting point for Newton. And the
intuitive fix of preserving total arm length by rolling the offset into `d_6` makes it *worse*: the
two offsets are orthogonal, so the errors add in quadrature (√2 × 57 = 80.6). Intuition gets this
backwards, which is the argument for ranking candidates by a measured metric rather than by
parameter magnitude.

---

## Phase 0 — prerequisites

`ImplementationThoughts.md:127` already identifies the gap: there is no robot-level baseline.
Nothing in approach 1 can be evaluated without one, because the deliverable is literally "robots
that move from unsolved to solved".

- **`scripts/robot_baseline.py`** — run every robot; record solved/unsolved, per-variable
  `solvemethod`, solve time, solution count into a checked-in file. Per BH's note
  (`ImplementationThoughts.md:159`) this is a *record to be diffed*, not a set of assertions:
  "does not solve" is a legitimate and expected entry.
- **Three incidental defects**, all cheap, all in the path of this work:
  1. `Chair6DOF` in `List` without a definition block → `UnboundLocalError`.
  2. `solChecker.py` is still Python 2 (`print` statements from line 207) and will not run — yet it
     is the natural starting point for the round-trip harness needed in 2.4.
  3. Any code building `unknown` objects outside `robot_params()` must replicate the `v.n = i` loop
     at `ik_robots.py:679-684`, or `get_variable_index()` calls `quit()` in the middle of the
     sum-of-angles scan. Every leaf proposed below constructs a robot programmatically, so this bites
     immediately. (It did.)

---

## Step 1.1 — find the smallest DH modification that creates a triple

New module **`ikbtbasics/dh_analysis.py`** — pure DH-table arithmetic, no FK and no symbolic solving,
so it runs in milliseconds and unit-tests without pickles.

1. **`pieper_triples(dh, ndof)`** → the satisfied triples, by kind (intersecting / parallel).
   **Restrict `j` to real joints.** The zero-padded rows required of sub-6-DOF robots manufacture
   spurious triples — `Brad`, a 3-DOF arm, reports five.
2. **`candidate_simplifications(dh, pvals, ndof)`** → for each *unsatisfied* triple, the set of DH
   entries that would have to be zeroed (intersection) or snapped to a multiple of π (parallel), each
   with its numeric magnitude. Symbolic entries with no `pvals` value are reported as *undecidable*,
   never silently skipped — `Sims11` has one (`d_2`).
3. **`displacement_metric(dh, dh_simp, pvals, ndof, n=2000)`** → mean and max position and orientation
   deviation over sampled joint space (`M.jlims` already holds limits). **This is the ranking key,
   not parameter magnitude**, for three reasons:
   - it puts length-zeroing and angle-snapping into comparable units;
   - it caught the compensation error in measurement 3 above;
   - it is exactly the quantity that predicts whether the Newton seed in 2.2 converges, so steps 1.1
     and 2.4 end up sharing a single number.
4. **`simplify(robot_name)`** → the ranked candidates plus the derived `dh`, `params`, `pvals`.
5. **`scripts/simplify_dh.py <Robot>`** — CLI reporting triples, ranked candidates with displacement,
   and the recommended table. This has standalone value as a *mechanical design* tool, independent of
   any solver: "your 57 mm offset is what costs you closed-form IK" is a useful thing to be able to
   tell a designer.

**Validation:** the detector must find a triple for every robot that currently solves, and the
no-triple set must be exactly the six in the table above. The expected answer is already measured, so
this is a real assertion rather than a rubber stamp.

---

## Step 2.2 — numeric refinement seeded by the closed-form solution

New module **`ikbtbasics/numeric_ik.py`**. Deliberately **not** a BT leaf: it produces no sympy
expressions and cannot call `unknown.set_solved()`, so dressing it as a solver leaf would corrupt the
solution-set machinery that every downstream generator consumes.

- **The symbolic Jacobian already exists and is already pickled.** `M.J66` (`kin_cl.py:454`) is built
  by velocity propagation whenever `JACOBIAN = True`, which is the default. No new symbolic
  computation is needed. It is expressed in frame 6, so refinement needs
  `J_0 = blkdiag(R_06, R_06) · J_66`.
- **Error metric:** position in mm; orientation as the rotation vector of `R_d · R(q)ᵀ`, scaled by a
  characteristic length so the 6-vector is dimensionally consistent. That scale factor drives the
  conditioning of the least-squares step and must be an explicit, documented knob.
- **Use damped least squares (Levenberg–Marquardt), not plain gradient descent.**
  `Δq = Jᵀ(JJᵀ + λ²I)⁻¹ e` behaves like Newton away from singularities and degrades gracefully *to*
  gradient descent as λ grows — which is precisely the singularity failure mode that step 2.4 asks
  about. Plain gradient descent would turn that failure mode into a permanent one.
- **Refine every closed-form branch independently**, then deduplicate on wrapped joint values while
  keeping branch labels. Preserving the multi-branch structure is the one thing this hybrid offers
  that a generic numerical IK does not, and it is the reason to build it this way.
- **Validate standalone, before any BT work:** on `Puma`/`KawasakiRS007L` (which solve exactly),
  perturb the true solution and confirm quadratic convergence; then on `KinovaLite`, seed from the
  `d_5 = 0` solution and measure convergence against true FK.

---

## Step 2.3 — behavior-tree and code-generation integration

Smaller than it looks, with one non-obvious wrinkle.

- **The symbolic branch does not signal failure as `b3.FAILURE`.** `comp_det` returns `SUCCESS` on
  the give-up path in order to break the outer `RepeatUntilSuccess`, and records the real outcome as
  `blackboard['no_progress']` (`comp_detect.py:137`). The top-level `Priority` therefore needs a
  small `b3.Condition` adapter converting `no_progress` into `FAILURE`. Get this wrong and the
  hybrid branch is silently unreachable.
- **Top level** becomes, in `build_default_bt()`:

  ```
  Priority([ Sequence([ symbolic_tree, symbolic_succeeded? ]),
             hybrid_sequence ])
  ```

  behind a `hybrid_enabled` flag defaulting **off** — following the `invariant_gen` precedent
  (`bt_assembly.py:157`), which makes the change behavior-preserving for all currently-solving robots
  by construction.
- **`hybrid_sequence`** = `Sequence([ simplify_dh_leaf, solve_simplified_leaf, mark_hybrid_leaf ])`.
  `solve_simplified_leaf` builds a *fresh* blackboard and ticks a nested `build_default_bt()` — legal
  in b3, and it avoids mutating the outer blackboard's `Robot`. Two traps:
  - the derived robot needs its **own pickle name** (`check_the_pickle()` compares DH tables and
    `quit()`s on mismatch — the experiment above used `KinovaLite_d5zero`);
  - it needs **fresh `unknown` objects**, because `set_solved()` mutates them in place.
- **`robot_params()` `quit()`s on unknown names**, so the derived robot cannot go through it.
  `load_robot()` needs a sibling taking an explicit DH table — a ~10-line refactor of `ik_driver.py`.
- **Codegen.** `output_python.py` gains `fk(q)` and `jac(q)` emission — `output_FK_python_code()`
  already proves the `str(M.T_06)` dump works, though it currently emits a module-level matrix with
  dummy joint values rather than a callable, so this doubles as a fix — plus a `refine()`
  implementing 2.2 and an `ikin_<Robot>(T, refine=True)` chaining them. The generated file must state
  prominently that the returned solutions are **numerical**, that the closed-form part solves a
  **different, simplified robot**, and it must return per-branch residuals and convergence flags.
- **C++ is phase 2**, after Python is validated. The existing C++ output contains no linear algebra
  at all, so it needs either Eigen (a new dependency) or ~80 lines of self-contained damped 6×6 solve.

---

## Step 2.4 — systematic study of the failure modes

Fast unit tests over synthetic DH tables (no FK, so they belong with the rest in
`tests/leavestest.py` — **next free class number is `TestSolver016`**; 015 is taken), plus a slow
robot-level study invoked separately.

Experiments, in order of value:

1. **Basin of attraction vs. simplification size.** Sweep the offending parameter — for `KinovaLite`,
   `d_5` = 0, 10, 20, … 57, 100, 200 — and plot convergence rate and iteration count against the
   displacement metric from 1.1. This answers "the simplified form may not be close enough to
   initialize the numerical method" with a predictive threshold instead of an anecdote. Cheap: one
   symbolic solve, many numeric runs.
2. **Singularity behavior.** Sample poses stratified by `σ_min(J)`; report success rate and the λ
   required per stratum.
3. **Branch loss — a structural limit, not a numerical one, and it must be stated plainly.** A 6R
   with a spherical wrist has at most 8 IK solutions; a general 6R has at most 16. Zeroing `d_5`
   therefore produces a seed set that can be *provably incomplete* for the real robot, and no amount
   of Newton polish recovers a branch that was never seeded. Mitigations to evaluate:
   - multi-start from perturbed seeds around each closed-form branch;
   - better, **numerical continuation**: sweep the parameter from 0 to its true value in steps,
     tracking each branch. The simplification defines a natural homotopy path, branches can split
     along it, and this is the principled version of the whole idea. This is the recommended fallback
     wherever plain Newton proves insufficient.
4. **Unreachable / joint-limit poses.** The closed-form stage already emits a `solvable_pose` flag;
   the hybrid must degrade gracefully rather than feeding NaN into Newton.
5. **Branch collision and mislabeling** — two seeds converging to one solution, or a seed converging
   into a neighbour's branch. Detectable by round-trip identity check.

---

## One caveat to settle before publishing anything

This changes what IKBT delivers. The JAIR position is *closed-form* IK; the hybrid output is
numerical, seeded by closed form. That is defensible, and the branch-completeness property is a
genuine advantage over generic numerical IK — but the LaTeX report and the generated code must say
clearly **which robot the closed-form equations actually describe**, or the artifacts become
misleading.

## Suggested order

1. Phase 0 — baseline and the three incidental defects.
2. Step 1.1 — standalone value even if nothing else lands.
3. Step 2.2 — validated on robots that already solve exactly.
4. Step 2.3 — behind a default-off flag.
5. Step 2.4 — the failure-mode study.
