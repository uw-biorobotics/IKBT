# Hybrid Symbolic–Numeric IK — Implementation Plan

> **Working plan, now tracked.** BH's review comments are preserved inline as `USR:`;  every one of
> them has been folded into the surrounding text. Phases A and B are **done** and their sections
> record what was actually built, which is not in every respect what was planned. Phases C-F are
> still plans. Insert further `USR:` comments anywhere and I will incorporate them.

Plan for [hybrid_plan.md](hybrid_plan.md), reordered per BH's direction: **build the high-level BT
changes first using stub leaves, prove the new tree reproduces today's successes and failures across
every robot, then implement and test each new leaf in the order it would be ticked.**

---

## Context

`futurework.md` item 1: when IKBT cannot solve a robot in closed form, simplify its DH parameters
until it can, solve the simplified robot symbolically, and correct the result numerically using the
symbolic Jacobian. Measurements in `hybrid_plan.md` show `KinovaLite` with `d_5 → 0` goes from
0-of-6 variables solved to all 6 in 27.7 s using the unmodified tree.

The Phase A baseline has since replaced this section's original guess of "six robots with no Pieper
triple". Measured over all 32 defined robots: **26 solve completely, 5 solve nothing, 1 crashes**.
The five that solve nothing are `{ArmRobo, Issue4, KawasakiRS05L, KinovaLite, Raven-II}`, and BH has
excluded `Raven-II` from the hybrid evaluation. See Phase C for why "solves nothing" and "has no
Pieper triple" are *not* the same set, and must not be asserted to be.

The deliverable is literally "robots that move from unsolved to solved", so nothing here is
measurable without a robot-level baseline. That is why the baseline comes first, before any tree
change.

### Why BT-first-with-stubs is the right order

If every leaf on the new hybrid branch stubs out to a status that makes the branch inert, the
restructured tree is **observably identical to today's**. The baseline diff then isolates exactly one
variable: the restructure. Each leaf afterwards goes from stub to real one at a time, and the same
diff proves what that leaf changed and nothing else. The tree shape is built once and never changes
again.

---

## What I verified before planning

`hybrid_plan.md`'s Phase 0 "suspect-text" is **substantially accurate**. Corrections that change the
work:

| Claim in the doc | Reality |
|---|---|
| `comp_detect.py:101` is an "empty `eqns_1u`" give-up condition | The real trigger is at `:127` — `ns == 0` **and** two consecutive passes with an identical progress signature `(ns, len(L1), len(L2), len(L3), len(kequation_aux_list))`. Empty `eqns_1u` only selects an extra diagnostic message (`:132`). |
| The symbolic branch does not signal failure | Already half-solved: `comp_det` sets `no_progress` (`:137`), and `ik_driver.py:109`/`solved_anything()` (`:122`) already gate `emit_outputs()`. Only the in-tree `Priority` needs a new adapter. |
| `build_default_bt()` is assumed to exist | It does — `ikbtfunctions/bt_assembly.py:243`, with `make_leaves()` (`:46`) and `build_worktools()` (`:212`) as documented extension points. `ikbtfunctions/ik_driver.py` already provides `load_robot`/`init_blackboard`/`run_solver`/`solved_anything`/`emit_outputs`. |
| USR: Don't worry about joint limits for this refactor.  `M.jlims` already holds joint limits | **False in any useful sense.** It is a hardcoded ±π 6×2 array in `mechanism.__init__` (`kin_cl.py:257`) that nothing in the repo ever reads or sets per-robot; prismatic joints get ±π as a *length*. Sampling ranges must be defined explicitly. |
| USR: Please de-duplicate the robot list.  Delete Chair6DOF and Issue4DZhang entries. 
|  There are 33 entries in `List` | 41 entries, 33 unique names, **32 with a definition block**. Six names are duplicated (`ICP5p5_A21`, `KR16`, `Issue4`, `MiniDD`, `Khat6DOF`, `DZhang`); `Chair6DOF` is the only listed-but-undefined one (`Issue4DZhang` now survives only in a comment). Note this differs from the "27 arms" figure — worth reconciling. |
| `fk_eqns/KinovaLite_d5zero_pickle.p` left in place | Not in the tree; `fk_eqns/` is gitignored. Measurement 2 must be re-run from a clean checkout. |
| USR: Just find an open number.  Later we can renumber:   Test class 015 taken, 016 free | True. But **010 is double-booked** (`x2y2_transform.py:290` and `output_cpp.py:293`), so don't treat the sequence as clean. Phase B has since taken **016/017/018**; Phase C uses **019**. |

Two hazards that shape the design:

- **`quit()` on the unhappy path, in four places.** USR: If these quit()'s are from incorrect input, corrupt data, etc.  Then it is OK for interactive use if they print informative msgs.    Otherwise if they occur inside a leaf it may be appropriate to replace them with "return(b3.FAILURE)".  `check_the_pickle()` (`ik_classes.py:135`),
  `Num_check()` (`pykinsym.py:84`), `get_variable_index()` (`ik_classes.py:596`), and
  `robot_params()` on an unknown name (`ik_robots.py:66`) all terminate the *process*. Any sweep over
  all robots must therefore run **one subprocess per robot**.
- USR: pickles and the fk_eqns/ directory can dissapear at any time.  The code should just default to computing the FK and Jacobian if the pickle is out of date or not found.  Consider adding the dh parameters to the pickle if appropriate.  **`kinematics_pickle()` ignores `dh` and `pvals` when a pickle already exists**  USR: just rename the derived robots with a suffix and it should keep the pickles separate and clear. 
  (`ik_classes.py:82-86`). A derived robot needs a pickle name that encodes *which* parameter was
  changed, or a re-ranked candidate silently reuses the wrong FK — or trips `check_the_pickle()` and
  kills the process.

Reusable pieces found, to be used rather than rewritten: `Link_N(al,a,d,th)` (`pykinsym.py:102`, a
pure-numpy DH link matrix — the right primitive for fast numeric FK, no sympy and no pickle),
`forward_kinematics_N()` (`kin_cl.py:526`, sympy `subs`-based, slow, and it requires sum-of-angles
keys such as `th_23` in the pose), `M.J66` (`kin_cl.py:454`, already pickled),
`ManipJacobian_S()` (`pykinsym.py:356`), and `bt_problems()` (`tests/bt_assembly_test.py:201`, a
shape-independent BT linter). There is **no** `lambdify` anywhere in the repo, and `M.Jacobian_N()`
referenced at `kin_cl.py:639` does not exist (the block is dead under `JACOBIAN and False`).

---

## Phase A — baseline and the incidental defects  ✅ DONE

Prerequisite for "reproduce our existing successes and failures". Per BH's note
(`OldWorkplans/ImplementationThoughts.md:173`) this is **a record to be diffed, not a set of
assertions**: "does not solve" is a legitimate and expected entry.

**`scripts/robot_baseline.py`** (`scripts/` currently holds only an empty `__init__.py`)

- Iterate the unique names in `robot_params()`'s `List`, **one subprocess per robot** (see the
  `quit()` hazard above), with a per-robot wall-clock timeout.
- Drive each child through `ik_driver.load_robot` → `bt_assembly.build_default_bt` →
  `ik_driver.run_solver`. Set `nodes['compDetect'].read_pause = 0` — it is `2` by default
  (`comp_detect.py:38`) and calls `time.sleep`.
- Record per robot: solved/total unknown count, `no_progress`, per-variable `unknown.solvemethod`
  (already accumulates e.g. `"best ranked, atan2(y,x)"`), `len(R.solutionSet)`, wall time, and the
  number of outer `comp_det` ticks (this is the measurement that answers BH's
  `RepeatUntilSuccess` question in step 2.3 — keep it or fold it in, decided by data).
- Write a checked-in machine-readable record plus a human-readable summary, and provide a
  `--diff <file>` mode that classifies each robot as unchanged / newly-solved / newly-unsolved /
  changed-method. Newly-solved is the *goal*, not a failure.

**Three incidental defects**

1. Delete `'Chair6DOF'` from `List` (`ik_robots.py:49`) — BH's explicit TODO. Also add a
   fall-through guard so any future listed-but-undefined name raises a named error instead of
   `UnboundLocalError: variables` at `ik_robots.py:682`. Leave the six duplicate entries alone but
   have the baseline runner de-duplicate.
2. USR: defer this if possible. `solChecker.py` is Python 2 from line 207 (`print truncated_pose`) and cannot be imported under
   py3. Rather than port the hand-pasted Puma expressions (`:92-171`), rewrite it as a **generic
   round-trip checker**: take the robot name on the command line, import
   `CodeGen/Python/IK_equations<name>.py`, sample poses, and *assert* max elementwise error against
   `forward_kinematics_N()` rather than printing for a human to read. This is the harness Phase F and
   step 2.4 both need. `tests/Wrist_IK_loop_test.py` is the working model.
3. Factor the `v.n = i` loop (`ik_robots.py:681-684`) into an exported helper (e.g.
   `number_unknowns(variables)`) and call it from every place that builds `unknown` objects
   programmatically. Skip it and `get_variable_index()` hits `v.n == 0` and calls `quit()` mid-scan
   (`ik_classes.py:594-596`).

**Verify:** baseline runs to completion over all 32 defined robots; `--diff` against itself reports
zero changes; `python3 -m tests.leavestest` and `python3 -m tests.test_chair_helper` still pass.

### As built

`scripts/robot_baseline.py`, plus the checked-in record in `tests/baselines/robot_baseline.{json,txt}`
and per-robot child logs under `logs/baseline/`. 32 robots in 328 s.

Deviations from the plan above, all deliberate:

- **The solve and the solution-set construction are recorded separately.** The child calls
  `run_solver(..., create_solutions=False)` and then `create_solution_set()` in its own `try`. On the
  plan's single-call design a throw out of `make_LHS_versions()` would cost the whole row — which is
  the thing being measured.
- **`PYTHONHASHSEED=0` in every child.** Parts of the solver iterate over sets, so the choice between
  equally-good solutions (and hence `unknown.solvemethod`) can vary run to run. Without pinning it,
  an empty diff means "nothing changed, probably".
- **`wall_s` and `comp_det_ticks` are recorded but NOT compared.** Wall time is noise, and the tick
  count is a property of the tree's shape — Phase B was expected to change it without changing a
  single solution, and it did.
- **Defect 2 (`solChecker.py`) deferred** per BH's `USR: defer this if possible`. Still needed by
  Phase F.
- **Defect 1 went further than planned:** `List` was replaced by a de-duplicated module-level
  `ROBOT_LIST` (41 entries → 32), since BH asked for de-duplication at the source rather than in the
  runner.
- **The `quit()` hazard was fixed rather than worked around** for the pickle path, per BH's
  `USR:` note that pickles may vanish at any time: `kinematics_pickle()` now recomputes a pickle that
  will not load or whose DH table no longer matches (`dh_tables_match()`), and `check_the_pickle()` is
  advisory and no longer calls `quit()`. The subprocess-per-robot design is still required for the
  other three `quit()` sites.

**Measured results** (`tests/baselines/robot_baseline.txt` is the readable form):

| | robots |
|---|---|
| solved completely | 26 |
| solved nothing | 5 — `ArmRobo`, `Issue4`, `KawasakiRS05L`, `KinovaLite`, `Raven-II` |
| crashed | 1 — `MiniDD` |
| **partially** solved | **0** |

Three findings that change later phases:

1. **No robot is ever partially solved.** Every one either solves completely or solves nothing. This
   makes the "should the hybrid fire on a partial solve" question moot for the current robot set —
   both policies produce an identical baseline today.
2. **The `RepeatUntilSuccess(x10)` budget is not slack** (BH's open question 5). The deepest solve is
   `UR5` at 9 outer passes, and the pass count tracks the unknown count closely (`Puma` 8, `KR16` 8,
   `JennyGuoSp24` 8). Keep 10; a robot with ≥10 unknowns would silently cap.
3. **`MiniDD` crashes before the BT ever ticks.** `ik_robots.py:441` gives it `vv = [0,1,1,1,1]` —
   five entries — while `forward_kinematics()` unconditionally reads `self.vv[5]`
   (`kin_cl.py:409`). It is a 5-DOF arm whose `vv` was never padded to 6 the way the DH table is
   required to be. Fixed by padding to `vv = [0,1,1,1,1,0]`, in its own commit because it moves
   `MiniDD` off `crash` and so must not be folded into a phase whose gate is an empty diff.
   `MiniDD` now solves 5/5 with 8 solutions.

   Two follow-ons worth recording. **`Sims11` has the same class of defect in the other direction** —
   `vv` is *seven* entries long against a six-row DH table. It is harmless today because nothing reads
   past `vv[5]`, and `Sims11` solves 5/5, so it was left alone; but it is an off-by-one somebody
   should look at, and it means `len(vv) == 6` is worth asserting rather than assumed. **And the DH
   6-row rule should extend to `vv`:** `CLAUDE.md` tells you to pad the DH table to six rows and says
   nothing about `vv`, which is exactly how this got in.

---

## Phase B — the BT skeleton, with every new leaf stubbed  ✅ DONE

All in `ikbtfunctions/bt_assembly.py`. Target shape:

```
Priority([ symbolic_branch, hybrid_branch ])

symbolic_branch = Sequence([ Priority([ RepeatUntilSuccess(solveRoutine, 10), Succeeder() ]),
                             symbolic_ok,
                             output_gen_full ])

hybrid_branch   = Sequence([ Inverter(pieper_id),
                             simplified_arm,
                             solve_simplified,
                             output_gen_hybrid ])
```

`solveRoutine` and everything under it is unchanged from `build_default_bt()`.

Four details that are easy to get wrong:

- **The `Priority([RepeatUntilSuccess(...), Succeeder()])` wrapper is required.**
  `RepeatUntilSuccess` returns FAILURE when it exhausts its 10 loops, which would abort the enclosing
  `Sequence` before `symbolic_ok` ever ticks and hand control to the hybrid branch. Today a
  loop-exhausted *partial* solve still emits outputs, and that must be preserved. This mirrors the
  existing precedent at `bt_assembly.py:280,284`.  USR: the RepeatUntilSuccess node can be moved iside the symbolic solver leaf, "solveRoutine" to eliminate this problem.  
- **`Inverter(pieper_id)`** is the gate, and it is diagnostic as well as functional: a robot that
  *has* a triple and still failed symbolically (`ArmRobo`, `Raven-II`) is a solver defect, not a
  geometry problem, so the hybrid must not fire. `b3.Inverter` exists at
  `b3/decorators/inverter.py:5`. USR: Raven-II is a super challenging robot because it has \alpha_j values which are not multiples of \pi/2 (i.e. sin/cos do not evaluate to +-{0,1}) Do NOT use RAVEN-II in the evaluation of the hybrid method (yet).
- **Codegen leaves must be opt-in.** `tests/test_chair_helper.py:14` documents that it deliberately
  does not call `emit_outputs()` and leaves `LaTex/`/`CodeGen/` alone; an unconditional codegen leaf
  would silently break that promise. Give `build_default_bt()` a default-off parameter (following the
  `invariantGen.enabled` precedent at `bt_assembly.py:160`) so library and test callers get a tree
  with no file side effects, and `ikSolver.py` opts in.

**Stub statuses, chosen so the hybrid branch is inert:**

| leaf | Phase B status | why |
|---|---|---|
| `symbolic_ok` | **real** | tiny, and it is the load-bearing adapter |
| `output_gen_full` | **real** | calls `R.create_solution_set()` then `ik_driver.emit_outputs(R, unks)`; returns SUCCESS |
USR: you can add the nodes below one at a time so that this is not a big deal. 
| `pieper_id` | stub → **SUCCESS** | `Inverter` turns it into FAILURE, so `hybrid_branch` never advances. Semantically honest: "pretend every robot has a triple ⇒ never simplify" |USR: I don't understand this line at all. 
| `simplified_arm` | stub → FAILURE | second line of defence |
| `solve_simplified` | stub → FAILURE | |
| `output_gen_hybrid` | stub → FAILURE | |

Because `output_gen_full` now owns `create_solution_set()`, `run_solver()` must stop doing it when
the tree owns codegen — thread that through rather than double-calling. Keep `solved_anything(bb)`
for `ikSolver.py`'s "no solution generated" message.

**Verify Phase B:** `bt_problems(bt)` returns `[]` (register the new leaves in `REQUIRED_SUPPORT` /
`OPTIONAL_LEAVES` and give every one a unique `.Name` — the linter flags unnamed and
duplicate-named nodes, and shared instances, which matters once a nested tree exists); the full
baseline diff is **empty** across all 32 robots; `tests.leavestest`, `tests.bt_assembly_test` and
`tests.test_chair_helper` pass. This is the checkpoint BH asked for.

### As built

Both of BH's steers above turned out to simplify the tree rather than complicate it, so the shape is
**smaller** than the target at the top of this section:

```
Priority[ symbolic_branch, hybrid_branch ]

symbolic_branch = Sequence[ symbolic_loop(x10, solveRoutine), output_gen_full ]

solveRoutine    = Sequence[ sub_transform,
                            RepeatUntilSuccess(x6, Sequence[ assigner, sum_id, worktools ]),
                            updateL,
                            comp_det ]        # unchanged

hybrid_branch   = hybrid_stub                 # always FAILs
```

- **`symbolic_loop` replaced `RepeatUntilSuccess` at the root** (`ikbtleaves/symbolic_loop.py`), per
  BH's `USR:` note at the first bullet. It runs the identical passes as a Python `while` inside
  `tick()`, which lets it *choose* its exit status: SUCCESS if ≥1 variable was solved, FAILURE if
  none. That killed **three** nodes at once — the `Priority([RepeatUntilSuccess, Succeeder()])`
  wrapper and the separate `symbolic_ok` adapter both disappear, because the loop node *is* the
  adapter. It keeps a `max_loop` attribute so `bt_problems()` still checks the budget, and sets
  `symbolic_passes` / `symbolic_exhausted` on the blackboard for the Phase A measurement.
- **The four-stub table is now one stub.** Per BH's `USR: you can add the nodes below one at a time`,
  `hybrid_branch` is a single `hybrid_stub` that always FAILs. This also disposes of the
  `pieper_id → SUCCESS` row BH could not parse: with `Inverter(pieper_id)` as the gate the hybrid runs
  when a robot has **no** triple, so keeping the branch inert would have required `pieper_id` to lie
  and claim every robot has one. One always-failing leaf says the same thing without the double
  negative. The inverter polarity is recorded in a test (`test_hybB`) because it is the thing that
  will get inverted by accident later.
- **Codegen is a leaf, default off** (`ikbtleaves/output_gen.py`, `output_gen_full`), following the
  `invariantGen.enabled` precedent. Disabled it does nothing whatsoever — not even
  `create_solution_set()` — so `run_solver()` keeps ownership on the ordinary path. Exactly one of the
  two must call it: it appends to `unknown.LHSversionNames` and is **not idempotent**.
  `build_default_bt(codegen=True)` hands the tail end to the tree, and `ikSolver.py` is the only
  caller that opts in (and then passes `create_solutions=False`).
- **The three new leaves went into `OPTIONAL_LEAVES`, not `REQUIRED_SUPPORT`.** The linter's stated
  philosophy is that the BT is an experimental surface and the file "does not compare the tree to a
  stored shape"; none of the three is needed to solve a robot (the outer loop can be a plain
  `RepeatUntilSuccess`, codegen can live in the caller, the hybrid does not exist yet). What the
  *shipped* tree guarantees is asserted directly instead — `test_btaR` (codegen off unless asked,
  including through the `nodes=` path) and `test_btaS` (hybrid inert, exactly one solve loop with a
  finite budget, `require_complete` off).

**Result:** `python3 -m scripts.robot_baseline --diff --no-save` → **32 robots, all unchanged**, exit
0. `bt_problems()` `[]` on both the default and the `codegen=True` tree; `tests.bt_assembly_test`
19/19; `tests.leavestest` clean over 6 consecutive runs; `tests.test_chair_helper` 3/3;
`tests.helpertest` 4/4. End to end, `ikSolver.py Wrist` writes both artifacts through the new leaf and
`ikSolver.py KinovaLite` writes nothing and says so.

Test classes added: **016** `symbolic_loop`, **017** `output_gen`, **018** `hybrid_ik`. Note for later
phases: a test double that lives in a leaf file must be named `test_*`, or `bt_assembly_test.py`'s
leaf-inventory scan picks it up as a real leaf.

**One loose end:** a single early `leavestest` run reported
`ERROR: runTest (ikbtleaves.x2y2_transform.TestSolver010)`. It has not recurred in 6 subsequent
full-suite runs and cannot be caused by the restructure (`x2y2_transform` does not import
`bt_assembly`, and 010 runs before the new tests in `suite3`), but the traceback was not captured, so
call it a pre-existing intermittent rather than resolved. Worth knowing the two run paths are not
equivalent: standalone runs `test_x2y2B_*` and `test_x2z2` as separate instances, while the suite runs
both through `runTest` sharing state — so a flake that only appears in the suite is plausible.

---

## Phases C–F — the leaves, in tick order

Each phase replaces exactly one stub and is verified by a baseline diff before the next begins.

### Phase C — `pieper_id`

New module **`ikbtbasics/dh_analysis.py`**: pure DH-table arithmetic, no FK and no symbolic solving,
so it runs in milliseconds and unit-tests without pickles.

`pieper_triples(dh, pvals, ndof)` → satisfied triples by kind.

#### The DH convention, confirmed against the code

`kin_cl.py:280` says it outright — *"standardize on the order `alpha N-1, a N-1, d N, theta N` for
the DH table columns"* — and `Link_S`/`Link_N` (`pykinsym.py:92,102`) are exactly Craig's
`Rot_x(α_{n-1}) Trans_x(a_{n-1}) Rot_z(θ_n) Trans_z(d_n)`. So 0-indexed row `r` holds
`[α_r, a_r, d_{r+1}, θ_{r+1}]`, as this plan said.

Deriving the geometry from that transform rather than from memory: **joint `n`'s axis is the Z line of
frame `{n}`**, because `θ_n` enters as `Rot_z(θ_n)` and `Trans_z(d_n)` only slides along that same
line. Expressed in frame `{n}`,

- axis `n`   passes through the origin along `z`
- axis `n+1` passes through `[a_n, −sin(α_n)·d_{n+1}, cos(α_n)·d_{n+1}]` along `[0, −sin(α_n), cos(α_n)]`

so axes `n`,`n+1` meet iff `a_n = 0` (at the origin of `{n}`), axes `n+1`,`n+2` meet iff
`a_{n+1} = 0` (at the origin of `{n+1}`), and all three share a point iff those two meeting points
coincide — `d_{n+1} = 0`. Every cell involved lives in rows `n` and `n+1`:

- axes `j, j+1, j+2` **intersect** iff `dh[j,1] == 0 ∧ dh[j+1,1] == 0 ∧ dh[j,2] == 0`
- axes `j, j+1, j+2` **parallel**  iff `sin(dh[j,0]) == 0 ∧ sin(dh[j+1,0]) == 0`

**The cell references are right, but `j` is a 1-based JOINT number used as a 0-based ROW index.**
That is the easy thing to get wrong here. `j` runs `1 … ndof−2`. Starting the loop at `j = 0` invents
a triple containing a nonexistent joint 0 and reads `a_0`/`d_1`, which are zero for most robots — so
it manufactures a spurious triple on almost every arm. With `j` correctly restricted, `Brad` (3 DOF)
reports **zero** triples, not the five this plan previously recorded.

#### Three corrections the verification turned up

All three were found by computing the axes numerically from `Link_N` and testing concurrency
geometrically, then comparing against the rule — **99 triples over all 32 robots, and after these
fixes the rule and the geometry agree on every one**, at three random configurations each.

1. **Use `.is_zero`, never `== 0`.** `sp.Float(0.0) == 0` is `False` in sympy, and the DH tables mix
   Integer `0` with Float `0.0` (`Stanford`, `Bartell`, `Palm13`, … use float zeros). With `== 0` the
   detector reports **no triples at all** for those robots — silently, and it looks like bad geometry
   rather than a broken test. `.is_zero` also returns `None` for an undecidable symbol, which is the
   three-valued answer this plan asks for, for free.
2. **The intersect rule needs a collinear-axes clause.** `a_n = 0 ∧ a_{n+1} = 0 ∧ d_{n+1} = 0` misses
   the case where two of the three axes are *the same line*, which happens iff `a = 0 ∧ sin(α) = 0`
   between them (α = π gives the same line, only reversed). Then there are only two distinct lines,
   and they are concurrent for **any** `d_{n+1}` — including when `d_{n+1}` is a prismatic joint
   *variable*, which is the common "prismatic joint sliding along the axis the next joint rotates
   about" design. This is exactly `Stanford`, and it also affects `Bartell`, `Palm13`, `Srisuan11` and
   `Raven-II`. Corrected rule:

   ```
   collinear(n, n+1)   ==  a_n   == 0  and  sin(al_n)   == 0
   collinear(n+1, n+2) ==  a_n+1 == 0  and  sin(al_n+1) == 0

   intersect(j) == a_j == 0 and a_j+1 == 0
                   and ( d_j+1 == 0 or collinear(j+1, j+2) or collinear(j, j+1) )
   ```
3. **There are no unvalued constants to worry about.** Every undecidable cell across all 32 robots is a
   *prismatic joint variable* (`d_2`/`d_3`/`d_4` on `ICP5p5_A21`, `Bartell`, `Sims11`, `Olson13`,
   `Stanford`, `Raven-II`, `Parkman13`, `Palm13`, `Srisuan11`) — **not** a symbol missing a `pvals`
   entry. So this plan's claim that "`Sims11` has one, `d_2`" is wrong: `d_2` is `Sims11`'s prismatic
   joint variable. For a prismatic variable the correct answer is "not identically zero", which
   falsy-`None` already gives, and the collinear clause then recovers the genuinely-concurrent cases.
   BH's `USR: Let's set missing pvals to a sensible value` therefore does not apply to the triple
   detector at all — it may still be needed for Phase D's displacement metric, which does need numbers.

Leaf `pieper_id(b3.Action)` in **`ikbtleaves/hybrid_ik.py`** — alongside `hybrid_stub`, which it will
sit in front of. (The plan originally said `ikbtleaves/simplify_dh.py`; Phase B already created
`hybrid_ik.py` as the home for the hybrid branch's leaves, so they go there.) It reads `Robot`,
computes triples from `R.Mech.DH` and `R.Mech.pvals`, sets `blackboard['pieper_triples']`, and returns
SUCCESS iff at least one triple exists.

#### Tree change — BH's shape, with the gate on the hybrid only

BH proposed, and this is what is built:

```
root     = Sequence[ analysis, report_gen ]
analysis = Priority[ symbolic_branch, hybrid_branch ]        # Priority == Selector
```

with `pieper_id` keeping standard `_id` semantics (SUCCESS iff a triple qualifies). Two clear wins
over what preceded it: **one** report generator instead of one per branch, and no always-SUCCESS
special case.

**One correction to the proposal.** BH's `analysis = Selector[ Sequence[pieper_id, symbolic_branch],
hybrid_branch ]` puts the gate ahead of the *symbolic* solver, which assumes Pieper's condition is
necessary — the very thing BH corrected earlier. Measured by building that exact tree and ticking it:
`Brad` and `Sims11` go from solving completely to solving **nothing**, because `symbolic_loop` never
ticks. It would break all **9** robots in the no-triple + solved cell (`Axtman13`, `Brad`, `DZhang`,
`ICP5p5_A21`, `Mackler13`, `MiniDD`, `Olson13`, `Sims11`, `Wachtveitl`).

So the gate moved to the hybrid branch only:

```
hybrid_branch = Sequence[ Inverter(pieper_id), hybrid_stub ]
```

`b3.Priority` tries the symbolic branch **first and unconditionally**, so a no-triple arm still gets
the full symbolic solver; the hybrid is reached only when symbolic came up empty *and* the geometry
really lacks the structure. `ArmRobo` and `Raven-II` — triple present, IKBT still failed — correctly
get no hybrid, which is the diagnostic BH wanted.

`Sims11` is the case that demonstrates it: no triple, solves 5/5, and its report now says "no
triple ... this does not prove that no closed form exists" — the report is its own evidence.

*Terminology, since BH asked:* **`b3.Priority` is the standard Selector** (a.k.a. Fallback) — ticks in
order, stops at the first non-FAILURE. **`b3.OrNode` is not a Selector**: it is a local 2017 addition
that runs *all* its children and returns SUCCESS if any succeeded. That difference is load-bearing at
`bt_assembly.py:196`, where `rank` needs both the tan and sin/cos candidates to choose between.

#### Superseded — why `pieper_id` was briefly always-SUCCESS

BH's added requirement is what forced this:  *"the pieper_id leaf should generate a concise statement
of its results (in LaTeX format) which can be incorporated in the final report, **regardless of which
branch ultimately produces a solution**."*

That cannot work with `pieper_id` inside the hybrid branch. `b3.Priority` stops at its first
non-FAILURE child, so when the symbolic branch succeeds the hybrid branch never ticks — and the 18
robots that solve symbolically would silently get no geometry section. `pieper_id` has to tick **ahead
of the split**:

```
root = Sequence[ pieper_id, Priority[ symbolic_branch, hybrid_branch ] ]
```

Two consequences, both improvements:

- **`pieper_id` always returns SUCCESS.** At the head of a Sequence a FAILURE would abort the whole
  solve, which is an absurd power for a leaf whose job is to *describe* the robot. Its answer goes on
  the blackboard instead — `pieper_triples`, plus a `pieper_ok` flag — and nothing is gated on its
  status. Note this breaks the repo's `_id` convention (SUCCESS iff something was identified); the
  convention exists for ID leaves paired with a solver in a Sequence, which this is not.
- **The `Inverter` disappears.** The hybrid branch's gate now reads `pieper_triples` off the
  blackboard and says what it means — "fire when there is no triple" — instead of expressing it as
  the inverse of a leaf that has to succeed in order to mean failure. That was the construction BH
  could not parse in the original Phase B table, and it is now gone for a structural reason rather
  than a stylistic one. **The gate must also refuse to fire when `pieper_ok` is False:** "we could not
  tell" is not "there is no triple".

`hybrid_stub` still always FAILs, so the branch stays inert and the **baseline diff must still be
empty**. `pieper_id` is registered in `OPTIONAL_LEAVES` with a unique `.Name`.

#### The report statement

`dh_analysis.pieper_latex(dh, pvals, ndof, robot_name)` returns a `\section{Joint Axis Geometry
(Pieper Condition)}` block. **`report_gen` generates it**, not `pieper_id` — which is what dissolves
the placement problem entirely: the statement is a *report* concern, so the report generator makes it,
and it therefore appears whichever branch won without anything having to tick in a special position.
`report_gen` stashes it on `Robot.pieper_latex` and `output_latex_solution()` appends it right after
Kinematic Parameters via `getattr(Robot, 'pieper_latex', None)` — a Robot restored from an older
pickle has no such attribute, and a missing statement must not break the report. A failure to build
the statement is a printed warning, never a blocked report.

It carries the **witness**, not just a verdict — which cells are zero — so a reader can check our
arithmetic against their own drawing, and so the collinear case is visibly distinct from the ordinary
one. `Stanford` renders as

> Axes $(2,3,4)$ intersect in a common point, since $a_2 = 0, a_3 = 0, \sin(\alpha_3) = 0$
> (axes 3 and 4 are collinear).

The wording is careful about the logic on purpose, because the tempting sentence is false. It says
Pieper's condition is **sufficient** and *not known to be necessary*, and for a robot with no triple it
says so does **not** prove no closed form exists — 9 of the 32 robots have no triple and solve
completely. A report claiming otherwise would be worse than no report, so both phrasings are asserted
in the tests.

Verified end to end: `Wrist`, `Stanford` and `Chair_Helper` reports all compile under `pdflatex` with
zero errors, which also covers escaping (`Chair\_Helper`) and `\alpha` rendering.

Deliberately **not** added to `output_FK_equations()` (the `fkOnly.py` report) — that path never ticks
the BT, so nothing would have written the statement. Worth doing if you want it there;  it would mean
calling `dh_analysis` directly from the FK generator.

*Naming:* **resolved.** `pieper_id`, node `Name = 'Pieper Triple ID'` — BH asked for the correct
spelling of Pieper (Donald Pieper, 1968; `hybrid_plan.md` writes `ID_Peiper`) and for consistency with
the other `_id` leaves (`algebra_id`, `sum_id`, `tan_id`).

#### Verify — and what must *not* be asserted

The original criterion here was wrong, and BH's `USR:` note is what breaks it:

> USR: in general, this rule is not proven.  Pieper condition is "sufficient" but not known to be
> "necessary"

That is decisive. Pieper's condition (three consecutive axes intersecting, or parallel) is
**sufficient** for a closed form to exist, and is *not* known to be necessary. So neither half of the
planned assertion holds:

- *solves ⇏ has a triple.* IKBT's rule set is not Pieper's construction; it can solve arms about which
  Pieper says nothing. "The detector must find a triple for every robot that currently solves" is
  therefore not a valid test.
- *no triple ⇏ unsolvable.* A general 6R still has a closed form (Raghavan–Roth, up to 16 solutions);
  it is just not one IKBT's leaves can find.

The one implication that *does* hold is the useful one:

> **has a triple ⇒ a closed form exists**, so a robot with a triple that IKBT fails on is an **IKBT
> defect**, not a geometry problem.

That is exactly the diagnostic BH wanted, and it is what the `Inverter(pieper_id)` gate encodes: don't
simplify an arm whose geometry is already good enough, because the bug is in the solver.

The measured Phase A baseline also replaces the doc's guessed set. `Wachtveitl` (6/6), `Olson13` (7/7)
and `Sims11` (5/5) **all solve completely**, so the planned no-triple set
`{KinovaLite, Issue4, KawasakiRS05L, Wachtveitl, Olson13, Sims11}` is wrong on three of its six
members. The robots that solve nothing are `{ArmRobo, Issue4, KawasakiRS05L, KinovaLite, Raven-II}` —
which matches `comp_detect.py`'s empty-`eqns_1u` comment exactly — plus `MiniDD`, which crashes for an
unrelated reason (see Phase A). Note that "solves nothing" is an *IKBT* fact, not a geometric one, so
it still cannot be used as the expected output of a geometry function.

**Hard assertions** — geometry only, no reference to solve outcomes. All of these have been
**measured** with the verified rule (see `scripts/axis_triple_check.py`), so they are known-good
expected values, not guesses:

- `Puma`, `Pumaoffset`, `KawasakiRS007L`, `KR16`, `Khat6DOF` report the classic spherical wrist:
  intersecting at (4,5,6).
- `UR5`, `Arm_3`, `Parkman13`, `JennyGuoSp24` report a **parallel** triple — `par(2,3,4)`,
  `par(1,2,3)`, `par(2,3,4)`, `par(3,4,5)` respectively. Worth having: parallel is the branch of the
  rule with no test coverage otherwise.
- `Stanford` reports `int(2,3,4)`, `int(3,4,5)`, `int(4,5,6)` — the collinear-axes regression test.
  With the plan's original rule it reported **none of them**, on two counts at once (float zeros and
  the missing collinear clause).
- `Brad` (3 DOF) reports **zero** triples — the regression test for restricting `j` to real joints.
- `KinovaLite`, `Issue4`, `KawasakiRS05L`, `Sims11`, `Olson13`, `Wachtveitl`, `MiniDD`, `DZhang`,
  `Mackler13`, `Axtman13`, `ICP5p5_A21` report **no** triples.
- Synthetic tables: an all-parallel table reports a parallel triple at every legal `j`; a table with
  `a_j ≠ 0` everywhere and no `sin(α)` zero reports none.
- Cross-check against the independent geometric oracle (axes computed from `Link_N`, concurrency
  tested numerically) — 99 triples over 32 robots, at several configurations each.

**The cross-tab — measured, not pending.** This was the intended deliverable of Phase C and the
verified rule already produces it:

| cell | n | robots |
|---|---|---|
| has triple + solved | 18 | `Arm_3`, `Bartell`, `Chair_Helper`, `Craig417`, `Frei13`, `JennyGuoSp24`, `KR16`, `KawasakiRS007L`, `Khat6DOF`, `Minder13`, `Palm13`, `Parkman13`, `Puma`, `Pumaoffset`, `Srisuan11`, `Stanford`, `UR5`, `Wrist` |
| **has triple + UNSOLVED** | **2** | **`ArmRobo`, `Raven-II`** — IKBT solver defects: a closed form exists and we are not finding it. Hybrid must not fire. |
| NO triple + solved | 9 | `Axtman13`, `Brad`, `DZhang`, `ICP5p5_A21`, `Mackler13`, `MiniDD`, `Olson13`, `Sims11`, `Wachtveitl` |
| **NO triple + UNSOLVED** | **3** | **`Issue4`, `KawasakiRS05L`, `KinovaLite`** — the hybrid's target population |

Two things fall out of this that matter more than the detector itself:

- **BH's prediction is confirmed exactly.** The has-triple-but-unsolved cell is `{ArmRobo, Raven-II}`
  and nothing else — precisely the two robots BH named as solver defects rather than geometry problems.
  Those are IKBT bugs with a guaranteed closed form, and worth their own work item.
- **Pieper is empirically not necessary**, exactly as BH said: **9 robots have no triple and solve
  completely**. Any test asserting "solves ⇒ has a triple" would have failed on nine arms.

So the hybrid's target population is **three robots** — `Issue4`, `KawasakiRS05L`, `KinovaLite` —
with `Raven-II` excluded anyway (its `α_j` are not multiples of π/2, so `sin`/`cos` do not evaluate to
`±{0,1}`; a hard case for reasons unrelated to simplification). `KinovaLite` is the known-good one,
already measured solving 6/6 with `d_5 → 0`.

New test class **`TestSolver019`** — 016, 017 and 018 were taken by Phase B — wired into
`tests/leavestest.py` by named import plus `suite3.addTest(...)`, with a `runTest()` method.
`python3 -m ikbtbasics.dh_analysis` for the module self-test.

### Phase D — `simplified_arm`  ✅ DONE

Adds to `dh_analysis.py`:

- `candidate_simplifications(dh, pvals, ndof)` → for each *unsatisfied* triple, the DH cells that
  would have to be zeroed (intersection) or snapped to a multiple of π (parallel), each with its
  numeric magnitude. At most 8 candidate triples × 2 condition types — enumerable exhaustively in
  milliseconds, not an optimization problem. Symbols with no `pvals` value are reported as
  **undecidable, never silently skipped** (`Sims11` has one, `d_2`). USR: Let's set missing pvals to a sensible value for now.   d_x = 1.0, angles = \pi/2
  → BH's defaults are implemented (`DEFAULT_LENGTH = 1.0`, `DEFAULT_ANGLE = π/2`) and any use of them
  is reported in the metric's `defaulted` field rather than hidden. But **measured, no robot needs
  them**: every undecidable triple-relevant cell across all 32 robots is a *prismatic joint variable*,
  not a missing `pvals` entry, so `Sims11`'s `d_2` is its joint variable and not an unvalued constant.
  The defaults are a safety net for future robots, not a live code path.
- `displacement_metric(dh, dh_simp, pvals, vv, ndof, n, seed)` → mean and max position and
  orientation deviation over sampled joint space. **Build this on `Link_N` (`pykinsym.py:102`)** —
  pure numpy, ~100× faster than `forward_kinematics_N`'s `subs`, and it sidesteps the sum-of-angles
  key requirement entirely. Since `M.jlims` is a dead placeholder, define and document the sampling
  ranges explicitly: ±π for revolute joints, and for prismatic joints a range derived from the DH
  length scale. Take a `seed` so results are reproducible.

Leaf `simplified_arm(b3.Action)`: puts the ranked candidate list, with numeric magnitudes, on the
blackboard; returns FAILURE if a required `pval` is missing or no candidate exists.

**Ranking key.** Task-space displacement, for three reasons the doc already establishes: it puts
length-zeroing and angle-snapping in comparable units; it caught the compensation error in
measurement 3 (rolling the 57 mm offset into `d_6` makes things *worse*, 80.6 mm vs 57.0, because the
offsets are orthogonal and add in quadrature); and it is cheap enough to run inside a leaf.

**Your TODO at `hybrid_plan.md:89` (joint-space error) is a separate, later function**, because the
only way to get "one of the joint position solutions of the approximate model" is to actually solve
the approximate model — minutes per candidate. So: `joint_seed_error(...)` samples `q_true`, computes
`T = FK_true(q_true)`, runs the simplified robot's generated `ikin_<name>(T)`, and reports
`min_k ‖wrap(q_k − q_true)‖` plus the fraction of samples where `ikin` returned `False`. It runs
**once on the chosen candidate** as the go/no-go on whether the seed is usable (your ~π yardstick),
not as the ranking key for all candidates. Flag if you want ranking itself moved to joint space —
that makes `simplified_arm` a batch tool rather than a leaf.

**Verify:** unit tests over synthetic DH tables; on `KinovaLite` the top candidate must be `d_5 = 0`
with mean ‖Δp‖ ≈ 57.0 mm and Δθ = 0, reproducing the doc's measurement. Baseline diff still empty.

#### As built — `ikbtbasics/dh_analysis.py`

One module carrying both phases' arithmetic, 11 tests (`TestSolver019`), no FK, no pickles, ~1.4 s.
`scripts/axis_triple_check.py` now validates **the shipped module** rather than a copy of the rule, so
the two cannot drift.

`pieper_triples` / `triple_report` / `has_pieper_triple` (Phase C's arithmetic),
`candidate_simplifications`, `displacement_metric`, `rank_candidates`.

**Candidate routes.** An unsatisfied triple can be bought in four ways, not one, and the extra three
matter:

- `zero_offsets` — zero whichever of `a_j`, `a_{j+1}`, `d_{j+1}` are non-zero. **Unavailable when
  `d_{j+1}` is a prismatic joint variable** — you cannot zero a joint variable. That route is reported
  `blocked` with a reason rather than silently omitted, because a missing route looks exactly like
  "this triple needs no work".
- `collinear_hi` / `collinear_lo` — zero the `a`s and snap one `α` to a multiple of π, making two of
  the three axes one line. Needs no `d_{j+1} = 0`, so it is *the* route for a prismatic arm.
- `parallel` — snap both `α`s to multiples of π.

**Both π-multiples are enumerated**, not just the nearest. `sin(α) = 0` is satisfied by 0 and by π
alike, but α = π flips the axis, so the two give materially different arms — and π/2, the commonest
value in these tables, is exactly equidistant from both. Which is cheaper is not decidable from the
angle, so both become candidates and the ranking settles it.

**Ranking key** is BH's angle/axis scalar from question 3, not position alone:
`mean(‖Δp‖ + w_rot·θ)`, with `θ` the rotation angle of `R_trueᵀ R_simp`. This is what puts zeroing a
length and snapping an angle in comparable units — the stated reason for ranking in task space at all.
`w_rot` defaults to `length_scale()` (one radian costs one characteristic link length) because **the
DH tables carry no units** — `KinovaLite` is in mm, `Issue4` in m — so BH's "1 metre per radian"
cannot be applied blind; pass `w_rot=1000.0` for a mm table to get exactly that convention.

`length_scale()` is the median magnitude of the non-zero constant `a`/`d` cells, and it also sets the
prismatic sampling range (±`length_scale`, revolute ±π). Both ranges are **declared here**, since
`M.jlims` is a dead ±π placeholder nothing sets per robot.

**Two numerical traps found while building it**, both of which produce plausible-looking wrong
numbers:

1. **The rotation angle must come from `atan2`, not `arccos((tr−1)/2)`.** `arccos` has infinite
   derivative at `R = I`, so an O(1e-16) rounding error in the trace emerges as **O(1e-8) radians** —
   and that is exactly the regime that matters, because a candidate which changes only a *length* has
   to measure a clean zero orientation error. `atan2(‖skew‖/2, (tr−1)/2)` is linear in the
   perturbation and returns exactly 0.0 for `R = I`.
2. **Resolve the constant cells once, outside the sampling loop.** Doing `subs()` per sample makes the
   metric far too slow to rank ~28 candidates inside a leaf. `compile_rows()` pre-resolves every
   constant to a float and leaves one slot per joint, so the loop is pure numpy over `Link_N`.

#### Measured — the metric reproduces both of `hybrid_plan.md`'s numbers

| candidate | mean ‖Δp‖ | Δθ |
|---|---|---|
| `d_5 → 0` | **57.000** | 0 |
| `d_5 → 0` *and* `d_6 += 57` ("compensated") | **80.610** = 57√2 | 0 |

The second is the doc's 80.6 mm, and it is an *independent* check: nothing in the code knows that
number, and it confirms the doc's explanation that the two offsets are orthogonal and add in
quadrature. Both are asserted in `TestSolver019`.

#### Measured — every target robot has a single-parameter winner

Ranked over all 28 candidates each (`rank_candidates`, n=120, seed=1):

| robot | top candidate | cost | next-best | margin |
|---|---|---|---|---|
| `KinovaLite` | `d_5: 57 → 0` | 57.0 | `d_4: 245 → 0` | 4.3× |
| `Issue4` | `d_5: 0.029 → 0` | 0.03 | `a_3, d_4 → 0` | 15× |
| `KawasakiRS05L` | `a_3: 80 → 0` | 80.0 | `a_2, a_3 → 0` | 4.3× |

So this plan's claim that "in each one a *single* DH parameter is responsible" now holds for **all
three** target robots, not just the one that was measured by hand. Note `Issue4` fails on `d_5` at the
wrist, structurally the same defect as `KinovaLite`.

One case worth knowing about, from `KawasakiRS05L`: snapping `al_5` scores `mean ‖Δp‖ = 0.00` exactly —
the wrist origin does not move — while costing 341 through the rotation term. That is correct, not a
quirk: with `a_5 = d_6 = 0` the frame origin is independent of `α_5`, but changing it changes *the axis
joint 6 rotates about*, which is a real loss of orientation capability. A position-only ranking would
have called that free and picked it first. It is the clearest argument for the combined scalar.

#### As built — the `simplified_arm` leaf

```
hybrid_branch = Sequence[ Inverter(pieper_id), simplified_arm, hybrid_stub ]
```

`SUCCESS` with `simplification_candidates` (ranked, cheapest first) and `simplification_choice` (the
winner) on the blackboard; `FAILURE` when nothing is buyable. Each candidate carries its edits and
their magnitudes, the derived DH table, and its measured cost — everything Phase E needs to build the
derived robot.

**It refuses to act when `pieper_ok` is False, and that is the load-bearing detail.**
`Inverter(pieper_id)` turns *both* "this arm has no triple" and "I could not read the DH table" into
SUCCESS, because `pieper_id` returns FAILURE for both. `simplified_arm` is the first thing downstream
that would act on that conclusion, so it is where the ambiguity has to be resolved. Simplifying an arm
because we failed to parse its parameters would be the worst available outcome: a derived robot,
solved perfectly, describing nothing.

`seed` is fixed (and `n_samples` is a leaf attribute, default 200) because the phase-gate discipline
needs a rerun to make the same choice.

**Measured, live through the real gate:**

| robot | gate + rank | candidates | choice | cost | wall |
|---|---|---|---|---|---|
| `KinovaLite` | SUCCESS | 28 | `d_5: 57 → 0` | 57.000 | 2.9 s |
| `Issue4` | SUCCESS | 28 | `d_5: 0.029 → 0` | 0.029 | 2.9 s |
| `KawasakiRS05L` | SUCCESS | 28 | `a_3: 80 → 0` | 80.000 | 2.6 s |
| `Puma` | **FAILURE** | — | — | — | 0.4 s |

`Puma` is the control: it *has* a triple, so `Inverter(pieper_id)` fails and the Sequence aborts before
`simplified_arm` ever ticks. ~2.9 s is affordable because the leaf ticks at most once per solve, and
only for an arm that already failed symbolically.

New structural test `test_btaV` asserts the leaf is inside the hybrid branch, sequenced *after* the
Pieper gate, and unreachable from the symbolic branch — ranking simplifications on the symbolic path
would be pure waste, and a simplification chosen for an arm that solved exactly would be a standing
invitation to use it.

**Not built:** a LaTeX section stating the chosen simplification. It would never appear yet —
`hybrid_stub` still FAILs, so the hybrid path produces no report at all. It belongs with Phase F's
requirement that the artifacts say which robot the equations actually describe, and `report_gen` being
shared means it is a few lines there rather than a second generator.

### Phase E — `solve_simplified`

Builds the derived robot and ticks a nested `build_default_bt()` on a **fresh** blackboard, which
avoids mutating the outer blackboard's `Robot`. Three traps:

- **Its own pickle name, encoding the modification** (e.g. `KinovaLite_d5_0`), so that a re-ranked
  candidate cannot silently reuse the wrong FK. `kinematics_pickle(rname, dh, ...)` takes the name
  independently of the DH table, so no change to `robot_params()` is needed. Phase A already removed
  the sharp edge here — a pickle whose DH table no longer matches is now recomputed rather than
  triggering `quit()` — but a distinct name is still wanted, both to keep the cache useful across
  candidates and because the name is what a human reads in `fk_eqns/`.
- **Fresh `unknown` objects**, because `set_solved()` mutates them in place — and they must go
  through `number_unknowns()` (`ikbtfunctions/ik_robots.py`), the Phase A helper.
- **A separate node instance set** for the nested tree; `bt_problems()` correctly rejects a node
  instance appearing twice, since b3 keys per-node state on the blackboard by node id.

**Verify:** `KinovaLite` reaches all 6 variables solved via the hybrid branch, matching the doc's
27.7 s measurement. The baseline diff now shows exactly the intended change — whichever robots Phase
C's cross-tab put in the **no triple + unsolved** cell move to a new "solved (hybrid)" status, and
**nothing else moves**. That is at most four robots and possibly two, not the "six" this plan
originally assumed: `Raven-II` is excluded, `ArmRobo` is expected to classify as a solver defect, and
`Wachtveitl` / `Olson13` / `Sims11` were never unsolved in the first place.

Note the baseline record will need a new status value (or a flag) to distinguish "solved (hybrid)"
from "solved"; `COMPARED` in `scripts/robot_baseline.py` already includes `status`, so the diff picks
it up for free, but `RANK` needs an entry or the classifier will call it `changed-status` rather than
`newly-solved`.

### Phase F — `numeric_ik` and `output_gen_hybrid`

New module **`ikbtbasics/numeric_ik.py`**.

- `pvals_numeric(M)` first — `M.pvals` is **not uniformly numeric**: `forward_kinematics()` writes
  the *strings* `'np.cos(...)'`/`'np.sin(...)'` into it for robots whose α is not a multiple of π/2
  (`kin_cl.py:296-297`; `Raven-II`, `ICP5p5_A21`). A naive `subs(M.pvals)` injects strings.
- `sp.lambdify` of `M.T_06` and `M.J66` with `pvals` substituted. Sum-of-angles symbols (`th_23`, …)
  appear in both and must be resolved from `R.kequation_aux_list` first — exactly what
  `output_python.py` already emits as `th_23 = th_2 + th_3`. Cross-check the lambdified FK against
  the `Link_N` implementation from Phase D in a test; two independent paths to the same matrix is
  cheap insurance. No new symbolic computation is needed — `J66` is already pickled.
- `J_0 = blkdiag(R_06, R_06) · J_66` (`J66` is expressed in frame 6).
- **Damped least squares (Levenberg–Marquardt), not plain gradient descent:**
  `Δq = Jᵀ(JJᵀ + λ²I)⁻¹ e`, which behaves like Newton away from singularities and degrades
  gracefully *to* gradient descent as λ grows — precisely the singularity failure mode step 2.4 asks
  about. Plain gradient descent would make that failure mode permanent.
- **Residual and metric — settled, see question 3.** The LM *step* uses the 6-vector residual
  `e = [Δp; w · rotvec(R_d Rᵀ)]`. Convergence is reported and thresholded on BH's scalar,
  `‖Δp‖ + (1 m) · θ`, where `θ` is the rotation angle of `R_d Rᵀ` in radians — so `w = 1 m/rad` and
  the step and the metric use the same rotation parameterisation and the same weighting. Frobenius is
  dropped: the angle/axis scalar is metrically interpretable (it states the position/orientation trade
  explicitly and adjustably) and is linear in the error, so one threshold means the same thing near and
  far from convergence.
- **Refine every closed-form branch independently**, then deduplicate on wrapped joint values while
  keeping branch labels. Preserving the multi-branch structure is the one thing this hybrid offers
  that a generic numerical IK does not, and it is the reason to build it this way.
- **A first-order failure mode the doc files under 2.4, item 4.** Generated IK returns `False` when
  `solvable_pose` is falsified by an out-of-domain `asin`/`acos` (`output_python.py:274-275, 327-330`).
  A pose reachable on the *true* robot can be unreachable on the *simplified* one, so the hybrid can
  get **no seed at all** — distinct from Newton diverging, and it needs an explicit fallback
  (multi-start from perturbed seeds) rather than feeding `NaN` into the refinement.

`output_gen_hybrid` then emits the simplified closed form plus the LM refinement. Two requirements:

- **Artifacts must say which robot the closed-form equations actually describe**, or they are
  misleading — the doc's one caveat before publishing. Name the artifacts for the *true* robot
  (`Robot.name` drives every output path: `output_latex.py:87`, `output_python.py:185`,
  `output_cpp.py:65`) and carry a header banner naming the derived DH table and the parameter that
  was changed.
- **C++ is phase 2**, after Python is validated. The existing C++ output contains no linear algebra
  at all, so it needs either Eigen (a new dependency) or ~80 lines of self-contained damped 6×6
  solve.

**Verify:** standalone first, before any tree wiring — on `Puma`/`KawasakiRS007L` (which solve
exactly), perturb the true solution and confirm quadratic convergence; then on `KinovaLite`, seed
from the `d_5 = 0` solution and measure convergence against true FK. Then end-to-end with the
rewritten `solChecker.py` from Phase A.

---

## Explicitly deferred

- **Step 1.1 item 5** — `scripts/simplify_dh.py` CLI. Lower priority per the doc, but it has
  standalone value as a mechanical-design tool: "your 57 mm offset is what costs you closed-form IK"
  is a useful thing to tell a designer.
- **Step 2.4** — the systematic failure-mode study (basin of attraction vs. simplification size,
  singularity stratification, branch loss and numerical continuation, joint-limit poses, branch
  mislabeling). Held for later per the doc. Note the branch-loss item is a *structural* limit, not a
  numerical one: a 6R with a spherical wrist has at most 8 IK solutions and a general 6R at most 16,
  so zeroing `d_5` can produce a provably incomplete seed set and no amount of Newton polish recovers
  a branch that was never seeded.

---

## Verification summary

```bash
# from the repo root, always
python3 -m tests.leavestest              # leaf suite (016/017/018 done; add 019 for Phase C)
python3 -m tests.bt_assembly_test        # bt_problems() linter over the new tree shape
python3 -m tests.test_chair_helper       # full solve; must still write nothing to LaTex/ or CodeGen/
python3 -m ikbtbasics.dh_analysis        # new module self-test (Phases C, D)
python3 -m scripts.axis_triple_check     # the DH rule vs. an independent geometric oracle

python3 -m scripts.robot_baseline                # capture (writes tests/baselines/)
python3 -m scripts.robot_baseline --diff         # the gate at every phase boundary; exit 1 if moved
python3 -m scripts.robot_baseline --diff --no-save          # gate without overwriting the record
python3 -m scripts.robot_baseline --robots KinovaLite Puma  # one or two robots only

python3 ikSolver.py KinovaLite           # the motivating robot: unsolved -> hybrid-solved
python3 ikSolver.py Puma                 # unchanged control
python3 solChecker.py KinovaLite         # numeric round-trip, generic rewrite (deferred from Phase A)
```

The phase gate is the same each time: **`--diff` shows only the change that phase was supposed to
make.** Phases B, C and D must show an empty diff; Phase E is the first one allowed to move a robot.
Use `--no-save` while iterating, so the checked-in baseline is not overwritten by the run that is
supposed to be measured against it.

The FK pickle cache is self-healing as of Phase A: `kinematics_pickle()` recomputes any pickle that
will not load or whose DH table no longer matches, and a `pvals`-only edit updates `M.pvals` in place
without a recompute (`pvals` never enter the symbolic FK). A change to the FK or sum-of-angles *code*
is still invisible to it — delete `fk_eqns/<name>_pickle.p` by hand for those.

---

## Questions — all five answered

BH's answers are preserved verbatim as `USR:`. What each one turned into:

1. **Robot count** — I measure 33 unique names / 32 with a definition block, not 27. Which set did
   you have in mind for "reproduce our existing successes and failures"? USR: please de-dupe, eliminate comment-only robots etc. and use the remaining number. 
   → **32.** `List` became a de-duplicated module-level `ROBOT_LIST` (41 entries → 32), `Chair6DOF`
   removed, and a fall-through guard added so a listed-but-undefined name says so instead of raising
   `UnboundLocalError`. Done in Phase A.
2. **Ranking metric (Phase D)** — task-space displacement to rank all candidates, joint-space seed
   error as go/no-go on the winner? Or must ranking itself be joint-space (making it a batch tool
   rather than a BT leaf)? USR: task space.
   → **Task space for ranking**, `joint_seed_error()` as the go/no-go on the winner only.
   `simplified_arm` stays a leaf.
3. **LM residual (Phase F)** — 6-vector twist residual with Frobenius norm reported, or the exact
   12-element Frobenius least-squares form?   Should be a scalar I belive.  Alternative to Frobenius: 
   convert the rotation submatrix to angle/axis and multiply the angle (in radians) by 1meter then add that to the magnitude of the XYZ position error. 
   → **BH's angle/axis scalar.** The reported convergence metric is
   `‖Δp‖ + (1 m) · θ`, where `θ` is the angle of the angle/axis form of `R_d Rᵀ` in radians. This is
   better than the Frobenius norm for the stated purpose: it is metrically interpretable (a
   1-radian orientation error counts as 1 m of position error, an explicit and adjustable trade), and
   it is linear in the error rather than quadratic-ish, so the threshold means the same thing near and
   far from convergence. The LM *step* still uses the 6-vector residual
   `e = [Δp; rotvec(R_d Rᵀ)]` — same rotation parameterisation, so the step and the metric agree — and
   the 1 m weight becomes the scale factor on the rotation block. Frobenius is dropped.
4. **Leaf naming** — `pieper_id` / `simplified_arm` (repo convention), or `ID_Peiper` /
   `Simplified_ARM` as written in `hybrid_plan.md`? (USR: try to make naming have correct spelling of Pieper and be consistent with other "id" and "solver" leaf names.)
   → **`pieper_id`, `simplified_arm`, `solve_simplified`, `output_gen_hybrid`.** Correct Pieper
   spelling, lowercase, `_id` suffix consistent with `algebra_id` / `sum_id` / `tan_id`.
5. **`RepeatUntilSuccess(x10)`** — I kept it and added a measurement (outer `comp_det` tick counts
   per robot) to answer your "is it still necessary" question with data rather than by inspection.
   Say if you would rather just fold it into `symbolic_branch` now. (USR: you could move this inside the solver leaf by implementing a code-based loop to enforce the "repeatuntilsuccess" limit.)
   → **Done, and the measurement says keep the budget.** The loop now lives inside
   `symbolic_loop.tick()` as a Python `while`, which is what let the node choose its own exit status
   and delete two other nodes. The data says 10 is a real limit, not slack: `UR5` uses 9 passes.

---

## Open question

One decision Phase A's data made free, so it is worth asking rather than assuming:

- **Should the hybrid fire on a *partial* symbolic solve, or only when nothing at all was solved?**
  `symbolic_loop.require_complete` is the switch, currently `False` (= fire only when nothing was
  solved), which reproduces the pre-existing `solved_anything()` contract exactly. Because **no robot
  in the current set is ever partially solved**, both settings produce an identical baseline today —
  so flipping it costs nothing now and can be flipped back. The argument for `True` is that a partial
  closed form is not usable IK, so a robot that stalls halfway is exactly as stuck as one that never
  started. The argument for `False` is that IKBT has always reported partial solves, and a future
  robot that stalls at 5-of-6 would silently stop getting its report.
