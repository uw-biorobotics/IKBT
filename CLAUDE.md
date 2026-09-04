# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

IKBT symbolically solves the closed-form inverse kinematics of a serial-chain manipulator. A behavior tree (BT)
selects among rule-based "solver leaves" (the algorithms a human expert would apply) and ticks repeatedly until
every joint variable is solved. Output is a LaTeX report plus generated Python and C++ code.  Theory of 
operation is documented in the peer-reviewed report:  @IKdocs/IKBT_JAIR_2019.pdf.

Dependencies: Python 3, `sympy`, `numpy` (LaTeX distribution needed only to compile the report). No packaging
files — everything runs from the repo root as modules.

## Commands

All commands must be run **from the repo root** (module imports are package-relative):

```bash
python3 ikSolver.py Wrist          # full IK solve (Wrist is the fast one; no arg -> Wrist)
python3 fkOnly.py <RobotName>      # forward kinematics + Jacobian only
python3 solChecker.py              # numerically check a solution against FK (robot/pose hard-coded at top)

python3 -m tests.leavestest        # main unit-test suite (basic classes + all leaves)
python3 -m tests.leavestest html   # same, writes an HTML report via HTMLTestRunner
python3 -m tests.helpertest        # helperfunctions only
python3 -m ikbtleaves.sincos_solver  # run one leaf's self-test (every leaf file has a __main__)
python3 -m ikbtbasics.kin_cl         # basic kinematic classes self-test

python3 -m scripts.robot_baseline           # solve every robot, record the outcome
python3 -m scripts.robot_baseline --diff    # ... and diff it against the checked-in record
python3 -m scripts.axis_triple_check       # DH joint-axis geometry vs. numeric FK (exit 1 on mismatch)

python3 -m scripts.check_solution_sets --robots Puma          # is the SYMBOLIC solution set correct?
python3 -m scripts.check_solution_sets --gate                 # ... exit 1 if a known-complete robot drops
python3 -m scripts.numerical_closed_loop_sol_check Puma       # is the GENERATED python IK correct?
python3 -m scripts.numerical_closed_loop_sol_check KinovaLite # ... same command for a HYBRID robot
python3 -m scripts.bt_path_gate                               # fast gate: every BT path, 5 robots
python3 -m scripts.bt_path_gate --list                        # what that gate asserts, and why
```

`scripts/robot_baseline.py` is the regression gate for anything that touches the tree or a solver
leaf: it solves all 32 robots (one subprocess each, `PYTHONHASHSEED=0`) and records solved/total,
per-variable `solvemethod`, `len(solutionSet)`, `comp_det` ticks and wall time.  `--diff` classifies
each robot as unchanged / newly-solved / newly-unsolved / changed-method and exits non-zero if
anything moved.  It **asserts nothing** — "does not solve" is a legitimate entry.  The record lives
in `tests/baselines/`; per-robot child output goes to `logs/baseline/`.

**`wall_s` is recorded but deliberately NOT compared** (`COMPARED` holds `status`, `n_solved`,
`n_unknowns`, `methods`, `n_solutions`, `solution_set_error`, `hybrid`) — the slow robots vary far too
much for a timing comparison to mean anything. Measured over back-to-back full sweeps, every robot
repeated at 1.0-1.1x **except** `DZhang` (20.7 s → 78.9 s, 3.8x) and `Issue4` (101 s, 187 s, 246 s,
then >900 s on identical code at `PYTHONHASHSEED=0`). `DEFAULT_TIMEOUT` was raised 900 → **1800 s**
for exactly that reason.

Compile the report: `cd LaTex && pdflatex ik_solution_<RobotName>.tex` (the generated file is standalone —
`IK_preamble.tex` and `IK_close.tex` are already spliced in; `ik_report_template.tex` is a legacy wrapper that
`\input`s a no-longer-generated `IK_solution.tex`).

Generated artifacts, not source: `fk_eqns/` (FK pickle cache), `CodeGen/Python/`, `CodeGen/Cpp/`,
`LaTex/ik_solution_*.tex`, `logs/`.

FK pickle cache directory or individual files can be deleted at any time without penalty (except some added 
execution time). 

## Architecture

### Pipeline (`ikSolver.py` is the whole story, top to bottom)

1. `robot_params(name)` (`ikbtfunctions/ik_robots.py`) returns `[dh, vv, params, pvals, unknowns]` for a named
   robot. This is the only file you edit to add a robot.
2. `kinematics_pickle()` (`ikbtbasics/ik_classes.py`) returns `[M, R, unknowns]` — a `mechanism`, a `Robot`, and
   the (possibly extended) unknown list. **It caches to `fk_eqns/<name>_pickle.p`.** FK computation and the
   sum-of-angles scan are slow, so they are done once. The cache is self-healing: a pickle that will not
   load, or whose DH table no longer matches, is silently recomputed and overwritten (`dh_tables_match()`).
   A `pvals`-only edit does not force a recompute — `pvals` never enter the symbolic FK — but the cached
   `M.pvals` is updated in place. `check_the_pickle()` is advisory; it no longer calls
   `quit()`. Any other change to the FK or SOA *code* still requires `rm fk_eqns/<name>_pickle.p` by hand.
3. `R.scan_for_equations(unknowns)` splits all scalar equations from the 4x4 matrix equations into
   `L1`/`L2`/`L3p` (1, 2, and 3+ unknowns) — these go on the blackboard.
4. The BT is assembled by `bt_assembly.build_default_bt()` and ticked. Solving is a side effect on the
   blackboard objects.
5. `R.create_solution_set()` then `ol.output_latex_solution()`, `op.output_python_code()`,
   `oc.output_cpp_code()` — all four inside the `output_gen_full` leaf when the tree is built with
   `codegen=True` (which only `ikSolver.py` does), otherwise `run_solver()` builds the solution set
   and the caller emits. Exactly one of the two must call `create_solution_set()`: it appends to
   `unknown.LHSversionNames` and is **not idempotent**.

### The behavior tree

`b3/` is a   locally modified copy of Behavior3Py (nodes return `b3.SUCCESS`/`FAILURE`/`RUNNING`; state
is stored on a `Blackboard`). The tree, built in `ikbtfunctions/bt_assembly.py`:

```
Sequence[ analysis, report_gen ]

analysis        = Priority[ symbolic_branch, hybrid_branch ]      # Priority == Selector

symbolic_branch = Sequence[ clear_state, symbolic_loop(x10, solveRoutine) ]

solveRoutine    = Sequence[ sub_transform,
                            RepeatUntilSuccess(x6, Sequence[ assigner, sum_id, worktools ]),
                            updateL,
                            comp_det ]

hybrid_branch   = Sequence[ pieper_geom_report,   # reports, ALWAYS SUCCESS
                            simplified_arm, install_simplified,
                            symbolic_branch (2nd instance set) ]

worktools = Priority[ algSol, Sequence[OrNode[tanSol, scSol], rank], Simu_Eqn_Sol, sacSol, x2z2_transform ]
```

**Node vocabulary.** `b3.Priority` is the standard **Selector** (a.k.a. Fallback): it ticks children in
order and stops at the first non-FAILURE. `b3.OrNode` is a local addition and is *not* a Selector — it
runs **all** its children and returns SUCCESS if any succeeded, which is load-bearing for `rank` (it
needs both the tan and sin/cos candidates to choose between).

`symbolic_loop` (`ikbtleaves/symbolic_loop.py`) replaced `b3.RepeatUntilSuccess(solveRoutine, 10)` at
the root. It runs the identical passes, but as a Python loop inside `tick()`, so it can **choose** its
exit status.  When `require_complete` is **True**: SUCCESS only when *every* unknown is solved, because 
in rare cases, a
closed form for some of the joints might not be valid inverse kinematics and must not be reported as an answer.
Nothing is discarded — the solved unknowns keep their solutions and still appear in the baseline
record; only the verdict changes. Setting it False restores "SUCCESS if anything was solved" and is
the hook for the future partial-analytic + lower-dimensional-numeric work.  

`report_gen` (`ikbtleaves/output_gen.py`) is a **single** generator at the end of the tree, ticked
after whichever branch produced the solution — the report is a property of the finished solve, not of
the branch that made it. `b3.Sequence` aborts on FAILURE, so a solve that got nowhere never reaches it
and no empty report is written. It also generates the joint-axis geometry statement into
`Robot.pieper_latex`, which `output_latex_solution()` places after Kinematic Parameters.

`pieper_geom_report` (`ikbtleaves/hybrid_ik.py`, was `no_pieper_id`) analyses the joint-axis geometry
and publishes `pieper_triples` / `pieper_ok` / `pieper_latex`. **It ALWAYS returns SUCCESS**, because
**PIEPER'S CONDITION GATES NOTHING, IN EITHER DIRECTION** (2026-08-31). Neither half of the
biconditional holds — the condition is *sufficient* for a closed form and is **not** known to be
necessary — so both gatings are unsound and both are measured:

- its **absence** cannot gate the symbolic branch. 9 of the 32 robots have no triple yet solve
  completely (Axtman13, Brad, DZhang, ICP5p5_A21, Mackler13, MiniDD, Olson13, Sims11, Wachtveitl);
  `Sequence[pieper_id, symbolic_branch]` would stop the symbolic solver ticking at all for those nine.
- its **presence** cannot gate the hybrid branch. This was the live defect. `ArmRobo` (triples (2,3,4) and (4,5,6)), `Panda`
  ((1,2,3)) and `Raven-II` ((1,2,3),(2,3,4),(3,4,5)) all satisfy the condition and all solve **0** of
  their unknowns. The old `Sequence[no_pieper_id, ...]` gate therefore turned away three arms whose
  only other answer was "unsolved", on the reasoning that "has a triple and still failed" is an IKBT
  defect rather than a geometry problem and must not be simplified. True, and beside the point: the
  user gets nothing either way, and "had a triple, failed symbolically, and the simplified arm
  solved" is exactly the report line that identifies the solver gap and points at it.

**"Does not gate" is a property of the leaf, not of the wiring.** The leaf returns SUCCESS
unconditionally — for an arm with a triple, for one without, and for a DH table it could not read at
all — so it sits **bare** in the hybrid `b3.Sequence` and needs no `Priority[.., Succeeder]` wrapper to
stop that Sequence treating a FAILURE as a gate: there is no adverse verdict to swallow. A leaf that
cannot FAIL also cannot gate anything wherever it is moved.
`tests/bt_assembly_test.py::test_btaT` ticks it over all three blackboards and asserts SUCCESS for
each, so restoring the old FAILURE-on-triple silently restores the gate and is caught.

**What gates the hybrid branch is `simplified_arm`**, an outcome rather than a proxy for one:
`candidate_simplifications()` skips triples that already qualify, so an arm satisfying the condition on
*every* triple yields nothing usable and the branch closes there by itself
(`TestSolver018.test_hybJ` is that arm).

Naming history, because two things about this leaf look like accidents and are not. It began as
`pieper_id` (SUCCESS when a triple *exists*) under a `b3.Inverter` — the only `Inverter` in the tree;
the polarity was folded into the node as `no_pieper_id`, then the verdict dropped entirely and the node
renamed to what it does. `tests/bt_assembly_test.py` still asserts the tree contains **no** `Inverter`
at all. What the `Inverter` could never express survives as **`pieper_ok`**: it turned "could not read
the table" into SUCCESS, indistinguishable from "no triple". With no gate left, that flag is the
**whole** of the protection — `simplified_arm` reads it and refuses — so it must stay False for "could
not run" and never be set optimistically. The geometry itself lives in `ikbtbasics/dh_analysis.py`;
`scripts/axis_triple_check.py` validates it against numeric FK.

 The hybrid approach requires a `simplified arm` which is a set of DH parameters close to the original
 arm but containing a Pieper triple. 
`simplified_arm` ranks the DH changes that would give the arm a triple, cheapest first by task-space
displacement, and publishes `simplification_candidates` / `simplification_choice`. It **refuses to run
when `pieper_ok` is False** — and that is no longer defense in depth but the only defense: now that
`pieper_geom_report` cannot FAIL, this check is all that stands between a DH table we could not parse
and a derived robot describing nothing. Its "no usable candidate" FAILURE is the branch's other and
main gate.
`install_simplified` then builds the derived robot — fresh `unknown` objects (`set_solved()` mutates in
place), its own pickle name (`KinovaLite_d_5_0`) — and leaves `hybrid_source` on the blackboard.

**The solver appears TWICE**, over two separate leaf sets (the second renamed ` (hybrid)`). One
*instance* in two tree slots really would collide — b3 keys per-node state on the blackboard by node id
and `bt_problems()` rejects it — but two instances are legal and get that state fresh for free. What is
not free is the unscoped application state, so **each solver starts with `clear_state`**, which wipes by
default and keeps by exception: it preserves the problem and the findings about the true robot
(`pieper_*`, `simplification_*`, `hybrid_source`) and drops everything else, notably `no_progress` and
`comp_det_signature` from the failed first solve. A keep-list rather than a clear-list, so a new
blackboard key that should survive fails loudly instead of leaking stale state silently.

Because `install_simplified` swaps the `Robot`, `pieper_geom_report` **snapshots** its LaTeX-formatted output report onto the
blackboard before the swap and `report_gen` prefers that snapshot — otherwise the report would describe
the simplified arm rather than the robot that was asked for. That snapshot is the reason the leaf still
ticks first in the branch even though nothing reads its verdict.

`hybrid_branch` implements the hybrid method (simplify the DH parameters until the robot solves,
then correct numerically). **`hybrid_stub` is gone** (2026-09-02). The branch used to end in that
always-FAIL leaf, which withheld the report because the closed form on the blackboard describes the
*derived* arm and emitting it as though it were the real robot is the one thing this method must
never do. That is still true; what changed is that there is now an honest way to present the answer,
so withholding it is no longer the only safe option. `report_gen` reads `hybrid_source` and writes a
hybrid report plus a two-phase Python module — see **Hybrid code generation** below.

The branch reports **only a COMPLETE derived solve.** The second solver is a `symbolic_loop` like the
first and carries `require_complete = True`, so a derived arm solved 2-of-7 FAILs the inner branch,
the hybrid `Sequence` FAILs with it, and `report_gen` never ticks. `ArmRobo` and `KawasakiRS05L` sit
on the far side of exactly that boundary and produce nothing.

Measured over the whole sweep now that Pieper's condition gates nothing, the branch reaches **five**
robots and is 2-for-5, up from the three it used to be admitted for:

| robot | derived arm | result |
|---|---|---|
| `KinovaLite` | `d_5: 57 → 0` | **solved 7/7** |
| `Panda` | `a_3, a_4 → 0` | **solved 6/6** — the gate used to turn this one away |
| `ArmRobo` | `d_4 → 0` | partial 2/7 — likewise turned away |
| `KawasakiRS05L` | `a_3: 80 → 0` | 0/7 |
| `Raven-II` | `a_5: 13 → 0` | 0/6 |

`Issue4` (excluded from the sweep) also simplifies via `d_5 → 0` and reaches 1 of 7. `simplified_arm`
commits to its cheapest candidate with no fall-through to the next-ranked one, which is why three of
the five fall short. All three shortfalls share a signature: `eqns_1u` is empty and stays empty, so
no ID node can fire.

Blackboard keys: `Robot`, `unknowns`, `curr_unk`, `counter`, `Tm`, `eqns_1u`, `eqns_2u`, `eqns_3pu`,
`no_progress` (comp_det gave up), `symbolic_passes` / `symbolic_exhausted` (set by `symbolic_loop`),
`pieper_triples` / `pieper_ok` / `pieper_latex` (set by `pieper_geom_report`),
`simplification_candidates` / `simplification_choice` (set by `simplified_arm`), `hybrid_source` (set
by `install_simplified` — names the derived robot, so nothing downstream reports a simplified solve as
though it solved the real arm).

### Leaf conventions (`ikbtleaves/`)

Most solvers are an **ID node + solve node in a `b3.Sequence`**: the ID node scans the equation lists for a
pattern it can handle and stashes state on the blackboard, the solve node does the sympy work. Each leaf file
also contains a `test_<name>_id` action that fabricates a blackboard for unit testing, plus a
`TestSolverNNN(unittest.TestCase)` class. `TEMPLATE_solver.pyX` is the starting point for a new leaf (the `X`
keeps it out of the import system). All leaves have a `BHdebug` flag, wired to commented-out per-robot debug
blocks in `ikSolver.py`.

Non-solver leaves: `assigner_leaf` (round-robins `curr_unk` over unsolved unknowns), `rank_leaf` (when both
tan and sin/cos solved a variable, picks fewer solutions / fewer dependencies, then calls `set_solved`),
`updateL` (re-scans equations and folds `R.kequation_aux_list` SOA definitions into L1/L2/L3p),
`comp_detect` (termination), `sum_id` (identifies sum-of-angles terms; the actual solving is left to the
algebra leaf), `sub_transform` / `x2y2_transform` (equation transforms, the latter is Craig eqn 4.65).

Test-class numbers are global and referenced by `tests/leavestest.py` (001 sincos, 002 algebra, 003 sinANDcos,
004 tan, 006 sub_transform, 007 updateL, 008 kin_cl, 009 helperfunctions, 010 x2y2 — note `output_cpp.py` also
defines a `TestSolver010` — 011 rank, 012 invariant_gen, 013 bt_assembly, 014 comp_detect,
015 output_latex, 016 symbolic_loop, 017 output_gen, 018 hybrid_ik, 019 dh_analysis,
020 clear_state, 021 progress, 022 numeric_ik, 023 parallel_triple,
024 output_hybrid_python, 025 subexpressions). A test double that lives in a
leaf file must be named `test_*`, or the `bt_assembly_test.py` leaf-inventory scan picks it up as a
real leaf.

### Numerical IK (`ikbtbasics/numeric_ik.py`)

The hybrid method's correction step: damped least squares (Levenberg-Marquardt) refining a closed-form seed
against the true arm's FK. **Standalone** — it takes an FK callable, a Jacobian callable, a seed and a
target pose, and knows nothing about the tree, so it is validated on robots that already solve
exactly (perturb a known-good pose, confirm it comes back) with no dependence on the hybrid branch.

`dq = J'(JJ' + lam^2 I)^-1 e`, with the residual `[dp ; w_rot*theta*axis]` and BH's scalar metric
`||dp|| + w_rot*theta` sharing one rotation parameterisation and one weight, so the step and the
stopping test agree about "closer". The Jacobian's rotation rows are scaled by `w_rot` to match.
Measured on Puma: 10/10 convergence from perturbations of 0.05 to 2.0 rad (3 to 19 iterations),
quadratic error sequence `1.9e-01 1.6e-02 3.4e-04 1.0e-07 2.3e-15`, and 10/10 even at the wrist
singularity `th_5 = 0` where `J` is rank-deficient and an undamped Newton step does not exist.

Four traps, all measured:

- **`w_rot` has no correct default.** BH's metric is `||dp|| + (1 m)*theta`, but nothing records a
  model's units and they are not consistent — Puma is in **metres** (`a_2 = 0.432`), KinovaLite in
  **millimetres** (`l_2 = 280`). So `w_rot` is an explicit parameter (1.0 for a metre model, 1000.0
  for a millimetre one); `w_rot_for()` offers a unit-free alternative of one arm-length per radian.
- **`pvals` USED TO BE non-numeric, and is not any more** (2026-09-03). `kin_cl` invents `ca_i`/`sa_i`
  for a twist that is not a multiple of 90°, and it used to store them as the *strings*
  `'np.cos(al_1)'` / `'np.sin(pi/4)'` — which `dh_analysis.numeric_pvals()` *drops*, right for its
  zero-tests and wrong here, since a dropped `ca1` survives as a free symbol and becomes a spurious
  `lambdify` argument. `forward_kinematics()` now evaluates them (α is a constant, so its sine and
  cosine are too) and every pval in every robot is a number. `pvals_numeric()` and
  `_lambdify_checked()` stay as the guard: the string is still the fallback for an α whose symbols
  have no numeric value, and nothing should build a callable with a symbol left over.

  That fix exposed a **pre-existing** defect: `output_python`/`output_cpp` declared `Robot.params`
  (what `ik_robots.py` *declared*) while the equations use `Mech.params` (which also holds what
  `forward_kinematics()` *invented*), so Craig417's generated module referenced `sa2` and never
  defined it — `NameError` on the first call. Only Craig417 and Raven-II generate those symbols and
  neither was in a checker's known-good list, which is how it survived.
- **Sum-of-angle symbols CANNOT be in `T_06`/`J66`** — and this is structural, not just measured
  (BH): **every link can have only one joint variable**, so a DH-derived link transform has nothing
  for a `th_23` to be. SOA terms arise *within* the FK equations as the sum-of-angles scan rewrites
  them, and that lands in the matrix equations (`matrix_equation.Ts`), never in the product `T_06` or
  in `J66`. The plan says they appear there and must be resolved from `kequation_aux_list`; they do
  not, and this is a property to rely on rather than a measurement to re-check per robot.
- **The DH table is always 6 rows, which is not the DOF.** `Craig417` and `ICP5p5_A21` are 4-DOF
  tables padded with `[0,0,0,0]`, so `dof_of()` counts the leading rows with a symbolic joint cell —
  and *not* the unknown list, which is inflated by SOA variables (`Craig417`: 5 unknowns, 4 joints).
  `J66` is stored 6x6 for every robot and its surplus columns are **not** zero, so
  `jacobian_callable()` slices to `[:, :ndof]`; handing those columns to the solver would let it move
  joints the arm does not have. `output_latex.py` slices the same way (via `numeric_ik.dof_of()`) as
  of 2026-08-26 — the report used to print six Jacobian columns for the 5-DOF `Chair_Helper`.

### Progress reporting (`ikbtfunctions/progress.py`)

Solves run from 1 s (`Wrist`) to minutes, and used to be silent throughout, so "is this working or
stuck?" had no answer. `symbolic_loop` now owns a `SolveProgress` and prints a banner plus **one
compact line per pass**:

```
  pass 4/10  solved 3/7 (+1: th_23)  eqns 1u/2u/3pu 13/21/32  1s this pass  11s total
            making progress -- 4 variables left -- about 14s more, at most 16s
```

The solved count is **monotonic** (`set_solved()` never un-solves), so rising = working. Flat with
changed equation pools = "still working"; flat twice running = "stuck, not slow" — which is
`comp_det`'s own stop condition, so the wording escalates exactly when the solve is about to end.
The ETA is a **range**, deliberately: the optimistic figure extrapolates cost per solved variable, the
pessimistic one is remaining budget x cost per pass, a genuine upper bound. It is clamped so the
optimistic number can never exceed the bound, and it is never a countdown — `comp_det` routinely
stops a solve well before the budget (pass 2 of 10 on `KawasakiRS05L`).

**`count_ops` was measured and rejected as the cost predictor.** Every expression IKBT simplifies is
under 50 ops on every robot (`Puma` 33, `KR16` 46, `KinovaLite` 44) while the call count varies 34x
(22 / 69 / 746) — and the *same* 18-op expression was seen taking 3.2 s and then 7.0 s in one run. Size
discriminates nothing; cost is driven by structure `count_ops` flattens away. Long runtime is
thousands of small sympy calls, not a few huge ones, so the meter counts **calls and cumulative
seconds** (37-98 % of wall clock is inside `sp.simplify`). `enable_sympy_meter()` wraps
`Basic.simplify` and is **opt-in** — only `ikSolver.py` calls it, since it patches a third-party
class. Individual calls over `slow_call_s` (default 2 s) print as they happen, which is the only
output possible *during* a blocking simplify and so doubles as the live heartbeat.

Two non-obvious requirements, both learned the hard way:

- **Everything flushes** (`_say()`). Python buffers stdout when it is not a tty, so on `> log`,
  `| tee`, and `robot_baseline`'s captured logs every line was withheld until exit — an Issue4 solve
  printed nothing for 12 minutes and then everything at once.
- **Reporting may never break a solve.** `pass_done()` and `finished()` are wrapped and degrade to a
  warning: they run on the hot path of every pass, and a solve that took minutes must not be thrown
  away because a status line would not format.

Output is **line-oriented, never `\r`-animated** — the sum-of-angles progress bar collapses in
`logs/baseline/<robot>.log` into one unreadable multi-kilobyte line, which is what made `Issue4`'s log
useless for diagnosis. Do not add animation.

**`comp_det`'s stall detection** (`comp_detect.py`). Termination and reportability used to be one
flag; they are now two decisions:

```
signature repeated  and (ns == 0 or eqns_1u empty)  ->  stop ticking
ns == 0                                            ->  no_progress  (nothing to report)
```

`no_progress` does *not* mean "stop": `run_solver()` skips `create_solution_set()` when it is set and
`solved_anything()` gates `emit_outputs()` on it — so setting it for a partial solve would **discard
the partial closed form**. Keeping them separate is what lets a stalled *partial* solve stop without
losing its result.

The reason to stop is **not** speed — these are hard problems and a long solve is legitimate. It is
that the node was continuing past its own proof: 
**once a pass leaves the solved set and every equation
pool exactly as it found them, the identical pass cannot produce a different result**.  

Two traps, both caught by the 32-robot gate rather than by inspection:

- **A repeated signature is NOT proof of being stuck.** `assigner_leaf` round-robins `curr_unk`, so a
  pass can change nothing merely because it was offered a variable it cannot solve yet — the next
  pass, offered another, succeeds. Traced on `ICP5p5_A21`: pass 4 changed nothing with `eqns_1u = 6`,
  and the solve went on to finish. Stopping on the repeat alone took `ICP5p5_A21` and `Parkman13` from
  `solved` to `partial`. An **empty `eqns_1u`** is the sound condition — no ID node can fire for *any*
  variable, so the assigner's cursor stops mattering. The `ns == 0` path keeps its original condition,
  so every previously-stopping robot stops exactly as before.
- **The signature compares equation CONTENTS, not `len()`** (`_pool_signature()`). Counts alone treat
  "swapped one equation for another" as "nothing happened".

`comp_detect.read_pause` was **2 s per tick** — a sleep so a human could read the scrolling status
wall. It is now **0**, not to reclaim the time but because what it compensated for is gone: one
compact line per pass means nothing scrolls past unread, and a pause that no longer buys legibility is
just a pause. `scripts/robot_baseline.py` already forced it to 0, so the record is unaffected.

### Hybrid code generation (`ikbtfunctions/output_hybrid_python.py`)

What a hybrid solve delivers, for `KinovaLite` (true) simplified to `KinovaLite_d_5_0` (derived):

| file | describes | written by |
|---|---|---|
| `LaTex/ik_solution_KinovaLite.tex` | **both arms** | `output_latex_solution(..., hybrid=, R_true=)` |
| `CodeGen/Python/IK_hybrid_KinovaLite.py` | the two phases | `write_hybrid_top()` |
| `CodeGen/Python/FK_numericKinovaLite.py` | true arm, FK **and Jacobian** | `write_fk_module(jacobian=True)` |
| `CodeGen/Python/IK_equationsKinovaLite_d_5_0.py` | derived arm, closed form | `output_python.output_python_code()` |
| `CodeGen/Python/FK_numericKinovaLite_d_5_0.py` | derived arm, FK | `write_fk_module(jacobian=False)` |

`FK_numeric*`, not `FK_equations*`: `output_python.output_FK_python_code()` already owns that name —
it is what `fkOnly.py` writes, and it is a different artifact (a readable module-level dump of the
symbolic `T_06` with dummy joint values, not a numeric callable). Sharing the filename would mean
whichever ran last silently replaced the other.

**WHICH NAME EACH ARTIFACT CARRIES IS THE LOAD-BEARING PROPERTY.** The two things a user reaches for —
the report and the module they import — carry the **true** robot's name, because that is the robot
they asked about. The closed form underneath carries the **derived** arm's name, because that is the
arm it actually describes. An `IK_equations<True>.py` on this path would be a simplified arm's
equations shipped under the real robot's name; `bt_path_gate` fails on it explicitly, comparing the
artifact set **exactly** in both namespaces — an unexpected file is a failure, not just a missing one.

**No C++ on the hybrid path.** Emitting the derived arm's C++ under the true name would ship precisely
that misleading artifact, and there is no C++ numeric correction to pair it with. A hybrid C++ target
is its own piece of work.

**Two entry points, because a seed is a choice.**

```python
ikin_KinovaLite_approx(T)        # PHASE I  -> list of joint vectors (approximate arm)
refine_KinovaLite(T, index)      # PHASE II -> DLS against the TRUE arm, seeded by branch `index`
refine_all_KinovaLite(T)         #             convenience: Phase II from every branch
```

The branches are different postures — elbow up or down, wrist flipped — not different spellings of one
answer, and which is wanted depends on obstacles, joint limits and where the arm is now. None of that
is known here, and damped least squares stays in the basin of the seed it is given, so the choice of
index *is* the choice of posture. Folding the two calls into one would pick a posture on the user's
behalf from information IKBT does not have.

**Phase I filters.** IKBT enumerates combinations of each unknown's solution branches and does not
discard the spurious ones, so a returned branch is a *candidate*. Phase I evaluates the approximate
arm's own FK on each and keeps those that reproduce `T` — which is why the derived arm's FK is
imported and not merely its IK.

**Four files that import each other, not one that inlines everything** (BH). The whole hazard of this
method is confusing the two arms; separate files named for the arm they describe make that visible
instead of burying both in one namespace, and the top level stays small enough to read in one sitting.

**The DLS loop is a COPY** (`REFINE_CORE`). The generated module stands on numpy alone, so it cannot
import `ikbtbasics.numeric_ik`. A drifting copy is worse than no copy — the library stays green while
what ships to users stops converging — so `TestSolver024` runs the emitted `solve_numeric()` and the
library one on Puma from the same seeds and requires the same `converged`, the same iteration count,
and `q` within 1e-12. The same test compares the emitted `fk_*`/`jacobian_*` against
`numeric_ik.fk_callable`/`jacobian_callable`, which is the only independent construction of those
kinematics; Phase II refines against the *generated* FK, so if that FK were the wrong arm's, every
check that used it would agree with it.

`w_rot` is **baked into the generated module per robot** (`w_rot_for()`, one characteristic arm length
per radian) rather than defaulted in the numerics — see the units trap under Numerical IK. The FK
parameters are baked in too, deliberately unlike `IK_equations*.py`'s module-level globals: those two
functions *are* the definition of "the true arm" that Phase II refines against, and a caller who
edited a link length there would move the target without moving the closed form that seeds it.

`expr_py()` refuses any sympy function the generated module does not import. Emitting it would produce
a file that imports cleanly and then dies with a `NameError` inside generated code, naming neither the
expression nor the robot. Measured, `T_06` and `J66` reduce to `sin`/`cos` and arithmetic with every
parameter resolved (KinovaLite: ~3 KB each).

### The fast gate (`scripts/bt_path_gate.py`)

Five robots, one solve each, **codegen on** — which `robot_baseline.py` deliberately runs with off, so
nothing else in the suite exercises `report_gen` end to end. ~45 s unloaded against the sweep's 5568 s.

| robot | path | artifacts |
|---|---|---|
| `Puma` | symbolic, 6 DOF, SOA + ranking | tex, py, cpp |
| `Chair_Helper` | symbolic, 5 DOF, prismatic | tex, py, cpp |
| `KinovaLite` | hybrid, derived arm solves 7/7 | tex, hybrid, fk (+ py, fk as the derived arm) |
| `ArmRobo` | hybrid, derived arm partial 2/7 | **nothing** |
| `KawasakiRS05L` | hybrid, derived arm 0/7 | **nothing** |

Per robot it asserts the branch (from `hybrid_source`, **not** inferred from the status —
`KawasakiRS05L` ends `unsolved` having gone all the way through `install_simplified`), the solved
count, the exact artifact set in both namespaces, and the derived arm's name. Then it hands off to
`numerical_closed_loop_sol_check.check()` for correctness, which detects the path itself.

Artifacts are judged by **freshness, not existence** — `LaTex/` and `CodeGen/` carry output from
earlier sessions for every one of these robots — and freshness is timed against the solve's own start,
because the derived arm's name is not known until `install_simplified` has run and its files therefore
cannot be snapshotted in advance.

Solver-method coverage is deliberately **not** this gate's job; `tests/leavestest.py` exercises every
leaf directly, and re-solving whole robots to reach a leaf pays minutes for what a unit test buys in
milliseconds. What only an end-to-end run shows is the wiring.

`scripts/numerical_closed_loop_sol_check.py` covers **both paths from one command**. `detect_path()`
reads which artifacts exist — `IK_hybrid_<name>.py` only the hybrid path writes, `IK_equations<name>.py`
only the symbolic path writes *under the true robot's name* — so a caller with a robot name needs no
idea which branch of the tree answered it. (A hybrid solve also writes `IK_equations<derived>.py`, but
that carries the derived arm's name and cannot be mistaken for this robot's closed form; the naming
contract is what makes the detection unambiguous.) If both somehow exist, it checks the newer and says
the other is stale rather than picking silently. Two tolerances, because they measure different things:
`TOL = 1e-7` for an evaluated expression, `HYBRID_TOL = 1e-6` for the output of an iterative solve that
stops at `metric <= 1e-9`.

The hybrid half is:
`q → T = FK_true(q) → Phase I → Phase II → FK_true(refined) == T`. **Every check goes through the TRUE
arm's FK**; judging Phase I against the simplified arm's FK would pass no matter how bad the
approximation was, confirming only that a closed form is a closed form. It reports the **seed
improvement** — pose error before and after Phase II — because a run that converges while improving
nothing would mean the simplification was never needed. The FK and entry points are taken from the
*shipped* module rather than rebuilt, so a generator that emitted the wrong arm's kinematics cannot
pass by being compared against itself.

### Core data model (`ikbtbasics/`, see also `IKdocs/classes.md`)

- `kin_cl.py`: `kequation` (LHS/RHS sympy exprs), `matrix_equation` (`Td` known LHS, `Ts` symbolic FK RHS),
  `unknown` (a joint variable: `solutions`, `dependencies`, `solutionNames`, `versionNames`, `solvemethod`,
  and `set_solved(R, unknowns)` which every solver must call), `mechanism` (DH table, `forward_kinematics()`,
  `get_mequation_set()`).
- `ik_classes.py`: `Robot` — owns `Mech`, `mequation_list`, `kequation_aux_list` (SOA definitions like
  `th_23 = th_2 + th_3`), `solution_nodes` (in solve order), `create_solution_set()` / `make_LHS_versions()`.
- `pykinsym.py` symbolic kinematics helpers; `solutionGraphV3.py` `Node`/`Edge`; `matching.py` is V2-era.

**Solutions vs. versions.** A variable has *solutions* (e.g. the two branches of an `asin`) and *versions* —
one per combination of its own solutions with those of everything it depends on. `create_solution_set()` builds
`solListMatrix` (rows = complete valid solution vectors, cols = unknowns in solve order) directly, row-doubling
as each multi-solution unknown is added; `solutionSet` is the same thing as a set of tuples, which is what the
LaTeX/Python/C++ generators consume. This replaced an older tree/graph-matching approach in June 2024 — code
mentioning `notation_collections`, `notation_graph_edges`, or `matching_func` is that legacy path
(`VERSION02 = False` in `ikSolver.py`). See `IKdocs/solExamp.pdf`.
`solIdxMatrix` records **which solution of each unknown each row uses**; `make_LHS_versions()` needs it and
must not infer it.

The one thing to be careful of in `create_solution_set()` is the row count. `block` is the number of
rows present *before* the current unknown is added, it is read once, and the loops must not re-read
`len(solListMatrix)` — the matrix grows as the unknown's solutions are paired in, and row `i` then
sits in block `i // block`, which **is** its solution index.  

**THE TWO NAMESPACES MUST STAY SEPARATE.** A *solution* name is `th_1s2` (branch 2 of `th_1`'s own equation);
a *version* name is `th_1v5` (row 5 of the solution matrix). Three defects, all fixed 2026-08-24 and all
present in `main` until then, came from mixing them:

- `kin_cl.set_solved()` built `versionNames` as `solutionNames[i % nsolutions]` — version names *were* solution
  names repeated;
- `create_solution_set()` seeded the first unknown's column from that list, so column 0 was in a different
  namespace from every other column;
- `make_LHS_versions()` then substituted `th_1 -> th_1s1`, emitting equations referencing a symbol nothing ever
  assigns. It also read `nsols` *after* its `enumerate` loop, leaking the **last** unknown's count, so
  `solutions[row % nsols]` always chose solution 0 and every version of a variable came out identical.

Measured: Puma went from **0 of 8** versions being evaluable to **8 of 8** reproducing the target pose to
~2e-16, and its generated Python from unloadable to 8-of-8 correct. 

**Two checkers, one property, two stages** — build `T = FK(q)`, solve IK at `T`, require
`FK(each solution)` to reproduce `T`. Nothing else checks solution *correctness*: `robot_baseline.py`
records only that every unknown got an expression, which is precisely how this survived.

- `scripts/check_solution_sets.py` runs **in process** and evaluates `R.FinalEqnMatrix` — it tests the
  symbolic solution set. `--gate` fails if a robot in `KNOWN_COMPLETE` drops below 100%.
- `scripts/numerical_closed_loop_sol_check.py` shells out to `ikSolver.py`, imports
  `CodeGen/Python/IK_equations<name>.py` and calls `ikin_*()` — it tests the **code generator**, a
  strictly later failure point, so a defect that lives only in codegen is invisible to the first one.

They share about 60% of their scaffolding (pvals resolution, the FK callable, the probe pose, the
report table); folding that into one module is worth doing, merging the two scripts is not.

**THE GENERATED CODE'S RETURN CONTRACT** (fixed 2026-08-26). `ikin_<Robot>(T)` returns a list of
solution branches, each a list of joint values **in DH chain order**, or `False` for an unreachable
pose. The order is stated in the generated module itself as `JOINT_NAMES`, and
`ikin_<Robot>_labeled(T)` returns the same answer as dicts. Three things it deliberately is not:

- **not sorted by name.** The rows used to be `g.sort()`ed as strings, so Puma came back
  `['th_1','th_23','th_2',...]` — `'th_23'` sorts between `'th_1'` and `'th_2'` — silently shifting
  every joint after it.
- **not solve order.** Solve order is an artifact of how the tree happened to crack this arm. Chain
  order comes from the DH table via `numeric_ik.joint_symbols()`, which is also why it must not be
  read off the unknown list (that list is extended with sum-of-angle variables).
- **no sum-of-angle variables.** `th_23` is still computed — later solutions depend on it — but it is
  an intermediate, not a joint. Returning it made a 6-DOF arm come back 7 wide, and in C++ it wrote a
  seventh column into a row declared `[64][6]`. The C++ now emits `IK_NJOINTS` / `IK_NBRANCHES` and
  one commented assignment per joint.

Row order comes from `Robot.solListMatrix` (an ordered list), never from `solutionSet` (a set of
tuples, so its iteration order moves with string hashing and the generated source stops being
diffable).

**An unknown could be solved TWICE — fixed 2026-08-26.** `set_solved()` now returns immediately when
`self.solved` is already True (first answer wins, since every later solution already depends on it).
Without the guard a second solve appended further solution names and a **duplicate node** to
`solution_nodes`, giving the variable a duplicate column and inflating the version count:
`ICP5p5_A21` reported 9 versions and `Parkman13` 18, where 2 and 4 are right. Both now report the
right count. **The guard does not touch the ranked retry**: the tan and sin/cos solvers deliberately
do not call `set_solved()` — `rank_leaf` calls it once, after choosing — so a call arriving with
`solved` already True is a genuine second solve in a later pass, never a ranking.

Still failing for an unrelated reason, and worth its own investigation: `Parkman13` (0/4) and `UR5`
(0/8) — the values evaluate but are wrong. Those were 0 before the guard as well; what the guard
fixed is the inflation, not the arithmetic. **`ICP5p5_A21` is no longer among them** (2026-09-03):
it went 0/2 → 1/1 when `Simu_Eqn_Sol` was promoted ahead of `sc_tan`, and it is now in
`check_solution_sets.KNOWN_COMPLETE`.

**THE DEFECT THAT PROMOTION FIXED, because it will recur.** A variable solved by `arcsin` gets two
solutions, `th` and `pi - th`, from an equation containing only `sin(u)`. Both satisfy *that*
equation — so every consistency check on the solution itself passes — but the rest of the FK still
constrains `cos(u)`, and the supplement flips its sign. Measured on `Chair_Helper`'s `th_2` over 10
random reachable poses, the supplementary branch was valid **0 times out of 20**: not a branch that
is right at some poses and wrong at others, but one that is wrong everywhere, occupying half the
advertised solution set with nothing to say which half. The equation it came from did not even exist
in the FK — `sub_transform` manufactured it by substituting `r_13` into the `Px` equation, which
collapses the `cos(th_2)` terms away — while `L1` already held the canonical pair `simu_solver`
needs. So the diagnosis to reach for when a robot scores less than 100%: look for an `arcsin` or
`arccos` in its `solvemethod`s, and check whether a `sin+cos` pair for that variable was available.

## Adding a robot

Edit `ikbtfunctions/ik_robots.py`: add the name to `ROBOT_LIST` at the top of the module (one entry per
robot — a duplicate makes any sweep over "all robots" solve the same arm twice), then add an
`if(name == 'X'):` block copied from a similar robot setting `dh`, `vv` (1 = rotary, 0 = prismatic), `variables`
(a list of `unknown(...)`), `params`, and optionally `pvals` (numeric values for verification). The DH table
must have **6 rows** — pad with `[0,0,0,0]` — and **`vv` must have 6 entries** to match: a shorter `vv`
kills `forward_kinematics()` with `IndexError` at `kin_cl.py:409`, which is how `MiniDD` sat broken.
Modified/Craig-style DH. Do not put bare numeric literals in the DH
matrix (sympy bug); use a symbolic constant declared in `params` with its value in `pvals`. Declare any new
symbols with `sp.var()`. A name in `ROBOT_LIST` with no `if` block is reported by name rather than
raising `UnboundLocalError`.

**`EXCLUDED_FROM_SWEEP`** holds robots that are defined and runnable by name but deliberately out of
the all-robots sweep, so they do not gate. `robot_params()` validates against
`ROBOT_LIST + EXCLUDED_FROM_SWEEP`, which is why the two lists must stay separate: dropping a name
from `ROBOT_LIST` alone also makes it **unrunnable**, leaving no way to investigate the very robot
that was excluded for being odd.

Currently `['Issue4']`, excluded for **wall time, not solving**. Its result is stable when it finishes
(`partial (hybrid)` 1/7 via `Issue4_d_5_0`); its duration is not — 97 s, 101 s, 187 s, 246 s, and
twice over 1790 s on identical code at `PYTHONHASHSEED=0`, a factor of 18. Since a timeout *is* a
`status` and `status` is compared, `--diff` twice reported it as a regression when nothing had changed.
Suspected cause (unconfirmed): `PYTHONHASHSEED` pins `str`/`bytes` hashing only, not the identity-based
default `object.__hash__`, so a `set` of solver objects iterates differently per run and ties between
equally-ranked solutions break differently. Not yet investigated.

## git etiquette

Please keep commit messages to 5 lines or less. 

# Future Work 

~~First priority.  Determine the answer to the following question:
   We have several robots which fail to meet Pieper's criterion but CAN be successfully solved.  Yes, this is expected because Pieper's criterion is sufficient but not necessary evidence for a solution.   In that case, why do we gate the hybrid solver with Pieper's criterion?   Why not just go to hybrid solver iff the symbolic solver fails?~~
   **ANSWERED AND IMPLEMENTED, 2026-08-31.** There was no good reason. `Priority[symbolic_branch,
   hybrid_branch]` with `require_complete = True` already meant "hybrid iff symbolic fails"; the
   Pieper test was a *second, inner* gate on top of that, and it was unsound in the same way the
   question says — the criterion is sufficient, not necessary, so its **presence** proves no more
   than its absence does. `no_pieper_id` is now `pieper_geom_report` and **always returns SUCCESS**:
   it still runs, for `pieper_triples` / `pieper_ok` / the `pieper_latex` snapshot, but it cannot
   gate anything. The branch's gate is `simplified_arm` finding a usable candidate. See the
   `pieper_geom_report` section above.

   Then we can proceed to final integration and test: 

~~We need to extend the behavior tree to make the full workflow:
  for success on symbolic solutions: output Latex Report, Python code, C++ code. 
  for failure on symbolic solutions: return a bigger Latex report detailing both arms (no solution for original arm which failed), Hybrid numerical IK code, Hybrid numerical C++ code.  
  
In the "Hybrid NUmerical IK code",  desired End effector config is the input.  The computation has two phases. 
Phase I takes as input the desired EE pose, and returns all solutions of the approximate arm. 
Phase II takes as input an integer which selects from among the previously returned approximate poses, and returns the numerically corrected joint values for the selected approximate solution.   

A new test should evaluate performance of the end-to-end hybrid solution generated codes similar to `scripts/numerical_closed_loop_sol_check.py`.~~
   **DONE except the C++, 2026-09-02.** `hybrid_stub` is gone, so the hybrid branch reports.
   `report_gen` reads `hybrid_source` and writes a both-arms LaTeX report plus the two-phase Python
   (`ikin_<Robot>_approx(T)` / `refine_<Robot>(T, index)`), every artifact named for the arm it
   describes — see **Hybrid code generation** above. The end-to-end test is
   `scripts/numerical_closed_loop_sol_check.py`, which now covers both paths and detects which
   applies, wired into `scripts/bt_path_gate.py`.

## Still open

1. **Hybrid C++.** The Python path is done; C++ is not. It needs the FK, the Jacobian and the
   damped-least-squares loop emitted in C++, which is a bigger job than the Python one because there
   is no `sp.pycode()` equivalent already in use here and no numpy to lean on. Deliberately NOT done
   by emitting the derived arm's C++ under the true robot's name — that ships exactly the misleading
   artifact the naming rules exist to prevent.

2. **The hybrid branch reports only a COMPLETE derived solve.** `require_complete = True` on the
   second solver means `ArmRobo` (2/7) and `KawasakiRS05L` (0/7) still deliver nothing. Both
   shortfalls share a signature — `eqns_1u` is empty and stays empty, so no ID node can fire — and
   `simplified_arm` commits to its cheapest candidate with no fall-through to the next-ranked one.
   Trying the next candidate when the first derived arm does not solve is the obvious next move.

3. **`Parkman13` (0/4) and `UR5` (0/8)** still fail
   `scripts/check_solution_sets.py`: the values evaluate but are wrong. Unrelated to the hybrid work
   and unexplained.

