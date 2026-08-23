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
for exactly that reason: `Issue4` sat close enough to the old ceiling that its *status* flipped
between `partial (hybrid)` and `timeout` run to run, and `status` **is** compared — so `--diff`
reported a regression that did not exist. A flaky gate trains you to ignore it.

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
   `M.pvals` is updated in place. `check_the_pickle()` is retained but advisory; it no longer calls
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

hybrid_branch   = Sequence[ no_pieper_id, simplified_arm, install_simplified,
                            symbolic_branch (2nd instance set), hybrid_stub ]

worktools = Priority[ algSol, Sequence[OrNode[tanSol, scSol], rank], Simu_Eqn_Sol, sacSol, x2z2_transform ]
```

**Node vocabulary.** `b3.Priority` is the standard **Selector** (a.k.a. Fallback): it ticks children in
order and stops at the first non-FAILURE. `b3.OrNode` is a local addition and is *not* a Selector — it
runs **all** its children and returns SUCCESS if any succeeded, which is load-bearing for `rank` (it
needs both the tan and sin/cos candidates to choose between).

`symbolic_loop` (`ikbtleaves/symbolic_loop.py`) replaced `b3.RepeatUntilSuccess(solveRoutine, 10)` at
the root. It runs the identical passes, but as a Python loop inside `tick()`, so it can **choose** its
exit status: SUCCESS if at least one variable was solved, FAILURE if none. `RepeatUntilSuccess`
returns FAILURE when it exhausts its loops, which would abort the enclosing `Sequence` and discard a
loop-exhausted *partial* solve — and wrapping it in `Priority([..., Succeeder()])` hides the real
failure too, leaving the tree unable to tell "solved nothing" from "ran out of passes". That FAILURE
is the gate on the hybrid branch. Measured over all 32 robots the deepest solve is UR5 at 9 passes,
so the budget of 10 is real and not slack.

`report_gen` (`ikbtleaves/output_gen.py`) is a **single** generator at the end of the tree, ticked
after whichever branch produced the solution — the report is a property of the finished solve, not of
the branch that made it. `b3.Sequence` aborts on FAILURE, so a solve that got nowhere never reaches it
and no empty report is written. It also generates the joint-axis geometry statement into
`Robot.pieper_latex`, which `output_latex_solution()` places after Kinematic Parameters.

`no_pieper_id` (`ikbtleaves/hybrid_ik.py`) is SUCCESS iff the arm has **no** Pieper triple, which is
exactly what the hybrid branch gates on, so it sits in the branch directly. It was `pieper_id` under
an `Inverter` — the only `Inverter` in the tree; the polarity was swapped instead so the branch reads
as the condition it gates on, and `tests/bt_assembly_test.py` now asserts the tree contains no
`Inverter` at all. The swap also buys correctness: `Inverter` could not tell "no triple" from "could
not read the table" (both became SUCCESS), so the branch opened on a parse failure; `no_pieper_id`
FAILs on an unreadable table, at the gate. It gates **only the hybrid branch**. It must never gate the symbolic branch: Pieper's condition is *sufficient* for a closed
form and is **not** known to be necessary — measured, 9 of the 32 robots have no triple and solve
completely (Axtman13, Brad, DZhang, ICP5p5_A21, Mackler13, MiniDD, Olson13, Sims11, Wachtveitl), and
`Sequence[pieper_id, symbolic_branch]` stops the symbolic solver ticking at all for those nine. It
publishes `pieper_triples` and `pieper_ok`, the latter distinguishing "no triple" (a real answer) from
"could not analyse the table". The geometry itself lives in `ikbtbasics/dh_analysis.py`;
`scripts/axis_triple_check.py` validates it against numeric FK.

`simplified_arm` ranks the DH changes that would give the arm a triple, cheapest first by task-space
displacement, and publishes `simplification_candidates` / `simplification_choice`. It **refuses to run
when `pieper_ok` is False** — now defence in depth, since `no_pieper_id` already FAILs on a table it
could not read, but simplifying on a parse failure would produce a derived robot describing nothing.
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

Because `install_simplified` swaps the `Robot`, `no_pieper_id` **snapshots** its LaTeX statement onto the
blackboard before the swap and `report_gen` prefers that snapshot — otherwise the report would describe
the simplified arm rather than the robot that was asked for.

`hybrid_branch` implements `hybrid_impl_plan.md` item 1 (simplify the DH parameters until the robot
solves, then correct numerically). It is no longer inert: `simplified_arm` and `install_simplified`
build a derived robot and the second solver really solves it, so the branch **does move
`scripts/robot_baseline.py --diff`**. Only the trailing `hybrid_stub` still always FAILs, which
withholds the report — the closed form on the blackboard describes the *derived* arm, and emitting it
as though it were the real robot is the one thing this method must never do. Removing that block is
Phase F's job, together with naming the artifacts for the true robot.

Measured, the branch is gated to three robots (the ones with no Pieper triple) and is 1-for-3:
`KinovaLite` solves 7/7 via `d_5: 57 → 0`; `KawasakiRS05L` simplifies (`a_3: 80 → 0`) but the derived
arm still solves 0/7; `Issue4` simplifies (`d_5 → 0`) and reaches only 1 of 7 -- the set's ONLY
partial solve. `ArmRobo` and `Raven-II` *have* triples, so the gate correctly declines them — those
are IKBT solver gaps, not geometry gaps. `simplified_arm` commits to its cheapest candidate with no
fall-through to the next-ranked one, which is why two of the three fall short. Both shortfalls share
a signature: `eqns_1u` is empty and stays empty, so no ID node can fire.

Blackboard keys: `Robot`, `unknowns`, `curr_unk`, `counter`, `Tm`, `eqns_1u`, `eqns_2u`, `eqns_3pu`,
`no_progress` (comp_det gave up), `symbolic_passes` / `symbolic_exhausted` (set by `symbolic_loop`),
`pieper_triples` / `pieper_ok` / `pieper_latex` (set by `no_pieper_id`),
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
020 clear_state, 021 progress). A test double that lives in a
leaf file must be named `test_*`, or the `bt_assembly_test.py` leaf-inventory scan picks it up as a
real leaf.

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
the partial closed form**, which IKBT has always reported. Keeping them separate is what lets a
stalled *partial* solve stop without losing its result. `Issue4` went from 705 s to ~100-250 s, still
`partial (hybrid)` 1/7, because it used to re-derive an identical state for nine ~80 s passes and
`comp_det`'s own summary admitted it: *"the 10-pass budget ran out; comp_det did not stop it."*

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
wall, costing ~18 s of an interactive `Puma`'s ~27 s. It is now **0**: one line per pass replaced the
wall. `scripts/robot_baseline.py` already forced it to 0, so the recorded baseline is unaffected.

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

## git etiquette

Please keep commit messages to 5 lines or less. 

# Future Work
Optionally, if user asks about future work, see @hybrid_impl_plan.md (the tracked working plan;
Phases A-E are done and record what was actually built, Phase F and the deferred items are the
remaining work) and @hybrid_plan.md (the original design rationale).

