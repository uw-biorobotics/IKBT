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
Priority[ symbolic_branch, hybrid_branch ]

symbolic_branch = Sequence[ symbolic_loop(x10, solveRoutine), output_gen_full ]

solveRoutine    = Sequence[ sub_transform,
                            RepeatUntilSuccess(x6, Sequence[ assigner, sum_id, worktools ]),
                            updateL,
                            comp_det ]

hybrid_branch   = hybrid_stub          # always FAILs -- placeholder, see below

worktools = Priority[ algSol, Sequence[OrNode[tanSol, scSol], rank], Simu_Eqn_Sol, sacSol, x2z2_transform ]
```

`symbolic_loop` (`ikbtleaves/symbolic_loop.py`) replaced `b3.RepeatUntilSuccess(solveRoutine, 10)` at
the root. It runs the identical passes, but as a Python loop inside `tick()`, so it can **choose** its
exit status: SUCCESS if at least one variable was solved, FAILURE if none. `RepeatUntilSuccess`
returns FAILURE when it exhausts its loops, which would abort the enclosing `Sequence` and discard a
loop-exhausted *partial* solve — and wrapping it in `Priority([..., Succeeder()])` hides the real
failure too, leaving the tree unable to tell "solved nothing" from "ran out of passes". That FAILURE
is the gate on the hybrid branch. Measured over all 32 robots the deepest solve is UR5 at 9 passes,
so the budget of 10 is real and not slack.

`hybrid_branch` is a stub for `futurework.md` item 1 (simplify the DH parameters until the robot
solves, then correct numerically). It always FAILs, so the `Priority` is currently a no-op wrapper
and the tree is observably identical to the one that had no branch at all — which is what lets
`scripts/robot_baseline.py --diff` prove the restructure moved nothing before any new solver exists.
See `ikbtleaves/hybrid_ik.py` for the leaves that will replace it.

Blackboard keys: `Robot`, `unknowns`, `curr_unk`, `counter`, `Tm`, `eqns_1u`, `eqns_2u`, `eqns_3pu`,
`no_progress` (comp_det gave up), `symbolic_passes` / `symbolic_exhausted` (set by `symbolic_loop`).

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
015 output_latex, 016 symbolic_loop, 017 output_gen, 018 hybrid_ik). A test double that lives in a
leaf file must be named `test_*`, or the `bt_assembly_test.py` leaf-inventory scan picks it up as a
real leaf.

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
Optionally, if user asks about future work, see the file @futurework.md

