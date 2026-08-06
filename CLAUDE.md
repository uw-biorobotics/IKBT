# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

IKBT symbolically solves the closed-form inverse kinematics of a serial-chain manipulator. A behavior tree (BT)
selects among rule-based "solver leaves" (the algorithms a human expert would apply) and ticks repeatedly until
every joint variable is solved. Output is a LaTeX report plus generated Python and C++ code.

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
```

Compile the report: `cd LaTex && pdflatex ik_solution_<RobotName>.tex` (the generated file is standalone —
`IK_preamble.tex` and `IK_close.tex` are already spliced in; `ik_report_template.tex` is a legacy wrapper that
`\input`s a no-longer-generated `IK_solution.tex`).

Generated artifacts, not source: `fk_eqns/` (FK pickle cache), `CodeGen/Python/`, `CodeGen/Cpp/`,
`LaTex/ik_solution_*.tex`, `logs/`.

## Architecture

### Pipeline (`ikSolver.py` is the whole story, top to bottom)

1. `robot_params(name)` (`ikbtfunctions/ik_robots.py`) returns `[dh, vv, params, pvals, unknowns]` for a named
   robot. This is the only file you edit to add a robot.
2. `kinematics_pickle()` (`ikbtbasics/ik_classes.py`) returns `[M, R, unknowns]` — a `mechanism`, a `Robot`, and
   the (possibly extended) unknown list. **It caches to `fk_eqns/<name>_pickle.p`.** FK computation and the
   sum-of-angles scan are slow, so they are done once. `check_the_pickle()` compares DH tables and tells you to
   delete the pickle; any other change to FK/SOA code also requires `rm fk_eqns/<name>_pickle.p`.
3. `R.scan_for_equations(unknowns)` splits all scalar equations from the 4x4 matrix equations into
   `L1`/`L2`/`L3p` (1, 2, and 3+ unknowns) — these go on the blackboard.
4. The BT is assembled by hand in `ikSolver.py` and ticked. Solving is a side effect on the blackboard objects.
5. `R.create_solution_set()` then `ol.output_latex_solution()`, `op.output_python_code()`, `oc.output_cpp_code()`.

### The behavior tree

`b3/` is a vendored, locally modified copy of Behavior3Py (nodes return `b3.SUCCESS`/`FAILURE`/`RUNNING`; state
travels on a `Blackboard`). The tree in `ikSolver.py`:

```
RepeatUntilSuccess(x10)
  Sequence[ sub_transform,
            RepeatUntilSuccess(x6, Sequence[ assigner, sum_id, worktools ]),
            updateL,
            comp_det ]

worktools = Priority[ algSol, Sequence[OrNode[tanSol, scSol], rank], Simu_Eqn_Sol, sacSol, x2z2_transform ]
```

Blackboard keys: `Robot`, `unknowns`, `curr_unk`, `counter`, `Tm`, `eqns_1u`, `eqns_2u`, `eqns_3pu`.

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
defines a `TestSolver010`).

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

Edit `ikbtfunctions/ik_robots.py`: add the name to `List` at the top of `robot_params()`, then add an
`if(name == 'X'):` block copied from a similar robot setting `dh`, `vv` (1 = rotary, 0 = prismatic), `variables`
(a list of `unknown(...)`), `params`, and optionally `pvals` (numeric values for verification). The DH table
must have **6 rows** — pad with `[0,0,0,0]`. Modified/Craig-style DH. Do not put bare numeric literals in the DH
matrix (sympy bug); use a symbolic constant declared in `params` with its value in `pvals`. Declare any new
symbols with `sp.var()`.

## Known rough edges

`ISSUE_NOTE.txt` tracks long-standing issues (3-parallel-axis sum-of-angles, `Arm_3`, UR5 regressions). Several
robots in `List` are listed as not yet solvable. `Puma`, `Chair_Helper`, `Wrist`, `KawasakiRS007L` are the
reliably-solving references — `ikSolver.py` ends with hard-coded assertions for `Chair_Helper`.
