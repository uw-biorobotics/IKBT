# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

IKBT symbolically computes the forward kinematic equations (FK) and Jacobian matrix, and then
solves the closed-form inverse kinematics of a serial-chain manipulator.  The current version adds
a symbolic-numerical "hybrid" method for arms which IKBT cannot solve symbolically.  A behavior tree (BT)
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

python3 -m tests.leavestest        # main unit-test suite (basic classes + all leaves)
python3 -m tests.leavestest html   # same, writes an HTML report via HTMLTestRunner
python3 -m tests.helpertest        # helperfunctions only
python3 -m ikbtleaves.sincos_solver  # run one leaf's self-test (every leaf file has a __main__)
python3 -m ikbtbasics.kin_cl         # basic kinematic classes self-test

python3 -m scripts.robot_baseline --gate    # fast gate: every BT path, 5 robots, codegen + IK check
python3 -m scripts.robot_baseline --gate --list  # what those five robots stand for
python3 -m scripts.robot_baseline --full    # all 32, codegen + IK check, record the outcome
python3 -m scripts.robot_baseline --full --diff  # ... and diff it against the checked-in record
python3 -m scripts.axis_triple_check       # DH joint-axis geometry vs. numeric FK (exit 1 on mismatch)

python3 -m scripts.check_solution_sets --robots Puma          # is the SYMBOLIC solution set correct?
python3 -m scripts.check_solution_sets --gate                 # ... exit 1 if a recorded robot drops
python3 -m scripts.numerical_closed_loop_sol_check Puma       # is the GENERATED python IK correct?
python3 -m scripts.numerical_closed_loop_sol_check KinovaLite # ... same command for a HYBRID robot
```

**`IKdocs/TESTING.md` is the one-page orientation** — what to run, when, what each command asserts,
and where every log and artifact lands. Read it before adding a test.

**`IKdocs/DEV_NOTES.md`** carries the rationale that used to be inline in the source: measured
numbers, dated decisions, and designs that were tried and abandoned. Nothing there is needed to
read or change IKBT; it is where a "why is it like this?" question gets answered.  

Compile the report: `cd LaTex && pdflatex ik_solution_<RobotName>.tex`  

Generated artifacts, not source: `fk_eqns/` (FK pickle cache), `CodeGen/Python/`, `CodeGen/Cpp/`,
`LaTex/` (everything in it), `logs/`.

`LaTex_src/` **is** source: `IK_preamble.tex` and `IK_close.tex` are read and inlined by
`output_latex.py`, so a generated report is self-contained. They live apart from `LaTex/`
precisely so that wiping the reports cannot take the templates with them
(`LaTex_src/cleanLaTexFolder` does that wipe, from the repo root).

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
3. `R.scan_for_equations(unknowns)` splits all   equations from the 4x4 matrix equations (each matrix element is an equation) into
   `L1`/`L2`/`L3p` (having 1, 2, and 3+ unknowns) — these go on the blackboard.
4. The BT is assembled by `bt_assembly.build_default_bt()` and ticked. The BT encodes the high level steps to 
produce a solution.
5. The BT implements two possible solution path. First, if it can find a symbolic solution to all unknowns, then it proceeds directly
to generate code and Latex output.  But, if the solver fails to find a symbolic solution we go to the "hybrid" method which involves 
1) finding a "low cost" modification to the DH parameters which creates a new solvable robot as close as possible to the original arm. 
2) Using the new solvable robot to get a set of approximate solutions.
3) Using a damped least squares numerical method to get the exact solution using FK and Jacobian Matrix of the original robot. 

### The behavior tree

`b3/` is a   locally modified copy of Behavior3Py (nodes return `b3.SUCCESS`/`FAILURE`/`RUNNING`; state
is stored on a `Blackboard`). 

**Node vocabulary.** `b3.Priority` is the standard **Selector** (a.k.a. Fallback) node: it ticks children in
order and stops at the first non-FAILURE. `b3.OrNode` is a local addition and is *not* a Selector — it
runs **all** its children and returns SUCCESS if any succeeded. 
 

`pieper_geom_report` (`ikbtleaves/hybrid_ik.py`, was `no_pieper_id`) analyses the joint-axis geometry
and publishes `pieper_triples` / `pieper_ok` / `pieper_latex`.   Pieper's condition states that if three 
consecutive axes are parallel or if they intersect at a point, then there is a symbolic solution.  
Pieper's condition is sufficient for a solution but not necessary.
 

 The hybrid approach requires a `simplified arm` which is a set of DH parameters close to the original
 arm but containing a Pieper triple. 
`simplified_arm` ranks the DH changes that would give the arm a triple, cheapest first by task-space
displacement, and publishes `simplification_candidates` / `simplification_choice`. 

**The solver appears TWICE**, over two separate leaf sets (the second renamed ` (hybrid)`). One
*instance* in two tree slots really would collide — b3 keys per-node state on the blackboard by node id
and `bt_problems()` rejects it — but two instances are legal and get that state fresh for free.  
 

### Leaf conventions (`ikbtleaves/`)

Most solvers are an **ID node + solve node in a `b3.Sequence`**: the ID node scans the equation lists for a
pattern it can handle and stashes state on the blackboard, the solve node does the sympy work. Each leaf file
also contains a `test_<name>_id` action that fabricates a blackboard for unit testing, plus a
`TestSolverNNN(unittest.TestCase)` class. `TEMPLATE_solver.pyX` is the starting point for a new leaf (the `X`
keeps it out of the import system). All leaves have a `BHdebug` flag, wired to commented-out per-robot debug
blocks in `ikSolver.py`.

Non-solver leaves: `assigner_leaf` (round-robins `curr_unk` over unsolved unknowns, but see **Offer a
determined variable first** below), `rank_leaf` (when both
tan and sin/cos solved a variable, picks fewer solutions / fewer dependencies, then calls `set_solved`),
`updateL` (re-scans equations and folds `R.kequation_aux_list` SOA definitions into L1/L2/L3p),
`comp_detect` (termination), `sum_id` (identifies sum-of-angles terms; the actual solving is left to the
algebra leaf), `sub_transform` / `x2y2_transform` (equation transforms, the latter is Craig eqn 4.65). 

**THE RULE, precisely.** An unsolved unknown is **determined** when `Robot.kequation_aux_list` holds a
*definition* of it — an equation whose LHS is that unknown's own symbol and whose RHS is free of
transcendental functions and mentions only joint variables that are **already solved**.  

### Numerical IK (`ikbtbasics/numeric_ik.py`)

The hybrid method's correction step: damped least squares (Levenberg-Marquardt) refining a closed-form seed
against the true arm's FK. **Standalone** — it takes an FK callable, a Jacobian callable, a seed and a
target pose, and knows nothing about the tree, so it is validated on robots that already solve
exactly (perturb a known-good pose, confirm it comes back) with no dependence on the hybrid branch.

`dq = J'(JJ' + lam^2 I)^-1 e`, with the residual `[dp ; w_rot*theta*axis]` and BH's scalar metric
`||dp|| + w_rot*theta` sharing one rotation parameterisation and one weight, so the step and the
stopping test agree about "closer". The Jacobian's rotation rows are scaled by `w_rot` to match.
It converges quadratically, and it converges at the wrist singularity `th_5 = 0` where an
undamped Newton step does not exist — which is what the damping is for. Convergence data:
@IKdocs/DEV_NOTES.md.

Two non-obvious requirements, both learned the hard way:

- **Everything flushes** (`_say()`). Python buffers stdout when it is not a tty, so under `> log`
  or `| tee` every line was withheld until exit — a solve printed nothing for 12 minutes and
  then everything at once.
- **Reporting may never break a solve.** `pass_done()` and `finished()` are wrapped and degrade to a
  warning: they run on the hot path of every pass, and a solve that took minutes must not be thrown
  away because a status line would not format.

Output is **line-oriented, never `\r`-animated** — a `\r` progress bar collapses in a captured log
into one unreadable multi-kilobyte line. Do not add animation.

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
 

**code generated by the hybrid path has two entry points, because a seed is a choice.**

```python
ikin_KinovaLite_approx(T)          # PHASE I   -> list of seed joint vectors (approximate arm)
refine_seed_KinovaLite(T, q_seed)  # PHASE IIa -> (q, error, iterations), DLS against the TRUE arm
refine_all_KinovaLite(T)           # PHASE II  -> Phase IIa over every seed, one dict each
```

**Phase IIa takes a seed, not an index** — it is the operational call, and the seed is normally
the branch nearest where the arm is now. Phase II is its wrapper over a seed list, for learning
once which postures the true arm can actually reach.

The branches are different postures — elbow up or down, wrist flipped — not different spellings of one
answer, and which is wanted depends on obstacles, joint limits and where the arm is now. None of that
is known here, and damped least squares stays in the basin of the seed it is given, so the choice of
index *is* the choice of posture. Folding the two calls into one would pick a posture on the user's
behalf from information IKBT does not have.
 

### Latex output: Equations that fit the page (`ikbtfunctions/texwidth.py`)

Long equations used to run off the right margin. They no longer do: the report is written, **measured**,
and the equations that did not fit are re-written. `ik_driver.write_latex_fitted()` drives it.

**PREDICTING WIDTH DOES NOT WORK — MEASURE IT.** Character count and `count_ops` were both tried
and both failed on data: they are properties of the expression, and overflow is a property of the
typeset line. The numbers are in @IKdocs/DEV_NOTES.md.


**Every equation is emitted on its own source line, preceded by a `%%IKBT-EQ <lhs>` comment**, which is
what makes a reported line number attributable. This is also why raw line length predicted so badly
before: a whole `align` block used to be one source line. 

**THE REMEDY IS `K_i` NAMING, applied where TeX says it is needed.** An equation measured too wide has
its pieces named whatever the dependency rule thinks, which shortens it without changing its shape. The
loop repeats while each pass turns up equations the previous one had not seen, since shortening one can
expose another.


### The solution graph as a figure (`ikbtfunctions/graph2latex.py`)

The report has always printed the dependency graph as a list of edges
(`Edge:th_4 depends on: th_23`), which is complete and nearly unreadable — the shape of the solve,
which variable unlocks which, is exactly what a list of pairs does not show. `solution_graph_tikz()`
draws it as a TikZ figure placed **before** the listing; the listing stays, because it is the exact data.

`\usepackage{tikz}` was already in `LaTex_src/IK_preamble.tex`, so no preamble change was needed.

- **Rows are dependency DEPTH, not solve order.** A variable sits one row below everything it needs, so
  every arrow runs downward. Those differ: the solver may reach `th_5` before `th_23` while `th_4` waits
  on both. Within a row the solve order is preserved, so it still reads left to right as the report
  discusses them.
- **Arrows point from a variable to what it DEPENDS ON**, matching the listing's own wording so the two
  cannot be read as contradicting each other. 
 
### Closed Loop Testing
`scripts/numerical_closed_loop_sol_check.py` thoroughly tests a robot by 1) solving by either method
2) choosing some reachable end effector configurations 3) enumerating all IK solutions for a configurations
4) re running the IK solutions through FK and comparing with the reachable EE configuration of step 2 within
a numerical tolerance.   For a perfect result, all returned IK solutions (vector of computed joint values)
must match the selected EE configuration of step 2.    
 
`scripts/numerical_closed_loop_sol_check.py` covers **both paths from one command**. 

**Solutions vs. versions.** A variable has *solutions* (e.g. the two branches of an `asin`) and *versions* —
one per combination of its own solutions with those of everything it depends on.  For example, see `IKdocs/solExamp.tex'.


`create_solution_set()` builds a rectangular array of solutions.  
`solListMatrix` (rows = complete valid solution vectors, cols = unknowns in solve order) directly, 
 

**THE TWO NAMESPACES MUST STAY SEPARATE.** A *solution* name is `th_1s2` (branch 2 of `th_1`'s own equation);
a *version* name is `th_1v5` (row 5 of the solution matrix).  

 

## git etiquette

Please keep commit messages to 5 lines or less. 

# Future Work   

## Still open

1. **Hybrid C++.** The Python path is done; C++ is not. It needs the FK, the Jacobian and the
   damped-least-squares loop emitted in C++, which is a bigger job than the Python one because there
   is no `sp.pycode()` equivalent already in use here and no numpy to lean on. Deliberately NOT done
   by emitting the derived arm's C++ under the true robot's name — that ships exactly the misleading
   artifact the naming rules exist to prevent.
 

