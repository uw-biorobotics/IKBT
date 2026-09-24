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

**`IKdocs/TESTING.md` is the one-page orientation** — what to run, when, what each command asserts,
and where every log and artifact lands. Read it (especially "Testing Overview") before adding a test.

**`IKdocs/DEV_NOTES.md`** carries the rationale that used to be inline in the source: measured
numbers, dated decisions, and designs that were tried and abandoned. Nothing there is needed to
read or change IKBT; it is where a "why is it like this?" question gets answered.  

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

python3 -m scripts.tube_map Puma           # solution graph as a subway map -> graphs/Puma_tube.svg

python3 -m scripts.check_solution_sets Puma                   # is the SYMBOLIC solution set correct? (10 random poses)
python3 -m scripts.check_solution_sets Puma --poses 20 --gate # ... exit 1 unless EVERY version reproduces EVERY pose
python3 -m scripts.numerical_closed_loop_sol_check Puma       # is the GENERATED python IK correct?
python3 -m scripts.numerical_closed_loop_sol_check KinovaLite # ... same command for a HYBRID robot
```

How to compile the report: `cd LaTex && pdflatex ik_solution_<RobotName>.tex`  

After compiling a report, please remove all LaTex junk (.log, .bbl, etc etc files).  Leave only .tex and .pdf files. 

This package produces generated code artifacts, which are not source code for the project itself:
`fk_eqns/` (FK pickle cache), `CodeGen/Python/`, `CodeGen/Cpp/`,
`LaTex/` (everything in it), `logs/`, `graphs/`.

`LaTex_src/` **is** source: `IK_preamble.tex` and `IK_close.tex` are read and inlined by
`output_latex.py`, so a generated report is self-contained. They live apart from `LaTex/`
 so that wiping the reports cannot take the templates with them
(`LaTex_src/cleanLaTexFolder` does that wipe, from the repo root).

FK pickle cache directory (fk_eqns) or individual pickle files within it can be deleted at any time without penalty (except some added 
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
5. The BT implements three possible solution paths, tried in order. First, if it can find a symbolic solution to all unknowns, then it proceeds directly
to generate code and Latex output.  If that fails, the "one-variable" method declares ONE unknown to be a known parameter and solves
the rest of the arm symbolically -- the true arm, unmodified -- leaving a 1-D numerical search over that parameter.  If that fails too,
the "hybrid" method involves
1) finding a "low cost" modification to the DH parameters which creates a new solvable robot as close as possible to the original arm. 
2) Using the new solvable robot to get a set of approximate solutions.
3) Using a damped least squares numerical method to get the exact solution using FK and Jacobian Matrix of the original robot. 

### The behavior tree

`b3/` is a   locally modified copy of Behavior3Py (nodes return `b3.SUCCESS`/`FAILURE`/`RUNNING`; state
is stored on a `Blackboard`). 

**Node vocabulary.** `b3.Priority` is the standard **Selector** (a.k.a. Fallback) node: it ticks children in
order and stops at the first non-FAILURE. `b3.OrNode` is a local addition and is *not* a Selector — it
runs **all** its children and returns SUCCESS if any succeeded. `b3.Sequence` node tics each of its
children in order but if any of the children returns FAIL, it returns FAIL. It returns SUCCESS of all 
children succeed.
 

`pieper_geom_report` (`ikbtleaves/hybrid_ik.py`, was `no_pieper_id`) analyses the joint-axis geometry
and publishes `pieper_triples` / `pieper_ok` / `pieper_latex`.   Pieper's condition states that if three 
consecutive axes are parallel or if they intersect at a point, then there is a symbolic solution.  
Pieper's condition is sufficient for a solution but not necessary.
 

 The hybrid approach requires a `simplified arm` which is a set of DH parameters close to the original
 arm but containing a Pieper triple. 
`simplified_arm` ranks the DH changes that would give the arm a triple, cheapest first by task-space
displacement, and publishes `simplification_candidates` / `simplification_choice`. 

**The solver appears THREE TIMES**, over three separate leaf sets (renamed ` (onevar)` and ` (hybrid)`). One
*instance* in three tree slots really would collide — b3 keys per-node state on the blackboard by node id
and `bt_problems()` rejects it — but separate instances are legal and get that state fresh for free.
A node repeated by a LOOP is not the same thing and does not collide, which is why the one-variable branch
retries with a single solver instance.  
 

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
(note that KinvaLite does not need the hybrid method now that we have the oneVariable Solver). But the examples above
work for some other robots that fall back to the hybrid numerical solver. 

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
(same caveat here - Kinova no longer needs the hybrid path)


**Phase IIa takes a seed, not an index** — it is the operational call, and the seed is normally
the branch nearest where the arm is now. Phase II is its wrapper over a seed list, for learning
once which postures the true arm can actually reach.

The branches are different postures — elbow up or down, wrist flipped — not different spellings of one
answer, and which is wanted depends on obstacles, joint limits and where the arm is now. None of that
is known here, and damped least squares should stay in the basin of the seed it is given, so the choice of
index *is* the choice of posture. Folding the two calls into one would pick a posture on the user's
behalf from information IKBT does not have.
 

### The one-variable branch (`ikbtleaves/onevar_ik.py`)

The newest strategy, and the one tried **before** hybrid: declare ONE unknown to be a known parameter and
the rest of the arm solution often falls out symbolically.  Method and 1-D search from Friedman et al., 2010
(`IKdocs/`), whose arm is `C-Arm` in `ik_robots.py`.

**NOTHING ABOUT THE ROBOT CHANGES.**  The DH table, the FK and the equations stay the true arm's;  the only
edit is removing one entry from the `unknowns` list.  `count_unknowns()` reads that list, so the symbol
becomes a constant everywhere at once, and `comp_det` — which iterates the same list — comes to mean "all
the others are solved".  No derived robot and no second FK, which is what separates this from
`install_simplified`.  It is also why this branch precedes hybrid: its answer is about the REAL arm.

`onevar_rank` measures, for every unknown, how many equations declaring it known would restock, and ranks by
**L1 count first** (then L2, then `count_ops`, then chain position).  L1 leads because a solver leaf can only
start from an equation in one unknown, and a robot that fails symbolically usually has none — C-Arm has 0 of
63.  A candidate that restocks nothing is dropped;  the rest are tried best-first, capped at
`max_candidates` (3).  The measurement is one equation scan per unknown — milliseconds, no sympy solving —
against the minutes a symbolic attempt costs.  Sum-of-angles variables are ordinary candidates
(`th_34` ranks third on C-Arm): assuming th_3 + th_4 known is as legitimate a one-parameter family as
assuming a joint known.

`install_known` declares the next candidate known and installs the reduced problem, one candidate per tick,
FAILing when the list is exhausted — which is what closes the retry loop.  Each attempt **reloads the robot**
(`fresh_problem()`, 20-30 ms from the FK cache), because a failed attempt leaves `solved` flags, candidate
solution lists and `R.solveN` behind.

**WHAT COMES OUT IS A CONDITIONAL CLOSED FORM**, exact only where the assumed value is right.  C-Arm solves
all remaining variables in 33 s with `th_2` assumed known, on the first candidate, with th_2 riding
through the equations as a parameter (for example: `0 = Px*sin(th_2) - Pz*cos(th_2) + d_1*cos(th_2)` is the one that
gives `d_1`).  It must
never be written out as `IK_equations<Robot>.py`, which means an unconditional IK for that robot —
`report_gen` refuses on `onevar_source`, the same naming discipline the hybrid branch follows.
 
**What a one-variable solve delivers**, for e.g. `C-Arm` with `th_2` assumed known:

| file | describes | written by |
|---|---|---|
| `LaTex/ik_solution_C-Arm.tex` | the equations **and the assumption** | `output_latex_solution(..., onevar=)` |
| `CodeGen/Python/IK_onevarC-Arm.py` | the 1-D search — **the entry point** | `output_onevar_python.write_onevar_top()` |
| `CodeGen/Python/IK_conditionalC-Arm.py` | the closed form, `th_2` an argument | `output_python.output_python_code(..., known=)` |
| `CodeGen/Python/FK_numericC-Arm.py` | this arm's FK (no Jacobian needed) | `output_numeric_common.write_fk_module()` |

`IK_conditional`, never `IK_equations`: that name means an unconditional inverse kinematics for the
robot, and these equations hold only where the assumed value is right. `ikin_C_Arm_given(T, th_2)`
takes the assumed value as an **argument**, not a module constant — it is not a property of the arm,
it is what the search sweeps — and returns a **complete** joint vector with `th_2` in its own chain
position, so it can go straight into FK. There is no C++ yet, for the same reason as the hybrid path.

`write_fk_module` and the emitted pose-error code moved to `ikbtfunctions/output_numeric_common.py`.
They were in `output_hybrid_python.py`, and this branch is not hybrid; that module re-exports them,
so every existing caller still works.

**The search** (`SEARCH_CORE` in `output_onevar_python.py`, emitted verbatim — a generated module
stands on numpy alone). Sampling is a **Van der Corput sequence**: each new point falls in the middle
of the largest untested gap, no end bias, and the sequence is a prefix of itself, so raising
`n_samples` refines everywhere and repeats nothing. **One error curve per solution branch** — the branches are
separate functions of the assumed variable with separate zeros and separate domains, and mixing them
would bracket minima no single branch has. Each bracketed minimum is refined by **golden section**,
and **a minimum is accepted only if it reaches zero**. 

Measured on C-Arm: `solve_C_Arm(T)` returns **8 of 8 solutions reproducing the pose** with errors of
1e-16 to 1e-14, and the joint vector the probe pose was built from is among them
(`python3 -m scripts.numerical_closed_loop_sol_check C-Arm`, which now detects this third path from
`IK_onevar<name>.py` and checks **completeness** as well as soundness — the question the other two
paths cannot ask, since a 1-D search can step over a basin narrower than its sample spacing).

Joint ranges and 2*pi wraps stay out of the search: they are one range test applied to the accepted
joint vectors afterwards.

#### Remaining problems
IK_onevarC-Arm.py chooses a random pose to evaluate.   After commenting out a line which fixed the random
numer seed, inconsistencies at several different random poses were exposed.  These may have been partially
addressed earlier.   

  1. Most residual magnitudes were in the range of 10^{-14} but sometimes they are/were as high as 10^{-7}
  2. Different poses had different number of found solutions (they should all have the same unless the pose 
  is EXACTLY at a singular configuration but at such a config, two identical solutions would be acceptable for 
  two branches.)
  3. Some solutions were missed, potentially caused by insufficient 1-D search resolution.
  
##### Potential remedies

  a. make sure that search resolution (by extending the Van der corput sequence) is sufficient to find all zeros

  b. consider elimination of the van der corput approach.  Does it really add value? or do we have to scan all 
  intervals of a given size and shrink that size?  
  
  c. review the logic which switches from a brute force scan to local optimization (golden ratio).   Are there
  bugs there?
  
  

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

### The solution graph as a subway map (`scripts/tube_map.py`)

The TikZ figure above draws the VARIABLES.  This draws the **versions**, London-Underground style:

| map | solve |
|---|---|
| column (a "fare zone") | one dependency LEVEL, left to right — one variable or several |
| station | one version of that variable (`th_4v5`) |
| line, one colour each | one complete solution — one row of `solListMatrix` |

**A column is a depth, not a solve position, because THE SOLUTION GRAPH IS NOT A TREE.**  Arm_3
solves `th_2` and then `th_123` and neither depends on anything;  Puma's `th_2` and `th_4` are
likewise independent.  Which went first is an artifact of which leaf the BT happened to fire, so
`dependency_levels()` puts variables of equal depth in one shaded band under a bracket reading
*independent — solved in either order*.  Variables at the same depth cannot depend on each other
(a dependency would push the dependent one strictly deeper), so the bracket's claim is sound.
Each variable still carries its own "Nth solved" label, which is the way back to the report.

A line is a walk from the first variable solved to the last, calling at the version of each variable
that solution uses.  Where solutions agree they run together and the shared version is drawn as an
**interchange** (the white capsule);  where they differ they part.  All lines start from one unnamed
station: the state before anything is solved.  Past 13 solutions the 13 real line colours repeat
dashed, then dotted, then dash-dot — no robot has ever needed it.

**The script imports nothing from IKBT** — no sympy, no pickle, no solve.  Its whole input is
`graphs/<robot>_graph.txt`, written by `output_latex.py` alongside the report (columns, dependency
edges, and the version matrix, in ~25 inline lines).  So redrawing is free, and the drawing code
cannot break a solve.  It is run **by hand**;  nothing in the pipeline calls it.

Output is a static SVG — open it in a browser or Inkscape, and export from there if a raster is wanted.
 
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
   artifact the naming rules exist to prevent.   An alternative to consider is a new script which could 
   generate C++ code by *translating* the python code to C++. 

 

