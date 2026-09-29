# New Branch dev_CppCodeGen

## Outcome (2026-09-29) — done

All seven steps landed. What is on the branch:

| new | what |
|---|---|
| `Cpp_src/` | source: `ikbt_types.h`, `ikbt_pose_error.h`, `ikbt_linalg.h`, `ikbt_dls.h`, `ikbt_search.h` |
| `ikbtfunctions/output_cpp.py` | rewritten — symbolic and conditional closed forms |
| `ikbtfunctions/output_cpp_common.py` | the expression layer, parameters, FK/Jacobian headers |
| `ikbtfunctions/output_cpp_hybrid.py` | the two phases |
| `ikbtfunctions/output_cpp_onevar.py` | the 1-D search's robot-specific half |
| `scripts/cpp_expr_check.py` | the two printers, on one expression |
| `scripts/cpp_closed_loop_check.py` | six modes: symbolic, `--compile-only`, `--fk`, `--dls`, `--onevar`, `--hybrid` |
| `TestSolver031` | in `tests.leavestest`, where the old generator's test never ran |

Measured, after a full 32-robot sweep (2455 s) with the gate passing:

- expression printer: 19 expressions, 222 evaluations, **0 disagreements, worst relative 0**,
  plus 12 out-of-domain contract cases, 0 disagreements
- **worst python/C++ disagreement across every robot and every path: 0.00e+00** (bit-exact)
- symbolic, 23 robots: every branch count meets its recorded expectation; Puma, Stanford,
  Khat6DOF, KawasakiRS007L all 8 of 8 at 1e-13 or better
- one-variable, 4 robots: **98 roots matched, none missing, none extra** (ArmRobo 16, C-Arm 28,
  KawasakiRS05L 14, KinovaLite 40), worst FK 1.2e-10
- hybrid, Panda: **12 refined postures matched**, none missing, none extra, worst FK 4.0e-11
- FK and Jacobian, all 32 robots: **0.00e+00** elementwise except Craig417 6.7e-15 and
  Raven-II 4.6e-13
- damped least squares vs `numeric_ik`, all 32: 1e-13, **identical iteration counts**
- C-Arm one-variable search: **9 ms/pose** against the python's 0.14 s
- every generated file compiles under `-Wall -Wextra` with no warnings
- `tests.leavestest`: 23 tests OK

Artifacts, from the sweep's own exact-set comparison:

| path | robots | emits |
|---|---|---|
| symbolic | 26 | `cpp, py, tex` |
| one variable | 4 | `cond, cpp_cond, cpp_fk, cpp_onevar, fk, onevar, tex` |
| hybrid | 1 | `cpp_fk, cpp_hybrid, fk, hybrid, tex` + derived `cpp, cpp_fk, fk, py` |

### Three python bugs found by building the twin

Fixing none of them by copying what python did:

1. `ALLOWED_FUNCS` held `'abs'` where sympy's class is `Abs`, so `expr_py()` refused every
   expression containing an absolute value.
2. The arcsine domain guard used a greedy regex over the printed RHS, tested the wrong
   quantity, and emitted a second unguarded assignment that overrode it. A generated module
   **raised** `ValueError: ... got 1.113` on an ordinary reachable Panda pose. BH's answer --
   check at the point of use -- removed the extraction entirely.
3. `sqrt` had the identical asymmetry (`math.sqrt` raises, `std::sqrt` gives NaN).

### And two robots out of UNCHECKABLE

Fixing (2) and (3) and returning the branches that EXIST rather than discarding all of them:

| robot | was | now |
|---|---|---|
| KR16 | `sqrt(): expected a nonnegative input, got -202.2` | **4 of 4**, worst 5.6e-16 |
| DZhang | `asin/acos: expected a number in range -1..1` | **1 of 2** (known-incomplete) |

The three that remain -- Arm_3, JennyGuoSp24, UR5 -- are ONE solver defect, not three: the
solution for a variable contains that variable. No domain check reaches it.

The open question below was resolved as recommended: `std::vector<JointVec>` everywhere, with
the fixed-array `ikin()` kept as a wrapper.

---

## Goal

Finish C++ code generation: every path the Python generator serves (symbolic,
one-variable, hybrid) should also emit C++, natively, with no AI translation
step and no new dependency on the user.

The Python generator is the specification.  For each Python emitter there is
exactly one C++ twin, derived from it mechanism by mechanism, and validated
against it before the next one is started.

## Why a fresh start, not a repair

`ikbtfunctions/output_cpp.py` predates every convention the Python generator
now follows.  It was written before `solListMatrix`, before `pvals` were
baked into generated code, before `sp.pycode` was adopted, and it has never
been compiled by any test.  Measured 2026-09-29 over the 26 `.cpp` files in
`CodeGen/Cpp/`:

| | count |
|---|---|
| `g++ -fsyntax-only` clean as shipped | **1 of 26** (Wrist, the only arm with no parameters) |
| clean after substituting `XXXXX -> 1.0` | 15 of 26 |
| still broken with parameters supplied | **11 of 26** |

The 11 break for four distinct reasons, and each one is a place where the
generator diverged from its Python sibling rather than a typo:

1. **Missing semicolon.**  `c.line('argument = ' + str(node.argument))`
   (`output_cpp.py:137`) emits no `;`.  7 robots.
2. **`arccos` / `arcsin` emitted as C++ function names.**  `node.solvemethod`
   is printed verbatim; `<math.h>` spells them `acos` / `asin`.
3. **The arcsine argument uses the UNVERSIONED symbol.**  C++ emits
   `argument = r_13*sin(th_1) - r_23*cos(th_1)`; Python emits
   `th_5v1 = acos(r_13*sin(th_1v1) - r_23*cos(th_1v1))`.  `th_1` does not
   exist, so it does not compile -- and *even if it did*, all four versions of
   `th_5` would share one argument and the `-acos(...)` branches would lose
   their sign.  This is a wrong answer, not just a broken build.  The Python
   generator avoids it by reading `solEqnVer.RHS`, which is versioned, instead
   of `node.argument`, which is not.
4. **`**` survives.**  The hand-rolled regex handles `x**2` and `sin(x)**2`
   but not `(Px - a_1)**2`.  Arm_3 and UR5.

Two more defects that compile silently:

5. **`double pi = 3.1415926;`** -- eight digits.  Any solution containing
   `pi/2` carries ~3.6e-8 of injected error.  Python uses `np.pi`.
6. **Parameters are never given their values.**  Every parameter is `XXXXX`,
   even for the 28 robots whose `pvals` are complete.  Python bakes them in.
   This alone is why no automated check of the C++ has ever been possible.

Repairing six independent divergences in a 415-line file that has no tests is
more work, and leaves less, than deriving the file from the Python generator
that is already correct and already validated.

## What is kept

**The b3 leaf and the driver contract, unchanged.**  Nothing in the behavior
tree changes:

```
report_gen (ikbtleaves/output_gen.py)
   -> ik_driver.emit_outputs(R, unks)          symbolic
   -> ik_driver.emit_onevar_outputs(R, unks, onevar)
   -> ik_driver.emit_hybrid_outputs(R, unks, hybrid, R_true)
        -> oc.output_cpp_code(R, R.solutionSet)      <- the call site that stays
```

`emit_outputs()` already calls `oc.output_cpp_code(R, R.solutionSet)`.  That
signature, that call site, and the artifact-naming discipline
(`IK_equations<Robot>` means an unconditional closed form; a conditional one is
never given that name) are the template.  Everything below the call is new.

Also kept: the `XXXXX`-for-a-missing-pval behaviour and the `CodeGen/HOWTO.txt`
paragraph explaining it.  A deliberate compile stop naming the line is the
right answer for a parameter nobody supplied; the bug is that it fires for
parameters that *were* supplied.

**Deleted:** the body of `output_cpp_code()`, the `cpp_output` class and its
`**` regex, `TestSolver010` (it reads a `Test_pickles/` directory that no
longer exists and calls `output_cpp_code(R)` with the wrong arity), and the
dead `import ikbtfunctions.output_cpp as oc` in `fkOnly.py`, which is never
called.

## Ground rules for the emitted C++

- **C++11, standard library only.**  No Eigen, no Boost, no build system.
  `g++ -std=c++11 -O2 IK_equationsPuma.cpp -o puma` with no `-I` and no `-l`
  beyond `-lm`.  This is the same rule the Python side lives by -- "a generated
  module stands on numpy alone" -- and it is the same reason the AI-translation
  route was rejected: a generated artifact must not make the user install
  anything to use it.
- **One self-contained translation unit per artifact**, mirroring one Python
  module per artifact.
- **`Cpp_src/` is source, on the `LaTex_src/` precedent.**  The shared numeric
  cores -- pose error, the 6x6 damped-least-squares solve, golden section --
  are written as real, compilable, testable C++ headers in `Cpp_src/`, and the
  generator *reads and inlines* them so the emitted file stays self-contained.
  `LaTex_src/IK_preamble.tex` already works exactly this way.  The alternative,
  a `POSE_ERROR_CORE`-style Python string literal, is the literal Python
  precedent but makes the numerics untestable except through a generated file.
  `Cpp_src/` sits apart from `CodeGen/Cpp/` so that wiping the artifacts
  (`CodeGen/cleanCodeGenOutput`) cannot take the sources with them.

## The derivation table

This is the whole method: each row is a Python mechanism and the C++ twin it
becomes.  Steps below apply the rows in dependency order.

| Python | C++ |
|---|---|
| `sp.pycode(e, fully_qualified_modules=False)` | `sp.cxxcode(e, standard='c++11')` |
| `ALLOWED_FUNCS` whitelist, `ValueError` if unmet | same whitelist, mapped to `<cmath>` |
| module-level `a_2 = 0.432` | `const double a_2 = 0.432;` at namespace scope |
| `a_2 = XXXXX  # USER must supply` | `const double a_2 = XXXXX;  // USER must supply` |
| `pi = np.pi` | `const double pi = M_PI;` |
| `JOINT_NAMES = ['th_1', ...]` | `const char* const JOINT_NAMES[] = {...};` + `NDOF` |
| `AUX_NAMES`, the `WARNING: not solved` comment | identical, as comments |
| `solvable_pose = False` | `bool solvable_pose = false;` |
| `solution_list.append([...])` | `out.push_back({...})` |
| `return False` on an unreachable pose | return an empty vector |
| `if __name__ == "__main__":` | `#ifdef IKBT_MAIN ... #endif` |
| `np.array([[...]])` 4x4 literal | `Mat4` = `std::array<double,16>`, row-major |
| `np.linalg.solve(A, e)`, A 6x6 | Gauss elimination, partial pivot -- `Cpp_src/ikbt_linalg.h` |
| `np.linalg.norm(v)` | `std::sqrt(dot(v,v))` |
| `solve_numeric()` returning a dict | `struct SolveResult { q, metric, iterations, converged, reason }` |
| `np.inf` as "no answer here" | `std::numeric_limits<double>::infinity()` |
| error cache `dict[float] -> list` | `std::map<double, std::vector<double>>` |
| `_labeled()` dict view | omitted; `JOINT_NAMES` is the index |

`sp.cxxcode` is the single most important row.  It emits `std::pow(x, 2)`,
`std::atan2`, `std::fabs`, `M_PI` -- correctly, for every expression -- and
retires defects 2, 4 and 5 above without anyone writing a regex.  It is in
sympy 1.14 (checked).

## Steps

Each step ends with a check that must pass before the next one starts.

### Step 0 -- branch and demolition

Branch `dev_CppCodeGen` off `RelCandidate2`.  Empty `output_cpp.py` down to the
call-site signature.  Drop the dead `fkOnly.py` import.  Everything still
imports; `python3 -m tests.leavestest` and
`python3 -m scripts.robot_baseline --gate` still pass, with no C++ written.

### Step 1 -- `ikbtfunctions/output_cpp_common.py`

The twin of `output_numeric_common.py`: `expr_cpp()`, the parameter/declaration
block, the header banner, `Cpp_src/` inlining, and a `cpp_identifier()`
alongside `py_identifier()`.

**Check:** a round-trip harness that takes a list of sympy expressions with
random numeric substitutions, evaluates them in Python and in a compiled C++
program, and requires agreement to 1e-14.  Seeded with the expressions that
broke the old generator: `(Px - a_1)**2`, `sin(th_1)**2`, `atan2(y, x) + pi/2`,
`-sqrt(...)`.

### Step 2 -- symbolic IK: `output_cpp_code(R, groups)`

The twin of `output_python.output_python_code(..., known=None)`, walking
`Robot.FinalEqnMatrix` and `solListMatrix` the same way, with the same
first-seen dedup on the LHS and the same joint/aux column split via
`nik.joint_symbols`.  Reads `solEqnVer.RHS`, never `node.argument`, which is
what fixes defect 3.

Parameters get their `pvals`, exactly as Python does, `XXXXX` only where there
is none.  `main()` moves behind `#ifdef IKBT_MAIN` so two robots can link into
one program, and it prints the solution rows rather than
`std::cout << sol_list`, which prints a pointer.

**Checks, three of them, and they are the deliverable of this step:**

- `scripts/cpp_compile_gate.py` -- every robot in `ROBOT_LIST` that emits C++
  must compile with `-Wall`.  Robots whose `pvals` are incomplete are expected
  to stop at `XXXXX` and are asserted to fail *that way and only that way*.
- `scripts/cpp_closed_loop_check.py` -- the twin of
  `numerical_closed_loop_sol_check.py`: pick q, `T = FK(q)`, run the compiled
  `ikin()`, feed every returned branch back through the Python FK, require
  `FK(branch) == T`.  Same `EXPECT` table, same `(good, total)` judgement.
- **Python agreement, branch for branch.**  Same robot, same pose: the C++
  `solution_list` must match the Python `ikin_*()` list row for row and column
  for column to 1e-12.  Stronger than the round-trip -- it catches version
  ordering and column-order mistakes that FK cannot see -- and it is the test
  that makes "derived from the Python generator" an assertion rather than a
  description.

All three skip cleanly when `g++` is absent, the way `write_latex_fitted()`
degrades without `pdflatex`.

Expect the five `UNCHECKABLE` robots (Arm_3, JennyGuoSp24, UR5, KR16, DZhang)
to fail in C++ for the same upstream reasons they fail in Python -- the
solution referencing the variable it solves for, and out-of-range asin/acos.
Those are solver defects, not codegen defects; the C++ gate should record them
against the same list rather than try to fix them.

### Step 3 -- FK and Jacobian: `write_fk_module_cpp()`

The twin of `output_numeric_common.write_fk_module()`.  `FK_numeric<name>.h/.cpp`,
`fk_<name>(q) -> Mat4` and `jacobian_<name>(q) -> 6 x ndof`, parameters baked
in, the same `ndof`-column truncation of `J66` and the same "pvals did not
resolve every parameter" guard.

**Check:** elementwise agreement with the Python `fk_*` / `jacobian_*` over
random q, 1e-12.

### Step 4 -- the conditional closed form

`output_cpp_code(R, groups, known='th_2')` -> `IK_conditional<Robot>.cpp`,
`ikin_<Robot>_given(T, th_2)`.  As in Python, this is the *same emitter with
one extra argument*, so it is nearly free once Step 2 lands -- and it is worth
doing here, before the search that uses it, for the same reason.

**Check:** agreement with `IK_conditional<Robot>.py` at the same assumed value.

### Step 5 -- hybrid: `output_cpp_hybrid.py`

`Cpp_src/ikbt_pose_error.h`, `ikbt_linalg.h` (the 6x6 solve), `ikbt_dls.h`, and
the generated `IK_hybrid_<Robot>.cpp` with the three entry points the Python
module has -- `ikin_<R>_approx`, `refine_seed_<R>`, `refine_all_<R>` -- and the
same reason for keeping the first two separate: the seed is a choice.

Hybrid before one-variable, although the BT tries them the other way round.
The DLS loop is ~100 lines against the search's ~470, it is the piece with an
existing pin (`TestSolver024` requires the generated refine and
`numeric_ik.solve_numeric()` to agree), and it is what shakes out the linear
algebra layer that has no Python counterpart to copy.

**Check:** same target, same seed, same converged q as the Python module, and
the `TestSolver024` pin extended to the C++ refine.

### Step 6 -- one-variable: `output_cpp_onevar.py`

`Cpp_src/ikbt_search.h` plus the generated `IK_onevar<Robot>.cpp`.  The
mechanisms port one for one and must be ported *all* of them, since each one
covers roots the others cannot see: the doubling ladder, the twin hunt, the
domain-edge probe, the one-curve-per-branch rule, the finite-domain extension
in `_local_minima`, and golden section stopping at float resolution.  The
`n_samples` field rides out with each solution, as in Python.

**Check:** on the same poses, the C++ search must return the same root set as
the Python search -- same count, each root within 1e-9.  Then C-Arm's own
measurement re-run natively: 160 random reachable poses against multistart
least squares, 888 of 888, nothing spurious.

### Step 7 -- wire up and document

`ik_driver.emit_hybrid_outputs()` and `emit_onevar_outputs()` gain their C++
calls.  `scripts/expected.py` gains `cpp_hybrid`, `cpp_onevar`, `cpp_cond`,
`cpp_fk` artifact kinds, and `artifacts_owed()` starts requiring C++ on all
three paths -- which is the moment "Still open #1" in CLAUDE.md is closed.
`CodeGen/HOWTO.txt`, `IKdocs/TESTING.md` and the CLAUDE.md architecture section
follow.

## Milestones

Steps 0-4 are a complete, compile-clean, twice-validated C++ symbolic path
covering most of the 32 robots, and are shippable on their own.  Steps 5 and 6
each add one fallback path and are independent of each other.

## Open question for BH

**The return type.**  The existing `ikin()` fills a fixed `double[NB][NJ]` and
returns `int`.  The numeric paths return a count that is not known at compile
time -- seeds, and roots found by the search -- so they want
`std::vector<std::array<double, NDOF>>`.

Recommendation: unify on the vector form (the file already includes
`<iostream>`, so it is not a C-only artifact), and emit a thin fixed-array
wrapper with the old signature so existing callers of `ikin()` keep working.
The alternative, `(double* out, int max_rows, int* n_out)` everywhere, keeps
one C-compatible style at the cost of a clumsier interface on all three paths.
