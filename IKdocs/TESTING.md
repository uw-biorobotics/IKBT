# Testing IKBT

Everything runs from the **repo root** — module imports are package-relative.

There are four questions worth asking of a change, and one command for each.

| # | question | command | takes |
|---|---|---|---|
| 1 | Do the classes and solver leaves still work? | `python3 -m tests.leavestest` | ~45 s |
| 2 | Is the tree still well-formed? | `python3 -m tests.bt_assembly_test` | ~5 s |
| 3 | Did any path through the tree stop delivering? | `python3 -m scripts.robot_baseline --gate` | ~5 min |
| 4 | Did anything move, anywhere, on any robot? | `python3 -m scripts.robot_baseline --full --diff` | hours |

Run 1–3 after any edit. Run 4 before a commit that touches the tree, a solver
leaf, or the code generators.

Why a particular threshold, timeout or expected count is what it is:
[DEV_NOTES.md](DEV_NOTES.md).

---

## 1. Unit tests — `tests/leavestest.py`

Exercises the basic classes and **every solver leaf** directly, with fabricated
blackboards. Each leaf file `ikbtleaves/*.py` carries its own
`TestSolverNNN(unittest.TestCase)` and a `test_<name>_id` action that builds the
fixture; `leavestest.py` collects them.

```bash
python3 -m tests.leavestest           # text output
python3 -m tests.leavestest html      # writes an HTML report
python3 -m tests.helpertest           # helper functions only
python3 -m ikbtleaves.sincos_solver   # one leaf's self-test
python3 -m ikbtbasics.kin_cl          # basic kinematic classes
```

Test-class numbers are **global** and referenced from `leavestest.py`. When you
add a leaf, take the next free number. A test double living inside a leaf file
must be named `test_*`, or the leaf-inventory scan in `bt_assembly_test.py`
mistakes it for a real leaf.

**This is where solver-method coverage belongs.** Re-solving a whole robot to
reach one leaf pays minutes for what a unit test buys in milliseconds.

## 2. Tree structure — `tests/bt_assembly_test.py`

Checks the tree is well-formed: no missing root, no empty composite, no class
used where an instance was meant, leaves paired with their ID nodes and in
order, no shared node or cycle, names unique, `codegen` off unless asked. It
also asserts the properties that are easy to undo by accident — that
`pieper_geom_report` **gates nothing**, that `report_gen` is last and shared,
that `simplified_arm` is the hybrid branch's gate, and that the tree contains
no `Inverter`.

The tree is the same for every robot, so this needs no robot and takes seconds.

## 3–4. The sweep — `scripts/robot_baseline.py`

**One program answers all three end-to-end questions.** It forks one child per
robot (`PYTHONHASHSEED=0`), solves, and hands the result back as fenced JSON.

1. **What solves** — status, solved count, per-variable method, version count,
   whether the hybrid branch took over.
2. **What crashed** — a child that dies, calls `quit()`, or hangs comes back as
   a `crash` or `timeout` row with its traceback, and the sweep carries on.
3. **Is the generated code correct** — with `--closed-loop`, every robot that
   produced artifacts is round-tripped through the code IKBT just wrote.

```bash
python3 -m scripts.robot_baseline --gate            # 5 robots, one per tree path
python3 -m scripts.robot_baseline --full            # all 32, codegen + IK check
python3 -m scripts.robot_baseline --full --diff     # ... and compare to the record
python3 -m scripts.robot_baseline --robots Puma Wrist --codegen --closed-loop
python3 -m scripts.robot_baseline --gate --list     # what the five robots stand for
```

`--gate` is the fast one: the five robots that cover the distinct **paths**
through the tree (symbolic 6-DOF, symbolic 5-DOF/prismatic, hybrid-complete,
hybrid-partial, hybrid-failed). It saves nothing and asserts everything.

**Most of the sweep asserts nothing.** "Does not solve" is a legitimate entry —
most of `ROBOT_LIST` does not solve completely. Only two things are asserted:

- **the artifacts a finished solve owes**, a *rule* applied to every robot
  (`scripts/expected.artifacts_owed`), not a per-robot list;
- **the closed-loop counts**, for the ten robots that have a recorded
  expectation in `scripts/expected.EXPECT`.

### Exit status

`0` = nothing to report. `1` = an expectation was missed, or (with `--diff`)
something moved. `2` = the run was refused (e.g. diffing a codegen record
against a non-codegen one).

### The record and the diff

`--diff` classifies every robot as unchanged / newly-solved / newly-unsolved /
changed-status / changed-count / changed-artifacts / changed-simplification /
changed-solutions, most alarming first.

**`methods` is recorded but deliberately NOT compared.** Which of two
equally-good solvers wins moves with almost any change to the tree and has
repeatedly moved without breaking anything, so `changed-method` was the diff's
most frequent verdict and its least informative. A gate whose usual answer is
known-benign is a gate you learn to ignore. What a leaf change must not do is
change the *answer* — that is what `n_solutions`, the artifact lists and the
closed-loop counts are for. Wall time is not compared either.

`--timeout` (default 1800 s) is a **hang backstop, not a performance target**.
Slow robots vary by 4x run to run; at 900 s a robot's recorded *status* flipped
between `partial (hybrid)` and `timeout`, and a flaky gate is worse than a slow
one.

## The correctness checkers

Both build a target the arm can certainly reach — `q → T = FK(q)` — solve IK at
`T`, and require `FK(each answer)` to reproduce `T`. Nothing has to be
hand-checked, and a failure is the solver's, never the target's.

| script | tests | stage |
|---|---|---|
| `scripts/check_solution_sets.py` | the **symbolic** solution set, in process (`R.FinalEqnMatrix`) | earlier |
| `scripts/numerical_closed_loop_sol_check.py` | the **generated Python**, imported and called | later |

```bash
python3 -m scripts.check_solution_sets Puma              # one robot, 10 random poses
python3 -m scripts.check_solution_sets Puma Stanford --poses 20 --gate
python3 -m scripts.numerical_closed_loop_sol_check Puma
python3 -m scripts.numerical_closed_loop_sol_check --keep     # don't re-solve
```

The second one is what the sweep calls (with `resolve=False`, so nothing is
solved twice). It covers **all three paths from one command**: `detect_path()`
reads which artifacts exist, so a caller with a robot name needs no idea which
branch answered it. The hybrid check is
`q → T = FK_true(q) → Phase I → Phase II → FK_true(refined) == T` — **every
check goes through the TRUE arm's FK**, because judging Phase I against the
simplified arm would pass no matter how bad the approximation was.

The one-variable check is `q → T = FK(q) → solve_*(T) → FK(each found) == T`,
**plus one question the other two cannot ask: was `q` itself among them.** A
closed form enumerates its branches and either contains the answer or does not;
a 1-D search can step over a basin narrower than its sample spacing, and the
only way to notice is to look for a solution you already know is there. The
note column says `probe recovered` or `PROBE NOT FOUND among N`.

Keep the first one for **diagnosis**: when the generated code is wrong, it tells
you whether the fault is in the closed form or in the code generator.

**It takes a robot name and holds no expectation table.** The closed loop is its
own oracle — `T` came from `FK`, so a version that does not reproduce it is
wrong and no stored count is needed to say so — and the sweep role belongs to
`robot_baseline`, which calls the *second* script. It solves the robot **once**
and then evaluates the version matrix at several random poses, because the
solve is pose-independent and is the only expensive step.

It fails on two defects:

- **a version that does not reproduce the pose.** Every version, not most of
  them — a partial score is a failure here, so Craig417 (2-of-4), MiniDD
  (1-of-4) and the rest of `expected.py`'s KNOWN INCOMPLETE list do not pass.
  They never had high-confidence solutions; why they fail is open, that they
  fail is not (BH, 2026-09-17). A version that evaluates *complex* is excused
  — that posture does not exist at that pose.
- **two versions that are identical** — the same arm configuration — whether or
  not that configuration is correct (compared mod 2π on revolute joints, since
  `th` and `th + 2π` place the arm the same way). A repeated *right* one
  inflates the solution count; a repeated *wrong* one is two spurious branches
  that are not even distinct from each other.
  There is a known producer: at the tangency case `A²+B² == C²` the two
  `sinANDcos` branches coincide while `nsolutions` stays 2 — see
  `test_scB_tangency_duplicate_solutions`.

It also reports when **which versions fail depends on the pose** — a stable
2-of-4 can mean two permanently broken branches or two-spurious-at-a-time with
the guilty pair rotating, and the counts cannot tell those apart. Craig417 is
the second kind.

Note this is **stricter than `EXPECT`**, which records those partial counts as
floors to be met. That table still governs `numerical_closed_loop_sol_check`
and the sweep, so the two checkers deliberately disagree about Craig417: the
sweep asks "did it get worse?", this asks "is it right?".

What it **cannot** see is a solution that is *missing*: every count it has
descends from the same leaves that built the solution set, so there is nothing
to disagree with.

## The one expectation table — `scripts/expected.py`

`EXPECT` maps robot → `(good, total)`: how many returned branches must reproduce
the pose, and how many branches there should be. Both numbers, because `good`
alone catches a solution going wrong while `total` catches a **spurious branch
coming back** — a real defect that leaves `good` untouched.

This was three tables (`KNOWN_COMPLETE`, `KNOWN_GOOD`, and a copy inside the old
`bt_path_gate.py`) with no mechanism to agree. If you improve a robot, update it
**here**, once.

The same module holds `artifacts_owed()` — which files a finished solve owes,
by path:

| path | under the true name | under the derived name |
|---|---|---|
| symbolic, complete | `tex`, `py`, `cpp` | — |
| one variable, complete | `tex`, `onevar`, `cond`, `fk` | — |
| hybrid, derived arm complete | `tex`, `hybrid`, `fk` | `py`, `fk` |
| anything incomplete | nothing | nothing |

Sets are compared **exactly**: an unexpected artifact is a failure, not just a
missing one. The characteristic hybrid defect is an *extra* file — a simplified
arm's equations shipped under the real robot's name. The one-variable path has
its own version of that defect: `py` (`IK_equations<robot>.py`) would claim an
unconditional closed form for equations that hold only at the right value, so
it ships `cond` (`IK_conditional<robot>.py`) and never `py`.

## Other checks

```bash
python3 -m scripts.axis_triple_check      # DH joint-axis geometry vs numeric FK
python3 -m tests.test_chair_helper        # full-solve regression, one robot
```

## Where the output goes

| path | what | keep? |
|---|---|---|
| `tests/baselines/robot_baseline.json` | the record `--diff` compares against | **checked in** |
| `tests/baselines/robot_baseline.txt` | the same, laid out for a person | **checked in** |
| `logs/baseline/<robot>.log` | one child's full stdout | generated |
| `LaTex/ik_solution_<robot>.tex` | the report | generated |
| `LaTex_src/IK_preamble.tex` | inlined into every report | **checked in** |
| `CodeGen/Python/IK_equations<robot>.py` | closed-form IK | generated |
| `CodeGen/Python/IK_hybrid_<robot>.py` | hybrid two-phase top level | generated |
| `CodeGen/Python/IK_onevar<robot>.py` | one-variable 1-D search (the entry point) | generated |
| `CodeGen/Python/IK_conditional<robot>.py` | closed form given one assumed value | generated |
| `CodeGen/Python/FK_numeric<robot>.py` | FK (+ Jacobian) callables | generated |
| `CodeGen/Cpp/IK_equations<robot>.cpp` | closed-form IK, C++ | generated |
| `fk_eqns/<robot>_pickle.p` | FK cache | generated, safe to delete |

Artifacts are judged by **freshness, not existence** — `LaTex/` and `CodeGen/`
carry output from earlier sessions for most robots, so "the file is there"
proves nothing. Freshness is timed against the solve's own start.

Deleting `fk_eqns/` costs only time. The cache is self-healing: a pickle that
will not load, or whose DH table no longer matches, is silently recomputed.

## Things that will bite you

- **Run from the repo root.** Nothing else works.
- **`create_solution_set()` is not idempotent.** Exactly one of the tree
  (`codegen=True`) and the caller may call it.
- **A record captured with codegen on cannot be diffed against one captured
  with it off** — `n_solutions` and the artifact lists do not mean the same
  thing. The sweep refuses rather than producing a nonsense diff.
- **`RECORD_VERSION` mismatches are rejected loudly.** Recapture the baseline
  rather than diffing across versions.
- **`Issue4` is in `EXCLUDED_FROM_SWEEP`** — for wall time, not for solving. It
  is still runnable by name.

## Do not pipe the sweep

```bash
python3 -m scripts.robot_baseline --full            # good
python3 -m scripts.robot_baseline --full | tee x.log # BAD: exit status is tee's
```

The sweep signals failure through its **exit status**, and a pipeline reports the status of its last
command. Piping to `tee` therefore turns a failing gate into a silent success — measured on
2026-09-05, when a sweep that correctly exited 1 on `Parkman13` was reported as exit 0. Redirect
instead (`> sweep.log 2>&1`), or `set -o pipefail` first.
