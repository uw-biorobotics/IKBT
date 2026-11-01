# Spurious solution branches: status, 2026-10-16

## The problem
Nine robots returned solution sets with extra, wrong versions. The true joint-pose is always among
the versions; the extras are **combinations of branches** that satisfy the equation each unknown was
solved from but break an equation no solver leaf used. Most are 4–5 DOF arms, where the 12 matrix
equations over-determine the unknowns. Which version is wrong changes with the end-effector-pose,
so no row can be dropped once and for all: the fix is ONE expression per unknown that picks the
right branch everywhere.

## What exists (commits `17a83f3`, `29eb699`)
`ikbtleaves/disambiguate.py`, one BT leaf after the symbolic loop. For each unknown with 2+
solutions it searches the unused equations (small subsets, Groebner elimination, at most one extra
unknown eliminated) for an expression that decides the branch. It accepts an expression only if it
reproduces the true joint value at 6 random joint-poses. Limits: 2 s per elimination (forked
child, killed), 20 s per unknown.

| robot | before | now | kind of split |
|---|---|---|---|
| Palm13, Sims11, Wachtveitl | 1/2 | **1/1** | `tan_solver` sign pair (`u.unsplit` record) |
| Craig417 | 2/4 | **2/2** | `±sqrt` |
| DZhang, Mackler13, MiniDD, Parkman13 | 1/4 | 1/4 | `asin`/`π−asin`, `±acos` |
| Panda | 8/16 | 8/16 | mixed genuine and spurious |
| Puma, Stanford, Wrist (genuine pairs) | | unchanged | correctly kept |

Full sweep: only those four robots moved; the baseline record is updated to them.

## Open problem 1: cost on robots with genuine branches
A genuine pair (Puma's elbow) has nothing that decides it, so the search runs to its 20 s budget
and finds nothing: **+20–25 s per solve** on such robots.

Why the leaf cannot tell genuine from spurious without searching:
- `tan_solver`'s `unsplit` record says *where* a split came from (an unsolved unknown's sign), not
  that it is spurious: on a 6-DOF arm the same pattern is the genuine wrist flip.
- `asin`, `±acos`, `±sqrt` splits carry no record at all, and are genuine (Puma) or spurious
  (Craig417) with nothing local to tell them apart.
- Asking symbolically "do the unused equations force this sign?" *is* the search.

**Proposed: a numeric detector as a gate.** At ~6 random joint-poses, `T = FK(q)`, evaluate every
branch combination and run it through FK. Complex = does not exist (fine); real but misses `T` =
spurious. If nothing is ever spurious, skip the search (~1 s). Per unknown: if flipping only `u`'s
branch turns a valid combination into a miss, `u` is implicated, and only those are searched
(matters for Panda). The detector decides *whether* to search; the answer stays symbolic.

## Open problem 2: the five unresolved robots
On MiniDD, deciding `th_4` needs at least `th_5` eliminated. A 300 s budget gave 211 eliminations,
138 timeouts, and 6 linear results, **all 6 failing the numerical check**. So a bigger budget does
not help. Options: allow two eliminated unknowns, or choose the deciding equations more cleverly.

## Branch census, 2026-10-17 (`scripts/branch_census.py`, analysis only)
Idea under test (`IKdocs/latest_thinking.md`): each solver records which of THREE states its
result is in, and the solution table is built and filtered only after every unknown is solved.

- **single**: one expression.
- **multiple**: 2+ branches decided by the unknown's own equation (`asin`/`π−asin`, `±acos`, `±sqrt`).
- **contingent**: 2+ branches decided by the SIGN of an expression containing a
  **forward-dependency**, i.e. an unknown solved AFTER this one.

The script solves every robot in its own subprocess. It wraps `disambiguate_robot` to snapshot
each unknown just before that leaf runs (the leaf clears `unsplit` on what it resolves), and again
after. No solver code is changed. Output: `logs/branch_census/census.txt` and `census.json`.

**Totals over 182 solved unknowns, 32 robots:** single 127, multiple 36, contingent 19.

- **Three states suffice.** No unknown fell outside them. No solution expression mentions a later
  unknown outright, and no sign split has a decider made only of earlier unknowns.
- **Every contingent case comes from `tan_solver`**: 19 cases on 17 robots. Each has exactly ONE
  forward-dependency.
- **That forward-dependency always has one solution and always LOOPS BACK** (19 of 19). It is
  solved (simultaneous eqn or algebra) from equations containing the contingent unknown, so each
  branch yields exactly one row. A forward-dependency independent of the contingent unknown never
  occurs.
- So the "stable/unstable loop" question reduces to a per-row check: does the row agree with the
  sign assumption of its own branch? It is checked once, with no iteration. Expected outcome:
  genuine (Puma's wrist flip, `th_4` by sign of `sin(th_5)`) means both rows agree; spurious
  (Palm13, Sims11, Wachtveitl) means one row does not. **Expected, not yet measured.**

| decider (sign of) | contingent unknown, robots |
|---|---|
| `sin(th_5)` | `th_4`: Puma, KR16, Khat6DOF, KawasakiRS007L, Stanford |
| `sin`/`cos` of a single joint | Palm13, Sims11, Bartell, Minder13, Chair_Helper, Frei13, Srisuan11, Wrist, Panda (`th_1` by `sin(th_2)`) |
| `sin` of a sum of angles | C-Arm, Wachtveitl, Olson13, Parkman13 |
| `−d_3` (prismatic) | Stanford `th_2` |

**Reach:** of the five unresolved robots, only Parkman13 and Panda have a contingent split. DZhang,
Mackler13 and MiniDD are pure **multiple** (arcsin/arccos), so a contingent-row filter cannot help
them. Open problem 2 stands.

**Caveat:** at a decider near zero (`sin(th_5) ≈ 0` is the wrist singularity) the sign test
needs a tolerance and must keep both rows.

## Bugs found and fixed on the way
- The leaf crashed silently on robots loaded from an FK cache that predates `unknown.unsplit`.
- The time limit covered only `sp.groebner`; a back-substitution on Panda ran 420–470 s. Found by
  pickling the slow call's arguments and replaying that one call; `test_disE` now guards it.

## Bugs found, not yet fixed
- **disambiguate errors out on JennyGuoSp24**: `TypeError: Cannot convert expression to float`
  (`logs/branch_census/JennyGuoSp24.log`, line 573). The leaf catches it and moves on, so the
  baseline sweep never showed it. `th_4` (arccos pair) is left unexamined.

## Next
1. **Measure the contingent predicate** (an extra mode of `branch_census`): at random joint-poses,
   evaluate every row of the 17 contingent robots. Check that the sign assumption holds exactly on
   the rows that reproduce `T` under FK. If so, the three-state design is validated before any
   refactor. It would also settle Open problem 1 for tan splits with no search.
2. Implement the numeric gate (robot and per-unknown); rerun leavestest, `--gate`, `--full --diff`.
3. Attack the five unresolved robots (two extra unknowns, or smarter equation choice).
4. Fix the JennyGuoSp24 `TypeError` in disambiguate.
5. Terminology: CLAUDE.md and DEV_NOTES still use bare "pose"/"posture" in places.
