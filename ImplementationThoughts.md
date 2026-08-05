# Implementation Tradeoffs: Integrating New Solution Strategies

Companion to [NewStrategies.md](NewStrategies.md). Considers how the three candidate methods
should be integrated: as additional leaves in the existing behavior tree in `ikSolver.py`, or as
separate solver programs with their own behavior trees.

Written August 2026.

## Framing

The decision is not a single global choice. It splits three ways, because the candidates differ
in whether they speak the language of the existing blackboard. Separating that question from the
question of file layout makes the tradeoffs much clearer.

## What actually determines the cost

There is one seam that every solution path must reach, regardless of file or tree:

```python
unknown.set_solved(R, unknowns)      # ikbtbasics/kin_cl.py:153
```

This computes dependencies, builds `solutionNames` / `versionNames`, appends the `Node` to
`R.solution_nodes`, and adds the graph edges. Downstream, `create_solution_set()` and the three
codegen modules consume only those structures.

So "separate solver with its own BT" does not mean a separate *system* — it means a separate
front end and tree sharing a common back end. Any design that forks the back end also forks the
LaTeX, Python, and C++ output. Once that boundary is fixed, the question becomes narrow:

> Does this candidate's leaf produce `kequation`s on the standard blackboard, or does it not?

## Candidates 1 and 3: separation buys nothing

**Candidate 3** is a pure equation *generator*. It emits scalar `kequation`s into `L1/L2/L3p` and
produces no solutions of its own. A separate BT for it would have to contain the entire existing
toolbox in order to do anything at all — duplicating the tree to host one transform leaf that
belongs naturally beside `sub_transform` and `sum_id`.

**Candidate 1** emits the same `kequation` type and commits through the same `set_solved`.
Mechanically it is one more `Priority` child. Its real cost — that `n_s = 4` breaks the
row-flipping logic in `create_solution_set()` — is paid identically in either layout, because the
solution-set builder is shared. A separate BT dodges none of it.

### The controllable risk

Candidate 1 is general enough to fire where a pattern matcher would have produced a cleaner
result. Ordering handles this: placed **last** in the `Priority` list, it can only fire where
everything else has already failed, which makes it behavior-preserving for currently-solving
robots by construction. The `rank` node is the precedent for a later upgrade, where it competes
on solution quality rather than acting purely as a fallback.

### The risk ordering does not fix

Runtime. The `Priority` sits inside `RepeatUntilSuccess(..., 6)` nested within
`RepeatUntilSuccess(..., 10)`, so a leaf can be ticked on the order of 60 times per solve.
Resultants are expensive. Candidate 1 needs a cheap degree/size pre-check before the real
computation, and probably memoization keyed on equation pairs.

## Candidate 2: separation is clearly right

This one genuinely does not fit the existing structure. It needs:

- twists `ξ = (v, ω)` rather than DH transforms,
- "equations" that are geometric predicates (point, point, axis, distance) rather than scalar
  LHS/RHS pairs,
- equation generation by applying both sides to strategically chosen points, rather than by
  scanning the twelve elements of a matrix equation.

Forcing that into the current tree leaves two bad options: pollute the blackboard with a parallel
key set that every existing leaf ignores, or have the PoE leaves marshal to and from
`kequation`s — which discards the compactness that motivated Paden–Kahan in the first place.

There is also research value in separating this one specifically. The paper's discussion (p. 484)
speculates that *"a custom Behavior Tree could solve additional robots or solve robots more
efficiently."* Two trees sharing a back end is the experiment that tests that claim directly:
same robot, same output pipeline, two solving strategies, comparable results.

### Practical trap

`kinematics_pickle()` stores `[m, R, unknowns]` under `fk_eqns/NAME_pickle.p`. A PoE front end
wants a different payload. Reusing that filename with a different shape will produce baffling
failures, especially given how fragile pickle compatibility already is (the README's advice is
essentially `rm -rf fk_eqns/`). Give it a distinct name from the start.

## The cost of separate top-level scripts

`ikSolver.py` is a 466-line script with everything at module level — no `main()`, no import
guard. Copying it twice yields three divergent copies of robot loading, pickle handling, the DH
check, blackboard initialization, solution-set generation, three codegen calls, and the
`Chair_Helper` assertions.

This repository has already been bitten by exactly this failure mode:

- `ISSUE_NOTE.txt` documents the duplicated `find_xy` in `ik_classes.py` and `sum_transform.py`,
  and the work required to unify them.
- `HTMLTestRunner.py` currently exists twice, at top level and in `tests/`.

### The refactor that makes both options cheaper

Extract the script into functions:

```python
def load_robot(name)          -> (M, R, unknowns)
def build_default_bt()        -> BehaviorTree
def run_solver(R, unknowns, bt) -> R
def emit_outputs(R, unknowns)
```

Each top-level solver then becomes roughly twenty lines: load, build *its* tree, run, emit.

This is also why leaf testing is currently awkward. `tests/HOWTO.txt` describes hand-assembling
2-leaf trees with a bespoke setup node for each test, which is largely a consequence of not being
able to import any of the real assembly without triggering a full solve.

## On fixing simu_id first

Do it before anything else, for measurement reasons more than functional ones. A new candidate
cannot be evaluated against a baseline that has a known-good solver switched off — any
improvement measured would partly be the re-enabling of paper-solver #5.

It is not risk-free despite being a "cleanup." If `simu_id` starts claiming variables again, the
*chosen* solver for some variables changes, which changes the solution expressions, which can
move the `Chair_Helper` assertions at the end of `ikSolver.py` and some leaf-test expectations.
That is information rather than breakage, but it should be budgeted for.

### Related gap: no robot-level regression suite

At present there are only leaf unit tests plus the inline `Chair_Helper` assertions. Before
touching the tree, a script that runs all ~19 robots and records solved/unsolved, solve time,
solution counts, and `solvemethod` per variable would pay for itself immediately.
`unknown.solvemethod` already accumulates that string (e.g. `"best ranked, atan2(y,x)"`), so most
of the necessary instrumentation already exists.

## Recommended plan

1. **Fix `simu_id`**, then capture a baseline across all robots.
2. **Refactor `ikSolver.py` into importable functions.** Cheap, and it is what makes a second BT
   affordable rather than a duplication liability.
3. **Candidate 3, then Candidate 1, into the main tree.** They speak the existing language, and
   Candidate 1 goes last in the `Priority` list so it acts as a pure fallback. Its real
   prerequisite is generalizing `create_solution_set()` past `n_s ∈ {1,2}`.
4. **Candidate 2 in its own file** with its own front end and BT, joining at `set_solved` so it
   inherits the entire output pipeline unchanged.

## Caution: scope creep in the Priority list

The `worktools` `Priority` already has five children inside two nested repeat loops, and the
~200 lines of commented-out per-robot debug flags in `ikSolver.py` suggest it is near the limit
of what can be debugged by inspection. If Candidates 1 and 3 both land, grouping the algebraic
toolbox into a named subtree would keep it legible.
