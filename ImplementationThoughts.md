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

---

## BH notes, August 2026

Direction agreed: **fix up the existing BT method first**, per the recommended plan above.

### The baseline must record expected failures, not just successes

Some robots do not currently succeed — Kinova among them. The baseline is therefore not a
pass/fail list but a *record of current status per robot*, in which "does not solve" is a
legitimate and expected entry.

This matters for how the baseline gets used:

- A robot moving from solved to unsolved is a regression.
- A robot moving from unsolved to solved is the *goal* of the new candidates, and needs to be
  distinguishable from an accidental change.
- A robot that stays unsolved is not a test failure and must not be reported as one, or the
  suite will be permanently red and get ignored.

So the baseline file should be checked in and diffed against, rather than encoded as assertions
that every robot must solve.

### A new higher-level test of the full solution process

Ideally this becomes a proper test at a higher level than the current leaf unit tests: one that
exercises the **full solution process** end to end, and that runs **after the leaf unit tests
pass**.

Rationale for that ordering: if a leaf is broken, the full-solution test will fail in a way that
is hard to attribute. Leaf tests localize the fault; the integration test then answers the
different question of whether the assembled tree still reaches the same solutions. Running the
expensive whole-robot test only once the cheap unit tests are green also keeps the normal
development cycle fast.

Open questions to settle when building it:

- Where it lives and how it is invoked, relative to `python3 -m tests.leavestest`.
- Which robots form the set — all of them, or a representative subset, given runtime.
- What is asserted per robot: solved/unsolved status at minimum; possibly also solution counts,
  the `solvemethod` chosen per variable, and solve time.
- How the FK pickle cache interacts with it, since a cold `fk_eqns/` makes the run far slower and
  a stale one can mask changes.

---

## Solver math audit, August 2026

Findings from numeric round-trip testing of `sinANDcos_solver` and `tan_solver`. The method:
substitute numbers into whatever expression the solver returns and check it actually satisfies the
original equation. This is form-independent, so unlike the existing assertions — which compare
against exact sympy expressions evidently captured from a run — it can tell "correct" from merely
"unchanged".

Both solvers' core formulas were **confirmed correct** this way. Everything below is around the
edges.

### Open question: complex solutions for unreachable poses

`sinANDcos_solve` computes `t = sqrt(A**2 + B**2 - C**2)`. When `C**2 > A**2 + B**2` the pose is
unreachable, `t` is imaginary, and the solver returns complex solutions rather than reporting
failure.

BH: this **may be a feature** — the real part of the complex solution is the closest reachable
pose, which is genuinely useful behavior. The open question is whether that was ever implemented
*consistently*, i.e. whether every leaf that can go complex does so with the same meaning, and
whether anything downstream (LaTeX, Python, C++ codegen, `solChecker`) knows to interpret it that
way or documents it for the user.

To look at in future:

- Which leaves can return complex solutions, and under what condition in each.
- Whether the real part is the closest reachable pose in every such case, or only in this one.
- Whether the generated code signals reachability to its caller at all, or silently returns `nan`.
- If the behavior is intentional, it should be named and tested rather than incidental — right now
  nothing records that it is deliberate.

Pending that decision, the new round-trip tests **guard** rather than fix: reachable cases assert
the solution is real, and `test_scB_unreachable_gives_complex` pins the current complex behavior as
a characterization test so any future change is a deliberate one.

### Bug (FIXED): negative cosine coefficient defeats `tan_solver`

`tan_id` and `tan_solve` decompose equations with an unconstrained sympy `Wild`:

```python
d2 = ectst.match(Cw*sp.cos(u.symbol) + Dw)
```

That pattern is ambiguous — `Cw*cos(x) + Dw` can match anything by setting
`Cw = (expr - Dw)/cos(x)` — and for a **negative** cos coefficient sympy returns exactly that
degenerate reading:

```
-3*d_3*cos(th_1) + 4.95   ->   {Cw: 4.95/cos(th_1),  Dw: -3*d_3*cos(th_1)}
```

The `count_unknowns(d2[Dw]) > 0` screen then rejects the equation, so the leaf **silently declines
a pair it can solve**. Flip the sign of the coefficient and the identical problem solves. This is a
lost-capability bug, invisible in the current tests because they only exercise positive
coefficients.

Fixed by excluding `sin(u)`/`cos(u)` from the `Wild`s, in both `tan_id` and `tan_solve`. The
`Wild`s must be constructed per-unknown, since the exclusion depends on `u.symbol`, so they are
built inside `tick()` rather than reusing the module-level ones. `two_eqn_m7` and
`sinANDcos_solver` already sidestep this by using `.coeff()`, and `two_eqn_m7` even carries a
comment recording the same lesson: *"previously used sp.match, which fails when expr too
complicated."* `tan_solver` was the one that did not get the memo.

BH decided to apply this ahead of the robot-level baseline rather than behind it: robot solutions
changing is acceptable so long as they are still (or now) correct. Covered by
`test_tanB_negative_cos_coefficient_solves`, which checks both signs solve and recover the same
angle.

**This is a capability increase, not just a repair** — `tan_solver` will now claim equation pairs
it previously declined, so it may win variables from other leaves and some robots may solve
differently, or newly solve. That makes the baseline diff more interesting, not less.

### Was masked by the above (also FIXED): assumption labels tracked the wrong coefficient

`tan_solve`'s two-branch path scales both `atan2` arguments to cancel a shared unsolved factor.
That scaling is angle-preserving only when the scale factor — which is `A2` (`d2[Cw]`) — is
positive, and the second branch covers the negative case. But the recorded assumptions are on `A1`:

```python
u.assumption.append(sp.Q.positive(d[Aw]))   # d[Aw] is A1, not A2
u.assumption.append(sp.Q.negative(d[Aw]))
```

while `tan_id` separately records `Q.nonzero(d2[Cw])` — on `A2`. When `A1` and `A2` share a sign
these agree by accident; when they carry opposite signs the branch labelled "A1 positive" is
actually the one valid for `A1` negative.

Harmless in itself: `u.assumption` and `global_assumptions` are only appended and printed, never
consumed by codegen or branch selection, and both branches are emitted so the correct one is always
present. It would become a real defect the moment anything selects a branch from these labels.

This was undemonstrable until the `Wild` bug was fixed, since opposite-sign coefficients were
exactly the case being rejected. With that fixed it was confirmed directly, with `d_3 = +2`:

| coefficients | valid branch | `assumption[0]` = `Q.positive(A1)` | |
|---|---|---|---|
| `A2 = +3*d_3` | `solutions[0]` | holds | agree |
| `A2 = -3*d_3` | `solutions[1]` | holds | **inverted** |

Fixed by recording `d2[Aw]` (A2) instead of `d[Aw]` (A1). Covered by
`test_tanB_assumption_labels_match_valid_branch`, which checks both sign combinations.

One trap worth recording for whoever writes the next such test: `bool()` on a sympy
`AppliedPredicate` is **always `True`** — it does not evaluate the predicate. The first version of
this test used `bool(a.subs(...))` and consequently passed vacuously. Evaluate
`a.arguments[0]` and compare against `a.function == sp.Q.positive` instead.

### Minor, `sinANDcos_solve`

- **Dead code containing a wrong formula.** `r = sp.sqrt(A*A+B*B)` is never used — its only
  consumer, `targument = C/r`, is commented out. Inside it, `if (A==B): r = sp.sqrt(2)*A` is wrong
  for `A < 0` (should be `|A|*sqrt(2)`). Harmless until someone uncomments the line below it.
- **Duplicate solutions at tangency.** When `A**2 + B**2 == C**2`, `t == 0` and both branches
  coincide, but `nsolutions` is still 2 — two identical rows into the version machinery.
- **A guard that cannot fire.** `assert(A*A+B*B != 0)` is a structural sympy comparison; for
  symbolic `A`, `B` it is always `True`, so it never catches the case it names.

### Method note

Reading code found the dead `r` and the assumption mismatch. It also produced one confident
hypothesis that was simply wrong (a suspected dropped `LHS` term in `sinANDcos` — `lhs = l_1 - l_1`
is an obscure way of writing zero, so nothing is dropped). The `Wild` bug, which is the most
consequential finding here, was found by *running* a case the existing tests never tried. Weight
future audits accordingly: the round-trip harness earns its keep, code reading is a way to decide
which cases to try.

Two bugs also stayed hidden because of a **dependency between them** — the assumption-labelling
error was unreachable while the `Wild` bug rejected the only inputs that expose it. Expect more of
this: fixing one defect makes the next one testable, so an audit pass should be re-run after each
fix rather than treated as one-shot.

Every fix in this section was mutation-tested: the fix was reverted and the suite re-run to confirm
the new test actually fails. Both did. That step is not optional here — the `simu_id` work began
precisely because a test had been passing against a solver that was returning nothing at all, and
the first draft of the assumption test passed vacuously for the `bool()` reason noted above.
