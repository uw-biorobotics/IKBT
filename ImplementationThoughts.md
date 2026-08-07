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

~~**This is a capability increase, not just a repair** — `tan_solver` will now claim equation pairs
it previously declined, so it may win variables from other leaves and some robots may solve
differently, or newly solve.~~

**Struck: this was overstated.** The degenerate match needs a loose numeric term, which the DH
convention should prevent, so real robots almost certainly never hit it. See *Correction: how
reachable the `Wild` bug actually is* below.

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

### `sincos_solver`: the same `Wild` bug, worse consequence (FIXED)

Asked whether auditing two leaves said anything about the rest, the answer turned out to be no —
the bug just fixed in `tan_solver` was **already present** in `sincos_solver`, one of the leaves not
yet examined. Same unconstrained `Wild`, but a worse outcome:

```
-3*sin(th_1) + 1/2 = 0      i.e. sin(th_1) = 1/6, identical to the positive case

positive coefficient:   asin(1/6)                  correct
negative coefficient:   asin(6*sin(th_1)**2)       a "solution" for th_1 containing th_1
```

`tan_solver` silently *declined*. `sincos_solver` emitted a wrong answer — and a plausible-looking
one that would flow straight into the LaTeX and codegen output.

Fixed the same way (`exclude=terms` on both Wilds), and additionally **reinstated the
self-containment guard**: `sinANDcos_solver` line 188 already had exactly the check that would have
caught this, commented out —

```python
#assert(not lhs.has(u.symbol)), 'Somethings wrong: solution contains itself! '
```

— so an `assert(not targument.has(u.symbol))` now runs in both the arcsin and arccos branches.
Defence in depth: mutation testing confirms that without `exclude=` the guard fires with a clear
message, and without either the tests catch the bad solution.

**Why this hid for so long is the important part.** `TestSolver001` is one of the better tests in
the repo — 12 assertions with a correct `ntests` count guard — and it missed this completely,
because every case it exercises has a positive coefficient. Comprehensive-looking assertions over a
narrow input set. Test *coverage* of the code was fine; coverage of the input space was not.

### Correction: how reachable the `Wild` bug actually is

BH's reaction to the above was that he did not recall this issue arising in any robot solution.
That is correct, and it downgrades the severity of both `Wild` fixes considerably.

The trigger is not a negative coefficient — it is a **numeric additive term**:

| expression | match |
|---|---|
| `-3*d_3*cos(th) + 4.95` | degenerate |
| `-3*d_3*cos(th) + 5` | degenerate |
| `-3*d_3*cos(th) + Px` | ok |
| `-l_1*cos(th) + Pz` | ok |
| `-l_1*cos(th) + Pz + 3` | degenerate |

Purely symbolic expressions match correctly regardless of sign. Since `CLAUDE.md` requires DH
entries to be symbolic constants declared in `params` rather than bare numeric literals, real robot
equations should not contain loose numerics, and the degenerate branch is not reached. This is
consistent with all four reference robots having solved correctly the whole time.

So the honest characterisation:

- These are **defensive hardening, not repairs of an active defect.** `exclude=` is strictly more
  precise than an unconstrained `Wild` and costs nothing, so the fixes are worth keeping — but they
  are unlikely to change any robot result.
- The claim in commit `864eb55` that the `tan_solver` fix is "a capability increase: some robots may
  solve differently or newly solve" is **overstated**. That commit is already on `main` and cannot be
  amended without a force-push, so the correction is recorded here instead. Expect the baseline diff
  to show nothing from these two changes.
- The genuinely dangerous case remains worth guarding: the failure mode in `sincos_solver` was a
  *wrong answer* rather than a decline, and the new `assert(not targument.has(u.symbol))` catches
  that class of fault whatever its cause. That guard has value independent of this particular
  trigger.

Residual risk is narrow but not zero: a robot whose `params` values get substituted early, or a
transform that introduces an integer term, could produce a loose numeric addend. The
`l_2 + 5` / `17` entries in the existing `sincos_solver` test fixture suggest the original authors
did expect such terms to appear.

### Remaining unaudited leaves

Running tally: three leaves examined closely, three with real defects (`two_eqn_m7` 3 bugs,
`tan_solver` 2, `sincos_solver` 1) — plus `sinANDcos` with latent issues only. Treat the untouched
leaves as unaudited, not as working.

Known leads, in priority order:

1. `algebra_solver.py:170` — same unconstrained `Wild` (`Aw*u.symbol+Bw`), and `A = d[Aw]` then
   `(LHS-B)/A` with no `d is None` check and no `A != 0` check.
2. `sum_id.py:81,88,216` — three more unconstrained matches, on a different pattern shape
   (`sin(thx + sgn*thy)`), so each needs its own analysis.
3. `x2y2_transform` — `TestSolver010` sets `ntests = 0` and then never increments or asserts on it,
   so its two assertions sit inside an `if` that may never fire; also squaring can introduce
   spurious roots.
4. `assigner_leaf`, `comp_detect`, `sum_id` — no test class at all, and all three are in the tree.
5. Re-run over `tan_solver` and `sincos_solver`: fixing one bug has twice now made the next one
   reachable.

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

## Solver-leaf audit round 2, August 2026

Second pass, this time over *all* the solution-finding leaves rather than two of them. Kept at
unit-test level by BH's instruction — leaf-level fixtures and probes, no full robot solves. Baseline
before starting: `python3 -m tests.leavestest` green, 21 tests.

Six findings, all reproduced by ticking the real leaves with a hand-built blackboard. Two were
fixed this round; four are recorded and left alone.

### FIXED: `algebra_solver` accepted anything that merely *mentioned* the unknown

`algebra_id`'s only screen was "does the equation contain `sin(u)`/`cos(u)`". Anything else
mentioning `u` was claimed, whatever its shape, and `algebra_solve` then divided by whatever an
unconstrained `Wild` returned. Three shapes verified to produce a "solution" that is a function of
the variable being solved — and `set_solved()` was called on all of them:

| input | emitted "solution" | marked solved |
|---|---|---|
| `0 = d_1**2 - l_1` | `d_1 = l_1/d_1` | yes |
| `r_11 = sin(th_1 + th_2)` (un-substituted SOA) | `th_1 = r_11*th_1/sin(th_1 + th_2)` | yes |
| `0 = -d_1*l_3 + 5` | `d_1 = d_1**2*l_3/5` | yes |
| `A` cancels during `expand()` | `zoo*(-l_1 + r_11)` | yes |

The third is the same degenerate-`Wild` trigger already fixed in `tan_solver` and `sincos_solver`
last round — `algebra_solver` was lead #1 on that round's list and this confirms it. The first two
are new, and are *not* variants of the `Wild` problem: they are the ID node's screen being far too
loose. `sin(th_1+th_2)` is the interesting one, because `has(sin(th_1))` is **False** for it, so the
existing sin/cos screen lets it straight through.

Fixed with a `linear_match(expr, sym)` helper shared by both nodes: `Aw*sym + Bw` with `sym`
excluded from both `Wild`s, plus an explicit `A == 0` rejection. Excluding `sym` is what gives the
match teeth — it is precisely what makes `sym**2` and `sin(th_1+th_2)` fail to match, since both
would need `Aw` to contain `sym`. One helper covers all four shapes above.

Two behaviour changes worth noting:

- `algebra_id` now `continue`s past an unusable equation instead of `break`ing, so an unusable
  equation early in `eqns_1u` no longer hides a usable one behind it. Small capability increase.
- `algebra_solve` now returns `b3.FAILURE` when it cannot decompose, instead of returning
  `b3.SUCCESS` having silently done nothing. That lets the `Priority` offer the variable to another
  leaf.

Per the previous round's correction, this is **defensive hardening, not a repair of an active
defect** — no evidence any reference robot reaches these shapes, and BH's position is that
self-referential solutions "have never been a big issue". Reachability against real robots is
**not** established and remains the open question if anyone wants to escalate it.

### FIXED: `sincos_solve`'s two branches disagreed about how to fail

```python
arcsin:  assert(d is not None), "..."       # AssertionError -> aborts the whole solve
arccos:  print(...); return b3.FAILURE      # BT continues
```

Verified: `0 = sin(th)**2 - l_1` crashed, `0 = cos(th)**2 - l_1` declined cleanly. The arccos
behaviour is the right one; arcsin now matches it.

Note the direction of travel: **the `exclude=terms` hardening from last round made this assert more
reachable, not less.** The `Wild`s now correctly refuse shapes they previously matched
degenerately, so `d is None` arises where it once did not. Tightening a matcher converts silent
wrong answers into hard failures, and every such site needs its failure path checked afterwards.
Worth applying that lesson to the other leaves touched last round.

### Recorded, not fixed

1. **`rank_leaf` — hard `IndexError`, crashes the whole solve.** `sincos_solve` appends to
   `u.sincos_eqnlist` in the arcsin branch but **not** the arccos branch; `rank_leaf.py:69` then does
   `u.eqntosolve = u.sincos_eqnlist[0]`. Reproduced end-to-end through the production composite
   `Sequence([OrNode([tanSol, scSol]), rank])`: a 1-unknown cos equation (arccos, 2 solutions) plus a
   sin/cos pair sharing an unsolved factor (tan, 2 solutions) makes `rank` tie-break to `"sincos"` and
   crash. **One missing line in `sincos_solve` is the fix.** Same function: the `else` tie-break
   branch reads `sincos_solutions[0]` / `tan_solutions[0]` unguarded — the `len(...) > 0` checks
   above it do not protect it.
2. **`x2y2_transform` guard precedence.** `if not u.symbol == th_3 or u.symbol == th_2:` parses as
   `(not (u.symbol == th_3)) or (u.symbol == th_2)`, so `th_2` is **rejected** and the `or` clause is
   dead — despite the comment above it saying the leaf is needed for "Th 2 or Th_3". Verified by
   truth table. Also in that leaf: `for u in unknowns: if temp_r.has(u.symbol): unknown = u` takes the
   **last** match including already-solved unknowns (and shadows the `unknown` class); if the last
   match is solved the useful equation is silently not appended and the leaf still returns `SUCCESS`.
3. **`tan_id` debug block clobbers the current unknown.** `for u in get_unknowns(unknowns, e1tmp):`
   rebinds `u`; the following `blackboard.set('curr_unk', u)` and `if u.solvable_tan` then act on the
   wrong unknown. Fires only under `BHdebug` — i.e. it corrupts exactly the runs you would be doing
   to diagnose something else.
4. **`assigner` hands back a solved variable** (`unknowns[0]`) once everything is solved. Combined
   with `tan_id` returning SUCCESS based on `u.solvable_tan` rather than on `found`, and neither
   `tan_solve` nor `sincos_solve` having a `not u.solved` guard, that appends duplicate solutions.
   Hard to reach in `ikSolver.py` (`RepeatUntilSuccess` breaks on the first SUCCESS) but reachable in
   the unit-test harnesses, which use fixed-count `b3.Repeater`.

### Test coverage, all solving leaves

| leaf | test | state |
|---|---|---|
| `two_eqn_m7` | `TestSolver005` | best in the repo — positive + negative cases, numeric round-trip |
| `sinANDcos_solver` | `TestSolver003` | good since round 1 |
| `sincos_solver` | `TestSolver001` | good; +1 this round for the decline path |
| `tan_solver` | `TestSolver004` | good, but the `test_number == 3` fixture is **never invoked** — and it is labelled "test equation that caused bug" |
| `algebra_solver` | `TestSolver002` | was 3 exact-expression asserts, all linear/positive; +7 this round |
| `sub_transform` | `TestSolver006` | 6 exact asserts; the `(-e1)` branch is never exercised |
| `x2y2_transform` | `TestSolver010` | weakest — `ntests = 0` set and never incremented or asserted, so its two bare `assert`s sit inside an `if` with nothing proving it fired |
| `rank_leaf` | **none** | in the tree |
| `assigner_leaf` | **none** | in the tree |
| `comp_detect` | **none** | in the tree |
| `sum_id` | **none** | its `__main__` prints *"This node is not currently used!"* and `quit()`s. **Stale** — `sum_id` IS in the tree; `sum_solve` is the unused one |

Four cross-cutting gaps, in the order they matter:

1. **No test ticks the composite `ikSolver.py` actually builds.** Every leaf test uses an isolated
   leaf or a hand-rolled subtree. Finding 1 above lives at the `OrNode([tanSol, scSol]) -> rank` seam
   and is invisible to any per-leaf test. This is the single biggest structural gap.
2. **Failure paths are untested everywhere.** Whether a leaf returns `FAILURE`, raises, or returns
   `SUCCESS` having done nothing was unspecified — and three of the six findings live there.
3. **Exact-expression assertions still dominate the un-audited leaves** (`sub_transform`, `x2y2`).
   Same critique as round 1: they pin form, not correctness.
4. The `ntests == N` idiom is good and mostly used; `x2y2` is the one place it is present but dead.

### Mutation testing

Both fixes were mutation-tested, and one mutation deliberately came back clean:

| mutation | result |
|---|---|
| `linear_match` without `exclude=` | 3 failures |
| `algebra_id` shape screen removed | 1 failure (`test_algB_scans_past_an_unusable_equation`) |
| `algebra_solve` self-reference guard removed, `exclude=` intact | **OK — no failure** |
| `sincos_solve` arcsin back to `assert()` | 1 failure |

The third is recorded honestly rather than quietly: with `linear_match` in place the
`sol.has(u.symbol)` guard in `algebra_solve` is genuinely unreachable, so no test can cover it. It
is kept because it states the invariant that matters and costs nothing — but this repo has a track
record of dead guards containing wrong formulas (`sinANDcos`'s `r = sqrt(2)*A`), so it is flagged
rather than assumed harmless.

The second mutation is also informative: removing the ID screen alone does **not** produce wrong
answers, because `algebra_solve`'s copy of the check catches them. The two layers are independently
covered, which is what defence in depth is supposed to look like.

### Open inconsistency

`sincos_solve` still uses `assert(not targument.has(u.symbol))` for its self-reference guard, while
`algebra_solve` now returns `b3.FAILURE` for the same condition. Both are defensible — an assert is a
loud tripwire for "this should be impossible", a FAILURE lets another leaf try — but they should not
differ by accident. BH to pick one.

### Design intent of `rank_leaf` / `assigner_leaf` (BH, Aug 2026)

Recording this because it is not recoverable from the code, and it changes how the two leaves
should be judged.

More than one leaf can often solve the current unknown, and the solutions are not equally good — an
`atan2(y,x)` form is preferable to a `±acos(...)` pair, because it is single-valued and better
conditioned. Expressing "try several solvers, then keep the nicest answer" did not fit cleanly into
the BT framework, which is built around first-success-wins. `assigner` + `OrNode` + `rank` is the
workaround: `OrNode` deliberately runs **both** `tanSol` and `scSol` (unlike a `Priority`, it does
not stop at the first success) so that `rank` can compare the two results afterwards and discard the
worse one.

So `rank` is hard-coded expert preference, not a generic mechanism, and its comparison — fewer
solutions first, then fewer dependencies — is the encoding of that preference. Two consequences:

- Judging `rank` as "untested logic" understates it: it is the only place in the tree that *discards*
  a correct solution, and the criterion it discards on is a design decision, not a derivable fact.
  A test should pin the preference (atan2 beats acos-pair), not just the mechanics.
- The `OrNode`-runs-both behaviour is load-bearing and easy to mistake for a bug. Anything that
  "optimises" it into a `Priority` silently disables ranking.

## KawasakiRS007L regression, August 2026 — and a correction to round 1

BH ran the reference robots and found `KawasakiRS007L`, which had solved for years, now aborting:

```
File "ikbtleaves/tan_solver.py", line 139, in tick
    assert(d1 is not None and d2 is not None), 'somethings wrong!'
AssertionError: somethings wrong!
```

**Cause: last round's `exclude=terms` hardening of `tan_id`.** Bisected — the crash reproduces with
this session's `algebra_solver` / `sincos_solver` changes reverted to `HEAD`, so it is not from
those. The offending pair, printed by instrumenting the assert:

```
unknown : th_2
sin eqn : 0 = -Pz + l_1 + l_2*sin(th_2) - l_3*cos(th_23)          -> matches fine
cos eqn : 0 = -Px + (l_2*cos(th_2) + l_3*sin(th_23))*cos(th_1)    -> d2 is None
```

`cos(th_2)` sits inside an **unexpanded product**, and `match()` is structural — it will not expand
to find it. Confirmed directly on that exact expression:

| Wilds | result |
|---|---|
| unconstrained (pre-Aug-2026) | `{Cw: 0, Dw: -Px + (l_2*cos(th_2) + l_3*sin(th_23))*cos(th_1)}` |
| `exclude=terms` (round 1) | `None` |
| `exclude=terms` + `.expand()` | `{Cw: l_2*cos(th_1), Dw: -Px + l_3*sin(th_23)*cos(th_1)}` |

So the assert was only ever safe because an unconstrained `Wild` **cannot fail to match**. Unusable
pairs were rejected a few lines later by the `count_unknowns(d2[Dw]) > 0` screen — `{Cw: 0, Dw: <whole
expr>}` trips it immediately. Making the match correct made it return `None`, and turned a graceful
rejection into a hard crash.

Fixed by skipping the pair (`continue`) instead of asserting, which reproduces the old outcome
exactly. Covered by `test_tanC_undecomposable_pair_declines_not_asserts`, which uses the real
Kawasaki expressions and asserts only that the leaf **declines** — mutation-tested by restoring the
assert.

`.expand()` before `.collect()` would make these pairs genuinely solvable, and is deliberately NOT
done: that is a capability change, not a regression fix. Worth considering separately — it would let
`tan_id` claim pairs it has never claimed, on every robot.

### The correction

Round 1 concluded, in *Correction: how reachable the `Wild` bug actually is*:

> These are **defensive hardening, not repairs of an active defect** ... unlikely to change any
> robot result. Expect the baseline diff to show nothing from these two changes.

**That was wrong for `tan_solver`.** The change did not merely fail to help — it broke a reference
robot outright, and the breakage sat undetected because nothing runs the robots.

Both the original severity claim and its correction reasoned about the *same* trigger, a loose
numeric additive term, and concluded DH-conforming robots cannot produce one. That reasoning was
sound and is still true. It was simply **not the only way `match()` behaviour changes**: an
unexpanded product is a completely different mechanism, and neither the claim nor the retraction
considered it. Two rounds of careful analysis of a single failure mode, and the actual defect was a
second one nobody enumerated.

The transferable lesson is narrower than "we were overconfident". When you tighten a matcher, the
question is not *"which inputs newly fail to match?"* — it is **"what does every caller do when the
match returns `None`?"** There were two callers. One (`count_unknowns` screen) degraded gracefully.
One asserted. Only the second mattered, and it could have been found by reading the call sites
rather than by reasoning about inputs at all.

Round 1 already recorded a version of this — *"fixing one defect makes the next one testable, so an
audit pass should be re-run after each fix"* — and this session's `sincos_solve` note said the
hardening made that assert *more* reachable. The pattern was identified twice and still not swept
for systematically. **Concrete action: grep every leaf for `assert` on a `match()` result.**

### Robot-level output is not reproducible

Discovered while diffing the reference robots before/after. Two runs of `Puma` with **identical
code** produce LaTeX differing in 26 lines; with `PYTHONHASHSEED=0` they are byte-identical.

Cause: `solutionSet` is a Python `set` of tuples, and the solution-graph edges iterate sets too, so
row order and edge order follow the randomized string hash. The differences are pure ordering — the
same rows, permuted.

This matters directly for the planned robot-level baseline suite (*Related gap: no robot-level
regression suite*): a naive `diff` of generated `.tex` will show spurious changes on **every** run
and the suite will be ignored within a week. Any such suite must either pin `PYTHONHASHSEED`, sort
before comparing, or compare parsed structures rather than text.

Separately, the pre-session `.tex` baselines differ from current output by LaTeX **whitespace only**
(line breaks inside `align`/`dmath`), meaning they predate an `output_latex.py` formatting change.
Worth regenerating them before trusting any before/after comparison.

### State after this session

All four reference robots solve (`Wrist`, `Puma`, `Chair_Helper`, `KawasakiRS007L` — exit 0, no
tracebacks, completion detector reached). Unit suite green at 23 tests.

## Fixes for open items 1-3, August 2026

All three mutation-tested; all four reference robots re-run afterwards.

### 1. `rank_leaf` IndexError — two defects, not one

**Root cause.** `sincos_solve` appended to `u.sincos_eqnlist` in its arcsin branch but not in its
arccos branch, while `rank` reads `u.sincos_eqnlist[0]` when it picks sincos. Fixed by appending in
both branches, with the invariant now stated in a comment: *sincos_eqnlist must be populated whenever
sincos_solutions is.*

**Second defect, found while fixing the first — and made more reachable by this session's own
item-4 change.** `rank` keyed its decision off the `solvable_*` flags and then indexed the solution
lists unguarded:

```python
if u.solvable_sincos and u.solvable_tan:
    if (len(u.sincos_solutions) < len(u.tan_solutions)) and (len(u.sincos_solutions) > 0): ...
    elif (len(u.sincos_solutions) > len(u.tan_solutions)) and (len(u.tan_solutions) > 0): ...
    else:
        sol_sin = u.sincos_solutions[0]      # <-- no guard, and this is the branch that can fail
```

Both `len(...) > 0` guards sit on branches that cannot fail; the bare `else`, which can, has none.
An ID node can set `solvable_*` and its solver then decline, leaving the flag `True` and the list
empty — and item 4 of this session turned `sincos_solve`'s arcsin `assert` into exactly such a
`return b3.FAILURE`, so the fix for one crash widened the window for another. **Third time this
session that tightening one thing made the next thing reachable.**

Fixed by keying the decision on the solution lists actually produced (`n_sc`, `n_tan`) rather than on
the flags, which removes every unguarded index at once. When neither solver produced anything, `rank`
now declines to call `set_solved()` — previously it marked the variable solved with no solution.

**Reachability, measured.** Every ranking decision across all four reference robots resolves to
`best ranked, atan2(y,x)` — tan always wins, so the `choosen == "sincos"` path is never taken and the
IndexError never bit in production. But `KawasakiRS007L`'s `th_6` is solved by **arccos and tan and
is ranked** (`th_6 (atan2(y,x), arccos, best ranked, atan2(y,x))`): it satisfies every precondition
except the tie-break outcome. One change in relative solution counts away from aborting the solve.
Not hypothetical — latent.

New `TestSolver011` in `rank_leaf.py`, registered in `tests/leavestest.py`. Four tests: the arccos
crash, the ID-fired-but-solver-declined crash, the core preference (tan's single `atan2` beats
sincos's `±acos` pair), and single-solver pass-through. Per the design note above, these pin the
*preference*, not just the mechanics.

### 2. `tan_solve`'s asserts — the same construct, one function away

`tan_solve` carried the identical assert-on-`match()` pattern that broke Kawasaki in `tan_id`:

```python
assert(d != None), fs
assert(count_unknowns(unknowns, d[Bw])==0), fs
```

In the tree these hold only because `tan_id` screened the *same* expressions with equivalent `Wild`s
before setting `solvable_tan` — an implicit invariant between two nodes with nothing enforcing it.
Converted to `return b3.FAILURE`. Covered by `test_tanD_solve_node_declines_bad_input_not_asserts`,
which deliberately bypasses `tan_id` to violate that invariant.

### 3. `x2y2_transform` guard precedence

```python
if not u.symbol == th_3 or u.symbol == th_2 :   ->   if u.symbol not in (th_2, th_3):
```

The old form parses as `(not (sym == th_3)) or (sym == th_2)`, so `th_2` was rejected and the `or`
clause was dead — only `th_3` ever got through, despite the comment above it naming both.

Also fixed in the same block: the "find the current unknown" loop assigned on every match and kept
the **last** one, which could be an already-solved unknown also present in `temp_r`; the following
`if not unknown.solved` then silently skipped appending the new equation while the leaf still
returned `SUCCESS`. It also shadowed the imported `unknown` class with a local of the same name. Now
searches for the first *unsolved* match, under the name `target`, and returns `FAILURE` if there is
none.

**Effect on the reference robots: none.** Verified by byte-comparing seed-pinned Puma output before
and after — identical. `x2y2` still fires exactly once per robot (Puma, Kawasaki), credited to
`th_3`, because `algSol` sits ahead of `x2z2_Solver` in the `Priority` and solves `th_2` by algebra
long before x2y2 is reached. So enabling `th_2` is latent capability with no current consumer. It is
still the intended behaviour, and the dead clause was certainly not.

New `test_x2y2B_th2_is_not_rejected` uses the cheap test-1 fixture (no Puma kinematics) and, unlike
the existing `test_x2z2`, actually counts its assertions.

### State

Unit suite green, 23 tests (was 21 at the start of the session). All four reference robots solve:
exit 0, no tracebacks, completion detector reached. Nothing committed.
