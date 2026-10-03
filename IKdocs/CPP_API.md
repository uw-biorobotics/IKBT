# The generated C++ — an API reference

**Who this is for:** you have run `python3 ikSolver.py <Robot>`, you have
`CodeGen/Cpp/<Robot>CppCode/<Robot>.cpp`, and you want to call it from an application of your own.

This lists every type and function you can call, says where each lives, and — the question that
sends people looking through the sources — says **who calls what**. Nothing in `Cpp_src/` calls a
generated file and no generated file calls another's internals; the two meet at exactly the places
named in "The wiring" below.

Every program in section 2 was compiled with `-Wall -Wextra` and run against the checked-in
generated code; the outputs shown are what they actually printed.

---

## 1. One file, one namespace

**Everything specific to a robot is in `CodeGen/Cpp/<Robot>CppCode/<Robot>.cpp`, inside
`namespace ikbt::<Robot>`** — forward kinematics, Jacobian, and whichever inverse kinematics the
behavior tree managed to produce. Everything generic is in `Cpp_src/`, which that file reaches by
a relative `#include`. So there is one question and one answer:

```cpp
#include "CodeGen/Cpp/PumaCppCode/Puma.cpp"

ikbt::Puma::ikin(T);
```

**The namespace is the qualification.** The arm the file is named for owns the plain names:
`fk()`, `jacobian()`, `ikin()`, `ikin_given()`, `solve()`, `refine_seed()`. You never write
`ikin_Puma` inside `namespace Puma`. For a robot whose name is not an identifier, the namespace
mangles (`C-Arm` → `ikbt::C_Arm`) and a namespace alias reads well:

```cpp
namespace CArm = ikbt::C_Arm;
CArm::solve(T);
```

**Two robots can be linked into one program.** `ikbt::Puma::ikin` and `ikbt::Wrist::ikin` are
different functions. This did not work before — see section 6.

### Which IK did my robot get?

IKBT has three solution paths and your robot got exactly one of them. **The file says which, in
`SOLUTION_PATH`**, and the functions follow from it:

| `SOLUTION_PATH` | what happened | your entry point |
|---|---|---|
| `"symbolic"` | a complete closed form | `ikin(T)` |
| `"onevar"` | closed form in one assumed variable, plus a 1-D search for it | `solve(T)` |
| `"hybrid"` | a nearby arm's closed form, corrected numerically | `ikin_approx(T)` then `refine_seed(T, q)` |

```bash
grep SOLUTION_PATH CodeGen/Cpp/PumaCppCode/Puma.cpp
```

**`fk()` and `jacobian()` are there whichever path ran.** The report has a Forward Kinematics
section and a Jacobian section on every path, so the code does too; the three paths' files differ
only where the methods genuinely differ.

On the **hybrid** path the file describes **two arms**. The true arm owns the plain names; the
*derived* (simplified) arm keeps a suffix — `ikin_Panda_a_3_0_a_4_0()`, `fk_Panda_a_3_0_a_4_0()` —
because it is a different robot and you must not have to look up which is which. It gets no
Jacobian; nothing wants one for it.

### The wiring

This is the part that is hard to see by grepping, because the two halves are written at different
times and live in different trees.

```
        YOUR APPLICATION
               |
               v
  <Robot>.cpp :: ikbt::Panda::refine_seed(T, q_seed)      (generated, per robot)
               |
               |  calls, passing fk and jacobian as the two callbacks
               v
  Cpp_src/ikbt_dls.h :: solve_numeric(fk, jac, q0, T_d, w_rot, ...)   (SOURCE, generic)
               |
               +--> Cpp_src/ikbt_linalg.h     :: solve6()       the 6x6 step
               +--> Cpp_src/ikbt_pose_error.h :: pose_error()   "how far from the goal"
               |
               v  (the callbacks, back in the same generated file)
  <Robot>.cpp :: ikbt::Panda::fk(q)  and  ikbt::Panda::jacobian(q)
```

So, for the two things that are hardest to find:

- **`solve_numeric()` has exactly one caller in the shipped code: `refine_seed()` in a hybrid
  robot's file** (plus `scripts/cpp_closed_loop_check.py --dls`, which pins it against the python).
  It is generic on purpose — pass it your own `FkFn`/`JacFn` and it will refine anything.
- **The "numerical FK function" it is passed is generated, not hand-written.** It is `fk()` in the
  same file, emitted by `ikbtfunctions/output_cpp_common.py::fk_body_cpp()`, alongside
  `jacobian()`, which is the other callback.
- **A symbolic robot has `fk()` too, but nothing in its own file calls it.** Puma's closed form is
  exact, so there is nothing to refine and `ikbt_dls.h` is never included. It is there for *you* —
  to check a posture by round-tripping it, or to drive `solve_numeric()` from a seed of your own.

### What else is in the directory

`CodeGen/Cpp/<Robot>CppCode/` holds `<Robot>.cpp` and nothing else, unless you ran
`python3 fkOnly.py <Robot>`, which writes a standalone `FK_numeric<Robot>.h` with `fk_<Robot>(q)`
and `jacobian_<Robot>(q)` — suffixed, because a bare header has no robot namespace to sit in.
**`fkOnly.py` deliberately does not write `<Robot>.cpp`**: it knows only the forward kinematics, and
letting it would mean running it after a solve silently discarded that solve's IK.

If you have a directory from an older IKBT, solving the robot again clears the superseded
`IK_equations*`, `IK_conditional*`, `IK_onevar*` and `IK_hybrid_*` files out of it. They still
compile, which is what makes them dangerous — nothing refreshes them again.

---

## 2. Quick start

Three complete programs, one per path. Build each with

```
g++ -std=c++11 -O2 myapp.cpp -o myapp -lm
```

**No `-I`, no library, no build system.** The `#include` path to the robot file is the only thing
that has to be right.

### 2a. Symbolic (Puma)

```cpp
#include "CodeGen/Cpp/PumaCppCode/Puma.cpp"
#include <cstdio>

namespace Puma = ikbt::Puma;

int main()
{
    //  A pose the arm certainly reaches: its own FK on a joint vector.
    const ikbt::JointVec q0 = {0.30, -0.80, 0.50, 1.10, 0.70, -0.40};
    const ikbt::Mat4 T = Puma::fk(q0);

    ikbt::SolutionList sols = Puma::ikin(T);          // 0..8 postures
    if (sols.empty()) {
        printf("no branch of the closed form exists at this pose\n");
        return 1;
    }
    for (size_t i = 0; i < sols.size(); ++i) {
        //  CHECK IT: round-trip the posture through the arm's own FK.
        const ikbt::PoseError e = ikbt::pose_error(Puma::fk(sols[i]), T, 0.4);
        printf("posture %zu  err=%.2e :", i, e.metric);
        for (size_t j = 0; j < sols[i].size(); ++j)
            printf(" %s=%8.4f", Puma::JOINT_NAMES[j], sols[i][j]);
        printf("\n");
    }
    return 0;
}
```

All eight come back at ~1e-16, and posture 1 is `q0` recovered exactly:

```
posture 0  err=1.24e-16 : th_1=  3.9837 th_2=  1.9160 th_3=  0.5000 th_4= -0.9574 th_5=  1.9730 th_6= -2.4012
posture 1  err=5.55e-17 : th_1=  0.3000 th_2= -0.8000 th_3=  0.5000 th_4=  1.1000 th_5=  0.7000 th_6= -0.4000
...
```

**Round-trip it like this and do not skip the check — a returned branch is a CANDIDATE.** IKBT
enumerates combinations of each unknown's solution branches and does **not** discard the ones that
fail the original equations, so a finite row is not by itself a correct one. Hand `Puma::ikin()` a
pose the arm cannot reach in *orientation* and it still returns eight finite rows; they match in
position and are wrong by up to 1.0 in the metric. This one check tells the two cases apart, and
it needs no extra include: `pose_error()` is pulled in by every robot file, on every path,
precisely so that a file can check its own work.

### 2b. One-variable (C-Arm)

```cpp
#include "CodeGen/Cpp/C-ArmCppCode/C-Arm.cpp"
#include <cstdio>

namespace CArm = ikbt::C_Arm;      // 'C-Arm' is not an identifier

int main()
{
    //  A pose we know is reachable: run the arm's own FK on a joint vector.
    const ikbt::JointVec q0 = {0.35, 0.60, -0.40, 0.90, 0.25, -0.70};
    const ikbt::Mat4 T = CArm::fk(q0);

    std::vector<ikbt::OneVarSolution> sols = CArm::solve(T);   // the 1-D search
    if (sols.empty()) {
        printf("no solution FOUND -- which is not quite 'unreachable'\n");
        return 1;
    }
    for (size_t i = 0; i < sols.size(); ++i) {
        printf("q =");
        for (size_t j = 0; j < sols[i].q.size(); ++j)
            printf(" %8.4f", sols[i].q[j]);
        printf("   %s=%7.4f  err=%.2e  branch=%d  n=%d\n",
               CArm::KNOWN_VARIABLE, sols[i].known_value,
               sols[i].error, sols[i].branch, sols[i].n_samples);
    }
    if (sols[0].n_samples >= CArm::MAX_SAMPLES)
        printf("WARNING: hit MAX_SAMPLES -- the set was still changing\n");
    return 0;
}
```

Eight postures in 9 ms, the first of them `q0` recovered to 7.9e-17:

```
q =   0.3500   0.6000  -0.4000   0.9000   0.2500  -0.7000   th_2= 0.6000  err=7.85e-17  branch=0  n=256
q =   1.5111  -1.1905  -0.5291   0.4102  -1.5700  -0.2131   th_2=-1.1905  err=1.76e-16  branch=0  n=256
...
```

`sols[i].q` is a **complete** joint vector in `ONEVAR_JOINT_NAMES` order, with the assumed variable
already in its own chain position, so it goes straight into `CArm::fk()`.

### 2c. Hybrid (Panda) — two steps, on purpose

```cpp
#include "CodeGen/Cpp/PandaCppCode/Panda.cpp"
#include <cstdio>

namespace Panda = ikbt::Panda;

int main()
{
    //  A pose we know the TRUE arm reaches: its own FK on a joint vector.
    const ikbt::JointVec q0 = {0.30, 0.55, -0.40, 0.85, 0.60, -0.25};
    const ikbt::Mat4 T = Panda::fk(q0);

    //  PHASE I -- seeds from the APPROXIMATE arm.  Not answers.
    const ikbt::SolutionList seeds = Panda::ikin_approx(T);
    printf("%zu seeds\n", seeds.size());
    if (seeds.empty())
        return 1;

    //  Pick the posture you want -- normally the seed nearest where the arm
    //  is now.  Here, just the first.
    //  PHASE IIa -- correct it against the TRUE arm.
    const ikbt::SolveResult r = Panda::refine_seed(T, seeds[0]);
    if (!r.converged) {
        printf("did not converge: %s (metric %.3e)\n", r.reason, r.metric);
        return 1;
    }
    for (size_t j = 0; j < r.q.size(); ++j)
        printf(" %s=%8.4f", Panda::HYBRID_JOINT_NAMES[j], r.q[j]);
    printf("\n  %d iterations, metric %.3e\n", r.iterations, r.metric);
    return 0;
}
```

Which prints:

```
8 seeds
 th_1=  0.3000 th_2=  0.5500 th_3= -0.4000 th_4=  0.8500 th_5=  0.6000 th_6= -0.2500
  4 iterations, metric 7.885e-11
```

**A seed that has not been through Phase II does not put the real robot where you asked.** The
closed form belongs to a *displaced* arm (for Panda, `a_3` and `a_4` zeroed — 0.106 of task-space
cost). Use `Panda::refine_all(T)` once, in a diagnostic, to learn which postures the true arm can
actually reach; use `refine_seed()` from then on.

---

## 3. `Cpp_src/` — the shared library (SOURCE, hand-written)

Five headers, header-only, everything `inline`. You get them by a **relative `#include`** from a
generated file — `../../../Cpp_src/ikbt_types.h` from a robot's directory. There is **one copy for
the whole package**, so a fix here reaches every robot already generated without regenerating
anything. Keep a robot's directory at its depth relative to `Cpp_src/`, or carry `Cpp_src/` along.

Line numbers are as of 2026-10-01 and will drift; the symbol names will not.

### 3.1 `ikbt_types.h` — types used everywhere

| symbol | line | meaning |
|---|---|---|
| `Mat4` | 51 | `std::array<std::array<double,4>,4>`, **row major**: `T[row][col]`, `T[0][3]` is x |
| `JointVec` | 55 | `std::vector<double>` — one joint vector. A vector, not an array, so the shared numerics are not templated on DOF |
| `SolutionList` | 60 | `std::vector<JointVec>` — one row per posture |
| `Matrix` | 65 | `std::vector<std::vector<double>>`, row major — what a Jacobian comes back as |
| `INF` | 67 | `std::numeric_limits<double>::infinity()` |

| function | line | what it does |
|---|---|---|
| `Mat4 identity4()` | 70 | 4x4 identity |
| `Mat4 from_array(const double T[4][4])` | 82 | interop with a plain C array |
| `void to_array(const Mat4 &M, double T[4][4])` | 92 | the other direction |
| `Mat4 mat_mul(const Mat4 &A, const Mat4 &B)` | 100 | 4x4 product |
| `bool all_finite(const Mat4 &T)` | 116 | every entry finite? |
| `bool all_finite(const JointVec &q)` | 126 | overload for a joint vector |

Also defines `M_PI`, `M_PI_2`, `M_PI_4`, `M_E` if absent — they are POSIX, not ISO C++, and
`-std=c++11` (as against `-std=gnu++11`) can leave them undefined while sympy's printer emits them.

### 3.2 `ikbt_pose_error.h` — one definition of "closer"

| symbol | line | meaning |
|---|---|---|
| `Mat3` | 25 | `std::array<std::array<double,3>,3>` |
| `struct PoseError` | 29 | `std::array<double,6> residual;` and `double metric;` |
| `double rotation_angle_axis(const Mat3 &Rerr, std::array<double,3> &axis)` | 45 | returns theta in `[0, pi]`, writes the unit axis |
| `PoseError pose_error(const Mat4 &T, const Mat4 &T_d, double w_rot)` | 88 | residual `[dp ; w_rot*theta*axis]` and metric `\|\|dp\|\| + w_rot*theta` |

`w_rot` is one characteristic arm length per radian — it is what puts a position error and an
orientation error in comparable units. The generated code bakes a value in as `W_ROT` /
`ONEVAR_W_ROT`, because nothing records a model's units and they are not consistent across the
robot set.

**The residual and the metric are returned together because they must never be computed two
different ways.** The refinement step and the stopping test share one rotation parameterisation and
one weight, so they cannot disagree about which of two joint vectors is nearer the goal.

`rotation_angle_axis` uses `atan2` of the skew norm, **not** `arccos((tr-1)/2)`. `arccos` has an
infinite derivative at `R = I`, so 1e-16 of rounding in the trace comes back as ~2.1e-8 radians —
and *quantised*, which turns the error curve near a root into a staircase that golden section
cannot see past. If you write your own pose error, do it this way.

### 3.3 `ikbt_linalg.h` — the only linear algebra

| function | line | what it does |
|---|---|---|
| `bool solve6(const double A[6][6], const std::array<double,6> &b, std::array<double,6> &x)` | 39 | Gaussian elimination with partial pivoting. **Returns `false` rather than throwing** on a singular or non-finite system, leaving `x` untouched |

`false` is not an error — in `solve_numeric()` a singular solve is a **rejected step**, which raises
the damping and tries again. That is what the damping is for.

### 3.4 `ikbt_dls.h` — damped least squares (the Phase II numerics)

| symbol | line | meaning |
|---|---|---|
| `FkFn` | 40 | `std::function<Mat4(const JointVec &)>` |
| `JacFn` | 41 | `std::function<Matrix(const JointVec &)>` |
| `struct SolveResult` | 46 | `q`, `metric`, `iterations`, `converged`, `reason`, `history` |
| `Matrix jacobian_base(const Mat4 &T, const Matrix &J66)` | 57 | `J_0 = blkdiag(R,R) . J_66` — frame 6 into the base frame |
| `SolveResult solve_numeric(...)` | 77 | the solver |

```cpp
SolveResult solve_numeric(const FkFn &fk, const JacFn &jac,
                          const JointVec &q0, const Mat4 &T_d,
                          double w_rot, double tol = 1e-9,
                          int max_iter = 100, double lam0 = 1e-3,
                          double lam_min = 1e-12, double lam_max = 1e12);
```

`dq = J'(JJ' + lam^2 I)^-1 e`. Levenberg-Marquardt: a step that improves the metric is accepted and
`lam` drops by 3; one that does not is rejected and `lam` rises by 3 (by 10 for a singular solve).

- **It never throws and never fails on a bad step.** A singular `solve6`, a non-finite FK, a
  non-finite `dq` — all are rejected steps.
- **`converged` means `metric <= tol`. Test it.** A large metric is a real answer: it says the arm
  does not reach `T_d` from this seed, or at all.
- `reason` is one of `"converged"`, `"max_iter"`, `"lam_max"`.
- `history` is the metric after every accepted step; `iterations == history.size() - 1`.
- **It stays in the basin of its seed.** That is why `refine_seed()` takes a seed rather than
  choosing one: the branches are different *postures*, and which you want depends on obstacles,
  joint limits and where the arm is now — none of which IKBT knows.
- It converges at the wrist singularity `th_5 = 0`, where an undamped Newton step does not exist.

**To use it on something that is not an IKBT robot**, hand it your own two callables:

```cpp
#include "Cpp_src/ikbt_dls.h"

ikbt::FkFn  myfk  = [](const ikbt::JointVec &q) { ...; return T; };
ikbt::JacFn myjac = [](const ikbt::JointVec &q) { ...; return J; };   // 6 x N, IN FRAME 6
ikbt::SolveResult r = ikbt::solve_numeric(myfk, myjac, q0, T_goal, 0.316);
```

Nothing above this line is IKBT-specific: `ikbt_dls.h` pulls in only `ikbt_types.h`,
`ikbt_linalg.h` and `ikbt_pose_error.h`, and no generated file at all. To drive it with an IKBT
robot's own kinematics instead, include that robot's file and wrap its two functions:

```cpp
#include "CodeGen/Cpp/PandaCppCode/Panda.cpp"

ikbt::FkFn  myfk  = [](const ikbt::JointVec &q) { return ikbt::Panda::fk(q); };
ikbt::JacFn myjac = [](const ikbt::JointVec &q) { return ikbt::Panda::jacobian(q); };
```

**The Jacobian must be expressed in frame 6**, which is what IKBT computes and stores as `J66`;
`solve_numeric()` rotates it with `jacobian_base()` itself. Hand it a base-frame Jacobian and the
rotation rows will be wrong.

### 3.5 `ikbt_search.h` — the 1-D search over the assumed variable

Generic where the python is emitted per robot (~470 lines per robot there), because `Cpp_src/` *is*
the library the python side does not have. Only three of these are things an application calls; the
rest are the mechanism, listed so you can read it.

**Types**

| symbol | line | meaning |
|---|---|---|
| `struct OneVarSolution` | 76 | `JointVec q`, `double known_value`, `int branch`, `double error`, `int n_samples` |
| `struct SearchConfig` | 85 | `lo`, `hi`, `periodic`, `n_samples`, `max_samples`, `accept_tol`, `dedup_tol`, `refine_iters` — defaults `[-pi, pi]`, periodic, 128, 4096, 1e-6, 1e-6, 80 |
| `BranchesFn` | 103 | `std::function<std::vector<JointVec>(const Mat4 &, double)>` — "what does the closed form give at this value" |
| `ErrorsFn` | 104 | `std::function<std::vector<double>(const Mat4 &, double)>` — "how wrong is each branch there" |
| `ErrorCache` | 106 | `std::map<double, std::vector<double>>` — memo keyed on the sample value |

**The entry point**

| function | line |
|---|---|
| `std::vector<OneVarSolution> onevar_solve(const Mat4 &T, const BranchesFn &, const ErrorsFn &, const SearchConfig &)` | 529 |

Climbs a **doubling ladder** — scan at `n_samples`, then 2n, 4n — and returns as soon as two
successive resolutions agree about the whole set of postures, or when doubling would pass
`max_samples`. Each returned solution carries the resolution it settled at in `n_samples`;
**a value equal to `max_samples` means the answer was still changing**. Solutions returned there are
still *sound* (each reaches `T` to within `accept_tol`) but their completeness is unproven.

An empty result means no solution was **found**, which is not quite "unreachable".

**The mechanism** (called by `onevar_solve`, useful for plotting or diagnosis)

| function | line | what it does |
|---|---|---|
| `sample_values(cfg, n)` | 118 | n ascending values. `lo + span*(k/n)`, not an accumulated step, so a doubling re-uses the coarse grid's cached values *exactly* |
| `wrap_value(t, cfg)` | 131 | back into `[lo, hi)` on a periodic domain |
| `cached_errors(T, v, cache, errors)` | 147 | `errors()` memoised on the value |
| `error_of(T, t, b, cache, errors, cfg)` | 158 | one branch's error at one value; `INF` past the end of the branch list |
| `scan_curves(T, n, cache, errors, cfg, values, curves)` | 177 | `curves[branch][i]`. **Gives every real sample two neighbours** — a wrap on a periodic domain, one extra step past each end on a finite one — without which a root at either end is never bracketed |
| `golden(f, a, b, iters, fx, xtol)` | 232 | golden section. Not a derivative method: near a solution the error is a V, not a parabola |
| `local_minima(curve)` | 272 | indices where the curve dips. A neighbour of `INF` is allowed and is often exactly where a solution is |
| `max_abs_diff(a, b)` | 285 | `INF` on a size mismatch |
| `dedup(found, tol)` | 301 | one entry per posture, keeping the most accurate |
| `roots_in(...)` | 321 | bracket every dip in one branch's curve and refine it. **Accepted only if it reaches `accept_tol`** — a minimum that stops above it is a feature of the curve, not a solution |
| `hunt_near(...)` | 357 | the **twin hunt**: re-scan one grid step around each root at 16x, three rounds |
| `domain_edges(curve)` | 408 | adjacent finite/infinite pairs |
| `hunt_edges(...)` | 420 | the **domain-edge probe**: bisect to the edge, then walk inward in halving steps |
| `solve_at(T, n, ...)` | 478 | every root at ONE resolution — grid, then edges, then twins |
| `same_postures(a, b, tol)` | 502 | the ladder's stopping test |

**Why three mechanisms and not one.** Roots hide in three ways and each mechanism finds cases the
others cannot. The ladder is the general one and is *not* proof — two successive grids can step over
the same pair. The twin hunt catches close pairs (C-Arm has roots 0.045 rad apart against a 0.049
sample spacing). The edge probe catches spikes narrower than any affordable grid: approaching from
inside, the curve *rises* first, so the nearest sample is a local **maximum** and the ordinary scan
never brackets it at any resolution. C-Arm has poses with a root within 1e-3 of `th_2 = ±pi/2`, in a
spike of slope 1000.

---
## 4. What is inside `<Robot>.cpp`

All of it is in `namespace ikbt::<Robot>`. The sections appear in the order below, which is
declaration order in one translation unit.

### 4.0 On every path

| symbol | what it is |
|---|---|
| `const char* const SOLUTION_PATH` | `"symbolic"`, `"onevar"` or `"hybrid"` — which path produced this file |
| `Mat4 fk(const JointVec &q)` | q -> 4x4 transform of frame 6 in the base frame |
| `Matrix jacobian(const JointVec &q)` | q -> 6 x NDOF Jacobian **expressed in frame 6** |
| `FK_JOINT_NAMES[]`, `N_FK_JOINT_NAMES`, `NDOF` | the DH **chain** order a `q` must be in |

Emitted by `output_cpp_common.fk_body_cpp()`. The parameters are **baked in**, not left as
globals to be overridden — the opposite of what the IK section does, and deliberately: these two
functions are the definition of "this arm".

Only the first `NDOF` columns of the Jacobian are emitted. `J66` is stored 6x6 for every robot and
the surplus columns of a short arm are **not** zero — a padded DH row still gets a column computed
for it — so handing them on would let a solver move joints the arm does not have.

### 4.1 `SOLUTION_PATH == "symbolic"`

| symbol | what it is |
|---|---|
| `SolutionList ikin(const Mat4 &T)` | the closed form. **One row per posture that EXISTS at `T`** |
| `int ikin_array(double T[4][4], double sol[IK_NBRANCHES][IK_NJOINTS])` | a plain C-array interface, for callers holding a `double[4][4]`. Returns 1 if any posture was found, 0 if none |
| `JOINT_NAMES[]`, `N_JOINT_NAMES` | the order of a returned row: which joint sits at each position |
| `AUX_NAMES[]`, `N_AUX_NAMES` | sum-of-angle intermediates: computed, **not returned** |
| `IK_NJOINTS`, `IK_NBRANCHES` | `IK_NBRANCHES` is the *maximum*; the returned count is usually smaller |
| link lengths (`a_2`, `d_4`, …) | baked in as `const double` from the robot's `pvals` |

**A non-finite row is a posture that does not exist, and only that row is dropped.** The returned
count therefore varies with the pose, and an empty list means none exist. (The python twin returns
`False` there; an empty list is the same information without a second return type.)

**`ikin_array`, not `ikin`.** The fixed-array wrapper has a name of its own so that `ikin` means
exactly one function.

**If you see `XXXXX` where a link length should be, that is deliberate** — the robot has a
parameter with no `pvals` entry, and the generator makes `g++` name the line rather than let a
missing length be silently defaulted.

### 4.2 `SOLUTION_PATH == "onevar"`

The closed form solves every *other* joint given a value for one assumed variable; `solve()`
searches for the values that are right.

| symbol | what it is |
|---|---|
| `std::vector<OneVarSolution> solve(const Mat4 &T, int n_samples = N_SAMPLES)` | **the entry point.** Every joint vector that reaches `T` |
| `SolutionList ikin_given(const Mat4 &T, double <known>)` | the conditional closed form. The assumed value is an **argument**, not a constant |
| `int ikin_array_given(double T[4][4], double sol[][], double <known>)` | fixed-array interface |
| `void sweep(const Mat4 &T, int n, std::vector<double> &values, std::vector<std::vector<double>> &curves)` | the raw scan at ONE resolution — for plotting, and for understanding a pose that comes back empty |
| `std::vector<JointVec> branches_at(const Mat4 &T, double value)` | the `BranchesFn`. Every branch at one assumed value |
| `std::vector<double> errors(const Mat4 &T, double value)` | the `ErrorsFn`. `INF` where a branch is undefined |
| `SearchConfig search_config(int n_samples = N_SAMPLES)` | the baked-in config |
| `ROBOT`, `KNOWN_VARIABLE`, `COND_KNOWN_VARIABLE` | the robot, and which variable was assumed known |
| `ONEVAR_JOINT_NAMES[]`, `ONEVAR_NDOF` | the order of `OneVarSolution::q` |
| `SEARCH_LO`, `SEARCH_HI`, `PERIODIC`, `N_SAMPLES`, `MAX_SAMPLES`, `ACCEPT_TOL`, `DEDUP_TOL`, `ONEVAR_W_ROT` | the search parameters |

Two things to know about `ikin_given()`:

- **It is the one function that keeps every row in place, NaNs and all.** `solve()` calls it at
  hundreds of values and builds one error curve per branch, **indexing by position**. Dropping a
  row at some values and not others would stitch one branch's error onto another's curve and wreck
  the per-branch domain edges the search hunts roots at. So test `all_finite()` yourself if you
  call it directly.
- **`ikin_given`, never `ikin`.** The plain name means an *unconditional* inverse kinematics for
  the robot, and these equations are exact only where the assumed value is right. (The python twin
  says the same thing with a file name: `IK_conditional<Robot>.py`, never `IK_equations<Robot>.py`.)

Nothing about the robot is approximated here — the DH table, the FK and the equations are the true
arm's. Measured on C-Arm over 160 random reachable poses: 888 of 888 true solutions found, none
spurious, worst round-trip pose error 2.4e-12, 0.14 s per pose.

### 4.3 `SOLUTION_PATH == "hybrid"`

**Two arms in one namespace.** The true arm owns the plain names; the derived arm keeps its suffix.

| symbol | what it is |
|---|---|
| `SolutionList ikin_approx(const Mat4 &T, bool filter_spurious = true)` | **PHASE I** — seeds from the approximate arm. `filter_spurious` drops branches that do not reach `T` on the approximate arm's own FK; pass `false` to see the raw list |
| `SolveResult refine_seed(const Mat4 &T, const JointVec &q_seed, double tol = 1e-9, int max_iter = 100)` | **PHASE IIa** — the operational call. DLS against the **true** arm |
| `std::vector<RefineRecord> refine_all(const Mat4 &T, double tol = 1e-9, int max_iter = 100)` | **PHASE II** — Phase IIa over every seed. `RefineRecord` is `{int index; JointVec q_seed; SolveResult result;}` |
| `TRUE_ROBOT`, `APPROXIMATE_ARM`, `DH_CHANGES` | which arm is which, and what was changed |
| `HYBRID_JOINT_NAMES[]`, `NDOF` | the order of `q_seed` and of `result.q` |
| `W_ROT`, `APPROX_TOL` | the metric weight; the Phase I acceptance tolerance |
| `ikin_<D>(const Mat4 &T)`, `ikin_array_<D>(...)` | the **derived** arm's closed form |
| `fk_<D>(q)`, `FK_JOINT_NAMES_<D>[]`, `NDOF_<D>` | the **derived** arm's forward kinematics. No Jacobian: nothing wants one for it |
| `JOINT_NAMES[]`, `AUX_NAMES[]`, `IK_NJOINTS`, `IK_NBRANCHES` | the row order and sizes of the derived arm's closed form |

**Phase IIa takes a seed, not an index.** The branches are different *postures* — elbow up or down,
wrist flipped — and which you want depends on obstacles, joint limits and where the arm is now.
None of that is known here, and damped least squares should stay in the basin of the seed it is
given, so the choice of seed *is* the choice of posture. Folding the two calls into one would pick
a posture on your behalf from information IKBT does not have.

**There is no plain `ikin()` on this path, and that is the point.** There is no closed form for the
true robot — that is why the path was taken — so a function claiming to be one would be a
simplified arm's equations under the real robot's name.

---

## 5. Things worth knowing before you debug something

**An out-of-domain arccosine is normal, even on a reachable pose.** "Reachable" means at least one
joint vector exists, not that every enumerated branch exists. IKBT does not discard spurious
branches; a posture can fail to exist at a pose the arm reaches; and on the hybrid path the closed
form belongs to a *displaced* arm that genuinely cannot reach everything the true one can (measured
39 mm apart on one Panda pose). It is data, not a fault.

**Every domain-restricted function returns NaN.** `acos`, `asin` and `sqrt` all say the
same thing when their argument leaves the domain: *this branch has no solution at this pose*. C++
returns NaN for all three, so there is no hoisted `if (fabs(arg) > 1)` anywhere — a check at the
point of use cannot be written down wrong, where a hoisted one has to re-derive the argument. (The
python twin rewrites them to `acos_dc`/`asin_dc`/`sqrt_dc`, which raise; `scripts/cpp_expr_check`
asserts the two languages agree case by case.)

**Every solution variable is initialised to NaN** — one deliberate divergence from python, where an
unbound local is an exception. An out-of-domain arccosine produces the same value, so both arrive
at the finiteness test by one route.

**Division by zero is the one asymmetry left.** `1.0/0.0` is `inf` in C++ and raises in python, and
it is an *operator*, so the NaN treatment does not reach it. `solve()` already handles it
(`branches_at()` lets IEEE produce the inf and `errors()` turns it into `INF`). What is exposed is a
bare `ikin()`. Not yet seen to bite — but if a posture comes back with an `inf` rather than a NaN,
this is why.

**`#ifdef IKBT_MAIN` is the C++ spelling of `if __name__ == "__main__":`.** Every robot file has one
self-test behind it, and exactly one. Build it with

```
g++ -std=c++11 -O2 -DIKBT_MAIN CodeGen/Cpp/PumaCppCode/Puma.cpp -o ik_test -lm && ./ik_test
```

`main()` has to be at global scope, so it sits outside both namespaces and reaches back in with
`using namespace ikbt::<Robot>;`. The `-DIKBT_MAIN` is why a robot file can be linked into your
program without a stray `main()`.

**`IKBT_DRIVER` and `IKBT_CHECK_DRIVER` are not yours.** They are appended by
`scripts/cpp_closed_loop_check.py` when it stages a generated file; nothing in a generated file
defines or tests them.

**Three robots solve "completely" and their generated code is still wrong** — Arm_3, JennyGuoSp24,
UR5. One defect, and it is the solver's, not the generator's: the solution for a variable contains
that variable. The C++ compiles, reads the self-reference as NaN and returns **no** branches, which
is an empty answer wearing "unreachable" as a disguise. If you are working with one of these, that
is the reason.

---

## 6. Build and directory rules

```
g++ -std=c++11 -O2 <file>.cpp -o <exe> -lm
```

**C++11, standard library only.** No Eigen, no Boost, no build system, no `-I`. The only linear
algebra needed is a 6x6 solve, which is 40 lines in `ikbt_linalg.h`. The rule is the same one that
says a generated python module stands on numpy alone: a generated artifact must not make you install
anything.

**Nothing needs `-I` because `#include "..."` resolves against the including file's own directory.**
`Cpp_src/` is reached by a path relative to the generated file (`../../../Cpp_src/...`, computed with
`os.path.relpath`, not spelled out), and the headers' own `#include "ikbt_*.h"` lines find each other
as siblings inside `Cpp_src/`.

**A robot's directory must keep its position relative to `Cpp_src/`.** To build it somewhere else,
carry `Cpp_src/` along at the same depth.

### Two robots in one program

This works:

```cpp
#include "CodeGen/Cpp/PumaCppCode/Puma.cpp"
#include "CodeGen/Cpp/WristCppCode/Wrist.cpp"

ikbt::Puma::ikin(T1);
ikbt::Wrist::ikin(T2);
```

**The namespace is what makes it work.** Every name a robot file defines is inside
`ikbt::<Robot>`, so two arms of equal DOF cannot collide at link time.

**`CodeGen/` is gitignored.** A working copy can be arbitrarily far behind its own source. If a
number looks wrong, regenerate before you debug.

---

## 7. Where each of these comes from

Every python emitter has one C++ twin, derived from it. The python generator is the specification;
the checks turn "derived from" into an assertion.

**The twins are functions, not files.** Python writes a module per artifact because a module is the
unit of import; C++ writes one translation unit per robot because that is what you compile. So each
C++ emitter below produces a *section* of `<Robot>.cpp`, and `output_cpp_robot.py` assembles the
file around them.

| python | C++ | emits |
|---|---|---|
| `output_python.output_python_code()` | `output_cpp.output_cpp_code()` | `ikin()` / `ikin_given()` |
| `output_numeric_common.write_fk_module()` | `output_cpp_common.fk_body_cpp()` | `fk()` / `jacobian()` |
| `output_hybrid_python.write_hybrid_top()` | `output_cpp_hybrid.write_hybrid_top_cpp()` | `ikin_approx()` / `refine_seed()` |
| `output_onevar_python.write_onevar_top()` | `output_cpp_onevar.write_onevar_top_cpp()` | `solve()` |
| — | `output_cpp_robot.py` | the file around them |
| `output_numeric_common.expr_py()` (`sp.pycode`) | `output_cpp_common.expr_cpp()` (`sp.cxxcode`) | one expression |
| `numeric_ik.solve_numeric()` / `DLS_CORE` | `Cpp_src/ikbt_dls.h` | — (source, not generated) |
| `output_numeric_common.POSE_ERROR_CORE` | `Cpp_src/ikbt_pose_error.h` | — |
| `output_onevar_python.SEARCH_CORE` | `Cpp_src/ikbt_search.h` | — |

`output_cpp_common.write_fk_module_cpp()` still writes a standalone `FK_numeric<R>.h`, for the three
callers that want FK on its own: `fkOnly.py`, and the `--fk` and `--dls` probes below. Its names stay
suffixed (`fk_<R>`), because a bare header has no robot namespace to sit in.

The last three rows are copies, and a copy is a liability, so they are pinned:

```bash
python3 -m scripts.cpp_expr_check                  # do the two expression printers agree?
python3 -m scripts.cpp_closed_loop_check --keep    # is the generated C++ correct, and does it match python?
python3 -m scripts.cpp_closed_loop_check --all --compile-only
python3 -m scripts.cpp_closed_loop_check --fk Puma        # FK/Jacobian, elementwise vs python
python3 -m scripts.cpp_closed_loop_check --dls Puma       # ikbt_dls.h vs numeric_ik, iteration for iteration
python3 -m scripts.cpp_closed_loop_check --onevar C-Arm   # same root set as the python search
python3 -m scripts.cpp_closed_loop_check --hybrid Panda   # same refined postures
```

`detect_path()` in that script reads `SOLUTION_PATH` out of the robot file, which is how one command
covers all three paths.

See `IKdocs/TESTING.md` for what each one asserts and where its log lands, and
`IKdocs/DEV_NOTES.md` for the measurements behind the decisions above.
