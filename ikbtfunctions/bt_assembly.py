#!/usr/bin/python
#
#   bt_assembly.py --  build the IKBT behavior tree
#
#   The tree can be built by anything that needs one:  the ikSolver.py CLI,
#   tests, batch runners, or a front end that swaps in its own solver set.
#
#       build_worktools()       -- the solver Priority.  THIS is the extension
#                                  point:  a new solution strategy is added here.
#       build_symbolic_branch() -- one complete symbolic solver
#       build_default_bt()      -- the whole tree
#       find()                  -- reach a node by Name, to set BHdebug
#
#   EVERY LEAF IS CONSTRUCTED WHERE IT IS WIRED.  There is no inventory
#   function and no dict of nodes:  a leaf is built in the builder that puts it
#   into the tree, next to the comment explaining why it is there.  A dict of
#   every leaf used to sit in front of this, and it cost more than it paid --
#   three production callers wanted one node each, while the dict needed
#   per-branch key suffixes and a reachability filter to keep it honest.
#
#   WHY THE TREE HOLDS THREE SEPARATE SOLVER INSTANCES (asked 2026-09-27).
#   b3 keys per-node state on the blackboard by node id, and ids are per
#   INSTANCE (b3/core/basenode.py).  One instance in two tree POSITIONS shares
#   one id, so its `is_open` flag and any RepeatUntilSuccess loop counter
#   collide;  bt_problems() rejects it.  A node repeated by a LOOP is not the
#   same thing and does not collide -- which is why the one-variable branch
#   retries with a single solver instance.
#
#   Clearing the blackboard between the branches instead would work in
#   principle and is not worth it:  the branches tick inside ONE Priority.tick(),
#   so at that moment every ancestor is `is_open` in the very dict you would be
#   wiping, and RepeatUntilSuccess._open() resets its counter to 0 -- silently
#   restarting the enclosing onevar loop's budget.  It would have to be a
#   selective per-subtree clear, which is more machinery than three cheap
#   instances.  (clear_state touches bb._base_memory only;  node state lives in
#   bb._tree_memory, which it never opens.)
#
#   Copyright 2017-2026 University of Washington
#
#   Developed by Dianmu Zhang and Blake Hannaford
#   BioRobotics Lab, University of Washington

import b3 as b3          # behavior trees

#  Solver leaves
from ikbtleaves.assigner_leaf   import assigner
from ikbtleaves.rank_leaf       import rank
from ikbtleaves.algebra_solver  import algebra_id, algebra_solve
from ikbtleaves.tan_solver      import tan_id, tan_solve
from ikbtleaves.sincos_solver   import sincos_id, sincos_solve
from ikbtleaves.sinANDcos_solver import sinandcos_id, sinandcos_solve
from ikbtleaves.two_eqn_m7      import simu_id, simu_solver

#  Transform / bookkeeping leaves
from ikbtleaves.invariant_gen   import invariant_gen
from ikbtleaves.parallel_triple import parallel_triple_transform
from ikbtleaves.sub_transform   import sub_transform
from ikbtleaves.sum_id          import sum_id      # detect and sub sum-of-angles
from ikbtleaves.updateL         import updateL
from ikbtleaves.comp_detect     import comp_det

#  Top-of-tree leaves.  symbolic_loop is the outer solve loop (it replaced
#  b3.RepeatUntilSuccess -- see below) and report_gen is codegen-as-a-leaf.
from ikbtleaves.symbolic_loop   import symbolic_loop
from ikbtleaves.output_gen      import report_gen
from ikbtleaves.hybrid_ik       import (pieper_geom_report,
                                        simplified_arm, install_simplified)
from ikbtleaves.onevar_ik       import onevar_rank, install_known
from ikbtleaves.clear_state     import clear_state


def find(node, name):
    '''The first node at or below `node` whose Name is `name`, or None.

       This is how a caller reaches a leaf to configure it:

           bt = build_default_bt()
           find(bt.root, 'Tangent ID').BHdebug = True

       Names are unique across the tree -- each branch builds its leaves with
       its own tag -- and bt_problems() enforces it, so "the first" is "the
       only".  Pass a branch rather than the root to reach a tagged copy.'''

    if node is None:
        return None
    stack = [node.root if isinstance(node, b3.BehaviorTree) else node]
    seen = []
    while stack:
        n = stack.pop(0)
        if any(x is n for x in seen):
            continue
        seen.append(n)
        if getattr(n, 'Name', None) == name:
            return n
        kids = list(getattr(n, 'children', None) or [])
        child = getattr(n, 'child', None)
        if child is not None and not isinstance(child, list):
            kids.append(child)
        stack.extend(k for k in kids if isinstance(k, b3.BaseNode))
    return None


def build_worktools(tag='', leaf_debug=False, solver_debug=False):
    '''Assemble the solver Priority, building its leaves as it wires them.

       This is the extension point for new solution strategies.  ORDER MATTERS:
       b3.Priority returns on its first non-FAILURE child, so ordering decides
       which strategy owns a variable, and a strategy placed last can only fire
       where everything before it has failed.  The list is

           Priority[algSol, Simu_Eqn_Sol, sc_tan, sacSol,
                    parallelTriple, invariantGen]

       Simu_Eqn_Sol sits ahead of sc_tan (BH, 2026-09-03) because sc_tan's
       arcsin branch was winning variables that simu_solver can pin exactly.
       arcsin returns TWO solutions, th and pi - th, from an equation holding
       only sin(u);  both satisfy it, but the rest of the FK still constrains
       cos(u), so one is wrong at every pose.  simu_solver reads the PAIR

           e1:  0 = A*sin(u) + B*cos(u) - C
           e2:  0 = A*cos(u) - B*sin(u) - D

       which fixes sin(u) and cos(u) separately and yields ONE atan2.

       invariantGen is last, which keeps it behaviour-preserving for robots that
       already solve.  Promoting it produces better solutions but costs much
       more time;  see IKdocs/DEV_NOTES.md.

       Craig417 (2/4), Parkman13 (0/4) and UR5 (0/8) are still open.

       `tag` is appended to every Name, because the tree carries this whole
       assembly three times over three separate instance sets and two nodes
       logging under one label make the tick log unreadable.'''

    ###  tangent solver
    tanID = tan_id()
    tanID.Name = 'Tangent ID' + tag
    tanID.BHdebug = leaf_debug

    tanSolver = tan_solve()
    tanSolver.Name = 'Tangent Solver' + tag
    tanSolver.BHdebug = solver_debug

    tanSol = b3.Sequence([tanID, tanSolver])
    tanSol.Name = 'TanID+Solv' + tag
    tanSol.BHdebug = leaf_debug

    ###  algebra solver
    algID = algebra_id()
    algID.Name = 'Algebra ID' + tag
    algID.BHdebug = leaf_debug

    algSolver = algebra_solve()
    algSolver.Name = 'Algebra Solver' + tag
    algSolver.BHdebug = False

    algSol = b3.Sequence([algID, algSolver])
    algSol.Name = 'Algebra ID and Solve' + tag
    algSol.BHdebug = solver_debug

    ###  sin(th) OR cos(th)
    scID = sincos_id()
    scID.Name = 'Sin Cos ID' + tag
    scID.BHdebug = solver_debug

    scSolver = sincos_solve()
    scSolver.Name = 'Sine Cosine Solver' + tag
    scSolver.BHdebug = leaf_debug

    scSol = b3.Sequence([scID, scSolver])
    scSol.Name = 'SinCos ID+Solve' + tag
    scSol.BHdebug = solver_debug

    ###  sin(th) AND cos(th) in the same eqn
    #  NOTE the Names here must differ from the sin-OR-cos leaves above:  Name is
    #  what shows up in the BT tick log, and two leaves sharing one Name makes
    #  that log unreadable (they were both "Sin Cos ID" until Aug 2026).
    sacID = sinandcos_id()
    sacID.Name = 'Sin AND Cos ID' + tag
    sacID.BHdebug = False

    sacSolver = sinandcos_solve()
    sacSolver.Name = 'Sin AND Cos Solver' + tag
    sacSolver.BHdebug = False

    sacSol = b3.Sequence([sacID, sacSolver])
    sacSol.Name = 'Sin AND Cos ID+Solve' + tag
    sacSol.BHdebug = solver_debug

    ###  two equations, one unknown
    SimuEqnID = simu_id()
    SimuEqnID.Name = 'Simultaneous Eqn ID' + tag
    SimuEqnID.BHdebug = False

    SimuEqnSolve = simu_solver()
    SimuEqnSolve.Name = 'Simultaneous Eqn solver' + tag

    Simu_Eqn_Sol = b3.Sequence([SimuEqnID, SimuEqnSolve])
    Simu_Eqn_Sol.Name = 'Simultaneous Eqn ID+Solve' + tag

    ###  assigner and rank
    #  rank is a deliberate workaround, not a solver:  when more than one leaf
    #  can solve the current unknown, it picks the nicer solution (e.g.
    #  atan2(y,x) over asin()).  That choice did not fit the BT framework cleanly.
    rankNode = rank()
    rankNode.Name = 'Rank Node' + tag

    ###  tan and sin/cos compete, then rank picks the nicer solution.
    #  b3.OrNode (unlike b3.Priority) runs ALL its children -- that is deliberate
    #  and load-bearing:  rank needs both candidate solutions to choose between.
    sc_tan = b3.Sequence([b3.OrNode([tanSol, scSol]), rankNode])
    sc_tan.Name = 'Tan/SinCos + Rank' + tag

    ###  x^2 + y^2 (Craig eqn 4.65) is NO LONGER IN THE TREE.  invariant_gen
    #  subsumes it:  the x2y2 trick is the ||P||^2 invariant for one particular
    #  pair of position equations, and invariant_gen emits that plus trace(R)
    #  and the column invariants over every pair.  x2z2_transform and its
    #  TestSolver010 remain in ikbtleaves/x2y2_transform.py, still unit-tested
    #  and re-wirable in one line here.  See IKdocs/DEV_NOTES.md.

    ###  Three consecutive PARALLEL axes -- the other half of Pieper's
    #  condition, and the half no solver leaf was written for.  It declines
    #  unless dh_analysis reports a `parallel` triple, which is pure DH
    #  arithmetic, so an arm without one pays only a table scan.  It restocks
    #  eqns_1u with the law-of-cosines equation for the middle joint of the
    #  triple;  the existing arccos/atan2/algebra leaves finish the job.
    #  See IKdocs/parallel_triple_refs.md.
    parallelTriple = parallel_triple_transform()
    parallelTriple.Name = 'Parallel Triple Transform' + tag
    parallelTriple.BHdebug = leaf_debug

    ###  kinematic invariant generator.  Generalizes the x2y2 trick:  emits
    #  ||P||^2 / trace(R) / P.col invariants, which typically carry fewer
    #  unknowns than any raw element equation.  To switch it off:
    #
    #       bt = build_default_bt()
    #       find(bt.root, 'Invariant Generator').enabled = False
    #
    invariantGen = invariant_gen()
    invariantGen.Name = 'Invariant Generator' + tag
    invariantGen.BHdebug = False
    #  ON.  The tree's only equation-restocking transform now that x2z2 is
    #  gone, and the leaves ahead of it in the Priority fall through only when
    #  they have all failed -- never, on a robot that solves cleanly.
    invariantGen.enabled = True

    worktools = b3.Priority([algSol, Simu_Eqn_Sol, sc_tan, sacSol,
                             parallelTriple, invariantGen])
    worktools.Name = 'Work Tools' + tag
    return worktools


def build_symbolic_branch(tag='', leaf_debug=False, solver_debug=False,
                          quiet=False, worktools=None):
    '''Assemble one complete symbolic solver, building its leaves as it wires them.

           Sequence[ clear_state, symbolic_loop(x20, solveRoutine) ]

           solveRoutine = Sequence[ sub_transform,
                                    RepeatUntilSuccess(x6, Sequence[assigner,
                                                       sum_id, worktools]),
                                    updateL,
                                    comp_det ]

       Called THREE TIMES by build_default_bt(), each with its own `tag`, so
       each branch gets its own instances (see the header) and its own labels in
       the tick log.

       `worktools` overrides the solver Priority, for callers experimenting with
       a different strategy order;  by default this builds its own.

       `quiet` is for batch sweeps:  no read_pause and no per-pass progress
       lines, set here because this is where both nodes are built.'''

    if worktools is None:
        worktools = build_worktools(tag=tag, leaf_debug=leaf_debug,
                                    solver_debug=solver_debug)

    asgn = assigner()
    asgn.Name = 'Assigner' + tag

    #  The SOA cases must be ID'd every pass so that algSol has equations to
    #  work on for the sum-of-angles variables.
    sumOfAnglesID = sum_id()
    sumOfAnglesID.Name = 'Sum of Angles ID' + tag
    sumOfAnglesID.BHdebug = False

    subtree = b3.RepeatUntilSuccess(
        b3.Sequence([asgn, sumOfAnglesID, worktools]), 6)
    subtree.Name = 'Solve Subtree' + tag

    ###  Equation transforms
    sub_trans = sub_transform()
    sub_trans.Name = 'Substitution Transform' + tag
    sub_trans.BHdebug = leaf_debug

    #  NOT named 'updateL':  that would rebind the imported class.
    updateLNode = updateL()
    updateLNode.Name = 'updateL Transform' + tag
    updateLNode.BHdebug = False

    compDetect = comp_det()
    compDetect.Name = 'Completion Detect' + tag
    compDetect.BHdebug = True
    if quiet:
        #  comp_det's read_pause exists so a human can read status scrolling
        #  past.  No human is reading a sweep.
        compDetect.read_pause = 0

    #  b3.Sequence aborts on its first FAILURE, so a failing sub_transform or
    #  solve subtree would stop updateL and comp_det from running at all -- and
    #  on a robot that solves nothing, that is every pass, leaving the tree with
    #  no termination logic in exactly the case that needs it.
    #
    #  Priority([x, Succeeder()]) hides x's failure, so the pass always
    #  reaches updateL and the completion detector.  Both are safe on a pass
    #  that achieved nothing:  updateL re-scans, comp_det only decides whether
    #  to stop.
    tryTransform = b3.Priority([sub_trans, b3.Succeeder()])
    tryTransform.Name = 'Sub Transform (optional)' + tag

    trySolve = b3.Priority([subtree, b3.Succeeder()])
    trySolve.Name = 'Solve Subtree (optional)' + tag

    solveRoutine = b3.Sequence([tryTransform, trySolve,
                                updateLNode, compDetect])
    solveRoutine.Name = 'Solve Routine' + tag

    #  The outer loop and its budget.  symbolic_loop reports what happened --
    #  SUCCESS if anything was solved, FAILURE if nothing was -- and that
    #  FAILURE is what admits the next branch.  See IKdocs/DEV_NOTES.md for
    #  why it is not b3.RepeatUntilSuccess.
    #
    #  Budget 20 (raised from 10, BH 2026-08-28):  the solvers now refuse
    #  equations that constrain nothing and fall through to invariant_gen, so a
    #  pass that used to end in a bogus solve ends in a restock, and real
    #  solves take more passes.
    symLoop = symbolic_loop(solveRoutine, 20)
    symLoop.Name = 'Symbolic Solver Loop' + tag
    symLoop.BHdebug = solver_debug
    if quiet:
        symLoop.progress = False

    ###  State hygiene.  Heads every solver:  drops the previous solve's
    #  leftover blackboard state (comp_det's verdict, the assigner's cursor)
    #  while keeping the problem and the findings about the true robot.  On the
    #  first solve there is nothing to drop;  on a retry it is what makes
    #  "wipe and retry" true.
    clearState = clear_state()
    clearState.Name = 'Clear Solver State' + tag
    clearState.BHdebug = leaf_debug

    branch = b3.Sequence([clearState, symLoop])
    branch.Name = 'Symbolic Branch' + tag
    return branch


def build_default_bt(leaf_debug=False, solver_debug=False, codegen=False,
                     quiet=False):
    '''Build the standard IKBT tree.  Returns the BehaviorTree.

           Sequence[ analysis, report_gen ]

           analysis        = Priority[ symbolic_branch, onevar_branch,
                                       hybrid_branch ]

           symbolic_branch = Sequence[ clear_state,
                                       symbolic_loop(x20, solveRoutine) ]

           onevar_branch   = Sequence[ onevar_rank,
                                       RepeatUntilSuccess(
                                           Sequence[ install_known,
                                                     symbolic_branch (onevar) ]) ]

           hybrid_branch   = Sequence[ pieper_geom_report,   # always SUCCESS
                                       simplified_arm,
                                       install_simplified,
                                       symbolic_branch (hybrid) ]

       The solver reports whether it got anywhere, so a SECOND strategy can be
       tried when it did not, and each branch can emit its own artifacts.

       codegen=False (the default) makes report_gen do nothing, so building a tree
       writes no files and run_solver() still owns create_solution_set().
       codegen=True hands the whole tail end to the tree, and the caller must
       then pass run_solver(..., create_solutions=False):  create_solution_set()
       appends, so running it twice is not the same as running it once.

       quiet=True is for batch sweeps:  no read_pause, no per-pass progress.

       To configure a leaf afterwards, reach it by Name:

           bt = build_default_bt()
           find(bt.root, 'Tangent ID').BHdebug = True
           find(bt.root, 'Completion Detect').FailAllDone = True'''

    symbolicBranch = build_symbolic_branch(leaf_debug=leaf_debug,
                                           solver_debug=solver_debug,
                                           quiet=quiet)

    #  The hybrid branch is admitted by the symbolic solver having FAILED --
    #  which is what the b3.Priority below does -- plus simplified_arm finding a
    #  usable candidate.  It is NOT conditioned on Pieper's condition:  the
    #  condition is sufficient for a closed form and not known to be necessary,
    #  so neither its presence nor its absence predicts whether IKBT can crack
    #  a given arm.  See IKdocs/DEV_NOTES.md.
    #
    #  pieper_geom_report heads the branch for WHAT IT STORES, not for what it
    #  returns -- pieper_triples, and the pieper_latex snapshot of the TRUE
    #  robot, which must be taken before install_simplified swaps the Robot.
    #  It always SUCCEEDs, so no Priority([x, Succeeder()]) wrapper is needed.
    #
    #  The branch ends with the solver, not a placeholder:  report_gen reads
    #  hybrid_source and writes a HYBRID report plus a two-phase python module,
    #  every artifact naming the arm it actually describes.
    pieperGeomReport = pieper_geom_report()
    pieperGeomReport.BHdebug = leaf_debug

    ###  Rank the DH changes that would give the arm a Pieper triple.
    #  THIS LEAF DECIDES whether the hybrid branch proceeds, since
    #  pieper_geom_report ahead of it cannot FAIL.  It refuses when
    #  pieper_triples is None (an unparseable DH table) and on finding no usable
    #  candidate -- the latter closes the branch on an arm that already
    #  satisfies Pieper everywhere, because candidate_simplifications() skips
    #  qualifying triples.
    simplifiedArm = simplified_arm()
    simplifiedArm.BHdebug = leaf_debug

    ###  Build the derived robot and install it, so the solver that follows
    #  solves the SIMPLIFIED arm.  Fresh unknown objects and its own pickle name.
    installSimplified = install_simplified()
    installSimplified.BHdebug = leaf_debug

    hybridBranch = b3.Sequence([pieperGeomReport, simplifiedArm,
                                installSimplified,
                                build_symbolic_branch(tag=' (hybrid)',
                                                      leaf_debug=leaf_debug,
                                                      solver_debug=solver_debug,
                                                      quiet=quiet)])
    hybridBranch.Name = 'Hybrid Branch'

    ###  Rank the unknowns by what declaring each one KNOWN would restock.
    #  THIS LEAF DECIDES whether the one-variable branch proceeds:  it refuses
    #  when no unknown, made known, restocks a single one-unknown equation, and
    #  the measurement costs one equation scan per unknown -- no sympy solving.
    onevarRank = onevar_rank()
    onevarRank.BHdebug = leaf_debug

    ###  Declare the next ranked candidate known and install the reduced
    #  problem, so the solver that follows solves one unknown fewer.  Fresh
    #  unknowns and a fresh Robot per attempt;  the DH table is untouched.
    installKnown = install_known()
    installKnown.BHdebug = leaf_debug

    #  ONE ATTEMPT PER LOOP.  install_known hands the solver the next candidate
    #  and FAILs when the ranked list is exhausted, so the Sequence fails, the
    #  loop tries again with the cursor advanced, and the branch closes when
    #  there is nothing left.  RepeatUntilSuccess loops INSIDE one tick, so the
    #  whole sweep happens in a single pass and stops at the first candidate
    #  that solves.
    #
    #  ONE instance of the solver serves every attempt:  a node repeated by a
    #  loop is not a node in two tree positions, and only the latter collides in
    #  b3's per-node blackboard state.  What does NOT reset itself is the
    #  application state, which is why each attempt reloads the robot
    #  (install_known) and each solver starts with clear_state.
    onevarAttempt = b3.Sequence([installKnown,
                                 build_symbolic_branch(tag=' (onevar)',
                                                       leaf_debug=leaf_debug,
                                                       solver_debug=solver_debug,
                                                       quiet=quiet)])
    onevarAttempt.Name = 'One Variable Attempt'

    #  The bound is onevar_rank.max_candidates + 1 -- the extra iteration is the
    #  one that discovers the list is empty.  It is also a REQUIRED safety net:
    #  an unbounded RepeatUntilSuccess would spin forever once install_known
    #  starts failing.  Raising max_candidates afterwards therefore needs the
    #  tree rebuilt.
    onevarLoop = b3.RepeatUntilSuccess(onevarAttempt,
                                       onevarRank.max_candidates + 1)
    onevarLoop.Name = 'One Variable Attempt Loop'

    onevarBranch = b3.Sequence([onevarRank, onevarLoop])
    onevarBranch.Name = 'One Variable Branch'

    #  b3.Priority (the standard Selector/Fallback) stops at its first
    #  non-FAILURE child, so each branch ticks ONLY when everything before it
    #  came up empty.  Any of the three can SUCCEED, and report_gen -- the
    #  shared node after this Priority -- reads onevar_source / hybrid_source to
    #  tell which.
    #
    #  ONE VARIABLE BEFORE HYBRID (BH, 2026-09-21), so a robot it can crack
    #  never reaches the hybrid method.  It solves the TRUE arm:  the DH table
    #  is untouched and the answer is exact wherever the 1-D search finds a
    #  root, where the hybrid method answers about a DERIVED arm and has to
    #  correct its way back numerically.  When both would work, the one that
    #  never approximated the robot is the one to keep.
    analysis = b3.Priority([symbolicBranch, onevarBranch, hybridBranch])
    analysis.Name = 'Analysis'

    ###  Report and code generation.  ONE generator, at the end of the tree,
    #  ticked after whichever branch produced the solution -- the report is a
    #  property of the finished solve, not of the branch that produced it.
    #
    #  DEFAULT OFF.  Enabled, this leaf owns create_solution_set() and writes
    #  LaTex/ and CodeGen/;  disabled it does nothing, and
    #  ik_driver.run_solver() owns the solution set instead.  EXACTLY ONE of the
    #  two must call create_solution_set():  it appends to
    #  unknown.LHSversionNames, so a second call would double it.
    #
    #  Off by default so that a test building a tree does not overwrite the
    #  repo's generated artifacts.  ikSolver.py opts in.
    reportGen = report_gen()
    reportGen.BHdebug = False
    reportGen.enabled = bool(codegen)

    topnode = b3.Sequence([analysis, reportGen])
    topnode.Name = 'Top Node'

    ikbt = b3.BehaviorTree()
    ikbt.root = topnode

    return ikbt
