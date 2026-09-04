#!/usr/bin/python
#
#   bt_assembly.py --  build the IKBT behavior tree
#
#   Extracted from ikSolver.py (which was a 466-line module-level script) so that
#   the tree can be built by anything that needs one:  the ikSolver.py CLI, unit
#   and integration tests, batch runners, and any future front end that wants to
#   swap in its own solver set.
#
#   The three entry points are:
#
#       make_leaves()       -- construct every leaf/composite node, return them in
#                              a dict so a caller can reach any node to set BHdebug
#       build_worktools()   -- assemble the solver Priority.  THIS is the extension
#                              point:  a new solution strategy is added here.
#       build_default_bt()  -- the whole tree, plus the node dict
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
from ikbtleaves.clear_state     import clear_state

#  (updateL and comp_det used to arrive in ikSolver.py by accident, via
#   `from ikbtleaves.tan_solver import *`.  Imported explicitly here.)


def make_leaves(leaf_debug=False, solver_debug=False):
    '''Construct every node the default tree needs and return them in a dict.

       Returning the dict (rather than just the tree) is what makes per-robot
       debugging tractable:

           bt, nodes = build_default_bt()
           nodes['tanID'].BHdebug = True

       which replaces the ~200 lines of commented-out debug blocks that used to
       sit at the bottom of ikSolver.py.'''

    n = {}

    ###  assigner and rank
    #  These two are a deliberate workaround, not a solver:  when more than one
    #  leaf can solve the current unknown, they pick the nicer solution (e.g.
    #  atan2(y,x) over asin).  That choice did not fit the BT framework cleanly.
    asgn = assigner()
    asgn.Name = "Assigner"
    n['asgn'] = asgn

    rankNode = rank()
    rankNode.Name = "Rank Node"
    n['rankNode'] = rankNode

    ###  tangent solver
    tanID = tan_id()
    tanID.Name = 'Tangent ID'
    tanID.BHdebug = leaf_debug
    n['tanID'] = tanID

    tanSolver = tan_solve()
    tanSolver.Name = "Tangent Solver"
    tanSolver.BHdebug = solver_debug
    n['tanSolver'] = tanSolver

    tanSol = b3.Sequence([tanID, tanSolver])
    tanSol.Name = "TanID+Solv"
    tanSol.BHdebug = leaf_debug
    n['tanSol'] = tanSol

    ###  algebra solver
    algID = algebra_id()
    algID.Name = "Algebra ID"
    algID.BHdebug = leaf_debug
    n['algID'] = algID

    algSolver = algebra_solve()
    algSolver.Name = "Algebra Solver"
    algSolver.BHdebug = False
    n['algSolver'] = algSolver

    algSol = b3.Sequence([algID, algSolver])
    algSol.Name = "Algebra ID and Solve"
    algSol.BHdebug = solver_debug
    n['algSol'] = algSol

    ###  sin(th) OR cos(th)
    scID = sincos_id()
    scID.Name = "Sin Cos ID"
    scID.BHdebug = solver_debug
    n['scID'] = scID

    scSolver = sincos_solve()
    scSolver.Name = "Sine Cosine Solver"
    scSolver.BHdebug = leaf_debug
    n['scSolver'] = scSolver

    scSol = b3.Sequence([scID, scSolver])
    scSol.Name = "SinCos ID+Solve"
    scSol.BHdebug = solver_debug
    n['scSol'] = scSol

    ###  sin(th) AND cos(th) in the same eqn
    #  NOTE the Names here must differ from the sin-OR-cos leaves above:  Name is
    #  what shows up in the BT tick log, and two leaves sharing one Name makes
    #  that log unreadable (they were both "Sin Cos ID" until Aug 2026).
    sacID = sinandcos_id()
    sacID.Name = "Sin AND Cos ID"
    sacID.BHdebug = False
    n['sacID'] = sacID

    sacSolver = sinandcos_solve()
    sacSolver.Name = "Sin AND Cos Solver"
    sacSolver.BHdebug = False
    n['sacSolver'] = sacSolver

    sacSol = b3.Sequence([sacID, sacSolver])
    sacSol.Name = "Sin AND Cos ID+Solve"
    sacSol.BHdebug = solver_debug
    n['sacSol'] = sacSol

    ###  x^2 + y^2 (Craig eqn 4.65) is NO LONGER IN THE TREE.  invariant_gen
    #  subsumes it:  the x2y2 trick is the ||P||^2 invariant for one particular
    #  pair of position equations, and invariant_gen emits that plus trace(R)
    #  and the column invariants over every pair.  Two reasons it went, both
    #  measured on UR5 (2026-08-28):
    #
    #    - it never fired.  Ticked 15 times over a 9-pass solve, FAILURE every
    #      time.  At the decisive tick its pair search tested 91 pairs and
    #      accepted none:  70 died because l1^2 + l2^2 still held unknowns, and
    #      of the 21 that survived, every one left TWO unknowns on the other
    #      side where the acceptance test demands exactly one.
    #    - it re-derived that same dead end from scratch on every tick, with no
    #      memo, at ~20 minutes a pass once the leaves ahead of it stopped
    #      short-circuiting it.
    #
    #  x2z2_transform and its TestSolver010 stay in ikbtleaves/x2y2_transform.py
    #  -- the class is still unit-tested and can be re-wired in one line here.

    ###  Three consecutive PARALLEL axes -- the other half of Pieper's
    #  condition, and the half no solver leaf was written for.  Gated on
    #  dh_analysis's `parallel` triples (pure DH arithmetic, milliseconds), so
    #  an arm without one pays a table scan and the leaf declines.  It restocks
    #  eqns_1u with the law-of-cosines equation for the middle joint of the
    #  triple;  the existing arccos/atan2/algebra leaves finish the job.
    #  See IKdocs/parallel_triple_refs.md.
    parallelTriple = parallel_triple_transform()
    parallelTriple.BHdebug = leaf_debug
    n['parallelTriple'] = parallelTriple

    ###  kinematic invariant generator (NewStrategies.md, Candidate 3)
    #  Generalizes the x2y2 trick:  emits ||P||^2 / trace(R) / P.col invariants,
    #  which typically carry fewer unknowns than any raw element equation.
    #
    #  OFF BY DEFAULT.  It works, it is tested, and it does subsume x2y2 -- but
    #  measured, it has not yet solved a robot that could not be solved without
    #  it, and it costs ~5x wall clock on Puma and Kawasaki for byte-identical
    #  output.  Wired in as a documented extension point.  To experiment:
    #
    #       bt, nodes = build_default_bt()
    #       nodes['invariantGen'].enabled = True
    #
    invariantGen = invariant_gen()
    invariantGen.Name = 'Invariant Generator'
    invariantGen.BHdebug = False
    #  ON.  It is the tree's only equation-restocking transform now that
    #  x2z2 is gone, and the leaves ahead of it in the Priority only fall
    #  through to it when they have all failed -- which on a robot that solves
    #  cleanly is never, so it costs those robots nothing.
    invariantGen.enabled = True
    n['invariantGen'] = invariantGen

    ###  two equations, one unknown
    SimuEqnID = simu_id()
    SimuEqnID.Name = 'Simultaneous Eqn ID'
    SimuEqnID.BHdebug = False
    n['SimuEqnID'] = SimuEqnID

    SimuEqnSolve = simu_solver()
    SimuEqnSolve.Name = 'Simultaneous Eqn solver'
    n['SimuEqnSolve'] = SimuEqnSolve

    Simu_Eqn_Sol = b3.Sequence([SimuEqnID, SimuEqnSolve])
    Simu_Eqn_Sol.Name = 'Simultaneous Eqn ID+Solve'
    n['Simu_Eqn_Sol'] = Simu_Eqn_Sol

    ###  Equation transforms
    sub_trans = sub_transform()
    sub_trans.Name = "Substitution Transform"
    sub_trans.BHdebug = leaf_debug
    n['sub_trans'] = sub_trans

    #  Sum-of-angles solving is done by the algebra node, but the SOA terms
    #  still have to be identified so that algSol has equations to work on.
    sumOfAnglesID = sum_id()
    sumOfAnglesID.Name = "Sum of Angles ID"
    sumOfAnglesID.BHdebug = False
    n['sumOfAnglesID'] = sumOfAnglesID

    #  NOTE: the instance is deliberately NOT called 'updateL'.  ikSolver.py used
    #  to do `updateL = updateL()`, rebinding the imported class over its own name.
    updateLNode = updateL()
    updateLNode.Name = "updateL Transform"
    updateLNode.BHdebug = False
    n['updateLNode'] = updateLNode

    compDetect = comp_det()
    compDetect.Name = "Completion Detect"
    compDetect.BHdebug = True
    n['compDetect'] = compDetect

    ###  State hygiene.  Heads every solver:  drops the previous solve's
    #  leftover blackboard state (comp_det's verdict, the assigner's cursor)
    #  while keeping the problem and the findings about the true robot.  On the
    #  first solve there is nothing to drop;  on the hybrid branch's second
    #  solve it is what makes "wipe and retry" true.
    clearState = clear_state()
    clearState.BHdebug = leaf_debug
    n['clearState'] = clearState

    ###  Report and code generation.  ONE generator, at the end of the tree,
    #  ticked after whichever branch produced the solution -- the report is a
    #  property of the finished solve, not of the branch that produced it.
    #
    #  DEFAULT OFF.  Enabled, this leaf owns create_solution_set() and writes
    #  LaTex/ and CodeGen/;  disabled it does nothing whatsoever, and
    #  ik_driver.run_solver() keeps ownership of the solution set exactly as
    #  before.  Exactly one of the two must call create_solution_set() -- it
    #  appends to unknown.LHSversionNames and is not idempotent.
    #
    #  Off by default because every unit test that builds a tree would otherwise
    #  overwrite the repo's generated artifacts;  tests/test_chair_helper.py
    #  documents that it leaves them alone.  ikSolver.py opts in.
    reportGen = report_gen()
    reportGen.BHdebug = False
    n['reportGen'] = reportGen

    ###  Joint-axis geometry (Pieper's condition) -- REPORTING, NOT A GATE.
    #  It answers "does this arm have a triple of consecutive joint axes that
    #  intersects or is parallel", publishes pieper_triples / pieper_ok /
    #  pieper_latex, and ALWAYS returns SUCCESS.  That is what lets it sit bare
    #  in the hybrid Sequence:  having a triple does not mean IKBT can solve the
    #  arm, so it is the wrong thing to gate on, and a leaf that cannot FAIL
    #  cannot gate anything wherever it is put.  See the long note in
    #  build_default_bt().  The report's geometry statement is generated by
    #  report_gen, not here:  this leaf ticks only on the hybrid path, and the
    #  statement has to appear whichever branch wins.
    pieperGeomReport = pieper_geom_report()
    pieperGeomReport.BHdebug = leaf_debug
    n['pieperGeomReport'] = pieperGeomReport

    ###  Rank the DH changes that would give the arm a Pieper triple.
    #  THIS IS THE BRANCH'S GATE.  pieper_geom_report ahead of it cannot FAIL,
    #  so admission rests on this leaf's two refusals:  pieper_ok False (a DH
    #  table nobody could parse -- now the only thing between that and a derived
    #  robot describing nothing), and no usable candidate, which is what closes
    #  the branch on an arm that already satisfies Pieper everywhere, since
    #  candidate_simplifications() skips triples that already qualify.
    simplifiedArm = simplified_arm()
    simplifiedArm.BHdebug = leaf_debug
    n['simplifiedArm'] = simplifiedArm

    ###  Build the derived robot and install it, so the solver that follows
    #  solves the SIMPLIFIED arm.  Fresh unknown objects and its own pickle name.
    installSimplified = install_simplified()
    installSimplified.BHdebug = leaf_debug
    n['installSimplified'] = installSimplified

    ###  tan and sin/cos compete, then rank picks the nicer solution.
    #  b3.OrNode (unlike b3.Priority) runs ALL its children -- that is deliberate
    #  and load-bearing:  rank needs both candidate solutions to choose between.
    sc_tan = b3.Sequence([b3.OrNode([tanSol, scSol]), rankNode])
    sc_tan.Name = "Tan/SinCos + Rank"
    n['sc_tan'] = sc_tan

    return n


def build_worktools(nodes):
    '''Assemble the solver Priority.

       This is the extension point for new solution strategies.  Order matters:
       b3.Priority returns on its first non-FAILURE child, so a strategy placed
       last can only fire on a tick where everything before it has already
       failed -- which makes it behavior-preserving for robots that already
       solve.  x2z2_Solver is the precedent for a transform living in this list.

       invariantGen is last for exactly that reason:  measured at PYTHONHASHSEED=0,
       Wrist / Puma / Chair_Helper / KawasakiRS007L all produce byte-identical
       LaTeX with it in the tree.

       Promoting it (swap it ahead of x2z2_Solver -- a one-line change here)
       measurably produces BETTER solutions:  its ||P||^2 invariant subsumes the
       x2y2 trick, so Puma th_3 goes from "x2z2 transform and sinANDcos" to plain
       "sinANDcos" with an identical solution expression and one fewer dependency
       (th_3 no longer depends on th_1, so the solution set is smaller).

       It is NOT promoted yet because it costs ~4.8x wall clock on Puma, all of it
       sp.simplify() over full FK expressions.  See ImplementationThoughts.md for
       the numbers and the list of ways to bring that down.

       SIMU_EQN_SOL IS PROMOTED AHEAD OF sc_tan, and that deliberately breaks the
       "put new things last" rule above (BH, 2026-09-03).  The rule exists to keep
       an addition behaviour-preserving;  this is not an addition, it is a
       correction of which solver gets first refusal, and it MUST change results.

       WHY.  Priority takes the first non-FAILURE, so ordering decides which
       strategy owns a variable -- and sc_tan's arcsin branch was winning
       variables that simu_solver can pin exactly.  arcsin returns TWO solutions,
       th and pi - th, from an equation containing only sin(u);  both satisfy that
       equation, but the rest of the FK still constrains cos(u), so one of them is
       wrong at every pose.  simu_solver reads a PAIR

           e1:  0 = A*sin(u) + B*cos(u) - C
           e2:  0 = A*cos(u) - B*sin(u) - D

       which fixes sin(u) and cos(u) separately and yields ONE atan2.

       Measured on Chair_Helper, whose th_2 is exactly this case.  Its arcsin came
       from an equation sub_transform had manufactured -- substituting r_13 into
       the Px equation collapses the cos(th_2) terms away -- while L1 already held
       the canonical pair.  Over 10 random reachable poses the supplementary
       branch was valid 0 times out of 20:  half the advertised solution set was
       unusable, with nothing to say which half.

           Chair_Helper   2 of 4 versions correct  ->  2 of 2
           ICP5p5_A21     0 of 2                   ->  1 of 1

       and the seven KNOWN_COMPLETE robots (Puma, Pumaoffset, Stanford, Khat6DOF,
       Olson13, Brad, Wrist) stay at 100%.  Note the version COUNTS drop:  a
       spurious branch that no longer exists is a smaller solution set, so
       robot_baseline's n_solutions moves for these robots.  That is the fix
       working, not a regression.

       Craig417 (2/4), Parkman13 (0/4) and UR5 (0/8) are NOT fixed by this and
       are still open.'''

    return b3.Priority([nodes['algSol'],
                        nodes['Simu_Eqn_Sol'],
                        nodes['sc_tan'],
                        nodes['sacSol'],
                        nodes['parallelTriple'],
                        nodes['invariantGen']])


def _reachable(node, seen=None):
    '''Every b3 node at or below `node`, by identity.'''
    seen = [] if seen is None else seen
    if any(n is node for n in seen):
        return seen
    seen.append(node)
    kids = list(getattr(node, 'children', None) or [])
    child = getattr(node, 'child', None)
    if child is not None and not isinstance(child, list):
        kids.append(child)
    for k in kids:
        if isinstance(k, b3.BaseNode):
            _reachable(k, seen)
    return seen


def rename_leaves(nodes, suffix):
    '''Append `suffix` to every node Name in a leaf set.

       Needed because the tree carries the symbolic solver TWICE -- once on the
       symbolic branch and once on the hybrid branch, over a simplified arm.  The
       two are separate INSTANCES (b3 keys per-node state by node id, so sharing
       one instance across two tree positions really would collide), but they
       start out with identical Names, and two nodes logging under one label make
       the tick log unreadable.  bt_problems() flags it.'''

    for node in nodes.values():
        nm = getattr(node, 'Name', None)
        if isinstance(nm, str):
            node.Name = nm + suffix
    return nodes


def build_symbolic_branch(nodes, tag='', solver_debug=False):
    '''Assemble one complete symbolic solver from one leaf set.

           Sequence[ clear_state, symbolic_loop(x10, solveRoutine) ]

           solveRoutine = Sequence[ sub_transform,
                                    RepeatUntilSuccess(x6, Sequence[assigner,
                                                       sum_id, worktools]),
                                    updateL,
                                    comp_det ]

       Called TWICE by build_default_bt(), over two separate leaf sets:  the
       hybrid branch re-solves a simplified arm with its own solver rather than
       hiding a second solve inside a leaf, so the tree shows what happens.

       Composite Names carry `tag` for the same reason the leaves do.'''

    worktools = build_worktools(nodes)
    worktools.Name = "Work Tools" + tag
    nodes['worktools'] = worktools

    #  The SOA cases must be ID'd every pass so that algSol has equations to
    #  work on for the sum-of-angles variables.
    subtree = b3.RepeatUntilSuccess(
        b3.Sequence([nodes['asgn'], nodes['sumOfAnglesID'], worktools]), 6)
    subtree.Name = "Solve Subtree" + tag
    nodes['subtree'] = subtree

    #  b3.Sequence aborts on its first FAILURE, so if sub_transform or the solve
    #  subtree fails, updateL and comp_det never run at all.  On a robot that
    #  solves NOTHING that is every pass -- measured: comp_det ticks 8 times on
    #  Puma and 0 times on KawasakiRS05L -- so the tree had no termination logic
    #  in exactly the case that needs it, and ran head-first into the report
    #  generator with an empty solution set.
    #
    #  Priority([x, Succeeder()]) swallows x's failure, so the pass always
    #  reaches updateL and the completion detector.  Both are safe to run on a
    #  pass that achieved nothing:  updateL just re-scans, and comp_det only
    #  reports and decides whether to stop.
    tryTransform = b3.Priority([nodes['sub_trans'], b3.Succeeder()])
    tryTransform.Name = "Sub Transform (optional)" + tag
    nodes['tryTransform'] = tryTransform

    trySolve = b3.Priority([subtree, b3.Succeeder()])
    trySolve.Name = "Solve Subtree (optional)" + tag
    nodes['trySolve'] = trySolve

    solveRoutine = b3.Sequence([tryTransform, trySolve,
                                nodes['updateLNode'], nodes['compDetect']])
    solveRoutine.Name = "Solve Routine" + tag
    nodes['solveRoutine'] = solveRoutine

    #  The outer loop and its budget.  This was b3.RepeatUntilSuccess(x10),
    #  which returns FAILURE when it exhausts its loops -- and a FAILURE at the
    #  head of a Sequence aborts the Sequence, so a loop-exhausted PARTIAL solve
    #  would never reach the report generator, though IKBT has always reported
    #  partial solves.  symbolic_loop runs the identical passes and then reports
    #  what happened:  SUCCESS if anything was solved, FAILURE if nothing was.
    #  That FAILURE is the gate on the hybrid branch.  (Measured over all 32
    #  robots the deepest solve was UR5 at 9 passes under the old tree.)
    #
    #  Raised 10 -> 20 (BH, 2026-08-28) now that the solvers refuse equations
    #  that constrain nothing and fall through to invariant_gen instead:  a
    #  pass that used to end in a bogus solve now ends in a restock, so real
    #  solves take more passes.  A DH table is solved once, so passes are cheap
    #  in the only currency that matters.
    symLoop = symbolic_loop(solveRoutine, 20)
    symLoop.Name = "Symbolic Solver Loop" + tag
    symLoop.BHdebug = solver_debug
    nodes['symLoop'] = symLoop

    branch = b3.Sequence([nodes['clearState'], symLoop])
    branch.Name = "Symbolic Branch" + tag
    nodes['symbolicBranch'] = branch
    return branch


def build_default_bt(leaf_debug=False, solver_debug=False, nodes=None,
                     codegen=False):
    '''Build the standard IKBT tree.  Returns (BehaviorTree, nodes dict).

           Sequence[ analysis, report_gen ]

           analysis        = Priority[ symbolic_branch, hybrid_branch ]

           symbolic_branch = Sequence[ clear_state,
                                       symbolic_loop(x10, solveRoutine) ]

           hybrid_branch   = Sequence[ pieper_geom_report,   # always SUCCESS
                                       simplified_arm,
                                       install_simplified,
                                       symbolic_branch (2nd instance set) ]

           solveRoutine    = Sequence[ sub_transform,
                                       RepeatUntilSuccess(x6, Sequence[assigner,
                                                          sum_id, worktools]),
                                       updateL,
                                       comp_det ]

       Everything from solveRoutine down is unchanged.  What is new is the top:
       the solver now reports whether it got anywhere, so a SECOND strategy can
       be tried when it did not, and each branch can emit its own artifacts.

       codegen=False (the default) leaves output_gen_full inert, so building a
       tree has no file side effects and run_solver() still owns
       create_solution_set().  codegen=True hands the whole tail end to the
       tree, and the caller must then pass run_solver(..., create_solutions=False)
       -- create_solution_set() is not idempotent.

       Pass `nodes` (from a prior make_leaves() call) to build a tree over nodes
       you have already customized.'''

    if nodes is None:
        nodes = make_leaves(leaf_debug=leaf_debug, solver_debug=solver_debug)

    symbolicBranch = build_symbolic_branch(nodes, solver_debug=solver_debug)

    #  THE HYBRID BRANCH IS NO LONGER GATED ON PIEPER'S CONDITION (BH,
    #  2026-08-31).  It is gated on the symbolic solver having failed -- which
    #  is what b3.Priority below already does -- plus simplified_arm finding a
    #  usable candidate.  An OUTCOME test, rather than a proxy for one.
    #
    #  The old inner gate was `Sequence[no_pieper_id, ...]` -- a leaf that
    #  FAILed when a triple existed, sitting bare in a Sequence that aborts on
    #  FAILURE.  It admitted the branch only for an arm with NO triple of
    #  consecutive joint axes that intersects or is parallel, on the reasoning
    #  that an arm which HAS a triple and still failed symbolically is an IKBT
    #  defect rather than a geometry problem and must not be simplified.  Two
    #  things are wrong with that:
    #
    #    - It is the same fallacy this tree is careful to avoid in the other
    #      direction.  Pieper's condition is SUFFICIENT for a closed form and is
    #      not known to be necessary, which is why the ABSENCE of a triple must
    #      never gate the symbolic branch (9 of the 32 robots have no triple and
    #      solve completely).  The mirror holds:  the PRESENCE of a triple does
    #      not imply IKBT can crack the arm.  Measured over the currently
    #      unsolved set, ArmRobo (triples (2,3,4) and (4,5,6)), Panda ((1,2,3))
    #      and Raven-II ((1,2,3),(2,3,4),(3,4,5)) all satisfy the condition and
    #      all solve 0 of their unknowns -- so for the three robots the gate
    #      turned away, its answer was "this is our bug, so you get nothing".
    #    - The honest gate already sits one node later.  candidate_simplifications()
    #      skips triples that ALREADY qualify, so an arm whose every triple
    #      qualifies yields no candidate and simplified_arm FAILs by itself.  No
    #      Pieper test is needed to close the branch in that case.
    #
    #  What the leaf is still needed for is its OUTPUT:  pieper_latex has to be
    #  snapshotted from the TRUE robot before install_simplified swaps it, and
    #  "had a triple, failed symbolically, and the simplified arm solved" is
    #  precisely the report line that identifies a solver gap and points at it.
    #  So the leaf stays and was renamed to what it now does --
    #  pieper_geom_report, always SUCCESS, ticked for its side effects.
    #
    #  b3.Priority still tries the symbolic branch FIRST and unconditionally,
    #  which is what keeps those nine no-triple robots solving exactly.

    #  A SECOND, complete symbolic solver over its own leaf set, applied to the
    #  simplified arm.  Two instances, not one instance twice:  b3 keys per-node
    #  state on the blackboard by node id, so one instance in two tree positions
    #  really would collide (bt_problems() rejects it).  Two instances get that
    #  state fresh for free;  what does NOT come free is the unscoped
    #  application state, which is why each solver starts with clear_state.
    #
    #  The alternative was to hide the second solve inside a leaf.  This way the
    #  tree shows what actually happens.
    hybrid_nodes = rename_leaves(
        make_leaves(leaf_debug=leaf_debug, solver_debug=solver_debug),
        ' (hybrid)')
    symbolicBranch2 = build_symbolic_branch(hybrid_nodes, tag=' (hybrid)',
                                            solver_debug=solver_debug)

    #  Merge the second set in under suffixed keys, but ONLY the nodes that
    #  actually ended up in the branch.  make_leaves() builds the whole
    #  inventory, including the hybrid-branch leaves (no_pieper_id, report_gen, the
    #  stub), and the second set's copies of those are never wired to anything.
    #  The nodes dict is contracted to hold nodes reachable from the root -- a
    #  dangling entry means somebody built a leaf and forgot to connect it, which
    #  is a real defect the test suite checks for, so it must not be manufactured
    #  here.
    in_branch = _reachable(symbolicBranch2)
    for k, v in hybrid_nodes.items():
        if any(n is v for n in in_branch):
            nodes[k + '_hybrid'] = v

    #  pieper_geom_report heads the branch for its SIDE EFFECTS -- pieper_triples,
    #  pieper_ok, and the pieper_latex snapshot of the TRUE robot, which has to be
    #  taken before install_simplified swaps the Robot.  It always returns
    #  SUCCESS, so it needs no Priority([x, Succeeder()]) wrapper to keep this
    #  Sequence from treating a verdict as a gate:  there is no adverse verdict.
    #  NO TRAILING STUB.  This branch used to end in hybrid_stub, an always-FAIL
    #  leaf whose job was to withhold the report:  the closed form on the
    #  blackboard describes the DERIVED arm, and emitting it as though it were
    #  the real robot is the one thing this method must never do.  That is still
    #  true, and it is now handled where it belongs -- report_gen reads
    #  hybrid_source and writes a HYBRID report and a two-phase python module,
    #  every artifact naming the arm it actually describes.  Withholding the
    #  answer was only ever right while there was no honest way to present it.
    hybridBranch = b3.Sequence([nodes['pieperGeomReport'],
                                nodes['simplifiedArm'],
                                nodes['installSimplified'],
                                symbolicBranch2])
    hybridBranch.Name = "Hybrid Branch"
    nodes['hybridBranch'] = hybridBranch

    #  b3.Priority (the standard Selector/Fallback node) stops at its first
    #  non-FAILURE child, so the hybrid branch is ticked ONLY when the symbolic
    #  solver came up completely empty.  It can now SUCCEED, which is what lets
    #  report_gen -- the shared node after this Priority -- see a hybrid solve at
    #  all;  what keeps that honest is hybrid_source on the blackboard, which
    #  report_gen reads to name every artifact for the arm it describes.
    analysis = b3.Priority([symbolicBranch, hybridBranch])
    analysis.Name = "Analysis"
    nodes['analysis'] = analysis

    topnode = b3.Sequence([analysis, nodes['reportGen']])
    topnode.Name = "Top Node"
    nodes['topnode'] = topnode

    #  Opt-in, last, so it applies whether or not the caller passed `nodes`.
    nodes['reportGen'].enabled = bool(codegen)

    ikbt = b3.BehaviorTree()
    ikbt.root = topnode

    return ikbt, nodes
