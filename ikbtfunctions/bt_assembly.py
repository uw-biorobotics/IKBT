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
from ikbtleaves.x2y2_transform  import x2z2_transform
from ikbtleaves.sub_transform   import sub_transform
from ikbtleaves.sum_id          import sum_id      # detect and sub sum-of-angles
from ikbtleaves.updateL         import updateL
from ikbtleaves.comp_detect     import comp_det

#  Top-of-tree leaves.  symbolic_loop is the outer solve loop (it replaced
#  b3.RepeatUntilSuccess -- see below), output_gen_full is codegen-as-a-leaf,
#  and hybrid_stub is the placeholder for the hybrid symbolic-numeric branch.
from ikbtleaves.symbolic_loop   import symbolic_loop
from ikbtleaves.output_gen      import report_gen
from ikbtleaves.hybrid_ik       import (hybrid_stub, pieper_id, simplified_arm,
                                        install_simplified)
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

    ###  x^2 + y^2 trick from Craig (eqn 4.65) -- needed for Puma and KawasakiRS007L
    x2z2_Solver = x2z2_transform()
    x2z2_Solver.Name = 'X2Y2 transform'
    x2z2_Solver.BHdebug = False
    n['x2z2_Solver'] = x2z2_Solver

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
    invariantGen.enabled = False
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

    ###  Joint-axis geometry (Pieper's condition) -- the hybrid branch's gate.
    #  SUCCESS iff the arm has a triple, so under an Inverter it admits exactly
    #  the arms that lack the structure.  The report's geometry statement is
    #  generated by report_gen, not here:  this leaf ticks only on the hybrid
    #  path, and the statement has to appear whichever branch wins.
    pieperID = pieper_id()
    pieperID.BHdebug = leaf_debug
    n['pieperID'] = pieperID

    ###  Rank the DH changes that would give the arm a Pieper triple.
    #  Reads pieper_ok and refuses to act if the analysis was unreliable -- the
    #  Inverter above cannot tell "no triple" from "could not tell".
    simplifiedArm = simplified_arm()
    simplifiedArm.BHdebug = leaf_debug
    n['simplifiedArm'] = simplifiedArm

    ###  Build the derived robot and install it, so the solver that follows
    #  solves the SIMPLIFIED arm.  Fresh unknown objects and its own pickle name.
    installSimplified = install_simplified()
    installSimplified.BHdebug = leaf_debug
    n['installSimplified'] = installSimplified

    ###  The hybrid symbolic-numeric branch (futurework.md item 1) -- a stub.
    #  Always FAILs, so the branch is inert and the tree is observably identical
    #  to the one that had no branch at all.  See ikbtleaves/hybrid_ik.py.
    hybridStub = hybrid_stub()
    hybridStub.BHdebug = False
    n['hybridStub'] = hybridStub

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
       the numbers and the list of ways to bring that down.'''

    return b3.Priority([nodes['algSol'],
                        nodes['sc_tan'],
                        nodes['Simu_Eqn_Sol'],
                        nodes['sacSol'],
                        nodes['x2z2_Solver'],
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
    #  robots the deepest solve is UR5 at 9 passes, so 10 is a real budget.)
    symLoop = symbolic_loop(solveRoutine, 10)
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

           hybrid_branch   = Sequence[ Inverter(pieper_id),
                                       simplified_arm,
                                       install_simplified,
                                       symbolic_branch (2nd instance set),
                                       hybrid_stub ]

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

    #  The hybrid branch is gated on the arm actually LACKING the structure:
    #  Inverter(pieper_id) succeeds exactly when no triple of consecutive joint
    #  axes intersects or is parallel.
    #
    #  The gate is on the HYBRID only, never on the symbolic branch.  Pieper's
    #  condition is sufficient for a closed form to exist and is NOT known to be
    #  necessary -- measured, 9 of the 32 robots have no triple and still solve
    #  completely -- so a Sequence[pieper_id, symbolic_branch] would stop the
    #  symbolic solver ticking at all for those nine.  b3.Priority tries the
    #  symbolic branch FIRST and unconditionally, which is what keeps them.
    noPieper = b3.Inverter(nodes['pieperID'])
    noPieper.Name = "No Pieper Triple"
    nodes['noPieper'] = noPieper

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
    #  inventory, including the hybrid-branch leaves (pieper_id, report_gen, the
    #  stub), and the second set's copies of those are never wired to anything.
    #  The nodes dict is contracted to hold nodes reachable from the root -- a
    #  dangling entry means somebody built a leaf and forgot to connect it, which
    #  is a real defect the test suite checks for, so it must not be manufactured
    #  here.
    in_branch = _reachable(symbolicBranch2)
    for k, v in hybrid_nodes.items():
        if any(n is v for n in in_branch):
            nodes[k + '_hybrid'] = v

    hybridBranch = b3.Sequence([noPieper,
                                nodes['simplifiedArm'],
                                nodes['installSimplified'],
                                symbolicBranch2,
                                nodes['hybridStub']])
    hybridBranch.Name = "Hybrid Branch"
    nodes['hybridBranch'] = hybridBranch

    #  b3.Priority (the standard Selector/Fallback node) stops at its first
    #  non-FAILURE child, so the hybrid branch is ticked ONLY when the symbolic
    #  solver came up completely empty.  hybrid_stub still always FAILs, which
    #  keeps the branch from reporting a solution it cannot yet finish -- the
    #  numeric refinement of Phase F goes where the stub is.
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
