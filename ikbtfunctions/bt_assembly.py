#!/usr/bin/python
#
#   bt_assembly.py --  build the IKBT behavior tree
#
#   The tree can be built by anything that needs one:  the ikSolver.py CLI,
#   tests, batch runners, or a front end that swaps in its own solver set.
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


def make_leaves(leaf_debug=False, solver_debug=False):
    '''Construct every node the default tree needs and return them in a dict.

       Returning the dict, not just the tree, is what makes per-robot debugging
       tractable:

           bt, nodes = build_default_bt()
           nodes['tanID'].BHdebug = True'''

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
    parallelTriple.BHdebug = leaf_debug
    n['parallelTriple'] = parallelTriple

    ###  kinematic invariant generator.  Generalizes the x2y2 trick:  emits
    #  ||P||^2 / trace(R) / P.col invariants, which typically carry fewer
    #  unknowns than any raw element equation.  To switch it off:
    #
    #       bt, nodes = build_default_bt()
    #       nodes['invariantGen'].enabled = False
    #
    invariantGen = invariant_gen()
    invariantGen.Name = 'Invariant Generator'
    invariantGen.BHdebug = False
    #  ON.  The tree's only equation-restocking transform now that x2z2 is
    #  gone, and the leaves ahead of it in the Priority fall through only when
    #  they have all failed -- never, on a robot that solves cleanly.
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

    #  NOT named 'updateL':  that would rebind the imported class.
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
    #  LaTex/ and CodeGen/;  disabled it does nothing, and
    #  ik_driver.run_solver() owns the solution set instead.  EXACTLY ONE of the
    #  two must call create_solution_set():  it appends to
    #  unknown.LHSversionNames and is not idempotent.
    #
    #  Off by default so that a test building a tree does not overwrite the
    #  repo's generated artifacts.  ikSolver.py opts in.
    reportGen = report_gen()
    reportGen.BHdebug = False
    n['reportGen'] = reportGen

    ###  Joint-axis geometry (Pieper's condition) -- REPORTING ONLY.
    #  Answers "does this arm have three consecutive joint axes that intersect
    #  or are parallel", publishes pieper_triples / pieper_ok / pieper_latex,
    #  and ALWAYS returns SUCCESS -- having a triple does not mean IKBT can
    #  solve the arm, so it decides nothing.  The report's geometry statement
    #  comes from report_gen, because this leaf ticks only on the hybrid path
    #  and the statement must appear whichever branch wins.
    pieperGeomReport = pieper_geom_report()
    pieperGeomReport.BHdebug = leaf_debug
    n['pieperGeomReport'] = pieperGeomReport

    ###  Rank the DH changes that would give the arm a Pieper triple.
    #  THIS LEAF DECIDES whether the hybrid branch proceeds, since
    #  pieper_geom_report ahead of it cannot FAIL.  It refuses on pieper_ok
    #  False (an unparseable DH table) and on finding no usable candidate --
    #  the latter closes the branch on an arm that already satisfies Pieper
    #  everywhere, because candidate_simplifications() skips qualifying
    #  triples.
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

       Craig417 (2/4), Parkman13 (0/4) and UR5 (0/8) are still open.'''

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

       The tree carries the symbolic solver TWICE, over two separate instances,
       and they start out with identical Names -- two nodes logging under one
       label make the tick log unreadable.  bt_problems() flags it.'''

    for node in nodes.values():
        nm = getattr(node, 'Name', None)
        if isinstance(nm, str):
            node.Name = nm + suffix
    return nodes


def build_symbolic_branch(nodes, tag='', solver_debug=False):
    '''Assemble one complete symbolic solver from one leaf set.

           Sequence[ clear_state, symbolic_loop(x20, solveRoutine) ]

           solveRoutine = Sequence[ sub_transform,
                                    RepeatUntilSuccess(x6, Sequence[assigner,
                                                       sum_id, worktools]),
                                    updateL,
                                    comp_det ]

       Called TWICE by build_default_bt(), over two separate leaf sets:  the
       hybrid branch re-solves a simplified arm with its own solver, so the tree
       shows what happens rather than hiding it in a leaf.

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

    #  b3.Sequence aborts on its first FAILURE, so a failing sub_transform or
    #  solve subtree would stop updateL and comp_det from running at all -- and
    #  on a robot that solves nothing, that is every pass, leaving the tree with
    #  no termination logic in exactly the case that needs it.
    #
    #  Priority([x, Succeeder()]) swallows x's failure, so the pass always
    #  reaches updateL and the completion detector.  Both are safe on a pass
    #  that achieved nothing:  updateL re-scans, comp_det only decides whether
    #  to stop.
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

    #  The outer loop and its budget.  symbolic_loop reports what happened --
    #  SUCCESS if anything was solved, FAILURE if nothing was -- and that
    #  FAILURE is what admits the hybrid branch.  See IKdocs/DEV_NOTES.md for
    #  why it is not b3.RepeatUntilSuccess.
    #
    #  Budget 20 (raised from 10, BH 2026-08-28):  the solvers now refuse
    #  equations that constrain nothing and fall through to invariant_gen, so a
    #  pass that used to end in a bogus solve ends in a restock, and real
    #  solves take more passes.
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
                                       symbolic_loop(x20, solveRoutine) ]

           hybrid_branch   = Sequence[ pieper_geom_report,   # always SUCCESS
                                       simplified_arm,
                                       install_simplified,
                                       symbolic_branch (2nd instance set) ]

           solveRoutine    = Sequence[ sub_transform,
                                       RepeatUntilSuccess(x6, Sequence[assigner,
                                                          sum_id, worktools]),
                                       updateL,
                                       comp_det ]

       The solver reports whether it got anywhere, so a SECOND strategy can be
       tried when it did not, and each branch can emit its own artifacts.

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

    #  The hybrid branch is admitted by the symbolic solver having FAILED --
    #  which is what the b3.Priority below does -- plus simplified_arm finding a
    #  usable candidate.  It is NOT conditioned on Pieper's condition:  the
    #  condition is sufficient for a closed form and not known to be necessary,
    #  so neither its presence nor its absence predicts whether IKBT can crack
    #  a given arm.  See IKdocs/DEV_NOTES.md.
    #
    #  A SECOND, complete symbolic solver over its own leaf set, applied to the
    #  simplified arm.  Two INSTANCES, not one instance twice:  b3 keys per-node
    #  state on the blackboard by node id, so one instance in two tree positions
    #  would collide (bt_problems() rejects it).  Two instances get that state
    #  fresh;  the unscoped application state does not, which is why each solver
    #  starts with clear_state.
    hybrid_nodes = rename_leaves(
        make_leaves(leaf_debug=leaf_debug, solver_debug=solver_debug),
        ' (hybrid)')
    symbolicBranch2 = build_symbolic_branch(hybrid_nodes, tag=' (hybrid)',
                                            solver_debug=solver_debug)

    #  Merge the second set in under suffixed keys, but ONLY the nodes that
    #  ended up in the branch:  make_leaves() builds the whole inventory, and
    #  the second set's hybrid-branch leaves are never wired to anything.
    #  The nodes dict must hold only nodes reachable from the root -- a dangling
    #  entry means a leaf was built and never connected, which the test suite
    #  treats as a defect.
    in_branch = _reachable(symbolicBranch2)
    for k, v in hybrid_nodes.items():
        if any(n is v for n in in_branch):
            nodes[k + '_hybrid'] = v

    #  pieper_geom_report heads the branch for its SIDE EFFECTS -- pieper_triples,
    #  pieper_ok, and the pieper_latex snapshot of the TRUE robot, which must be
    #  taken before install_simplified swaps the Robot.  It always SUCCEEDs, so
    #  no Priority([x, Succeeder()]) wrapper is needed.
    #
    #  The branch ends with the solver, not a stub:  report_gen reads
    #  hybrid_source and writes a HYBRID report plus a two-phase python module,
    #  every artifact naming the arm it actually describes.
    hybridBranch = b3.Sequence([nodes['pieperGeomReport'],
                                nodes['simplifiedArm'],
                                nodes['installSimplified'],
                                symbolicBranch2])
    hybridBranch.Name = "Hybrid Branch"
    nodes['hybridBranch'] = hybridBranch

    #  b3.Priority (the standard Selector/Fallback) stops at its first
    #  non-FAILURE child, so the hybrid branch ticks ONLY when the symbolic
    #  solver came up empty.  Either branch can SUCCEED, and report_gen -- the
    #  shared node after this Priority -- reads hybrid_source to tell which.
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
