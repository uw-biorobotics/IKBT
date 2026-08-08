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
    sacID = sinandcos_id()
    sacID.Name = "Sin Cos ID"
    sacID.BHdebug = False
    n['sacID'] = sacID

    sacSolver = sinandcos_solve()
    sacSolver.Name = "Sine Cosine Solver"
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


def build_default_bt(leaf_debug=False, solver_debug=False, nodes=None):
    '''Build the standard IKBT tree.  Returns (BehaviorTree, nodes dict).

           RepeatUntilSuccess(x10)
             Sequence[ sub_transform,
                       RepeatUntilSuccess(x6, Sequence[assigner, sum_id, worktools]),
                       updateL,
                       comp_det ]

       Pass `nodes` (from a prior make_leaves() call) to build a tree over nodes
       you have already customized.'''

    if nodes is None:
        nodes = make_leaves(leaf_debug=leaf_debug, solver_debug=solver_debug)

    worktools = build_worktools(nodes)
    worktools.Name = "Work Tools"
    nodes['worktools'] = worktools

    #  The SOA cases must be ID'd every pass so that algSol has equations to
    #  work on for the sum-of-angles variables.
    subtree = b3.RepeatUntilSuccess(
        b3.Sequence([nodes['asgn'], nodes['sumOfAnglesID'], worktools]), 6)
    subtree.Name = "Solve Subtree"
    nodes['subtree'] = subtree

    solveRoutine = b3.Sequence([nodes['sub_trans'], subtree,
                                nodes['updateLNode'], nodes['compDetect']])
    solveRoutine.Name = "Solve Routine"
    nodes['solveRoutine'] = solveRoutine

    topnode = b3.RepeatUntilSuccess(solveRoutine, 10)   # max 10 loops
    topnode.Name = "Top Node"
    nodes['topnode'] = topnode

    ikbt = b3.BehaviorTree()
    ikbt.root = topnode

    return ikbt, nodes
